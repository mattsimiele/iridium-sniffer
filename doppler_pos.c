/*
 * Stationary receiver positioning from Iridium IRA Doppler observations.
 * Original implementation Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental: broadcast orbit errors are correlated; reported uncertainty
 * is conditional on the local orbit fit and is NOT an integrity bound.
 */
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "doppler_pos.h"
#include "wgs84.h"

#define NS 1000000000ULL
#define MAX_SATS 128
#define PER_SAT 200
#define MAX_MEAS (MAX_SATS * PER_SAT)
#define WINDOW_S 600.0
#define FRESH_S 30.0
#define ORBIT_WINDOW_S 120.0
#define MIN_MEAS 12
/* Conservative engineering gates, to be calibrated with real captures. */
#define ORBIT_SIGMA_MPS 20.0
#define MAX_RMS_MPS 50.0
#define MAX_HORIZONTAL_SIGMA_M 10000.0

typedef struct { double p[3], freq, sigma_hz; uint64_t t; } observation;
typedef struct { observation obs[PER_SAT]; int count; } track;
typedef struct { double p[3], v[3], rate, sigma; int sat, active; } measurement;
static track tracks[MAX_SATS];
static measurement measurements[MAX_MEAS];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t latest_ns;
static double latest_monotonic;
static double height = NAN, previous[4];
static int have_previous;

static double seconds_between(uint64_t a, uint64_t b) {
    return a >= b ? (double)(a-b)/1e9 : -(double)(b-a)/1e9;
}
static double monotonic_seconds(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec/1e9;
}
static double dot(const double *a, const double *b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static double norm(const double *a) { return sqrt(dot(a,a)); }
static void rotate_z(double angle, const double *p, double *q) {
    double c=cos(angle), s=sin(angle);
    q[0]=c*p[0]-s*p[1]; q[1]=s*p[0]+c*p[1]; q[2]=p[2];
}
/* Small scaled systems only. Reject weak pivots instead of damping away rank
 * loss. Navigation uses column scaling; orbit time is scaled to [-1,1]. */
static int inverse4(const double a[4][4], double inverse[4][4]) {
    double b[4][8], scale=0;
    for (int i=0;i<4;i++) for(int j=0;j<4;j++) {
        if (!isfinite(a[i][j])) return 0;
        b[i][j]=a[i][j]; b[i][j+4]=(i==j);
        if (fabs(a[i][j])>scale) scale=fabs(a[i][j]);
    }
    if (scale==0) return 0;
    for(int k=0;k<4;k++) {
        int pivot=k;
        for(int i=k+1;i<4;i++) if(fabs(b[i][k])>fabs(b[pivot][k])) pivot=i;
        if(fabs(b[pivot][k]) < scale*1e-12) return 0;
        for(int j=0;j<8;j++) {double tmp=b[k][j];b[k][j]=b[pivot][j];b[pivot][j]=tmp;}
        double d=b[k][k];
        for(int j=0;j<8;j++) b[k][j]/=d;
        for(int i=0;i<4;i++) if(i!=k) {
            double f=b[i][k];
            for(int j=0;j<8;j++) b[i][j]-=f*b[k][j];
        }
    }
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) inverse[i][j]=b[i][j+4];
    return 1;
}

/* Fit a cubic trajectory in a common inertial frame oriented like ECEF at
 * the observation epoch. Rotate each ECEF sample by omega*(t-t0) FIRST.
 * Return the fitted state in ECEF, including -omega x position in velocity.
 * No absolute UTC/GMST is needed for this relative frame transformation.
 * This is a local smoother, not a precision ephemeris replacement. */
static int orbit_state(const track *tr, int index, double *p, double *v) {
    double a[4][4]={{0}}, rhs[4][3]={{0}}, inv[4][4];
    uint64_t t0=tr->obs[index].t;
    double lo=0, hi=0; int n=0;
    for(int i=0;i<tr->count;i++) {
        double dt=seconds_between(tr->obs[i].t,t0);
        if(fabs(dt)>ORBIT_WINDOW_S) continue;
        double u=dt/ORBIT_WINDOW_S, x[4]={1,u,u*u,u*u*u}, q[3];
        rotate_z(OMEGA_EARTH*dt,tr->obs[i].p,q);
        for(int j=0;j<4;j++) {
            for(int k=0;k<4;k++) a[j][k]+=x[j]*x[k];
            for(int k=0;k<3;k++) rhs[j][k]+=x[j]*q[k];
        }
        if(dt<lo) lo=dt;
        if(dt>hi) hi=dt;
        n++;
    }
    if(n<8 || hi-lo<60 || !inverse4(a,inv)) return 0;
    for(int k=0;k<3;k++) {
        p[k]=v[k]=0;
        for(int j=0;j<4;j++) {p[k]+=inv[0][j]*rhs[j][k];v[k]+=inv[1][j]*rhs[j][k]/ORBIT_WINDOW_S;}
    }
    v[0]+=OMEGA_EARTH*p[1]; v[1]-=OMEGA_EARTH*p[0];
    return norm(p)>7050e3 && norm(p)<7250e3 && norm(v)>6500 && norm(v)<8500;
}

/* All positions/velocities are ECEF. Receiver velocity is zero (stationary).
 * x[3] is common frequency bias expressed in range-rate units, m/s. */
static double predict(const measurement *m, const double x[4], double h[4]) {
    double l[3]={m->p[0]-x[0],m->p[1]-x[1],m->p[2]-x[2]}, r=norm(l);
    if(r<1 || !isfinite(r)) return NAN;
    double rate=dot(l,m->v)/r;
    if(h) {
        for(int j=0;j<3;j++) h[j]=-m->v[j]/r+l[j]*rate/(r*r);
        h[3]=1;
    }
    return rate+x[3];
}
static int height_row(const double *x, double h[4], double *residual) {
    if(!isfinite(height)) return 0;
    double lat,lon,alt;
    ecef_to_geodetic(x,&lat,&lon,&alt);
    if(!isfinite(alt)) return 0;
    lat*=M_PI/180;lon*=M_PI/180;
    h[0]=cos(lat)*cos(lon);h[1]=cos(lat)*sin(lon);h[2]=sin(lat);h[3]=0;
    *residual=height-alt;
    return 1;
}
static double objective(int n, const double *x) {
    double cost=0;
    for(int i=0;i<n;i++) if(measurements[i].active) {
        double e=(measurements[i].rate-predict(&measurements[i],x,NULL))/measurements[i].sigma;
        if(!isfinite(e)) return INFINITY;
        cost+=e*e;
    }
    double h[4],e;
    if(height_row(x,h,&e)) cost+=e*e/(30*30);
    return cost;
}
static int normal_matrix(int n,const double *x,double a[4][4],double b[4]) {
    memset(a,0,16*sizeof(double));memset(b,0,4*sizeof(double));
    for(int i=0;i<=n;i++) {
        double h[4],e,sigma;
        if(i==n) {if(!height_row(x,h,&e)) continue;sigma=30;}
        else {
            if(!measurements[i].active) continue;
            e=measurements[i].rate-predict(&measurements[i],x,h);
            sigma=measurements[i].sigma;
        }
        if(!isfinite(e)) return 0;
        for(int j=0;j<4;j++) {
            b[j]+=h[j]*e/(sigma*sigma);
            for(int k=0;k<4;k++) a[j][k]+=h[j]*h[k]/(sigma*sigma);
        }
    }
    return 1;
}
/* Scaled LM with actual objective acceptance. One implementation for initial
 * and robust re-solves. Unregularized covariance/rank are checked separately. */
static int optimize(int n,double *x) {
    double lambda=0.001;
    for(int iter=0;iter<200;iter++) {
        double a[4][4],b[4],s[4],scaled[4][4],inv[4][4],delta[4]={0},candidate[4];
        if(!normal_matrix(n,x,a,b)) return 0;
        for(int j=0;j<4;j++) {if(a[j][j]<=0) return 0;s[j]=sqrt(a[j][j]);}
        for(int j=0;j<4;j++) for(int k=0;k<4;k++) scaled[j][k]=a[j][k]/(s[j]*s[k]);
        for(int j=0;j<4;j++) scaled[j][j]+=lambda;
        if(!inverse4(scaled,inv)) return 0;
        for(int j=0;j<4;j++) for(int k=0;k<4;k++) delta[j]+=inv[j][k]*b[k]/s[k]/s[j];
        double step=norm(delta),factor=step>200000?200000/step:1;
        for(int j=0;j<4;j++) {delta[j]*=factor;candidate[j]=x[j]+delta[j];}
        double old=objective(n,x), next=objective(n,candidate);
        if(isfinite(next) && next<=old+1e-12) {
            memcpy(x,candidate,4*sizeof(double));
            lambda=fmax(lambda*.3,1e-9);
            if(norm(delta)<0.1 && fabs(delta[3])<0.001) return 1;
        } else {lambda*=10;if(lambda>1e10) return 0;}
    }
    return 0;
}
static int active_count(int n,int *satellites) {
    int used[MAX_SATS]={0},count=0;*satellites=0;
    for(int i=0;i<n;i++) if(measurements[i].active) {count++;used[measurements[i].sat]=1;}
    for(int i=0;i<MAX_SATS;i++) *satellites+=used[i];
    return count;
}
static int covariance(int n,const double *x,double *horizontal) {
    double a[4][4],b[4],s[4],scaled[4][4],inv[4][4],r[3][3];
    if(!normal_matrix(n,x,a,b)) return 0;
    for(int j=0;j<4;j++) {if(a[j][j]<=0)return 0;s[j]=sqrt(a[j][j]);}
    for(int j=0;j<4;j++) for(int k=0;k<4;k++) scaled[j][k]=a[j][k]/(s[j]*s[k]);
    if(!inverse4(scaled,inv)) return 0;
    double lat,lon,alt,var=0;
    ecef_to_geodetic(x,&lat,&lon,&alt);ecef_to_enu_matrix(lat,lon,r);
    for(int axis=0;axis<2;axis++) for(int j=0;j<3;j++) for(int k=0;k<3;k++)
        var+=r[axis][j]*inv[j][k]/s[j]/s[k]*r[axis][k];
    if(!isfinite(var)||var<=0)return 0;
    *horizontal=sqrt(var);return 1;
}

void doppler_pos_init(void) {
    pthread_mutex_lock(&lock);
    memset(tracks,0,sizeof(tracks));latest_ns=0;have_previous=0;
    height=NAN;latest_monotonic=0;
    pthread_mutex_unlock(&lock);
}
void doppler_pos_set_height(double h) {
    pthread_mutex_lock(&lock);height=(isfinite(h)&&h>=-500&&h<=10000)?h:NAN;
    have_previous=0;pthread_mutex_unlock(&lock);
}
int doppler_pos_add_observation(const ira_data_t *ira,double f,uint64_t t,double sigma) {
    if(!ira || ira->sat_id<0 || ira->sat_id>=MAX_SATS || !t ||
       !isfinite(f)||fabs(f-DOPPLER_IRA_HZ)>60000 || !isfinite(sigma)||sigma<=0) return 0;
    observation o={.freq=f,.t=t,.sigma_hz=sigma};
    for(int j=0;j<3;j++) {if(ira->pos_xyz[j]<-2048||ira->pos_xyz[j]>2047)return 0;o.p[j]=4000.0*ira->pos_xyz[j];}
    if(norm(o.p)<7050e3 || norm(o.p)>7250e3) return 0;
    pthread_mutex_lock(&lock);
    track *tr=&tracks[ira->sat_id];
    if(latest_ns && seconds_between(latest_ns,t)>WINDOW_S) {pthread_mutex_unlock(&lock);return 0;}
    int k=0;while(k<tr->count && tr->obs[k].t<t)k++;
    if(k<tr->count && tr->obs[k].t==t) {pthread_mutex_unlock(&lock);return 0;}
    /* Quantization-aware speed check against both temporal neighbors. */
    for(int i=k-1;i<=k;i++) if(i>=0 && i<tr->count) {
        double dt=fabs(seconds_between(t,tr->obs[i].t));
        double d[3];for(int j=0;j<3;j++)d[j]=o.p[j]-tr->obs[i].p[j];
        if(dt<120 && norm(d)>10000*dt+7000) {pthread_mutex_unlock(&lock);return 0;}
    }
    if(tr->count && seconds_between(t,tr->obs[tr->count-1].t)>WINDOW_S) {tr->count=0;k=0;}
    if(tr->count==PER_SAT) {
        if(k==0) {pthread_mutex_unlock(&lock);return 0;}
        memmove(tr->obs,tr->obs+1,(PER_SAT-1)*sizeof(observation));tr->count--;k--;
    }
    memmove(tr->obs+k+1,tr->obs+k,(tr->count-k)*sizeof(observation));tr->obs[k]=o;tr->count++;
    if(t>latest_ns){latest_ns=t;latest_monotonic=monotonic_seconds();}
    pthread_mutex_unlock(&lock);return 1;
}
void doppler_pos_add_measurement(const ira_data_t *ira,double f,uint64_t t) {
    (void)doppler_pos_add_observation(ira,f,t,50.0);
}
static void empty_solution(doppler_solution_t *out) {
    memset(out,0,sizeof(*out));out->lat=out->lon=out->alt=out->hdop=NAN;
    out->residual_rms_mps=out->horizontal_sigma_m=out->clock_drift_mps=NAN;
}
/* Called with lock held: immutable buffers throughout each solve. */
static int solve_locked(uint64_t now,doppler_solution_t *out) {
    empty_solution(out);out->epoch_ns=latest_ns;
    if(!latest_ns)return 0;
    if(now<latest_ns || seconds_between(now,latest_ns)>FRESH_S){out->status=DOPPLER_STALE;return 0;}
    int n=0;
    for(int sat=0;sat<MAX_SATS;sat++) {
        track *tr=&tracks[sat];
        for(int i=0;i<tr->count;i++) {
            observation *o=&tr->obs[i];
            if(seconds_between(now,o->t)>WINDOW_S)continue;
            measurement *m=&measurements[n];
            if(!orbit_state(tr,i,m->p,m->v))continue;
            m->rate=-(C_LIGHT/DOPPLER_IRA_HZ)*(o->freq-DOPPLER_IRA_HZ);
            m->sigma=hypot(ORBIT_SIGMA_MPS,o->sigma_hz*C_LIGHT/DOPPLER_IRA_HZ);
            m->sat=sat;m->active=1;n++;
        }
    }
    out->n_measurements=active_count(n,&out->n_satellites);
    if(n<MIN_MEAS||out->n_satellites<2)return 0;
    double x[4]={0};
    if(have_previous)memcpy(x,previous,sizeof(x));
    else {
        for(int i=0;i<n;i++)for(int j=0;j<3;j++)x[j]+=measurements[i].p[j]/n;
        double lat,lon,alt;ecef_to_geodetic(x,&lat,&lon,&alt);
        if(!isfinite(lat)||!isfinite(lon)){out->status=DOPPLER_GEOMETRY;return 0;}
        geodetic_to_ecef(lat,lon,isfinite(height)?height:0,x);
    }
    out->status=DOPPLER_REJECTED;
    /* Fixed array extent n, independent active mask. Never reuse stale rows. */
    for(int pass=0;pass<4;pass++) {
        if(!optimize(n,x)){have_previous=0;return 0;}
        int removed=0;
        for(int i=0;i<n;i++)if(measurements[i].active) {
            double residual=fabs(measurements[i].rate-predict(&measurements[i],x,NULL));
            if(residual>4*measurements[i].sigma){measurements[i].active=0;removed++;}
        }
        out->n_measurements=active_count(n,&out->n_satellites);out->n_rejected=n-out->n_measurements;
        if(out->n_measurements<MIN_MEAS||out->n_satellites<2||out->n_rejected>n/4)return 0;
        if(!removed)break;
        if(pass==3)return 0; /* never publish an unrefitted candidate */
    }
    double sum=0;
    for(int i=0;i<n;i++)if(measurements[i].active){double e=measurements[i].rate-predict(&measurements[i],x,NULL);sum+=e*e;}
    out->residual_rms_mps=sqrt(sum/out->n_measurements);
    if(!covariance(n,x,&out->horizontal_sigma_m) || out->horizontal_sigma_m>MAX_HORIZONTAL_SIGMA_M){out->status=DOPPLER_GEOMETRY;return 0;}
    double lat,lon,alt;ecef_to_geodetic(x,&lat,&lon,&alt);
    if(!isfinite(alt)||alt< -500||alt>10000||out->residual_rms_mps>MAX_RMS_MPS)return 0;
    if(have_previous){double d[3];for(int j=0;j<3;j++)d[j]=x[j]-previous[j];if(norm(d)>50000){have_previous=0;return 0;}}
    memcpy(previous,x,sizeof(x));have_previous=1;
    out->lat=lat;out->lon=lon;out->alt=alt;out->clock_drift_mps=x[3];
    out->converged=1;out->status=DOPPLER_VALID;return 1;
}
int doppler_pos_solve_at(uint64_t now,doppler_solution_t *out) {
    pthread_mutex_lock(&lock);int ok=solve_locked(now,out);pthread_mutex_unlock(&lock);return ok;
}
int doppler_pos_solve(doppler_solution_t *out) {
    pthread_mutex_lock(&lock);
    double age=latest_ns?fmax(0,monotonic_seconds()-latest_monotonic):0;
    uint64_t now=latest_ns;
    if(age>FRESH_S){empty_solution(out);out->epoch_ns=latest_ns;out->status=DOPPLER_STALE;pthread_mutex_unlock(&lock);return 0;}
    int ok=solve_locked(now,out);pthread_mutex_unlock(&lock);return ok;
}
const char *doppler_status_name(doppler_status_t s) {
    switch(s){case DOPPLER_VALID:return "valid";case DOPPLER_STALE:return "stale";
    case DOPPLER_GEOMETRY:return "poor_geometry";case DOPPLER_REJECTED:return "rejected";
    default:return "collecting";}
}
