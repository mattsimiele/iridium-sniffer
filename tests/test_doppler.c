/* Independent circular-orbit regression fixtures. */
#include "../doppler_pos.c"
#include <assert.h>
#include <stdio.h>
static void state(double t,int sat,double *p,double *v) {
    double r=7158137, w=sqrt(GM_EARTH/(r*r*r)), a=w*(t-120)+(sat-1)*0.10;
    double u[3],enu[3][3],q[3],dq[3],tangent[3];
    geodetic_to_ecef(40,-75,0,u);double len=norm(u);
    ecef_to_enu_matrix(40,-75,enu);
    for(int j=0;j<3;j++) {
        u[j]/=len;tangent[j]=cos(sat*1.1)*enu[0][j]+sin(sat*1.1)*enu[1][j];
        q[j]=r*(u[j]*cos(a)+tangent[j]*sin(a));
        dq[j]=r*w*(-u[j]*sin(a)+tangent[j]*cos(a));
    }
    rotate_z(-OMEGA_EARTH*t,q,p);rotate_z(-OMEGA_EARTH*t,dq,v);
    v[0]+=OMEGA_EARTH*p[1];v[1]-=OMEGA_EARTH*p[0];
}
int main(int argc,char **argv) {
    FILE *fixture=argc==2?fopen(argv[1],"w"):NULL;
    if(argc==2)assert(fixture);
    if(fixture)fputs("timestamp_ns,sat_id,x4km,y4km,z4km,frequency_hz,sigma_hz\n",fixture);
    assert(seconds_between(1,2000000001ULL)==-2);
    assert(fabs(DOPPLER_IRA_HZ-1626270833.3333333)<1e-6);
    track tr={0};
    for(int i=0;i<49;i++){double v[3];state(i*5,0,tr.obs[i].p,v);tr.obs[i].t=(1000+i*5)*NS;tr.count++;}
    for(int i=0;i<49;i++) {
        double p[3],v[3],truth[3],tv[3],d[3];assert(orbit_state(&tr,i,p,v));state(i*5,0,truth,tv);
        for(int j=0;j<3;j++)d[j]=v[j]-tv[j];
        assert(norm(d)<3);
    }
    double x[4];geodetic_to_ecef(40,-75,100,x);x[3]=12;
    measurement m={0};state(70,1,m.p,m.v);double h[4];predict(&m,x,h);
    for(int j=0;j<4;j++){double a=x[j];x[j]=a+0.1;double plus=predict(&m,x,NULL);x[j]=a-0.1;double minus=predict(&m,x,NULL);x[j]=a;assert(fabs((plus-minus)/0.2-h[j])<1e-8);}
    doppler_pos_init();doppler_pos_set_height(100);
    for(int i=0;i<49;i++) for(int sat=0;sat<3;sat++) {
        state(i*5,sat,m.p,m.v);ira_data_t ira={0};ira.sat_id=sat;
        for(int j=0;j<3;j++)ira.pos_xyz[j]=(int)llround(m.p[j]/4000);
        double f=DOPPLER_IRA_HZ-predict(&m,x,NULL)*DOPPLER_IRA_HZ/C_LIGHT;
        if(fixture)fprintf(fixture,"%llu,%d,%d,%d,%d,%.17g,5\n",(unsigned long long)((1000+i*5)*NS),sat,ira.pos_xyz[0],ira.pos_xyz[1],ira.pos_xyz[2],f);
        assert(doppler_pos_add_observation(&ira,f,(1000+i*5)*NS,5));
        assert(!doppler_pos_add_observation(&ira,f,(1000+i*5)*NS,5));
        assert(!doppler_pos_add_observation(&ira,NAN,(1001+i*5)*NS,5));
    }
    doppler_solution_t sol;assert(doppler_pos_solve_at(1240*NS,&sol));
    double got[3],delta[3];geodetic_to_ecef(sol.lat,sol.lon,sol.alt,got);
    for(int j=0;j<3;j++)delta[j]=got[j]-x[j];
    printf("quantized synthetic 3D error: %.1f m; RMS %.2f m/s\n",norm(delta),sol.residual_rms_mps);
    assert(norm(delta)<5000);assert(sol.n_satellites==3);assert(isnan(sol.hdop));
    /* Outlier refit must retain array extent and count masks correctly. */
    int baseline_measurements=sol.n_measurements,baseline_rejected=sol.n_rejected;
    tracks[0].obs[20].freq += 2000;
    tracks[1].obs[30].freq -= 2000;
    assert(doppler_pos_solve_at(1240*NS,&sol));
    assert(sol.n_rejected==baseline_rejected+2 && sol.n_measurements==baseline_measurements-2);
    /* Two IDs with exactly the same instantaneous state have no spatial rank. */
    for(int i=0;i<12;i++){measurements[i]=m;measurements[i].active=1;measurements[i].sigma=20;measurements[i].sat=i%2;}
    double uncertainty;assert(!covariance(12,x,&uncertainty));
    assert(!doppler_pos_solve_at(1271*NS,&sol));assert(sol.status==DOPPLER_STALE && isnan(sol.lat));
    doppler_pos_init();assert(!doppler_pos_solve_at(1240*NS,&sol));assert(!sol.converged && isnan(sol.lat));
    /* Earlier arrivals must be inserted, not unsigned-underflow rejected. */
    ira_data_t late={0};late.sat_id=0;
    for(int j=0;j<3;j++)late.pos_xyz[j]=(int)llround(tr.obs[2].p[j]/4000);
    assert(doppler_pos_add_observation(&late,DOPPLER_IRA_HZ,1010*NS,5));
    for(int j=0;j<3;j++)late.pos_xyz[j]=(int)llround(tr.obs[1].p[j]/4000);
    assert(doppler_pos_add_observation(&late,DOPPLER_IRA_HZ,1005*NS,5));
    assert(tracks[0].obs[0].t==1005*NS && tracks[0].count==2);
    if(fixture)assert(fclose(fixture)==0);
    puts("Doppler regressions passed");return 0;
}
