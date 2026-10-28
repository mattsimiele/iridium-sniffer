#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "doppler_pos.h"
static void output(uint64_t now) {
    doppler_solution_t s;int ok=doppler_pos_solve_at(now,&s);
    printf("{\"time_ns\":%" PRIu64 ",\"status\":\"%s\",\"measurements\":%d,\"satellites\":%d,\"rejected\":%d",
        now,doppler_status_name(s.status),s.n_measurements,s.n_satellites,s.n_rejected);
    if(ok)printf(",\"lat\":%.9f,\"lon\":%.9f,\"alt_m\":%.3f,\"rms_mps\":%.5f,\"horizontal_sigma_m\":%.3f,\"bias_mps\":%.5f",s.lat,s.lon,s.alt,s.residual_rms_mps,s.horizontal_sigma_m,s.clock_drift_mps);
    puts("}");
}
/* Private normalized stream from replay.py: H height, O observation, S epoch. */
int main(void) {
    char line[512],extra;doppler_pos_init();
    while(fgets(line,sizeof(line),stdin)) {
        uint64_t t;double f,sigma,h;ira_data_t ira={0};
        if(line[0]=='H' && sscanf(line,"H %lf %c",&h,&extra)==1)doppler_pos_set_height(h);
        else if(line[0]=='S' && sscanf(line,"S %" SCNu64 " %c",&t,&extra)==1)output(t);
        else if(line[0]=='O' && sscanf(line,"O %" SCNu64 " %d %d %d %d %lf %lf %c",&t,&ira.sat_id,&ira.pos_xyz[0],&ira.pos_xyz[1],&ira.pos_xyz[2],&f,&sigma,&extra)==7) {
            if(!doppler_pos_add_observation(&ira,f,t,sigma))fprintf(stderr,"Rejected observation at %" PRIu64 "\n",t);
        } else {fputs("Malformed replay stream\n",stderr);return 2;}
    }
    return ferror(stdin)||ferror(stdout)?2:0;
}
