/**
 * @file ntp_read.c
 */

#include <ntp_read.h>

#include <stdio.h>
#include <stdlib.h>

double ntp_offset;

//To get synchronized time, symply ADD this offset value to all timestamps received.  
double get_ntp_offset(){
    FILE *ntpFp;
    char ntpResult[NTP_PATH_SIZE];
    double offsetms = 0;

    //Parse string from NTP
    ntpFp = popen("/usr/bin/ntpq -pn", "r");

    //Only try to read if it's not a NULL pointer
    if(ntpFp != NULL){
        //Skip first two LINES of ntpq's output
        for(unsigned i = 0; i < 2; ++i){
            if(fgets(ntpResult, NTP_PATH_SIZE, ntpFp) == NULL){
                return NTP_ERR_NUM;
            }
        }

        //On the third line, skip everything before the OFFSET
        for(unsigned i = 0; i < 8; ++i){
            if(fscanf(ntpFp, "%s", ntpResult) == EOF){
                return NTP_ERR_NUM;
            }
        }

        //Next thing in the buffer should be the offset for the first NTP server defined in the /etc/ntp.conf file
        
        if(fscanf(ntpFp, "%s", ntpResult) != EOF){
            offsetms = atof(ntpResult);
        }
             
    }//if(ntpFp != NULL)

    //Close the filestream
    pclose(ntpFp);

    return offsetms;

}