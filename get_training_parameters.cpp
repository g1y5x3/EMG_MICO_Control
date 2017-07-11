// ==================== Header files ====================
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <wiringPi.h>
#include <bcm2835.h> 
#include <itpp/itbase.h>
#include "GUSSS.h"

using namespace std;
using namespace itpp;


#define MAX_SIG						5				// maximum possible number of signatures
#define NRS 		    			4				// default number of signatures, previously it was 4
#define WGTS_OPT					0				// feature weights' option (0/same, 1/different)


int main(int argc, char **argv)
{
	char    directory[300]="Sample_Dir";		// target directory name for storing the data
	
    // --- Final Gesture Variables for training/Reconigton ---
	mat 	emg_signal;							
	mat 	data_all[MAX_SIG];
	
	// ----- Training Parameters -----
    int N;
    int L = 1875;    
    ivec 	segD;
	int 	S;							// default number of signatures considered (3 or 4)
	mat 	Sigs;
	mat 	mav_aves[MAX_SIG];
	mat 	mav_covs[MAX_SIG];
	mat 	zc_aves[MAX_SIG];
	mat		zc_covs[MAX_SIG];
	mat		gr_aves[MAX_SIG];
	mat		gr_covs[MAX_SIG];
	double 	zc_thr;
	vec 	feat_wgts;
	int 	weightOpt = WGTS_OPT;				// default option for feature weights (Don't use them)
    int     num_class = 4;                      // default number of training classes
	int 	D = 3;								// default number of segments for the signals.
    // ----- File Variables -----
	FILE *fdata_all = NULL;
	FILE *flabel = NULL;
    char fullname[100];
    char *gesture_names[5] = {(char*)"Gesture1", (char*)"Gesture2", (char*)"Gesture3",(char*)"Gesture4", (char*)"Gesture5"};
    float temp = 0.0;

    int i = 1;
    int labels[5];
    int temp_index;
    // ----- Reading Inputs
    while (i < argc) 
	{
		if ((argv[i][0] == '-') && (argv[i][1] == 'd')) 
		{
			sprintf(directory, "%s", argv[i+1]);
			i = i + 2; 
		}
        
		else
			i++;

	}
    
    printf("The number of training data for each gesture:");
    scanf("%d", &N);
    printf("The number of gestures:");
    scanf("%d", &S);

    printf("Assign labels:\n");
    printf("Gesture1:");
    scanf("%d",&labels[0]);
    printf("Gesture2:");
    scanf("%d",&labels[1]);
    printf("Gesture3:");
    scanf("%d",&labels[2]);
    printf("Gesture4:");
    scanf("%d",&labels[3]);

    for(int g = 0; g < S; g++)
    {
        // Initialize the data variable
        temp_index = labels[g] - 1;
        data_all[temp_index] = zeros(N,L);
        
        for(i = 0; i < N; i++)
        {
		    sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/%s_%03d.txt", directory, gesture_names[g], i+1);
            fdata_all = fopen(fullname, "r");
				
            for(int k = 0; k < L; k++)	// samples' loop
			{
				fscanf(fdata_all, "%f", &temp);
                data_all[temp_index](i,k) = temp;
			}
			fclose(fdata_all);
	
        }

//        cout << data_all[g] << endl;

    }

    printf("Finished Loading Data!\n");
 
    segD = zeros_i(D+1);
   	for(i = 0; i <= D; i++)
		segD(i) = round_i(1.0 * i * L / D);


    i = get_training_parameters_MM(data_all,&segD, &Sigs,S,mav_aves,mav_covs,zc_aves, zc_covs, gr_aves, gr_covs, &zc_thr, &feat_wgts, weightOpt, directory);
  
    // Save all the label assignments
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Param/label_map.txt", directory);
    flabel = fopen(fullname, "w");
    for(i = 0; i < S; i++)
        fprintf(flabel, "%d\n", labels[i]);

    fclose(flabel);

    printf("Finished Getting Training Parameters!\n");

	return 0;
}
