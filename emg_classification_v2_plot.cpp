/*
 * 			   Name: emg_classification
 *
 *		 Created on: Auguest 6, 2016 (Originally Nov 4, 2013)
 *			 Author: Yixiang Gao
 *			Version: I cannot remember how many people have touched it.
 *
 *  Copyright (C) 2016 ViGIR - Vision Guided and Intelligent Robotics Lab
 *                             (http://vigir.missouri.edu)
 * 
 *  Written by Luis Alberto Rivera <larmbc@mail.missouri.edu>
 *  Modified by G. N. DeSouza <DeSouzaG@missouri.edu>
 *  Modified by Yixiang Gao <yg5d6@mail.missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *         
 * 			
 * 			keep this copyright notice,
 *          do not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 */

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
#include "enum.h"
#include "spi.h"
#include "ads1256.h"
#include "GUSSS.h"


//==================== Parameters ====================

//---------- Not sure about the functionality ----------
#define WGTS_OPT					0				// feature weights' option (0/same, 1/different)


//---------- Moving average window ----------
#define NOISE_WIN					200				// Set to have approx.  ~ 150 ms  // when sampling
#define THR_WIN						2000			// For the threshold window. Set to ~ 2 sec.
#define SIG_FIRST_PART	 			100				// Go back from the flagindex, count as signal pts.
#define HIGH_THR_FACTOR				1.0				// Moving average window threshold
#define LOW_THR_FACTOR				1.0

//---------- Training and testing configuration ----------
#define NUMBER_CHANNELS				1				// the number of channels being used
#define MAX_SIG						5				// maximum possible number of signatures
#define NRS 		    			1				// default number of signatures, previously it was 4
#define TRAIN_SAMPLES				10				// suggested training sample is 20 per gesture
#define MAX_NR_TR		   			50				// maximum number of training signals allowed
#define SIG_DURATION	            500				// default signal length, in mili-seconds
#define BUFFER_SIZE					3750			// Circular Buffer size

//---------- ADC(ads1256) configuration ----------
#define SAMPLING_FACTOR	   			3.75			// ADC sampling rate of the AVR, in kilo-samples/second

//---------- Debug configuration ----------
#define PRINTF						1				// 0 to prevent any printf, 1 to print the main messages
													// 2 to print every detail


//==================== Main ====================
int main(int argc, char **argv) 
{

#if PRINTF > 0
	printf("\nWelcome to ViGIR Lab!\n");
	fflush(stdout);
#endif

// ---------------- Variables ----------------


	// ----- General Variables -----
	char    directory[300]="Sample_Dir";		// target directory name for storing the data
	char    gesture_folder[300]="Sample";
	int     count = 0;							// represent different states while collecting gestures
	int     i = 1, j = 0;						// general loop variable
	int 	N = TRAIN_SAMPLES;					// default number of training/testing signals
//	double 	low_thr_fac = LOW_THR_FACTOR;
	int 	L;									// for the number of samples per signal
	int 	g;									// g - gesture, loop variable
	int 	k = 0;
	int 	train_was_req = 0;					// indicates if training is required (1) or not (0)
	int 	flagindex = 0;						// the buffer index when there is a gesture detected!
	int 	D = 3;								// default number of segments for the signals.
	
	// ----- Gesture Parameters -----
/*
 * 		noiseV: 		to have a noise level in Volts; factor: for converting read values to Volts
 *	   	thresholdV: 	used to detect when the sEMG signal started (in Volts)
 *	   	gaV: 		this value (in Volts) is added to the noiseV to get the thresholdV: 0.010-0.035
 *					when using the forearm muscle, 0.010 when using the eyebrows.
 *		gamV: 		equivalent to gaV, but in mili-volts. Used to capture an input of the program.
 *		mov_ave 		(moving average) and  are used to detect the sEMG signals	
 */	
	// --- Variables for storing the threshold ---
	float	noiseV[NUMBER_CHANNELS];
	float 	thresholdV[NUMBER_CHANNELS];
	float 	gaV;
	int     gamV = 100;													// default threshold constant, in mili-Volts
	
	// --- Variables for capturing noise signals ---
	float      *buffer_noise_cont[NUMBER_CHANNELS];		
	float 	average_noise;												// average noise for the noise buffer
	
	// --- Variables for capturing the actual acitivity signals ---
	float     *buffer[NUMBER_CHANNELS];
	int     buffer_size = round_i(BUFFER_SIZE*SAMPLING_FACTOR);			// 4s worth of data	
	int 	buffer_index = 0;	
	int     noise_win = round_i(NOISE_WIN*SAMPLING_FACTOR);				// 150ms worth of data	
	float 	mov_ave[NUMBER_CHANNELS];
	float 	dc_offset[NUMBER_CHANNELS];
	bool 	detected_resting;
	int     sig_first_part = round_i(SIG_FIRST_PART*SAMPLING_FACTOR);	// 65 mili-seconds worth of data
	int     sig_duration = SIG_DURATION;								// default value, may change if input by user	
	int     thr_win = round_i(THR_WIN*SAMPLING_FACTOR);					// threshold window, 2 seconds worth of data
	float 	average_signal;												// average signal to center the samples for the gesture 
    int     dummy;
	
    char *gesture_names[5] = {(char*)"Gesture1", (char*)"Gesture2", (char*)"Gesture3",(char*)"Gesture4", (char*)"Gesture5"};

	// --- Final Gesture Variables for training/Reconigton ---
	mat 	emg_signal;							
	mat 	data_all[MAX_SIG];
	
	// ----- Training Parameters -----
	ivec 	segD;
	int 	S = NRS;							// default number of signatures considered (3 or 4)
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
	
	// ----- 	 -----
	int cl_opt = 0;								// classification option (0 - distance, 1 - confidence)
	char mode = 'e';							// default mode (exit)

	FILE *fdata_all = NULL;
    char fullname[300];

// ---------------- Allocating Memories for the gesture variables ----------------

	if ((buffer[0] = (float *)malloc(buffer_size*sizeof(float))) == NULL) 
	{
		printf("Error allocating buffer\n");
		exit(-1); 
	}  // added sanity checks on the return of the malloc

	// Allocate noise buffers
	if ((buffer_noise_cont[0] = (float *)malloc(buffer_size*sizeof(float))) == NULL) 
	{
		printf("Error allocating noise buffer\n");
		exit(-1); 
	}  // added sanity checks on the return of the malloc

// ---------------- Handling of the Inputs ----------------

#if PRINTF > 1
	printf("\nNow Handling of the inputs (V3)\n");
	fflush(stdout);
#endif

	if((argc < 2) || (argc > 30)) 
	{
		printf("Usage:\nsudo %s <-d directory> <-t thr_offset> "
				"<-L sig_duration> \n", argv[0]);
		return -1; 
	}
	else while (i < argc) 
	{
		if((argv[i][0] == '-') && (argv[i][1] == 'm'))
		{
			for(j = 0; j < (int)strlen(argv[i+1]); j++)
			{
				if(argv[i+1][j] == 't')
					train_was_req = 1;
				if((argv[i+1][j] == 'r')||(argv[i+1][j] == 'c')||(argv[i+1][j] == 's'))
					mode = argv[i+1][j];
			}
			i = i + 2;
		}
		if ((argv[i][0] == '-') && (argv[i][1] == 'd')) 
		{
			sprintf(directory, "%s", argv[i+1]);
			i = i + 2; 
		}
		else if ((argv[i][0] == '-') && (argv[i][1] == 'f')) 
		{
			sprintf(gesture_folder, "%s", argv[i+1]);
			i = i + 2; 
		}
		else if ((argv[i][0] == '-') && (argv[i][1] == 't')) 
		{
			gamV = atoi(argv[i+1]);
			i = i + 2; 
		}
		else if ((argv[i][0] == '-') && (argv[i][1] == 'L')) 
		{
			sig_duration = atoi(argv[i+1]);
			if (sig_duration < 200)
				sig_duration = 200;
			else if (sig_duration > 500)
				sig_duration = 500;
			i = i + 2;
		}
		else
			i++;

	}

#if PRINTF > 1
	printf("\nDone Handling of the inputs\n");
	fflush(stdout);
#endif	

// ---------------- Configure the ADC ----------------

#if PRINTF > 0
	printf("\nResetting the ADC.\n");
	fflush(stdout);
#endif
	
	// Initialization and AD configuration
	if (!initializeSPI()) return 1;
	setBuffer(1);

	setPGA(PGA_GAIN1);
	setDataRate(DRATE_3750);
	
	// Set single-ended analog input channel.
	setSEChannel(AIN2);
	delayus(8);
	
	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(20); 	// min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

#if PRINTF > 0
	printf("Getting the Threshold.\n");
	fflush(stdout);
#endif

	// Getting the average noise and then calculate the threshold
	
	gaV = gamV/1000.0;		// prepare the threshold value from reading parameters
	
	for (j = 0; j<3; j++) 
	{
		noiseV[0] = 0.0;
        for (i = 0; i < thr_win; i++)
		{
// Message for testing the ADC reading
#if PRINTF > 1
			printf("%.3f\n",readADS1256());
			fflush(stdout);
#endif

			noiseV[0] += readADS1256()/thr_win;      // ADC value from AVR
		}
		thresholdV[0] = noiseV[0] + gaV; 
	}

#if PRINTF > 0
	printf("Noise Average = %.3f\nThreshold = %.3f\n", noiseV[0], thresholdV[0]);
	fflush(stdout);
#endif
	
	
	// -------------------- Main Client Loop --------------------
	while (1) 
	{
		// -------------------- Training mode --------------------
		if(train_was_req == 1)	// we need to train
		{
			
#if PRINTF > 0
			printf("\nTraining mode!\n");
			fflush(stdout);
#endif

			if(N <= 0)
			{
				std::cout << "ERROR" << endl;
				return -1;
			}
			if(N > MAX_NR_TR)
				N = MAX_NR_TR;	// limit the # of training signals to a pre-defined maximum

			L = round_i(sig_duration*SAMPLING_FACTOR);
			emg_signal = zeros(1,L); 					// need to be changed from vector to matrix
			
			detected_resting = false;
			buffer_index = 0;
			
#if PRINTF > 0
				printf("Pleasing Stay Rest......\n");
				fflush(stdout);
#endif
				// Fill the entire noise buffer 
				average_noise = 0;
				for(i= 0; i< buffer_size; i++)
				{
					buffer_noise_cont[0][i] = readADS1256();

					average_noise += buffer_noise_cont[0][i];
				}
								
				// Calculate the average noise to center the signal
				average_noise = average_noise / buffer_size;			
				
#if PRINTF > 0
				printf("Noise Buffer Filled! Now Training the Gesture...\n");
				fflush(stdout);
#endif
            
			// -------------------- Gestures' Loop --------------------
            printf("Which gesture to be trained?\n");
            scanf("%d",&g);
            g--;

			data_all[g] = zeros(N,L);

#if PRINTF > 0
			std::cout << N << " samples will be saved for Gesture " << g+1 << ", under directory: " << directory << endl << endl;
#endif

			k = 0;		// reset counter for the gestures

			// create the directory for subject if it does not exist
			sprintf(fullname, "mkdir -m o-w /home/pi/EMG_MICO/Training/%s\n", directory);
			dummy = system(fullname);

			// ----- Main training loop (gets all training signals for the current gesture -----
			while(k < N)
			{
				average_signal = 0.0;			
                mov_ave[0] = 0;

				if (detected_resting == false) 
				{
					count = -noise_win;
					dc_offset[0] = 0.0;
				}
				else count = 0;

#if PRINTF > 0
				std::cout << "Number of training signals left: " << (N-k) << endl;
#endif
				
				// circular buffer to get the samples
				while(1)
				{
					// read ADC value from AVR
					buffer[0][buffer_index] = readADS1256();
					
					// State:1 Get the current average DC offset
					// Should only be enetered once if signal rest has always been detected
					if(count < -1)
					{
						dc_offset[0] += buffer[0][buffer_index] / noise_win;
						count++;
					}
					// State:2 Calculate the beginning value of moving average
					// Should only be entered once as well
					else if(count == -1)
					{
                        cout << dc_offset[0] << endl;
//						mov_ave[0] = dc_offset[0];
						mov_ave[0] = 0;
//						for(int j = 0; j < noise_win; j++)
//						{
//							// Read the value which was collected for calculating the average DC offset 
//							// to be the first value of moving average window
//							mov_ave[0] += abs(buffer[0][buffer_index - (noise_win - j - 1)] - dc_offset[0]);
//						}
//						mov_ave[0] = mov_ave[0]/noise_win;
						count++;
					}
					// State: 3 Search for EMG activity
					// We now have the moving average, and we need to update it and see if that updated
					// value exceeds the threshold. If so, we start collecting the actual EMG signal
					else if(count == 0)  
					{
						// Continue to calculate the moving average while reading from the ADC
						mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win + buffer_size) % buffer_size] - dc_offset[0])) / noise_win;

						if ((mov_ave[0] > HIGH_THR_FACTOR * (thresholdV[0] - dc_offset[0])))
						{
							// Once the moving average exceeds the threshold, mark the current buffer index
							flagindex = buffer_index;
							// Get the total value to center the signal later
							count++;
                            cout << "state 3" << endl;
						}
					}
					// State: 4 Search for EMG resting while collecting gestures
					// Once start to collect the signal, the break condition will be either detecting the signal rest
					// or done collecting the entire length of 2000 samples
					else if((count > 0) && (count <= L - sig_first_part))   
					{
						mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win + buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
//						average_signal += buffer[0][buffer_index];
						if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0])) 
						{
                            cout << "state 4" << endl;
							detected_resting = true;
							break;
						}
						count++;
					}
					// State 5: Search for EMG resting after gesture has been collected
					// If the gesture takes longer than 500ms, wait till the signal start to rest to avoid misreading the next time.
					else
					{
						mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win+buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
						if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0]))
						{
                            cout << "state 5" << endl;
							detected_resting = false;
							break;
						}

						count++;
					}

					buffer_index = (buffer_index+1) % buffer_size;
				}	// end of circular buffer to get the samples	

				// ----- prepare the signal -----
				
#if PRINTF > 1
				printf("prepare signal vector......\n");
				fflush(stdout);
#endif

//				// Get the final total value of the gesture after adding the first 65ms of signals
//				for(i = 0; i < sig_first_part; i++)
//					average_signal += (buffer[0][(flagindex - sig_first_part + i + buffer_size) % buffer_size]);
//
//			    average_signal = average_signal / L;
//
//				// Calculate the average value based on the acutal length of the signal	
//				if((count + sig_first_part) > L)
//					average_signal = average_signal / L;
//				else
//					average_signal = average_signal / (count + sig_first_part);
					

				for(i = 0; i < L; i++)
				{
					if (i < (count+sig_first_part))
						emg_signal(0,i) = (buffer[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - 1.65;	
					else
						emg_signal(0,i) = (buffer_noise_cont[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - 1.65;
//						emg_signal(0,i) = (buffer_noise_cont[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - average_noise;
				}

				data_all[g].set_row(k,emg_signal.get_row(0));

				sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/%s_%03d.txt", directory, gesture_names[g], k+1);
				fdata_all = fopen(fullname, "w");
				
				for(i = 0; i < L; i++)	// samples' loop
				{
					fprintf(fdata_all, "%+1.7e\n", data_all[g](k,i));
				}
				fclose(fdata_all);

#if PRINTF > 0
				printf("The training signal was accepted.\n");
				fflush(stdout);
#endif
				//Plot the data using gnuplot
				sprintf(fullname, "gnuplot -e \"set yrange [-1.5 : 1.5]; plot '/home/pi/EMG_MICO/Training/%s/%s_%03d.txt' with lines; pause 1; exit\"", directory, gesture_names[g], k+1);
				system(fullname);                  

				// next training gesture
				k++;

				// Message to hold still, to prevent users from doing the gesture too soon...

				sleep(1);	// A small pause to avoid misreadings...

			} //end of main training loop
			
//			// ----- Calculate Signatures and other training parameters -----
//			
//#if PRINTF > 0
//			printf("\nCalculating the Parameters for the Signatures.\n");
//			fflush(stdout);
//#endif
//			// get the index vector for the segments.
//			segD = zeros_i(D+1);
//			for(i = 0; i <= D; i++)
//				segD(i) = round_i(1.0 * i * L / D);
//
//			// the following calculates all training parameters and saves everything into
//			// files. The parameters will be available in all those matrices, so there is no
//			// need to exit the program. I just need to go to either the use, the smooth or
//			// the classification mode.
//			i = get_training_parameters_MM(data_all,&segD, &Sigs,S,mav_aves,mav_covs,zc_aves,
//					zc_covs, gr_aves, gr_covs, &zc_thr, &feat_wgts, weightOpt, directory);
//					
//			if(i < 0)
//			{
//				perror("Calculation of signatures and training parameters failed\n");
//				exit(-1);
//			}
//			if(i > 0)
//			{
//				printf("There was a problem with the covariances: %d\n", i);
//				exit(-1);
//			}
//
//#if PRINTF > 0
//			printf("Finished Getting the Signatures.\n");
//			fflush(stdout);
//#endif

			// Free the matrices containing all the training signals. Not needed anymore
			for(g = 0; g < S; g++)		// 3, 4 or 5
				data_all[g].~Mat();
			
			// The end of the training mode. Exit the program.
			mode = 'e';
				
		}	//end of training
		
		
	
		// -------------------- End of Training mode --------------------

		if(mode == 'e')	// this means that the user doesn't want to use the interface
		{
			printf("\nExiting the program. See you next time~\n");
			exit(1);
		}
	
		else				// go for the "recognition" mode
		{
			int training_okay = 1;		
			
			if(train_was_req == 0) // we didn't train in this instance of the program so we have to retrieve the parameters from the files
			{
				// Retrieve the signatures and training parameters, and put them in variables so
				// that they can be used for using the system ("use" or "classification" mode).
				training_okay = get_parameters_from_files_MM(directory, &S, &D, &L,
						&zc_thr, &Sigs, mav_aves, mav_covs, zc_aves, zc_covs, gr_aves,
						gr_covs, &feat_wgts);

				// If parameters for the current user are missing, signal that training is needed
				if(training_okay == 0)
				{
					train_was_req = 1;	// This will trigger the training
					continue;			// To the main loop, so that training can be done.
				}
			}

			// USE THE FOLLOWING MODE AS A BASIS OF YOUR APPLICATION. 
			// -------------------- Recognition mode --------------------
			if(mode == 'r')	// to get here, the mode had to be 'r'
			{
#if PRINTF > 0
				printf("\nRecognition mode!(Accuracy Test)\n");
				printf("Pleasing Stay Rest......\n");
				fflush(stdout);
#endif
				// Fill the entire noise buffer 
				average_noise = 0;
				for(i= 0; i< buffer_size; i++)
				{
					buffer_noise_cont[0][i] = readADS1256();
					average_noise += buffer_noise_cont[0][i];
				}
				// Calculate the average noise to center the signal
				average_noise = average_noise / buffer_size;	
							
				int movement = 0;
				
				emg_signal = zeros(1,L); 					// need to be changed from vector to matrix
				mat conf = ones(S,1);
				int buffer_index = 0;						//index for buffer
				bool detected_resting = false;
				
				segD = zeros_i(D+1);						// segment indices
				for(i = 0; i <= D; i++)
					segD(i) = round_i(1.0 * i * L / D);		
					
#if PRINTF > 0
				printf("Initialization Finished!\n");
				fflush(stdout);
#endif


				int ges_num;
				int err_num[4] = {0};

                int labels[5];
	            FILE *flabel = NULL;
                // read from the label map            
                sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Param/label_map.txt", directory);
                
                flabel = fopen(fullname, "r");
                for(i = 0; i < S; i++)\
                    fscanf(flabel, "%d", &labels[i]);

                fclose(flabel);


                int temp;
//				int tot_ges_num = 4;
				int tot_rep_num = 5;
				
                printf("Which LABEL to test? ");
                scanf("%d", &temp);
                for(i = 0; i < S; i++)
                {
                    if( labels[i] == temp )
                        ges_num = i;
                }

//				for(ges_num = 0; ges_num < tot_ges_num; ges_num++)
//				{

					int rep_num;
					
#if PRINTF > 0
					std::cout << "Testing for Gesture " << ges_num+1  << endl; 
					fflush(stdout);
#endif
						
			        // create the directory for subject if it does not exist
			        sprintf(fullname, "mkdir -m o-w /home/pi/EMG_MICO/Testing/%s\n", directory);
			        dummy = system(fullname);

					for(rep_num = 0; rep_num < tot_rep_num; rep_num++)
					{
												
#if PRINTF > 0
						printf("Waiting for the gesture %d...\n", tot_rep_num - rep_num);
						fflush(stdout);
#endif

						average_signal = 0.0;				

						if (detected_resting == false) 
						{
							count = -noise_win;
							dc_offset[0] = 0.0;
						}
						else count = 0;

						// circular buffer to get the samples
                        while(1)
                        {
                            // read ADC value from AVR
                            buffer[0][buffer_index] = readADS1256();
                            
                            // State:1 Get the current average DC offset
                            // Should only be enetered once if signal rest has always been detected
                            if(count < -1)
                            {
                                dc_offset[0] += buffer[0][buffer_index] / noise_win;
                                count++;
                            }
                            // State:2 Calculate the beginning value of moving average
                            // Should only be entered once as well
                            else if(count == -1)
                            {
                                cout << dc_offset[0] << endl;
        //						mov_ave[0] = dc_offset[0];
                                mov_ave[0] = 0;
        //						for(int j = 0; j < noise_win; j++)
        //						{
        //							// Read the value which was collected for calculating the average DC offset 
        //							// to be the first value of moving average window
        //							mov_ave[0] += abs(buffer[0][buffer_index - (noise_win - j - 1)] - dc_offset[0]);
        //						}
        //						mov_ave[0] = mov_ave[0]/noise_win;
                                count++;
                            }
                            // State: 3 Search for EMG activity
                            // We now have the moving average, and we need to update it and see if that updated
                            // value exceeds the threshold. If so, we start collecting the actual EMG signal
                            else if(count == 0)  
                            {
                                // Continue to calculate the moving average while reading from the ADC
                                mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win + buffer_size) % buffer_size] - dc_offset[0])) / noise_win;

                                if ((mov_ave[0] > HIGH_THR_FACTOR * (thresholdV[0] - dc_offset[0])))
                                {
                                    // Once the moving average exceeds the threshold, mark the current buffer index
                                    flagindex = buffer_index;
                                    // Get the total value to center the signal later
                                    count++;
                                    cout << "state 3" << endl;
                                }
                            }
                            // State: 4 Search for EMG resting while collecting gestures
                            // Once start to collect the signal, the break condition will be either detecting the signal rest
                            // or done collecting the entire length of 2000 samples
                            else if((count > 0) && (count <= L - sig_first_part))   
                            {
                                mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win + buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
        //						average_signal += buffer[0][buffer_index];
                                if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0])) 
                                {
                                    cout << "state 4" << endl;
                                    detected_resting = true;
                                    break;
                                }
                                count++;
                            }
                            // State 5: Search for EMG resting after gesture has been collected
                            // If the gesture takes longer than 500ms, wait till the signal start to rest to avoid misreading the next time.
                            else
                            {
                                mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win+buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
                                if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0]))
                                {
                                    cout << "state 5" << endl;
                                    detected_resting = false;
                                    break;
                                }

                                count++;
                            }

                            buffer_index = (buffer_index+1) % buffer_size;
                        }	// end of circular buffer to get the samples	

#if PRINTF > 1
						printf("Signal detected (recognition mode)\n");
						fflush(stdout);
#endif
//        				// Get the final total value of the gesture after adding the first 65ms of signals
//	        			for(i = 0; i < sig_first_part; i++)
//		        			average_signal += (buffer[0][(flagindex - sig_first_part + i + buffer_size) % buffer_size]);
//
//			             average_signal = average_signal / L;

				        for(i = 0; i < L; i++)
				        {
					        if (i < (count+sig_first_part))
						        emg_signal(0,i) = (buffer[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - 1.65;	
					        else
						        emg_signal(0,i) = (buffer_noise_cont[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - 1.65;
				        }


#if PRINTF > 1
						printf("\n Signal Prepared successfully\n");
						fflush(stdout);
#endif

        				sprintf(fullname, "/home/pi/EMG_MICO/Testing/%s/%s_%03d.txt", directory, gesture_names[ges_num], rep_num+1);
	        			fdata_all = fopen(fullname, "w");
				
		        		for(i = 0; i < L; i++)	// samples' loop
			        		fprintf(fdata_all, "%+1.7e\n", emg_signal(0,i));

        				fclose(fdata_all);

				        //Plot the data using gnuplot
				        sprintf(fullname, "gnuplot -e \"set yrange [-1.5 : 1.5]; plot '/home/pi/EMG_MICO/Testing/%s/%s_%03d.txt' with lines; pause 1; exit\"", directory, gesture_names[ges_num], rep_num+1);
				        system(fullname);                  


					
						movement = EMG_classify_MM(emg_signal, &Sigs, mav_aves, mav_covs,
												   zc_aves, zc_covs, gr_aves, gr_covs, &segD,
												   &feat_wgts, zc_thr, &conf, cl_opt);

                        if(movement == 0)
                            movement = 4;

						printf("Movement: %d\n\n", movement);
						fflush(stdout);
							
						if(movement != temp)
							err_num[ges_num]++;

                        sleep(0.5);
													
					}
					
						std::cout << "Test Gesture " << ges_num + 1 << " Finished! Total Error: " << err_num[ges_num] << endl << endl;

//				} // end of while(1)
				
				std::cout << "Test Result:" << endl;				
				
//				for(ges_num = 0; ges_num < tot_ges_num; ges_num++)
//				{
						std::cout << "Gesture " << ges_num+1<< " has " << err_num[ges_num] << " error!";
						std::cout << " Accuracy is:" << (double)(tot_rep_num - err_num[ges_num]) / tot_rep_num * 100 << "%" << endl;
//				}
				
				mode = 'e';		// exit the program		
				
			}// end of if statement to determine if mode was 'r'
			

		}//end of else e mode


	}  //end of main client loop
	free(buffer[0]);
	
	// Stop continuous mode for ADC
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();	
	
	return 0;

}	// end of main function



