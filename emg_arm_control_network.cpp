/*
 * 			   Name: 
 *
 *		 Created on: Jun 21th, 2017 (Originally Nov 4, 2013)
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
#include <libkindrv/kindrv.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "enum.h"
#include "spi.h"
#include "ads1256.h"
#include "GUSSS.h"

//==================== Parameters for Classification ====================

//---------- Not sure about the functionality ----------
#define WGTS_OPT					0				// feature weights' option (0/same, 1/different)


//---------- Moving average window ----------
#define NOISE_WIN					150				// Set to have approx.  ~ 150 ms  // when sampling
#define THR_WIN						2000			// For the threshold window. Set to ~ 2 sec.
#define SIG_FIRST_PART	 			90				// Go back from the flagindex, count as signal pts.
#define HIGH_THR_FACTOR				1.8				// Moving average window threshold
#define LOW_THR_FACTOR				1.2

//---------- Training and testing configuration ----------
#define NUMBER_CHANNELS				1				// the number of channels being used
#define MAX_SIG						5				// maximum possible number of signatures
#define NRS 		    			4				// default number of signatures, previously it was 4
#define TRAIN_SAMPLES				10				// suggested training sample is 20 per gesture
#define MAX_NR_TR		   			20				// maximum number of training signals allowed
#define SIG_DURATION	            500				// default signal length, in mili-seconds
#define BUFFER_SIZE					3750			// Circular Buffer size

//---------- ADC(ads1256) configuration ----------
#define SAMPLING_FACTOR	   			3.75			// ADC sampling rate of the AVR, in kilo-samples/second

//---------- Debug configuration ----------
#define PRINTF						1				// 0 to prevent any printf, 1 to print the main messages
													// 2 to print every detail


// Print the error messages for network
void error(const char *msg)
{
    perror(msg);
    exit(0);
}													
	
//==================== Main ====================
int main(int argc, char **argv) 
{

#if PRINTF > 0
	printf("\nWelcome to ViGIR Lab!\n\n");
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
	delayus(8); 	// min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

#if PRINTF > 0
	printf("Getting the Threshold.\n");
	fflush(stdout);
#endif

	// Getting the average noise and then calculate the threshold
	
	gaV = gamV/1000.0;		// prepare the threshold value from reading parameters
	
	for (j = 0; j<3; j++) 
	{
		noiseV[0] = 0.0;
//		printf("threshold win: %d\n", thr_win);
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
			for(g = 0; g < S; g++)		// 3, 4 or 5
			{

				data_all[g] = zeros(N,L);

#if PRINTF > 0
				std::cout << N << " samples will be saved for Gesture " << g+1 << ", under directory: " << directory << endl << endl;
#endif

				k = 0;		// reset counter for the gestures

				// ----- Main training loop (gets all training signals for the current gesture -----
				while(k < N)
				{
					average_signal = 0.0;				

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
							mov_ave[0] = 0.0;
							for(int j = 0; j < noise_win; j++)
							{
								// Read the value which was collected for calculating the average DC offset 
								// to be the first value of moving average window
								mov_ave[0] += abs(buffer[0][buffer_index - (noise_win - j - 1)] - dc_offset[0]);
							}
							mov_ave[0] = mov_ave[0]/noise_win;
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
							}
						}
						// State: 4 Search for EMG resting while collecting gestures
						// Once start to collect the signal, the break condition will be either detecting the signal rest
						// or done collecting the entire length of 2000 samples
						else if((count > 0) && (count <= L - sig_first_part))   
						{
							mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win + buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
							average_signal += buffer[0][buffer_index];
							if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0])) 
							{
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
								detected_resting = true;
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

					// Get the final total value of the gesture after adding the first 65ms of signals
					for(i = 0; i < sig_first_part; i++)
						average_signal += (buffer[0][(flagindex - sig_first_part + i + buffer_size) % buffer_size]);
					// Calculate the average value based on the acutal length of the signal	
					if((count + sig_first_part) > L)
						average_signal = average_signal / L;
					else
						average_signal = average_signal / (count + sig_first_part);
						

					for(i = 0; i < L; i++)
					{
						if (i < (count+sig_first_part))
							emg_signal(0,i) = (buffer[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - average_signal;	
						else
							emg_signal(0,i) = (buffer_noise_cont[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - average_noise;
					}

					data_all[g].set_row(k,emg_signal.get_row(0));

#if PRINTF > 0
					printf("The training signal was accepted.\n");
					fflush(stdout);
#endif
					k++;

					// Message to hold still, to prevent users from doing the gesture too soon...

					sleep(1);	// A small pause to avoid misreadings...

				} //end of main training loop
				
				
			} //end of gesture loop
			
			// ----- Calculate Signatures and other training parameters -----
			
#if PRINTF > 0
			printf("\nCalculating the Parameters for the Signatures.\n");
			fflush(stdout);
#endif
			// get the index vector for the segments.
			segD = zeros_i(D+1);
			for(i = 0; i <= D; i++)
				segD(i) = round_i(1.0 * i * L / D);

			// the following calculates all training parameters and saves everything into
			// files. The parameters will be available in all those matrices, so there is no
			// need to exit the program. I just need to go to either the use, the smooth or
			// the classification mode.
			i = get_training_parameters_MM(data_all,&segD, &Sigs,S,mav_aves,mav_covs,zc_aves,
					zc_covs, gr_aves, gr_covs, &zc_thr, &feat_wgts, weightOpt, directory);
					
			if(i < 0)
			{
				perror("Calculation of signatures and training parameters failed\n");
				exit(-1);
			}
			if(i > 0)
			{
				printf("There was a problem with the covariances: %d\n", i);
				exit(-1);
			}

#if PRINTF > 0
			printf("Finished Getting the Signatures.\n");
			fflush(stdout);
#endif

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
                
				// ----- Configure the Socket -----

				int sockfd, portno;			
				struct sockaddr_in serv_addr;		// stores the address for the server
				struct hostent *server;				
				char buffer_socket[10];
    
				// Convert the argument into integers
				portno = atoi("51900");
    
				// Create a new socket, using internet domain, stream socket, and TCP protocal
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd < 0) 
				error("ERROR opening socket");
    
				// The variable argv[1] contains the name of a host on the internet
				server = gethostbyname("10.14.1.235");
				if (server == NULL) 
				{
					fprintf(stderr,"ERROR, no such host\n");
					exit(0);
				}
    
				bzero((char *)&serv_addr, sizeof(serv_addr));		// initializes serv_addr to zeros
				serv_addr.sin_family = AF_INET;						// sets the address family
				// Copy the settings from the server
				bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);	
				serv_addr.sin_port = htons(portno);					// sets the port number
    
				// Establish a connection to the server
				if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
					error("ERROR connecting");

#if PRINTF > 0
				printf("Network Configuration Complete!\n\n");
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

				while(1)		//use the same variable as in training mode
				{
#if PRINTF > 0
					printf("Waiting for the gesture...\n");
//					printf("Waiting for the gesture %d...\n", tot_rep_num - rep_num);
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
						// read ADC value from ADS1256()						
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
							mov_ave[0] = 0.0;
							for(int j = 0; j < noise_win; j++)
								// Read the value which was collected for calculating the average DC offset 
								// to be the first value of moving average window
								mov_ave[0] += abs(buffer[0][buffer_index - (noise_win - j - 1)] - dc_offset[0]);

							mov_ave[0] = mov_ave[0]/noise_win;
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
							}
						}
						// State: 4 Search for EMG resting while collecting gestures
						// Once start to collect the signal, the break condition will be either detecting the signal rest
						// or done collecting the entire length of 2000 samples
						else if((count > 0) && (count <= L - sig_first_part))   
						{
							mov_ave[0] += (abs(buffer[0][buffer_index] - dc_offset[0]) - abs(buffer[0][(buffer_index - noise_win+buffer_size) % buffer_size] - dc_offset[0])) / noise_win;
							average_signal += buffer[0][buffer_index];
							if (mov_ave[0] < LOW_THR_FACTOR * (thresholdV[0] - dc_offset[0])) 
							{
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
								detected_resting = true;
								break;
							}
							count++;
						}

						buffer_index = (buffer_index+1) % buffer_size;
					}	
					// end of circular buffer to get the samples

#if PRINTF > 1
					printf("Signal detected (recognition mode)\n");
					fflush(stdout);
#endif

					// prepare the signal
					// Get the final total value of the gesture after adding the first 65ms of signals
					for(i = 0; i < sig_first_part; i++)
						average_signal += (buffer[0][(flagindex - sig_first_part + i + buffer_size) % buffer_size]);
					// Calculate the average value based on the acutal length of the signal	
					if((count + sig_first_part) > L)
						average_signal = average_signal / L;
					else
						average_signal = average_signal / (count + sig_first_part);
				
					for(i = 0; i < L; i++)
					{
						if (i < (count+sig_first_part))
							emg_signal(0,i) = (buffer[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - average_signal;	
						else
							emg_signal(0,i) = (buffer_noise_cont[0][(flagindex-sig_first_part+i+buffer_size) % buffer_size]) - average_noise;
					}

#if PRINTF > 1
					printf("\n Signal Prepared successfully\n");
					fflush(stdout);
#endif
				
					movement = EMG_classify_MM(emg_signal, &Sigs, mav_aves, mav_covs,
											   zc_aves, zc_covs, gr_aves, gr_covs, &segD,
											   &feat_wgts, zc_thr, &conf, cl_opt);
					printf("Movement: %d\n\n", movement);
					fflush(stdout);
					bzero(buffer_socket, 10);
					
					switch(movement)
					{
						case 1:
							buffer_socket[0] = '1';
							break;
						case 2:
							buffer_socket[0] = '2';
							break;
						case 3:
							buffer_socket[0] = '3';
							break;
						case 0:
							buffer_socket[0] = '4';
							break;														
						default:
							break;
					}
					
					write(sockfd, buffer_socket, strlen(buffer_socket));					

                    usleep(50);
				} 
				// end of while(1)
				
                movement = 5;				
				mode = 'e';		// exit the program		
				
			}// end of if statement to determine if mode was 'r'
			

		}//end of else e mode


	}  //end of main client loop
	free(buffer[0]);
	
	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();	
    
	return 0;

}	// end of main function



