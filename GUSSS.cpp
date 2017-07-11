/*
 * EMG_unified_functions.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: lulorivera
 *
 *  Copyright (C) 2013 ViGIR - Vision Guided and Intelligent Robotics Lab
 *  (http://vigir.missouri.edu)
 *  Written by Luis Alberto Rivera <larmbc@mail.missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 * This file has all the functions that are required for EMG_unified.cpp.
 * They were taken from EMG_client_demo_small_changes_v2.cpp and
 * EMG_server_demo_small_changes_v3.cpp.
 *
 */

// ------------------ Header files -------------------------
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <bcm2835.h> 
#include "spi.h"
#include "ads1256.h"
#include "GUSSS.h"
#include <itpp/itbase.h>
#include <cmath>

using namespace std;
using namespace itpp;

// ------------------------------- GUSSS functions -----------------------------------------

/* Zero Crossing (ZC) counter
 * This function counts how many times the sign of a particular signal changes.
 * Assuming that the signal is continuous in time, the calculated number is a lower bound
 * to the number of zero crossings of the signal. This function sets to zero all signal
 * points with absolute value below the specified threshold. Then, it counts how many times
 * the compensated signal changes sign. The function is not otpimized in the Matlab sense
 * (matrix and vector operations), because it is to be translated into a C function.
 * Inputs:	y - the signal to be analyzed.
 *			L - the length of the incoming signal
 *			thr - threshold, to compensate ZC due to noise
 * Outputs: estimated number of zero crossings.										*/
int Zero_crossings(float *y, int L, float thr)
{
	int i, zc = 0, pivot = 0, pivotsign = 0;
	float *y_fixed = (float *)malloc(L*sizeof(float));

	for(i = 0; i < L; i++)
	{
		if(abs(y[i]) < thr)
			y_fixed[i] = 0.0;
		else
			y_fixed[i] = y[i];

		// determine where the first nonzero value is
		if(pivotsign == 0)
		{
			if(y_fixed[i] > 0.0)
			{
				pivot = i;
				pivotsign = 1;
			}
			else if(y_fixed[i] < 0.0)
			{
				pivot = i;
				pivotsign = -1;
			}
		}
	}

	// Count how many sign changes there are. A zero is considered the same sign
	// as the previous one.
	for(i = pivot+1; i < L; i++)
	{
		if((y_fixed[i] > 0.0) && (pivotsign == -1))
		{
			zc++;
			pivotsign = 1;
		}
		else if((y_fixed[i] < 0.0) && (pivotsign == 1))
		{
			zc++;
			pivotsign = -1;
		}
	}

	free(y_fixed);
	return zc;
}


// Takes an array with all training signals of a particular gesture and calculates the
// average mean absolute values (MAV) and the corresponding standard deviations.
// Input: data - array with the training signals as row vectors
//			 N - total number of signals in data
//		  segD - vector with the D + 1 positions where the signals are to be
//				 segmented from, including 0 and L = used_datapts:
//				 0 = segD[0] < segD[1] < ... < segD[D-2] < segD[D-1] = L
//			 D - number of segments: D = length(segD) - 1;
// 	   mav_ave - array where the averages of the MAVs are to be stored (each element
//				 is the average for each segment of the signals)
//	   mav_std - array where the standard deviations of the MAVs are to be stored (each
//				 element corresponds to each segment of the signals).
void get_MAV_features(float **data, int N, int *segD, int D, float *mav_ave, float *mav_std)
{
	int i, n, d;
	Stat DataMAV;
	float mav_all[N][D], bias_factor;

	bias_factor = sqrt((float)(1.0*N/(1.0*(N-1))));

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{
			for(i = segD[d]; i < segD[d+1]; i++)
				DataMAV.sample(abs(data[n][i]), false);

			mav_all[n][d] = DataMAV.avg();
			DataMAV.clear();
		}
	}

	for(d = 0; d < D; d++)
	{
		for(n = 0; n < N; n++)
			DataMAV.sample(mav_all[n][d], false);

		mav_ave[d] = DataMAV.avg();
		mav_std[d] = DataMAV.sigma()*bias_factor;;
		DataMAV.clear();
	}
}

// The only difference with respect to get_MAV_features() is that we retrieve all the
// MAV values. Useful for calculating the weight of these features, for the classification.
// Additional input: mav_all - array to store all the MAV values
void get_MAV_features_v2(float **data, int N, int *segD, int D, float *mav_ave, float *mav_std,
						 float **mav_all)
{
	int i, n, d;
	Stat DataMAV;
	float bias_factor;

	bias_factor = sqrt((float)(1.0*N/(1.0*(N-1))));

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{
			for(i = segD[d]; i < segD[d+1]; i++)
				DataMAV.sample(abs(data[n][i]), false);

			mav_all[n][d] = DataMAV.avg();
			DataMAV.clear();
		}
	}

	for(d = 0; d < D; d++)
	{
		for(n = 0; n < N; n++)
			DataMAV.sample(mav_all[n][d], false);

		mav_ave[d] = DataMAV.avg();
		mav_std[d] = DataMAV.sigma()*bias_factor;;
		DataMAV.clear();
	}
}


// Takes an array with all training signals of a particular gesture and calculates the
// average number of zero counts (ZC) and the corresponding standard deviations.
// Input: data - array with the training signals as row vectors
//			 N - total number of signals in data
//		  segD - vector with the D + 1 positions where the signals are to be
//				 segmented from, including 0 and L = used_datapts:
//				 0 = segD[0] < segD[1] < ... < segD[D-2] < segD[D-1] < segD[D] = used_datapts
//			 D - number of segments: D = length(segD) - 1;
//		   thr - threshold for the ZC function
// 	    zc_ave - array where the averages of the number of ZCs are to be stored (each
//				 element is the average for each segment of the signals)
//	    zc_std - array where the standard deviations of the # of ZCs are to be stored
//				 (each element corresponds to each segment of the signals).
void get_ZC_features(float **data, int N, int *segD, int D, float thr, float *zc_ave,
					 float *zc_std)
{
	int i, n, d, L = 0;
	Stat DataZC;
	float zc_all[N][D], bias_factor; // y[USED_DATAPTS];
	float *y = (float *)malloc(segD[D]*sizeof(float));

	bias_factor = sqrt((float)(1.0*N/(1.0*(N-1))));

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{
			L = 0;
			for(i = segD[d]; i < segD[d+1]; i++)
			{
				y[i-segD[d]] = data[n][i];
				L++;
			}
			zc_all[n][d] = 1.0*Zero_crossings(y, L, thr);
		}
	}

	for(d = 0; d < D; d++)
	{
		for(n = 0; n < N; n++)
			DataZC.sample(zc_all[n][d], false);

		zc_ave[d] = DataZC.avg();
		zc_std[d] = DataZC.sigma()*bias_factor;;
		DataZC.clear();
	}
}

// The only difference with respect to get_ZC_features() is that we retrieve all the
// ZC values. Useful for calculating the weight of these features, for the classification.
// Additional input: zc_all - array to store all the ZC values
void get_ZC_features_v2(float **data, int N, int *segD, int D, float thr, float *zc_ave,
						float *zc_std, float **zc_all)
{
	int i, n, d, L = 0;
	Stat DataZC;
	float bias_factor;
	float *y = (float *)malloc(segD[D]*sizeof(float));

	bias_factor = sqrt((float)(1.0*N/(1.0*(N-1))));

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{
			L = 0;
			for(i = segD[d]; i < segD[d+1]; i++)
			{
				y[i-segD[d]] = data[n][i];
				L++;
			}
			zc_all[n][d] = 1.0*Zero_crossings(y, L, thr);
		}
	}

	for(d = 0; d < D; d++)
	{
		for(n = 0; n < N; n++)
			DataZC.sample(zc_all[n][d], false);

		zc_ave[d] = DataZC.avg();
		zc_std[d] = DataZC.sigma()*bias_factor;;
		DataZC.clear();
	}
}


// Calculates the GUSSS ratio for a given test signal and signature
// Inputs:	Y - Testing signal
//		   Sp - Signature to be injected
//			L - Length of the signals
// Output: GUSSS ratio
float GUSSS_ratio_EMG_v2(mat Y, mat Sp, int L)
{
	int j, nrIC;
	float ratio, normsp = 0.0, norm0 = 0.0, norm1 = 0.0, error0 = 0.0, error1 = 0.0;
	mat Mix = zeros(2,L);			// Mix matrix
	mat A;							// for the mixing and de-mixing matrices
	mat icasig;						// for the independent components
//	mat A_init = zeros(2,2);		// initial guess for the mixing matrix

	// Initial guess for the mixing matrix
//	A_init(0,0) = 1.0;	A_init(0,1) = 0.5;
//	A_init(1,0) = 1.0;	A_init(1,1) = 1.5;

	for(j = 0; j < L; j++)
	{
		// constructs Mix matrices to be ICA-analyzed [Y; Sp + Y]
		Mix(0,j) = Y(0,j);
		Mix(1,j) = Sp(0,j) + Y(0,j);		// inject signature
	}

//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	Fast_ICA Test(Mix);		// ICA object

	// Analysis and ratios for all signatures
	Test.set_non_linearity(FICA_NONLIN_GAUSS);	// Set GAUSS non-linearity
	Test.set_approach(FICA_APPROACH_DEFL);		// Use deflation approach : IC are computed one by one
	Test.set_max_num_iterations(MAX_ITER);	// 100
	Test.set_epsilon(EPSILON);			// 0.01

//	printf("Before setting initial guess matrix\n");
//	Test.set_init_guess(guess);		// there is a bug with this function...
//	printf("After setting initial guess matrix\n");
//
//	printf("Been here, before ICA\n");
	Test.separate();	// Perform ICA

	A = Test.get_mixing_matrix();		// Mixing matrix
	icasig = Test.get_independent_components();
	nrIC = Test.get_nrof_independent_components();

	if(nrIC == 0)
	{
		ratio = 100.0;
	}
	else if(nrIC == 1)
	{
		ratio = 0.0;
	}
	else
	{
		for(j = 0; j < L-1; j++)	// for some reason, icasig signals are L-1 long
		{
			if(abs(Sp(0,j)) > normsp)
				normsp = abs(Sp(0,j));
			if(abs(icasig(0,j)) > norm0)
				norm0 = abs(icasig(0,j));
			if(abs(icasig(1,j)) > norm1)
				norm1 = abs(icasig(1,j));
		}

		// To determine which column corresponds to the coefficient cp (by looking at the
		// recovered, separated signals. One has to be equal (or very close) to the
		// injected signature. We calculate how different is icasig0 and icasig1 wrt the
		// signature Sp. We can then infer if cp is in A(0,0) or A(0,1).
		for(j = 0; j < L-1; j++)
		{
			Sp(0,j) = Sp(0,j)/normsp;
			icasig(0,j) = icasig(0,j)/norm0;
			icasig(1,j) = icasig(1,j)/norm1;
			//error0 += abs(Sp(0,j) - (A(0,0)/abs(A(0,0)))*icasig(0,j));
			//error1 += abs(Sp(0,j) - (A(0,1)/abs(A(0,1)))*icasig(1,j));
			error0 += abs(abs(Sp(0,j)) - abs(icasig(0,j)));
			error1 += abs(abs(Sp(0,j)) - abs(icasig(1,j)));
		}

		if(error0 <= error1)
			ratio = abs(A(0,1)/A(0,0));	// Sp matches icasig0, cp is in the 1st column
		else
			ratio = abs(A(0,0)/A(0,1));	// Sp matches icasig1, cp is in the 2nd column
	}

	return(ratio);
}


// Function to get the signatures and other parameters needed for the classification
// algorithm. It includes the calculation of the MAVs and ZCs from one or more segments
// or the signal.
// This function is the equivalent of the Matlab functions get_training_testing_signatures_v2.m,
// get_MAV_features.m and get_ZC_features.m.
// Inputs: directory - string with the user name, which serves as the name of the direc-
//					   tory where the training signals are to be found, and where the
//					   calculated signatures and parameters will be stored.
//				segD - vector with the D + 1 positions where the signals are to be
//					   segmented from, including 0 and L = used_datapts:
//					   0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
//				   D - number of segments: D = length(segD) - 1;
// Output: -1 if there was any error (e.g. opening files), and a positive number if
//		   everything went well (value not important).
int get_signatures_v2(char directory[], int *segD, int D)
{
	int i, j, nr_training = 0, dummy, L;
	char fullname[FNBUFFER], value[30];
	FILE *fnr = NULL, *fdata = NULL, *fsign = NULL;
	FILE *fmavs = NULL, *fmavs_stdv = NULL, *fzc_ave = NULL, *fzc_stdv = NULL;
	float **Stop_training_signals, **Left_training_signals, **Right_training_signals;
	float **signatures1;
	float **mav_aves, **mav_stds, **zc_aves, **zc_stds;
	float *aves_temp, *stds_temp;
	float thr = 0.0;
	Stat DataSignature;

	L = segD[D];	// = used_datapts

	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_training.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_training file for reading" << endl;
		return -1;
	}
	dummy = fscanf(fnr, "%d", &nr_training);
	fclose(fnr);

	// --- allocate memory -------------------
	signatures1 = (float **)calloc(nrS, sizeof(float *));
	for(i = 0; i < nrS; i++)
	{
		signatures1[i] = (float *)calloc(L, sizeof(float));
	}

	Stop_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Left_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Right_training_signals = (float **)calloc(nr_training, sizeof(float *));
	for(i = 0; i < nr_training; i++)
	{
		Stop_training_signals[i] = (float *)calloc(L, sizeof(float));	// USED_DATAPTS
		Left_training_signals[i] = (float *)calloc(L, sizeof(float));	// USED_DATAPTS
		Right_training_signals[i] = (float *)calloc(L, sizeof(float));	// USED_DATAPTS
	}

	mav_aves = (float **)calloc(D, sizeof(float *));
	mav_stds = (float **)calloc(D, sizeof(float *));
	zc_aves = (float **)calloc(D, sizeof(float *));
	zc_stds = (float **)calloc(D, sizeof(float *));
	for(i = 0; i < D; i++)
	{
		mav_aves[i] = (float *)calloc(nrS, sizeof(float));
		mav_stds[i] = (float *)calloc(nrS, sizeof(float));
		zc_aves[i] = (float *)calloc(nrS, sizeof(float));
		zc_stds[i] = (float *)calloc(nrS, sizeof(float));
	}

	aves_temp = (float *)calloc(D, sizeof(float));
	stds_temp = (float *)calloc(D, sizeof(float));


// ---------------- MAVS and SIGNATURES for the STOP signals ----------------------------
	for(i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Stop_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Stop_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Stop_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][0] = aves_temp[i];
		mav_stds[i][0] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Stop_training_signals[i][j], false);

		signatures1[0][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ---------------- MAVS and SIGNATURES for the LEFT signals ----------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Left_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Left_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Left_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][1] = aves_temp[i];
		mav_stds[i][1] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Left_training_signals[i][j], false);

		signatures1[1][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ------------------- MAVS and SIGNATURES for the RIGHT signals ------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Right_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Right_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Right_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][2] = aves_temp[i];
		mav_stds[i][2] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Right_training_signals[i][j], false);

		signatures1[2][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ---------------------------- Threshold for ZC ----------------------------------------
	for(i = 0; i < D; i++)
		for(j = 0; j < nrS; j++)
			thr += mav_aves[i][j];

	thr = thr/(1.0*nrS*D);		// this threshold is necessary for the ZC function
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the STOP signals -----------------------------
	get_ZC_features(Stop_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][0] = aves_temp[i];
		zc_stds[i][0] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the LEFT signals -----------------------------
	get_ZC_features(Left_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][1] = aves_temp[i];
		zc_stds[i][1] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the RIGHT signals -----------------------------
	get_ZC_features(Right_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][2] = aves_temp[i];
		zc_stds[i][2] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// ----------------------- Save to files ------------------------------------------------
	// opens file to store signatures ---------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/signatures1.txt", directory);
	fsign = fopen(fullname, "w");
	if(fsign == 0)
	{
		cerr << "Error: Could not open signature file1 for writing" << endl;
		return -1;
	}

	for(i = 0; i < nrS; i++)
	{
		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
		{
			sprintf(value, "%+1.7e  ", signatures1[i][j]);
			fwrite(value, sizeof(char), 16, fsign); // save to file
		}
		fwrite("\n", sizeof(char), sizeof(char), fsign);
	}
	fclose(fsign);

//	// if a there is a new way of getting the signatures, use signatures2.
//	memset(fullname, 0, FNBUFFER);
//	sprintf(fullname,"../../Training/%s/signatures2.txt", directory);
//	fsign = fopen(fullname, "w");
//	if(fsign == 0)
//	{
//		cerr << "Error: Could not open signature file2 for writing" << endl;
//		return -1;
//	}
//
//	for(i = 0; i < nrS; i++)
//	{
//		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
//		{
//			sprintf(value, "%+1.7e  ", signatures1[i][j]); // change to signatures2[i][j]
//			fwrite(value, sizeof(char), 16, fsign); // save to file
//		}
//		fwrite("\n", sizeof(char), sizeof(char), fsign);
//	}
//	fclose(fsign);


	// opens file to store mav_aves------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_aves.txt", directory);
	fmavs = fopen(fullname, "w");
	if(fmavs == 0)
	{
		cerr << "Error: Could not open signature mavs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrS; j++)
		{
			sprintf(value, "%1.7e  \t", mav_aves[i][j]);
			fwrite(value, sizeof(char), 16, fmavs); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs); // save to file
	}
	fclose(fmavs);		// closes file with mav_aves


	// opens file to store mavs_stds ----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_stds.txt", directory);
	fmavs_stdv = fopen(fullname, "w");
	if(fmavs_stdv == 0)
	{
		cerr << "Error: Could not open mav_stdvs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrS; j++)
		{
			sprintf(value, "%1.7e  \t", mav_stds[i][j]);
			fwrite(value, sizeof(char), 16, fmavs_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs_stdv); // save to file
	}
	fclose(fmavs_stdv);		// closes file with mav_aves


	// opens file to store zc_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_aves.txt", directory);
	fzc_ave = fopen(fullname, "w");
	if(fzc_ave == 0)
	{
		cerr << "Error: Could not open signature zc_ave file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrS; j++)
		{
			sprintf(value, "%1.7e  \t", zc_aves[i][j]);
			fwrite(value, sizeof(char), 16, fzc_ave); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_ave); // save to file
	}
	fclose(fzc_ave);	// closes file with ZC averages


	// opens file to store zc_stds ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_stds.txt", directory);
	fzc_stdv = fopen(fullname, "w");
	if(fzc_stdv == 0)
	{
		cerr << "Error: Could not open zc_stdv file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrS; j++)
		{
			sprintf(value, "%1.7e  \t", zc_stds[i][j]);
			fwrite(value, sizeof(char), 16, fzc_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_stdv); // save to file
	}
	fclose(fzc_stdv);		// closes file with ZC standard deviations
//---------------------------------------------------------------------------------------

	// This is just to allow the group to modify the files just created -----------------
	sprintf(fullname, "chmod g+w -R /home/pi/EMG_MICO/Training/%s\n", directory);
	dummy = system(fullname);
	dummy++;	// this is just to avoid a warning...

	// Free memory that was allocated
	free(Stop_training_signals);
	free(Left_training_signals);
	free(Right_training_signals);
	free(mav_aves);
	free(mav_stds);
	free(zc_aves);
	free(zc_stds);
	free(aves_temp);
	free(stds_temp);

	return 1;	// should be >= 0
}

// Function to get the signatures and other parameters needed for the classification
// algorithm. It includes the calculation of the MAVs and ZCs from one or more segments
// or the signal.
// This function is the equivalent of the Matlab functions get_training_testing_signatures_v2.m,
// get_MAV_features.m and get_ZC_features.m.
// Inputs: directory - string with the user name, which serves as the name of the direc-
//					   tory where the training signals are to be found, and where the
//					   calculated signatures and parameters will be stored.
//				segD - vector with the D + 1 positions where the signals are to be
//					   segmented from, including 0 and L = used_datapts:
//					   0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
//				   D - number of segments: D = length(segD) - 1;
//				 nrs - number of signatures (i.e., number of gestures to be used)
// Output: -1 if there was any error (e.g. opening files), and a positive number if
//		   everything went well (value not important).
int get_signatures_v3(char directory[], int *segD, int D, int nrs)
{
	int i, j, nr_training = 0, dummy, L;
	char fullname[FNBUFFER], value[30];
	FILE *fnr = NULL, *fdata = NULL, *fsign = NULL;
	FILE *fmavs = NULL, *fmavs_stdv = NULL, *fzc_ave = NULL, *fzc_stdv = NULL;
	float **Stop_training_signals, **Left_training_signals, **Right_training_signals;
	float **Forward_training_signals;
	float **signatures;
	float **mav_aves, **mav_stds, **zc_aves, **zc_stds;
	float *aves_temp, *stds_temp;
	float thr = 0.0;
	Stat DataSignature;

	L = segD[D];	// = used_datapts

	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_training.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_training file for reading" << endl;
		return -1;
	}
	dummy = fscanf(fnr, "%d", &nr_training);
	fclose(fnr);

	// --- allocate memory -------------------
	signatures = (float **)calloc(nrs, sizeof(float *));
	for(i = 0; i < nrs; i++)
	{
		signatures[i] = (float *)calloc(L, sizeof(float));
	}

	Stop_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Left_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Right_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Forward_training_signals = (float **)calloc(nr_training, sizeof(float *));

	for(i = 0; i < nr_training; i++)
	{
		Stop_training_signals[i] = (float *)calloc(L, sizeof(float));
		Left_training_signals[i] = (float *)calloc(L, sizeof(float));
		Right_training_signals[i] = (float *)calloc(L, sizeof(float));
		Forward_training_signals[i] = (float *)calloc(L, sizeof(float));
	}

	mav_aves = (float **)calloc(D, sizeof(float *));
	mav_stds = (float **)calloc(D, sizeof(float *));
	zc_aves = (float **)calloc(D, sizeof(float *));
	zc_stds = (float **)calloc(D, sizeof(float *));
	for(i = 0; i < D; i++)
	{
		mav_aves[i] = (float *)calloc(nrs, sizeof(float));
		mav_stds[i] = (float *)calloc(nrs, sizeof(float));
		zc_aves[i] = (float *)calloc(nrs, sizeof(float));
		zc_stds[i] = (float *)calloc(nrs, sizeof(float));
	}

	aves_temp = (float *)calloc(D, sizeof(float));
	stds_temp = (float *)calloc(D, sizeof(float));


// ---------------- MAVS and SIGNATURES for the STOP signals ----------------------------
	for(i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Stop_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Stop_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Stop_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][0] = aves_temp[i];
		mav_stds[i][0] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Stop_training_signals[i][j], false);

		signatures[0][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ---------------- MAVS and SIGNATURES for the LEFT signals ----------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Left_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Left_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Left_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][1] = aves_temp[i];
		mav_stds[i][1] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Left_training_signals[i][j], false);

		signatures[1][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ------------------- MAVS and SIGNATURES for the RIGHT signals ------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Right_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Right_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features(Right_training_signals, nr_training, segD, D, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][2] = aves_temp[i];
		mav_stds[i][2] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Right_training_signals[i][j], false);

		signatures[2][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ------------------- MAVS and SIGNATURES for the FORWARD signals ----------------------
	if(nrs == 4)
	{
		for (i = 0; i < nr_training; i++)
		{
			memset(fullname, 0, FNBUFFER);
			sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Forward_%03d.txt", directory, i+1);
			fdata = fopen(fullname, "r");
			if(fdata == 0)
			{
				cerr << "Error: Could not open signal file for reading" << endl;
				return -1;
			}

			for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
				dummy = fscanf(fdata, "%f", &Forward_training_signals[i][j]);

			fclose(fdata);
		}

		get_MAV_features(Forward_training_signals, nr_training, segD, D, aves_temp, stds_temp);

		for(i = 0; i < D; i++)
		{
			mav_aves[i][3] = aves_temp[i];
			mav_stds[i][3] = stds_temp[i];
		}

		for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
		{
			for(i = 0; i < nr_training; i++)
				DataSignature.sample(Forward_training_signals[i][j], false);

			signatures[3][j] = DataSignature.avg();
			DataSignature.clear();
		}

		DataSignature.clear();
	}
// --------------------------------------------------------------------------------------

// ---------------------------- Threshold for ZC ----------------------------------------
	for(i = 0; i < D; i++)
		for(j = 0; j < nrs; j++)
			thr += mav_aves[i][j];

	thr = thr/(1.0*nrs*D);		// this threshold is necessary for the ZC function
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the STOP signals -----------------------------
	get_ZC_features(Stop_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][0] = aves_temp[i];
		zc_stds[i][0] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the LEFT signals -----------------------------
	get_ZC_features(Left_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][1] = aves_temp[i];
		zc_stds[i][1] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the RIGHT signals -----------------------------
	get_ZC_features(Right_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][2] = aves_temp[i];
		zc_stds[i][2] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the FORWARD signals --------------------------
	if(nrs == 4)
	{
		get_ZC_features(Forward_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp);

		for(i = 0; i < D; i++)
		{
			zc_aves[i][3] = aves_temp[i];
			zc_stds[i][3] = stds_temp[i];
		}
	}
// --------------------------------------------------------------------------------------

// ----------------------- Save to files ------------------------------------------------
	// opens file to store signatures ---------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/signatures1.txt", directory);
	fsign = fopen(fullname, "w");
	if(fsign == 0)
	{
		cerr << "Error: Could not open signature file1 for writing" << endl;
		return -1;
	}

	for(i = 0; i < nrs; i++)
	{
		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
		{
			sprintf(value, "%+1.7e  ", signatures[i][j]);
			fwrite(value, sizeof(char), 16, fsign); // save to file
		}
		fwrite("\n", sizeof(char), sizeof(char), fsign);
	}
	fclose(fsign);

//	// if a there is a new way of getting the signatures, use signatures2.
//	memset(fullname, 0, FNBUFFER);
//	sprintf(fullname,"../../Training/%s/signatures2.txt", directory);
//	fsign = fopen(fullname, "w");
//	if(fsign == 0)
//	{
//		cerr << "Error: Could not open signature file2 for writing" << endl;
//		return -1;
//	}
//
//	for(i = 0; i < nrs; i++)
//	{
//		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
//		{
//			sprintf(value, "%+1.7e  ", signatures[i][j]); // change to signatures2[i][j]
//			fwrite(value, sizeof(char), 16, fsign); // save to file
//		}
//		fwrite("\n", sizeof(char), sizeof(char), fsign);
//	}
//	fclose(fsign);


	// opens file to store mav_aves------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_aves.txt", directory);
	fmavs = fopen(fullname, "w");
	if(fmavs == 0)
	{
		cerr << "Error: Could not open signature mavs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", mav_aves[i][j]);
			fwrite(value, sizeof(char), 16, fmavs); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs); // save to file
	}
	fclose(fmavs);		// closes file with mav_aves


	// opens file to store mavs_stds ----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_stds.txt", directory);
	fmavs_stdv = fopen(fullname, "w");
	if(fmavs_stdv == 0)
	{
		cerr << "Error: Could not open mav_stdvs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", mav_stds[i][j]);
			fwrite(value, sizeof(char), 16, fmavs_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs_stdv); // save to file
	}
	fclose(fmavs_stdv);		// closes file with mav_aves


	// opens file to store zc_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_aves.txt", directory);
	fzc_ave = fopen(fullname, "w");
	if(fzc_ave == 0)
	{
		cerr << "Error: Could not open signature zc_ave file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", zc_aves[i][j]);
			fwrite(value, sizeof(char), 16, fzc_ave); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_ave); // save to file
	}
	fclose(fzc_ave);	// closes file with ZC averages


	// opens file to store zc_stds ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_stds.txt", directory);
	fzc_stdv = fopen(fullname, "w");
	if(fzc_stdv == 0)
	{
		cerr << "Error: Could not open zc_stdv file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", zc_stds[i][j]);
			fwrite(value, sizeof(char), 16, fzc_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_stdv); // save to file
	}
	fclose(fzc_stdv);		// closes file with ZC standard deviations
//---------------------------------------------------------------------------------------

	// This is just to allow the group to modify the files just created -----------------
	sprintf(fullname, "chmod g+w -R /home/pi/EMG_MICO/Training/%s\n", directory);
	dummy = system(fullname);
	dummy++;	// this is just to avoid a warning...

	// Free memory that was allocated
	free(Stop_training_signals);
	free(Left_training_signals);
	free(Right_training_signals);
	free(Forward_training_signals);
	free(mav_aves);
	free(mav_stds);
	free(zc_aves);
	free(zc_stds);
	free(aves_temp);
	free(stds_temp);

	return 1;	// should be >= 0
}

// Function to get the signatures and other parameters needed for the classification
// algorithm. It includes the calculation of the MAVs and ZCs from one or more segments
// or the signal.
// NEW: this function calculates the weights to be associated to the different features.
// 		The weights are stored in a file, like other training parameters.
//		There is an input which is an option for actually calculating the weights, or for
//		using the default value of 1.0 for all the weights.
// Inputs: directory - string with the user name, which serves as the name of the direc-
//					   tory where the training signals are to be found, and where the
//					   calculated signatures and parameters will be stored.
//				segD - vector with the D + 1 positions where the signals are to be
//					   segmented from, including 0 and L = used_datapts:
//					   0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
//				   D - number of segments: D = length(segD) - 1;
//				 nrs - number of signatures (i.e., number of gestures to be used)
//		   weightOpt - 1, for actually calculating the weights
//					   0, for assigning a weight of 1.0 for all the features
// Output: -1 if there was any error (e.g. opening files), and a positive number if
//		   everything went well (value not important).
int get_signatures_v4(char directory[], int *segD, int D, int nrs, int weightOpt)
{
	int i, j, nr_training = 0, dummy, L;
	char fullname[FNBUFFER], value[30];
	FILE *fnr = NULL, *fdata = NULL, *fsign = NULL;
	FILE *fmavs = NULL, *fmavs_stdv = NULL, *fzc_ave = NULL, *fzc_stdv = NULL;
	FILE *f_weights = NULL;
	float **Stop_training_signals, **Left_training_signals, **Right_training_signals;
	float **Forward_training_signals;
	float **signatures;
	float **mav_aves, **mav_stds, **zc_aves, **zc_stds;
	float *aves_temp, *stds_temp;
	float thr = 0.0;
	Stat DataSignature;

	// NEW, for the weights
	float *feature_values;		// allocated later, after retrieving nr_training
	int *TrueLabels;			// allocated later, after retrieving nr_training
	float *mus = (float*)malloc(nrs*sizeof(float));
	float *stds = (float*)malloc(nrs*sizeof(float));
	float **Stop_mav_all, **Left_mav_all, **Right_mav_all, **Forward_mav_all;
	float **Stop_zc_all, **Left_zc_all, **Right_zc_all, **Forward_zc_all;
	float *mav_wgts = (float*)malloc(D*sizeof(float));
	float *zc_wgts = (float*)malloc(D*sizeof(float));


	L = segD[D];	// = used_datapts

	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_training.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_training file for reading" << endl;
		return -1;
	}
	dummy = fscanf(fnr, "%d", &nr_training);
	fclose(fnr);

	// --- allocate memory -------------------
	signatures = (float **)calloc(nrs, sizeof(float *));
	for(i = 0; i < nrs; i++)
	{
		signatures[i] = (float *)calloc(L, sizeof(float));
	}

	Stop_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Left_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Right_training_signals = (float **)calloc(nr_training, sizeof(float *));
	Forward_training_signals = (float **)calloc(nr_training, sizeof(float *));

	// NEW, for the weights
	Stop_mav_all = (float **)calloc(nr_training, sizeof(float *));
	Left_mav_all = (float **)calloc(nr_training, sizeof(float *));
	Right_mav_all = (float **)calloc(nr_training, sizeof(float *));
	Forward_mav_all = (float **)calloc(nr_training, sizeof(float *));
	Stop_zc_all = (float **)calloc(nr_training, sizeof(float *));
	Left_zc_all = (float **)calloc(nr_training, sizeof(float *));
	Right_zc_all = (float **)calloc(nr_training, sizeof(float *));
	Forward_zc_all = (float **)calloc(nr_training, sizeof(float *));

	for(i = 0; i < nr_training; i++)
	{
		Stop_training_signals[i] = (float *)calloc(L, sizeof(float));
		Left_training_signals[i] = (float *)calloc(L, sizeof(float));
		Right_training_signals[i] = (float *)calloc(L, sizeof(float));
		Forward_training_signals[i] = (float *)calloc(L, sizeof(float));

		// NEW, for the weights
		Stop_mav_all[i] = (float *)calloc(D, sizeof(float *));
		Left_mav_all[i] = (float *)calloc(D, sizeof(float *));
		Right_mav_all[i] = (float *)calloc(D, sizeof(float *));
		Forward_mav_all[i] = (float *)calloc(D, sizeof(float *));
		Stop_zc_all[i] = (float *)calloc(D, sizeof(float *));
		Left_zc_all[i] = (float *)calloc(D, sizeof(float *));
		Right_zc_all[i] = (float *)calloc(D, sizeof(float *));
		Forward_zc_all[i] = (float *)calloc(D, sizeof(float *));
	}

	mav_aves = (float **)calloc(D, sizeof(float *));
	mav_stds = (float **)calloc(D, sizeof(float *));
	zc_aves = (float **)calloc(D, sizeof(float *));
	zc_stds = (float **)calloc(D, sizeof(float *));
	for(i = 0; i < D; i++)
	{
		mav_aves[i] = (float *)calloc(nrs, sizeof(float));
		mav_stds[i] = (float *)calloc(nrs, sizeof(float));
		zc_aves[i] = (float *)calloc(nrs, sizeof(float));
		zc_stds[i] = (float *)calloc(nrs, sizeof(float));
	}

	aves_temp = (float *)calloc(D, sizeof(float));
	stds_temp = (float *)calloc(D, sizeof(float));

	// NEW, for the weights
	feature_values = (float *)malloc(nrs*nr_training*sizeof(float));
	TrueLabels = (int *)malloc(nrs*nr_training*sizeof(int));


// ---------------- MAVS and SIGNATURES for the STOP signals ----------------------------
	for(i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Stop_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Stop_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features_v2(Stop_training_signals, nr_training, segD, D, aves_temp, stds_temp, Stop_mav_all);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][0] = aves_temp[i];
		mav_stds[i][0] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Stop_training_signals[i][j], false);

		signatures[0][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ---------------- MAVS and SIGNATURES for the LEFT signals ----------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Left_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Left_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features_v2(Left_training_signals, nr_training, segD, D, aves_temp, stds_temp, Left_mav_all);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][1] = aves_temp[i];
		mav_stds[i][1] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Left_training_signals[i][j], false);

		signatures[1][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ------------------- MAVS and SIGNATURES for the RIGHT signals ------------------------
	for (i = 0; i < nr_training; i++)
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Right_%03d.txt", directory, i+1);
		fdata = fopen(fullname, "r");
		if(fdata == 0)
		{
			cerr << "Error: Could not open signal file for reading" << endl;
			return -1;
		}

		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
			dummy = fscanf(fdata, "%f", &Right_training_signals[i][j]);

		fclose(fdata);
	}

	get_MAV_features_v2(Right_training_signals, nr_training, segD, D, aves_temp, stds_temp, Right_mav_all);

	for(i = 0; i < D; i++)
	{
		mav_aves[i][2] = aves_temp[i];
		mav_stds[i][2] = stds_temp[i];
	}

	for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
	{
		for(i = 0; i < nr_training; i++)
			DataSignature.sample(Right_training_signals[i][j], false);

		signatures[2][j] = DataSignature.avg();
		DataSignature.clear();
	}

	DataSignature.clear();
// --------------------------------------------------------------------------------------

// ------------------- MAVS and SIGNATURES for the FORWARD signals ----------------------
	if(nrs == 4)
	{
		for (i = 0; i < nr_training; i++)
		{
			memset(fullname, 0, FNBUFFER);
			sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/Forward_%03d.txt", directory, i+1);
			fdata = fopen(fullname, "r");
			if(fdata == 0)
			{
				cerr << "Error: Could not open signal file for reading" << endl;
				return -1;
			}

			for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
				dummy = fscanf(fdata, "%f", &Forward_training_signals[i][j]);

			fclose(fdata);
		}

		get_MAV_features_v2(Forward_training_signals, nr_training, segD, D, aves_temp, stds_temp, Forward_mav_all);

		for(i = 0; i < D; i++)
		{
			mav_aves[i][3] = aves_temp[i];
			mav_stds[i][3] = stds_temp[i];
		}

		for(j = 0; j < L; j++)		// before: USED_DATAPTS instead of L
		{
			for(i = 0; i < nr_training; i++)
				DataSignature.sample(Forward_training_signals[i][j], false);

			signatures[3][j] = DataSignature.avg();
			DataSignature.clear();
		}

		DataSignature.clear();
	}
// --------------------------------------------------------------------------------------

// ---------------------------- Threshold for ZC ----------------------------------------
	for(i = 0; i < D; i++)
		for(j = 0; j < nrs; j++)
			thr += mav_aves[i][j];

	thr = thr/(1.0*nrs*D);		// this threshold is necessary for the ZC function
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the STOP signals -----------------------------
	get_ZC_features_v2(Stop_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp, Stop_zc_all);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][0] = aves_temp[i];
		zc_stds[i][0] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the LEFT signals -----------------------------
	get_ZC_features_v2(Left_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp, Left_zc_all);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][1] = aves_temp[i];
		zc_stds[i][1] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the RIGHT signals -----------------------------
	get_ZC_features_v2(Right_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp, Right_zc_all);

	for(i = 0; i < D; i++)
	{
		zc_aves[i][2] = aves_temp[i];
		zc_stds[i][2] = stds_temp[i];
	}
// --------------------------------------------------------------------------------------

// -------------------- Zero Crossings for the FORWARD signals --------------------------
	if(nrs == 4)
	{
		get_ZC_features_v2(Forward_training_signals, nr_training, segD, D, thr, aves_temp, stds_temp, Forward_zc_all);

		for(i = 0; i < D; i++)
		{
			zc_aves[i][3] = aves_temp[i];
			zc_stds[i][3] = stds_temp[i];
		}
	}
// --------------------------------------------------------------------------------------

// ---------------- Calculate the weights of the different features ---------------------
	for(i = 0; i < D; i++)
	{
		// --- MAV weights ---------------
		for(j = 0; j < nr_training; j++)
		{
			feature_values[j] = Stop_mav_all[j][i];
			TrueLabels[j] = STOP_COM;
			feature_values[j+nr_training] = Left_mav_all[j][i];
			TrueLabels[j+nr_training] = LEFT_COM;
			feature_values[j+2*nr_training] = Right_mav_all[j][i];
			TrueLabels[j+2*nr_training] = RIGHT_COM;
			if(nrs == 4)
			{
				feature_values[j+3*nr_training] = Forward_mav_all[j][i];
				TrueLabels[j+3*nr_training] = FORWARD_COM;
			}
		}

		for(j = 0; j < nrs; j++)
		{
			mus[j] = mav_aves[i][j];
			stds[j] = mav_stds[i][j];
		}

		if(weightOpt == 1)
			mav_wgts[i] = get_feature_weight(feature_values, nrs*nr_training, mus, stds, nrs, TrueLabels, WA, WB);
		else
			mav_wgts[i] = 1.0;

		// --- ZC weights ---------------
		for(j = 0; j < nr_training; j++)
		{
			feature_values[j] = Stop_zc_all[j][i];
			feature_values[j+nr_training] = Left_zc_all[j][i];
			feature_values[j+2*nr_training] = Right_zc_all[j][i];
			if(nrs == 4)
			{
				feature_values[j+3*nr_training] = Forward_zc_all[j][i];
			}
		}

		for(j = 0; j < nrs; j++)
		{
			mus[j] = zc_aves[i][j];
			stds[j] = zc_stds[i][j];
		}

		if(weightOpt == 1)
			zc_wgts[i] = get_feature_weight(feature_values, nrs*nr_training, mus, stds, nrs, TrueLabels, WA, WB);
		else
			zc_wgts[i] = 1.0;
	}
// --------------------------------------------------------------------------------------

// ----------------------- Save to files ------------------------------------------------
	// opens file to store signatures ---------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/signatures1.txt", directory);
	fsign = fopen(fullname, "w");
	if(fsign == 0)
	{
		cerr << "Error: Could not open signature file1 for writing" << endl;
		return -1;
	}

	for(i = 0; i < nrs; i++)
	{
		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
		{
			sprintf(value, "%+1.7e  ", signatures[i][j]);
			fwrite(value, sizeof(char), 16, fsign); // save to file
		}
		fwrite("\n", sizeof(char), sizeof(char), fsign);
	}
	fclose(fsign);

//	// if a there is a new way of getting the signatures, use signatures2.
//	memset(fullname, 0, FNBUFFER);
//	sprintf(fullname,"../../Training/%s/signatures2.txt", directory);
//	fsign = fopen(fullname, "w");
//	if(fsign == 0)
//	{
//		cerr << "Error: Could not open signature file2 for writing" << endl;
//		return -1;
//	}
//
//	for(i = 0; i < nrs; i++)
//	{
//		for(j = 0; j < L; j++)	// before: USED_DATAPTS instead of L
//		{
//			sprintf(value, "%+1.7e  ", signatures[i][j]); // change to signatures2[i][j]
//			fwrite(value, sizeof(char), 16, fsign); // save to file
//		}
//		fwrite("\n", sizeof(char), sizeof(char), fsign);
//	}
//	fclose(fsign);


	// opens file to store mav_aves------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_aves.txt", directory);
	fmavs = fopen(fullname, "w");
	if(fmavs == 0)
	{
		cerr << "Error: Could not open signature mavs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", mav_aves[i][j]);
			fwrite(value, sizeof(char), 16, fmavs); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs); // save to file
	}
	fclose(fmavs);		// closes file with mav_aves


	// opens file to store mavs_stds ----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_stds.txt", directory);
	fmavs_stdv = fopen(fullname, "w");
	if(fmavs_stdv == 0)
	{
		cerr << "Error: Could not open mav_stdvs' file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", mav_stds[i][j]);
			fwrite(value, sizeof(char), 16, fmavs_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fmavs_stdv); // save to file
	}
	fclose(fmavs_stdv);		// closes file with mav_aves


	// opens file to store zc_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_aves.txt", directory);
	fzc_ave = fopen(fullname, "w");
	if(fzc_ave == 0)
	{
		cerr << "Error: Could not open signature zc_ave file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", zc_aves[i][j]);
			fwrite(value, sizeof(char), 16, fzc_ave); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_ave); // save to file
	}
	fclose(fzc_ave);	// closes file with ZC averages


	// opens file to store zc_stds ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_stds.txt", directory);
	fzc_stdv = fopen(fullname, "w");
	if(fzc_stdv == 0)
	{
		cerr << "Error: Could not open zc_stdv file for writing" << endl;
		return -1;
	}

	memset(value, 0, 30);
	for(i = 0; i < D; i++)
	{
		for(j = 0; j < nrs; j++)
		{
			sprintf(value, "%1.7e  \t", zc_stds[i][j]);
			fwrite(value, sizeof(char), 16, fzc_stdv); // save to file
		}
		fwrite("\n", sizeof(char), 1, fzc_stdv); // save to file
	}
	fclose(fzc_stdv);		// closes file with ZC standard deviations


	// opens file to store mav_wgts -----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_weights.txt", directory);
	f_weights = fopen(fullname, "w");
	if(f_weights == 0)
	{
		cerr << "Error: Could not open mav_weights file for writing" << endl;
		return -1;
	}

	for(i = 0; i < D; i++)
	{
		fprintf(f_weights, "%1.2f    ", mav_wgts[i]);
	}
	fclose(f_weights);		// closes file with MAV weights


	// opens file to store zc_wgts ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_weights.txt", directory);
	f_weights = fopen(fullname, "w");
	if(f_weights == 0)
	{
		cerr << "Error: Could not open zc_weights file for writing" << endl;
		return -1;
	}

	for(i = 0; i < D; i++)
	{
		fprintf(f_weights, "%1.2f    ", zc_wgts[i]);
	}
	fclose(f_weights);		// closes file with ZC weights

//---------------------------------------------------------------------------------------


	// This is just to allow the group to modify the files just created -----------------
	sprintf(fullname, "chmod g+w -R /home/pi/EMG_MICO/Training/%s\n", directory);
	dummy = system(fullname);
	dummy++;	// this is just to avoid a warning...

	// Free memory that was allocated
	free(Stop_training_signals);			free(Left_training_signals);
	free(Right_training_signals);			free(Forward_training_signals);
	free(mav_aves);		free(mav_stds);		free(zc_aves);	free(zc_stds);
	free(aves_temp);	free(stds_temp);

	free(feature_values);	free(TrueLabels);	free(mus);	free(stds);
	free(Stop_mav_all);	free(Left_mav_all);	free(Right_mav_all);	free(Forward_mav_all);
	free(Stop_zc_all);	free(Left_zc_all);	free(Right_zc_all);		free(Forward_zc_all);
	free(mav_wgts);		free(zc_wgts);

	return 1;	// should be >= 0
}


/* This function reads the extracted signal and classifies it according to the GUSSS
 * ratio and the MAVs and ZC criteria. ----------------- OBSOLETE -----------------------
 * It implements the ICA algorithm for the ratios for the entire signal, and segments the
 * signal according to the segD array for the MAV and # of ZC.
 * Inputs:	vector with the testing signal (Y); number of points (L); matrix with the
 * 			signatures (S); array with the MAVs (mav_aves); array with the standard
 * 			deviations of the MAVs(mav_stds); arrays with the Zero Count averages (zc_aves)
 * 			and the corresponding standard deviations (zc_stds). Also, the idex(ces) where
 * 			to divide the incoming signal (segment it to get the individual ZC parameters):
 * 			that is D (integer or array of integers, if the signal is to be divided into
 * 			more than two segments).
 * Output:	the assigned class.														*/
int EMG_classify_v2(mat Y, int L, mat S, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D)
{
// --------------------- Definitions, initializations, etc ------------------------------
	int i, j, d, movement = 0, Lcount = 0;
	float thr = 0.0, temp = 0.0, normratios = 0.0;
	float *y, *mav, *normmavdist, *zc, *normzcdist, *zc_stdv_temp;
	int Movements[nrS] = {STOP_COM, LEFT_COM, RIGHT_COM};

	mat Sp = zeros(1, L);			// for the signatures
	mat ratios = zeros(1,nrS);				// to store the ratios
	mat mav_dist = zeros(D,nrS);			// for MAV distances
	mat zc_dist = zeros(D, nrS);			// for the ZC distances
	mat class_dist = zeros(1,nrS);			// for the final distances (classification)

	y = (float *)calloc(L, sizeof(float));				// maximum possible length
	mav = (float *)calloc(D, sizeof(float));			// MAV of every segment
	normmavdist = (float *)calloc(D, sizeof(float));	// needed when normalizing MAV distances
	zc = (float *)calloc(D, sizeof(float));				// number of ZC of every segment
	normzcdist = (float *)calloc(D, sizeof(float));		// needed when normalizing ZC distances
	zc_stdv_temp = (float *)calloc(D, sizeof(float));	// used in case there are stds == 0

//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	// Analysis and ratios for all signatures
	for(i = 0; i < nrS; i++)
	{
		for(j = 0; j < L; j++)
			Sp(0,j) = S(i,j);

		ratios(0,i) = (double)GUSSS_ratio_EMG_v2(Y, Sp, L);
	}

// ----------------------------------- MAVs ---------------------------------------------
	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		mav[d] = 0.0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			mav[d] += abs(Y(0,j));
		}
		mav[d] = mav[d]/(1.0*Lcount);
	}

	// get the distances from the test MAV to the average MAVs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			mav_dist(d,j) = abs((mav_aves[d][j] - mav[d])/mav_stds[d][j]);
		}
	}

// -------------------------------- Zero Counts -----------------------------------------
	for(d = 0; d < D; d++)
		for(j = 0; j < nrS; j++)
			thr += mav_aves[d][j];

	thr = thr/(1.0*nrS*D);		// this threshold is necessary for the ZC function

	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			y[j-segD[d]] = Y(0,j);
		}
		zc[d] = 1.0*Zero_crossings(y, Lcount, thr);
	}

// Be careful about ZC standard deviations being zero... Calculate 'just in case' values
	for (d = 0; d < D; d++)		// How many segments
	{
		zc_stdv_temp[d] = 1.0*USED_DATAPTS;	// big number, just to initialize
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			// get the minimum, non zero standard deviation
			if((zc_stds[d][j] != 0.0) && (zc_stdv_temp[d] > zc_stds[d][j]))
				zc_stdv_temp[d] = zc_stds[d][j];
		}
		zc_stdv_temp[d] = zc_stdv_temp[d]/2.0;	// replace 0 with 1/2 of the minimum non-zero
	}

	// get the distances from the test ZC to the average ZCs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			if(zc_stds[d][j] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stds[d][j]);
			else if(zc_stdv_temp[d] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stdv_temp[d]);
			else
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/0.1);
		}
	}

// ------------------------- Normalization and classification --------------------------
	// Normalization of ratios and distances
	for(d = 0; d < D; d++)
	{
		normmavdist[d] = 0.0;
		normzcdist[d] = 0.0;
	}

	for (j = 0; j < nrS; j++)
	{
		normratios += ratios(0,j);
		for(d = 0; d < D; d++)
		{
			normmavdist[d] += mav_dist(d,j);
			normzcdist[d] += zc_dist(d,j);
		}
	}

	ratios = ratios/normratios;
	for(d = 0; d < D; d++)
	{
		for (j = 0; j < nrS; j++)
		{
			mav_dist(d,j) = mav_dist(d,j)/normmavdist[d];
			zc_dist(d,j) = zc_dist(d,j)/normzcdist[d];
		}
	}

	temp = -1.0;		// initialize
	for(i = 0; i < nrS; i++)
	{
		class_dist(0,i) = (float)pow(1.0*ratios(0,i),2);

		for(d = 0; d < D; d++)
			class_dist(0,i) += (float)pow(1.0*mav_dist(d,i),2) + (float)pow(1.0*zc_dist(d,i),2);

		if((class_dist(0,i) < temp) || (temp < 0.0))
		{
			temp = class_dist(0,i);
			movement = Movements[i];
		}
	}

	// free allocated memory
	free(y);
	free(mav);
	free(normmavdist);
	free(zc);
	free(normzcdist);
	free(zc_stdv_temp);

	return movement;
}

/* This function reads the extracted signal and classifies it according to the GUSSS
 * ratio and the MAVs and ZC criteria.
 * It implements the ICA algorithm for the ratios for the entire signal, and segments the
 * signal according to the segD array for the MAV and # of ZC.
 * This function differs from v2 in that if the classifier is not certain, then an output
 * of 0 will indicate this fact.
* Inputs:	vector with the testing signal (Y); number of points (L); matrix with the
 * 			signatures (S); array with the MAVs (mav_aves);
 * 			array with the standard deviations of the MAVs(mav_stds); arrays with the
 * 			Zero Count averages (zc_aves) and the corresponding standard deviations
 * 			(zc_stds). Vector (segD) with the D + 1 positions where the signals are to be
 *			segmented from, including 0 and L = used_datapts:
 *			0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
 *			Number of segments (D = length(segD) - 1); classification threshold (cl_thr)
 * Output:	the assigned class (1, 2 or 3), or 0 if the classification was ambiguous.	*/
int EMG_classify_v3(mat Y, int L, mat S, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr)
{
// --------------------- Definitions, initializations, etc ------------------------------
	int i, j, d, movement = 0, Lcount = 0;
	float thr = 0.0, normratios = 0.0;
	float *y, *mav, *normmavdist, *zc, *normzcdist, *zc_stdv_temp;
	int Movements[nrS] = {STOP_COM, LEFT_COM, RIGHT_COM};

	mat Sp = zeros(1, L);		// for the signatures
	mat ratios = zeros(1,nrS);				// to store the ratios
	mat mav_dist = zeros(D,nrS);			// for MAV distances
	mat zc_dist = zeros(D, nrS);			// for the ZC distances
	//mat class_dist = zeros(1,nrS);			// for the final distances (classification)
	float class_dist[nrS] = {0.0};			// for the final distances (classification)

	y = (float *)calloc(L, sizeof(float));				// maximum possible length
	mav = (float *)calloc(D, sizeof(float));			// MAV of every segment
	normmavdist = (float *)calloc(D, sizeof(float));	// needed when normalizing MAV distances
	zc = (float *)calloc(D, sizeof(float));				// number of ZC of every segment
	normzcdist = (float *)calloc(D, sizeof(float));		// needed when normalizing ZC distances
	zc_stdv_temp = (float *)calloc(D, sizeof(float));	// used in case there are stds == 0

//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	// Analysis and ratios for all signatures
	for(i = 0; i < nrS; i++)
	{
		for(j = 0; j < L; j++)
			Sp(0,j) = S(i,j);

		ratios(0,i) = (double)GUSSS_ratio_EMG_v2(Y, Sp, L);
	}

// ----------------------------------- MAVs ---------------------------------------------
	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		mav[d] = 0.0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			mav[d] += abs(Y(0,j));
		}
		mav[d] = mav[d]/(1.0*Lcount);
	}

	// get the distances from the test MAV to the average MAVs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			mav_dist(d,j) = abs((mav_aves[d][j] - mav[d])/mav_stds[d][j]);
		}
	}

// -------------------------------- Zero Counts -----------------------------------------
	for(d = 0; d < D; d++)
		for(j = 0; j < nrS; j++)
			thr += mav_aves[d][j];

	thr = thr/(1.0*nrS*D);		// this threshold is necessary for the ZC function

	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			y[j-segD[d]] = Y(0,j);
		}
		zc[d] = 1.0*Zero_crossings(y, Lcount, thr);
	}

// Be careful about ZC standard deviations being zero... Calculate 'just in case' values
	for (d = 0; d < D; d++)		// How many segments
	{
		zc_stdv_temp[d] = 10000.0;	// big number, just to initialize
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			// get the minimum, non zero standard deviation
			if((zc_stds[d][j] != 0.0) && (zc_stdv_temp[d] > zc_stds[d][j]))
				zc_stdv_temp[d] = zc_stds[d][j];
		}
		zc_stdv_temp[d] = zc_stdv_temp[d]/2.0;	// replace 0 with 1/2 of the minimum non-zero
	}

	// get the distances from the test ZC to the average ZCs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrS; j++)	// How many signatures
		{
			if(zc_stds[d][j] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stds[d][j]);
			else if(zc_stdv_temp[d] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stdv_temp[d]);
			else
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/0.1);
		}
	}

// ------------------------- Normalization and classification --------------------------
	// Normalization of ratios and distances
	for(d = 0; d < D; d++)
	{
		normmavdist[d] = 0.0;
		normzcdist[d] = 0.0;
	}

	for (j = 0; j < nrS; j++)
	{
		normratios += ratios(0,j);
		for(d = 0; d < D; d++)
		{
			normmavdist[d] += mav_dist(d,j);
			normzcdist[d] += zc_dist(d,j);
		}
	}

	ratios = ratios/normratios;
	for(d = 0; d < D; d++)
	{
		for (j = 0; j < nrS; j++)
		{
			mav_dist(d,j) = mav_dist(d,j)/normmavdist[d];
			zc_dist(d,j) = zc_dist(d,j)/normzcdist[d];
		}
	}

	for(i = 0; i < nrS; i++)
	{
		class_dist[i] = (float)pow(1.0*ratios(0,i),2);

		for(d = 0; d < D; d++)
			class_dist[i] += (float)pow(1.0*mav_dist(d,i),2) + (float)pow(1.0*zc_dist(d,i),2);
	}

	itpp::Vec<float> class_dist_vec(class_dist,nrS); // create vector w/ elements of class_disrt
	Sort <float> mysort;	// Sorting object
	ivec sort_ind;			// vector of integers to store the sorted indexes

	// sort the distances and get the indexes (ascending order)
	sort_ind = mysort.sort_index(0, nrS-1, class_dist_vec);

	// if the two lowest distance values are closer than the classification threshold,
	// then return 0, meaning that the classification was ambiguous.
	if((class_dist[sort_ind(1)] - class_dist[sort_ind(0)]) > cl_thr)
		movement = Movements[sort_ind(0)];
	else
		movement = 0;		// classification was ambiguous

	// free allocated memory
	free(y);
	free(mav);
	free(normmavdist);
	free(zc);
	free(normzcdist);
	free(zc_stdv_temp);

	return movement;
}

/* This function reads the extracted signal and classifies it according to the GUSSS
 * ratio and the MAVs and ZC criteria.
 * It implements the ICA algorithm for the ratios for the entire signal, and segments the
 * signal according to the segD array for the MAV and # of ZC.
 * This function differs from v3 in that the number of signatures to be tested is an
 * input variable.
 * Inputs:	vector with the testing signal (Y); number of points (L); matrix with the
 * 			signatures (S); number of signatures (nrs); array with the MAVs (mav_aves);
 * 			array with the standard deviations of the MAVs(mav_stds); arrays with the
 * 			Zero Count averages (zc_aves) and the corresponding standard deviations
 * 			(zc_stds). Vector (segD) with the D + 1 positions where the signals are to be
 *			segmented from, including 0 and L = used_datapts:
 *			0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
 *			Number of segments (D = length(segD) - 1); classification threshold (cl_thr)
 * Output:	the assigned class (1, 2 or 3), or 0 if the classification was ambiguous.	*/
int EMG_classify_v4(mat Y, int L, mat S, int nrs, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr)
{
// --------------------- Definitions, initializations, etc ------------------------------
	int i, j, d, movement = 0, Lcount = 0;
	float thr = 0.0, normratios = 0.0;
	float *y, *mav, *normmavdist, *zc, *normzcdist, *zc_stdv_temp;
	int Movements[4] = {STOP_COM, LEFT_COM, RIGHT_COM, FORWARD_COM};

	mat Sp = zeros(1, L);				// for the signatures
	mat ratios = zeros(1,nrs);			// to store the ratios
	mat mav_dist = zeros(D,nrs);		// for MAV distances
	mat zc_dist = zeros(D, nrs);		// for the ZC distances
	float *class_dist;					// for the final distances (classification)

	class_dist = (float *)calloc(nrs, sizeof(float));
	y = (float *)calloc(L, sizeof(float));				// maximum possible length
	mav = (float *)calloc(D, sizeof(float));			// MAV of every segment
	normmavdist = (float *)calloc(D, sizeof(float));	// needed when normalizing MAV distances
	zc = (float *)calloc(D, sizeof(float));				// number of ZC of every segment
	normzcdist = (float *)calloc(D, sizeof(float));		// needed when normalizing ZC distances
	zc_stdv_temp = (float *)calloc(D, sizeof(float));	// used in case there are stds == 0

//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	// Analysis and ratios for all signatures
	for(i = 0; i < nrs; i++)
	{
		for(j = 0; j < L; j++)
			Sp(0,j) = S(i,j);

		ratios(0,i) = (double)GUSSS_ratio_EMG_v2(Y, Sp, L);
	}

// ----------------------------------- MAVs ---------------------------------------------
	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		mav[d] = 0.0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			mav[d] += abs(Y(0,j));
		}
		mav[d] = mav[d]/(1.0*Lcount);
	}

	// get the distances from the test MAV to the average MAVs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			mav_dist(d,j) = abs((mav_aves[d][j] - mav[d])/mav_stds[d][j]);
		}
	}

// -------------------------------- Zero Counts -----------------------------------------
	for(d = 0; d < D; d++)
		for(j = 0; j < nrs; j++)
			thr += mav_aves[d][j];

	thr = thr/(1.0*nrs*D);		// this threshold is necessary for the ZC function

	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			y[j-segD[d]] = Y(0,j);
		}
		zc[d] = 1.0*Zero_crossings(y, Lcount, thr);
	}

// Be careful about ZC standard deviations being zero... Calculate 'just in case' values
	for (d = 0; d < D; d++)		// How many segments
	{
		zc_stdv_temp[d] = 10000.0;	// big number, just to initialize
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			// get the minimum, non zero standard deviation
			if((zc_stds[d][j] != 0.0) && (zc_stdv_temp[d] > zc_stds[d][j]))
				zc_stdv_temp[d] = zc_stds[d][j];
		}
		zc_stdv_temp[d] = zc_stdv_temp[d]/2.0;	// replace 0 with 1/2 of the minimum non-zero
	}

	// get the distances from the test ZC to the average ZCs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			if(zc_stds[d][j] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stds[d][j]);
			else if(zc_stdv_temp[d] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stdv_temp[d]);
			else
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/0.1);
		}
	}

// ------------------------- Normalization and classification --------------------------
	// Normalization of ratios and distances
	for(d = 0; d < D; d++)
	{
		normmavdist[d] = 0.0;
		normzcdist[d] = 0.0;
	}

	for (j = 0; j < nrs; j++)
	{
		normratios += ratios(0,j);
		for(d = 0; d < D; d++)
		{
			normmavdist[d] += mav_dist(d,j);
			normzcdist[d] += zc_dist(d,j);
		}
	}

	ratios = ratios/normratios;
	for(d = 0; d < D; d++)
	{
		for (j = 0; j < nrs; j++)
		{
			mav_dist(d,j) = mav_dist(d,j)/normmavdist[d];
			zc_dist(d,j) = zc_dist(d,j)/normzcdist[d];
		}
	}

	for(i = 0; i < nrs; i++)
	{
		class_dist[i] = (float)pow(1.0*ratios(0,i),2);

		for(d = 0; d < D; d++)
			class_dist[i] += (float)pow(1.0*mav_dist(d,i),2) + (float)pow(1.0*zc_dist(d,i),2);
	}

	itpp::Vec<float> class_dist_vec(class_dist,nrs); // create vector w/ elements of class_disrt
	Sort <float> mysort;	// Sorting object
	ivec sort_ind;			// vector of integers to store the sorted indexes

	// sort the distances and get the indexes (ascending order)
	sort_ind = mysort.sort_index(0, nrs-1, class_dist_vec);

	// if the two lowest distance values are closer than the classification threshold,
	// then return 0, meaning that the classification was ambiguous.
	if((class_dist[sort_ind(1)] - class_dist[sort_ind(0)]) > cl_thr)
		movement = Movements[sort_ind(0)];
	else
		movement = 0;		// classification was ambiguous

	// free allocated memory
	free(y);
	free(mav);
	free(normmavdist);
	free(zc);
	free(normzcdist);
	free(zc_stdv_temp);

	return movement;
}

/* This function reads the extracted signal and classifies it according to the GUSSS
 * ratio and the MAVs and ZC criteria.
 * It implements the ICA algorithm for the ratios for the entire signal, and segments the
 * signal according to the segD array for the MAV and # of ZC.
 * This function differs from v4 in that we use the weights for the features that we
 * calculated during training.
 * Inputs:	vector with the testing signal (Y); number of points (L); matrix with the
 * 			signatures (S); number of signatures (nrs); array with the MAVs (mav_aves);
 * 			array with the standard deviations of the MAVs(mav_stds); arrays with the
 * 			Zero Count averages (zc_aves) and the corresponding standard deviations
 * 			(zc_stds). Vector (segD) with the D + 1 positions where the signals are to be
 *			segmented from, including 0 and L = used_datapts:
 *			0 = segD[0] < segD[1] < ... < segD[D-1] < segD[D] = L
 *			Number of segments (D = length(segD) - 1); classification threshold (cl_thr)
 *			Weights for the MAV features (mav_wgts) and for the ZC features (zc_wgts).
 * Output:	the assigned class (1, 2 or 3), or 0 if the classification was ambiguous.	*/
int EMG_classify_v5(mat Y, int L, mat S, int nrs, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr,
					float *mav_wgts, float *zc_wgts)
{
// --------------------- Definitions, initializations, etc ------------------------------
	int i, j, d, movement = 0, Lcount = 0;
	float thr = 0.0, normratios = 0.0;
	float *y, *mav, *normmavdist, *zc, *normzcdist, *zc_stdv_temp;
	int Movements[4] = {STOP_COM, LEFT_COM, RIGHT_COM, FORWARD_COM};

	mat Sp = zeros(1, L);				// for the signatures
	mat ratios = zeros(1,nrs);			// to store the ratios
	mat mav_dist = zeros(D,nrs);		// for MAV distances
	mat zc_dist = zeros(D, nrs);		// for the ZC distances
	float *class_dist;					// for the final distances (classification)

	class_dist = (float *)calloc(nrs, sizeof(float));
	y = (float *)calloc(L, sizeof(float));				// maximum possible length
	mav = (float *)calloc(D, sizeof(float));			// MAV of every segment
	normmavdist = (float *)calloc(D, sizeof(float));	// needed when normalizing MAV distances
	zc = (float *)calloc(D, sizeof(float));				// number of ZC of every segment
	normzcdist = (float *)calloc(D, sizeof(float));		// needed when normalizing ZC distances
	zc_stdv_temp = (float *)calloc(D, sizeof(float));	// used in case there are stds == 0


//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	// Analysis and ratios for all signatures
	for(i = 0; i < nrs; i++)
	{
		for(j = 0; j < L; j++)
			Sp(0,j) = S(i,j);

		ratios(0,i) = (double)GUSSS_ratio_EMG_v2(Y, Sp, L);
	}

// ----------------------------------- MAVs ---------------------------------------------
	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		mav[d] = 0.0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			mav[d] += abs(Y(0,j));
		}
		mav[d] = mav[d]/(1.0*Lcount);
	}

	// get the distances from the test MAV to the average MAVs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			mav_dist(d,j) = abs((mav_aves[d][j] - mav[d])/mav_stds[d][j]);
		}
	}

// -------------------------------- Zero Counts -----------------------------------------
	for(d = 0; d < D; d++)
		for(j = 0; j < nrs; j++)
			thr += mav_aves[d][j];

	thr = thr/(1.0*nrs*D);		// this threshold is necessary for the ZC function

	for(d = 0; d < D; d++)
	{
		Lcount = segD[d+1] - segD[d];		// = 0;
		for(j = segD[d]; j < segD[d+1]; j++)
		{
			y[j-segD[d]] = Y(0,j);
		}
		zc[d] = 1.0*Zero_crossings(y, Lcount, thr);
	}

// Be careful about ZC standard deviations being zero... Calculate 'just in case' values
	for (d = 0; d < D; d++)		// How many segments
	{
		zc_stdv_temp[d] = 10000.0;	// big number, just to initialize
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			// get the minimum, non zero standard deviation
			if((zc_stds[d][j] != 0.0) && (zc_stdv_temp[d] > zc_stds[d][j]))
				zc_stdv_temp[d] = zc_stds[d][j];
		}
		zc_stdv_temp[d] = zc_stdv_temp[d]/2.0;	// replace 0 with 1/2 of the minimum non-zero
	}

	// get the distances from the test ZC to the average ZCs obtained from training
	for (d = 0; d < D; d++)		// How many segments
	{
		for (j = 0; j < nrs; j++)	// How many signatures
		{
			if(zc_stds[d][j] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stds[d][j]);
			else if(zc_stdv_temp[d] != 0.0)
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/zc_stdv_temp[d]);
			else
				zc_dist(d,j) = abs((zc_aves[d][j] - zc[d])/0.1);
		}
	}

// ------------------------- Normalization and classification --------------------------
	// Normalization of ratios and distances
	for(d = 0; d < D; d++)
	{
		normmavdist[d] = 0.0;
		normzcdist[d] = 0.0;
	}

	for (j = 0; j < nrs; j++)
	{
		normratios += ratios(0,j);
		for(d = 0; d < D; d++)
		{
			normmavdist[d] += mav_dist(d,j);
			normzcdist[d] += zc_dist(d,j);
		}
	}

	ratios = ratios/normratios;
	for(d = 0; d < D; d++)
	{
		for (j = 0; j < nrs; j++)
		{
			mav_dist(d,j) = mav_dist(d,j)/normmavdist[d];
			zc_dist(d,j) = zc_dist(d,j)/normzcdist[d];
		}
	}

	for(i = 0; i < nrs; i++)
	{
		class_dist[i] = 1.0*((float)pow(ratios(0,i),2));

		for(d = 0; d < D; d++)
			class_dist[i] += mav_wgts[d]*((float)pow(mav_dist(d,i),2)) + zc_wgts[d]*((float)pow(zc_dist(d,i),2));
	}

	itpp::Vec<float> class_dist_vec(class_dist,nrs); // create vector w/ elements of class_disrt
	Sort <float> mysort;	// Sorting object
	ivec sort_ind;			// vector of integers to store the sorted indexes

	// sort the distances and get the indexes (ascending order)
	sort_ind = mysort.sort_index(0, nrs-1, class_dist_vec);

	// if the two lowest distance values are closer than the classification threshold,
	// then return 0, meaning that the classification was ambiguous.
	if((class_dist[sort_ind(1)] - class_dist[sort_ind(0)]) > cl_thr)
		movement = Movements[sort_ind(0)];
	else
		movement = 0;		// classification was ambiguous

	// free allocated memory
	free(y);
	free(mav);
	free(normmavdist);
	free(zc);
	free(normzcdist);
	free(zc_stdv_temp);

	return movement;
}



// Function to prevent the optimizer to ignore a while loop in the ADC reading function.


// This function is for sub-sampling a particular signal, i.e, adjust the number of
// points of the signal to match the desired length. This works also to increase the
// number of points of a given signal.
// Inputs: yc - array with the original signal
//			C - length of the original signal
//		   yn - array to store the sub-sampled signal
//			N - desired length of the sub-sampled signal
void sub_sample(float *yc, int C, double *yn, int N)
{
	int n, a, b;
	float tn, m;

	for(n = 0; n < N; n++)
	{
		tn = 1.0*n*(C-1)/(N-1);
		a = (int)floor(tn);
		b = (int)ceil(tn);
		if(b > a)
		{
			m = (yc[b] - yc[a])/(1.0*(b-a));	// slope
			yn[n] = m*(tn - 1.0*a) + yc[a];
		}
		else
			yn[n] = yc[a];
	}
}



// Function that checks the value of the push button data, during training.
// If valid (1 or 2), the program is terminated. The client_status.txt file is modified
// as necessary.
void push_buttons_training(unsigned char pb_data)
{
	int client_status;
	FILE *fclient_status = NULL;
	char value[30];

	if(pb_data == 32)	// button for canceling entire training process
	{
#if PRINTF > 0
		printf("Received from push button pipe: 1\n");
		fflush(stdout);
#endif
		client_status = 2;	// "use" mode code

		fclient_status = fopen("client_status.txt", "w");
		if(fclient_status == 0)
		{
			cerr << "Error: Could not client_status.txt" << endl;
		}
		else
		{
			sprintf(value, "%d\n", client_status);
			fwrite(value, sizeof(char), 2, fclient_status); // save to file
			fclose(fclient_status);
		}

		exit(0);
	}

	if(pb_data == 64)	// button for reseting current gesture training
	{
#if PRINTF > 0
		printf("Received from push button pipe: 2\n");
		fflush(stdout);
#endif
		// don't need to change the client_status file
		exit(0);
	}

#if PRINTF > 0
	printf("Invalid value for the push button pipe: %d\n", pb_data);
	fflush(stdout);
#endif

}



// Calculates the Rand index.
// Inputs: 	LC - Labels obtained from clustering algorithm
//			LP - External labels
//			 N - size of LC and LP integer arrays
// Output:	 R - Rand statistic
float Rand_index(int *LC, int *LP, int N)
{
	float R;
	int a = 0, b = 0, c = 0, d = 0, n, m;

	for(n = 0; n < N-1; n++)
	{
		for(m = n+1; m < N; n++)
		{
			if(LC[n] == LC[m])	// points in the same cluster, Clustering alg.
			{
				if(LP[n] == LP[m])	// points in the same cluster, external
					a++;
				else				// points in different clusters, external
					b++;
			}
			else				// points in different clusters, Clustering alg.
			{
				if(LP[n] == LP[m])	// points in the same cluster, external
					c++;
				else				// points in different clusters, external
					d++;
			}

		}
	}

	R = 1.0*(a+d)/(1.0*(a+b+c+d));
	return(R);
}



// Function to calculate the weight for a particular feature to be used in the distance
// classification algorithm.
// Inputs: values - array with all the feature values
//				N - number of features (length of values)
//			  mus - array with the mean values of each class
//			 stds - array with the standard deviation values of each class
//			  nrs - number of classes (length of mus and stds)
//			   TL - true labels of the values
//				a - zero-weight threshold (0 <= a < b <= 1)
//				b - one-weight threshold
// Output: weight value
float get_feature_weight(float *values, int N, float *mus, float *stds, int nrs, int *TL,
						 float a, float b)
{
	if((a >= b) || (a < 0.0) || (b > 1.0))
		return(-1.0);	// invalid a and b values

	float weight, pr = 0.0, stdv_temp = 0.0;
	int n, s;
	int *DL = (int *)malloc(N*sizeof(int));
	float *class_dist = (float *)calloc(nrs, sizeof(float));
	int Movements[4] = {STOP_COM, LEFT_COM, RIGHT_COM, FORWARD_COM};
	Sort <float> mysort;	// Sorting object
	ivec sort_ind;			// vector of integers to store the sorted indexes
	itpp::Vec<float> class_dist_vec(class_dist, nrs); // create object (values not important here)
	itpp::Vec<float> standard_devs(stds, nrs); // create object (values not important here)

	// sort the std devs and get the indexes (ascending order)
	sort_ind = mysort.sort_index(0, nrs-1, standard_devs);

	for(s = 0; s < nrs; s++)	// How many signatures
	{	// get the minimum, non zero standard deviation
		if(stds[sort_ind(s)] > 0.0)
		{
			stdv_temp = stds[sort_ind(s)]/2.0;	// 1/2 of the minimum non-zero std. dev.
			break;
		}
	}

	if(stdv_temp == 0.0)	// means that all the standard deviations were 0
		stdv_temp = 1.0;	// doesn't really matter the value

	for(n = 0; n < N; n++)
	{
		for(s = 0; s < nrs; s++)
		{
			if(stds[s] != 0.0)	// avoid dividing by zero
				class_dist_vec.set(s, abs(values[n]-mus[s])/stds[s]);
			else
				class_dist_vec.set(s, abs(values[n]-mus[s])/stdv_temp);
		}
		// sort the distances and get the indexes (ascending order)
		sort_ind = mysort.sort_index(0, nrs-1, class_dist_vec);
		DL[n] = Movements[sort_ind(0)];

		if(DL[n] == TL[n])
			pr = pr + 1.0;		// count how many were correctly classified
	}

	pr = pr/(1.0*N);

	weight = (pr-a)/(b-a);
	if(weight < 0.0)
		weight = 0.0;
	if(weight > 1.0)
		weight = 1.0;

	free(DL);
	free(class_dist);

	return(weight);
}

//**********************************************************************************************

// This function calculates all the training parameters from a given set of training
// signals.
// Inputs: data_all - ptr to an array of matrices containing the training signals
//			   segD - ptr to a vector with the D + 1 positions where the signals are to
//					  be segmented from, including 0 and L = used_datapts:
//					  0 = segD(0) < segD(1) < ... < segD(D-2) < segD(D-1) = L
//			   Sigs - ptr to a matrix to store the signatures of the gestures.
//				  S - number of signatures
//		   mav_aves - ptr to an array of matrices for storing the MAV mean vectors.
//		   mav_covs - ptr to an array of matrices for storing the MAV covariance matrices
//			zc_aves - ptr to an array of matrices for storing the ZC mean vectors.
//			zc_covs - ptr to an array of matrices for storing the ZC covariance matrices
//			gr_aves - ptr to an array of matrices for storing the GR mean vectors.
//			gr_covs - ptr to an array of matrices for storing the GR covariance matrices
//				zc_thr - ptr to store the threshold value used for the calculation of ZCs
//		  feat_wgts - ptr to store the weights for the different features
//		   wgts_opt - option for the weights. 0 - equal weights
//		  directory - user name. This will be used when saving the parameters to files,
//					  which is done by calling another function inside this one.
// Output: the output value of save_to_files_MM, which can be -1, 0 or >0 (see that funct.)
int get_training_parameters_MM(mat *data_all, ivec *segD, mat *Sigs, int S,
							   mat *mav_aves, mat *mav_covs, mat *zc_aves, mat *zc_covs,
							   mat *gr_aves, mat *gr_covs, double *zc_thr,
							   vec *feat_wgts, int wgts_opt, char directory[])
{
	int j, k, L, N, cov_errors = 0;
	L = data_all[0].cols();		// number of samples per signal
	N = data_all[0].rows();		// number of training signals per gesture
	*Sigs = zeros(S,L);			// initialize
	mat mav_all, zc_all, gr_all; // not used, but needed as parameters for some functions

	// calculate the signatures. Approach: average of the training signals
	for(k = 0; k < S; k++)
	{
		for(j = 0; j < L; j++)
		{
			(*Sigs)(k,j) = mean(data_all[k](0,N-1,j,j));
		}
		// normalize signature
		(*Sigs).set_row(k,(1/max(abs((*Sigs).get_row(k))))*(*Sigs).get_row(k));
	}

    cout << "Complete Getting signatures!" << endl;
	*zc_thr = 0.0;		// initialize the threshold for the ZC calculations

	// get MAV parameters
	for(k = 0; k < S; k++)
	{
		if(get_MAV_features_MM(&data_all[k], segD, &mav_aves[k], &mav_covs[k], &mav_all) == 1)
		{
			*zc_thr += mean(mav_aves[k])/S;
		}
		else
		{
			printf("\nProblem with the data, MAV covariance matrix is singular\n");
			cov_errors |= MAV_COV_ERR;
		}

	}
    cout << "Complete Getting MAVs" << endl;

	// get the ZC and GR parameters
	for(k = 0; k < S; k++)
	{
		if(get_ZC_features_MM(&data_all[k], segD, *zc_thr, &zc_aves[k], &zc_covs[k], &zc_all) == 0)
		{
			printf("\nProblem with the data, ZC covariance matrix is singular\n");
			cov_errors |= ZC_COV_ERR;
		}

		if(get_GR_features_MM(&data_all[k], Sigs, &gr_aves[k], &gr_covs[k], &gr_all) == 0)
		{
			printf("\nProblem with the data, GR covariance matrix is singular\n");
			cov_errors |= GR_COV_ERR;
		}
	}

	*feat_wgts = ones(3);	// default: equal weights.
	if(wgts_opt == 1)
	{
		// if there is an alternative way of calculating the weights, put it here...
	}

	// save all the training parameters to files, and return (1 if success)
	return(save_to_files_MM(directory, S, (*segD).size()-1, L, N, *zc_thr, Sigs, mav_aves,
		mav_covs, zc_aves, zc_covs, gr_aves, gr_covs, feat_wgts, data_all, cov_errors));
}

// Takes a matrix with all training signals of a particular gesture and calculates the
// mean vector and covariance matrix of the Mean Absolute Value (MAV) feature vectors.
// Input: data - ptr to the matrix with the training signals as row vectors
//		  segD - ptr to a vector with the D + 1 positions where the signals are to be
//				 segmented from, including 0 and L = used_datapts:
//				 0 = segD(0) < segD(1) < ... < segD(D-2) < segD(D-1) = L
// 	   mav_ave - ptr to a matrix where the mean vector of the MAVs is to be stored
//				 (D-dimensional vector)
//	   mav_cov - ptr to a matrix where the cov matrix of the MAVs is to be stored (D-by-D).
//	   mav_all - ptr to an N-by-D matrix to store all the MAV values
// Output: an integer indicating if the covariance matrix is invertible (1) or not (0)
int get_MAV_features_MM(mat *data, ivec *segD, mat *mav_ave, mat *mav_cov, mat *mav_all)
{
	int n, d;
	int N = (*data).rows();		// total number of signals in data
	int D = (*segD).size() - 1;	// number of segments, i.e. dimensionality of data

	*mav_all = zeros(N,D);		// initialize matrix

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{		// get the mean absolute value of the dth segment of the nth signal
			(*mav_all)(n,d) = mean(abs((*data)(n,n,(*segD)(d),(*segD)(d+1)-1)));
		}
	}

	// get statistics and return whether the covariance matrix is invertible or not
	return(get_Mu_and_Sigma(mav_all, mav_ave, mav_cov, BIAS_OPT));
}


// Takes a matrix with all training signals of a particular gesture and calculates the
// mean vector and covariance matrix of the MZero Crossing (ZC) feature vectors.
// Input: data - ptr to a matrix with the training signals as row vectors
//		  segD - ptr to a vector with the D + 1 positions where the signals are to be
//				 segmented from, including 0 and L = used_datapts:
//				 0 = segD(0) < segD(1) < ... < segD(D-2) < segD(D-1) = L
//		zc_ave - ptr to a matrix where the mean vector of the ZCs is to be stored
//				 (D-dimensional vector)
//		zc_cov - ptr to amatrix where the cov matrix of the ZCs is to be stored (D-by-D).
//		zc_all - ptr to an N-by-D matrix to store all the ZC values
// Output: an integer indicating if the covariance matrix is invertible (1) or not (0)
int get_ZC_features_MM(mat *data, ivec *segD, double zc_thr, mat *zc_ave, mat *zc_cov, mat *zc_all)
{
	int n, d;
	int N = (*data).rows();		// total number of signals in data
	int D = (*segD).size() - 1;	// number of segments, i.e. dimensionality of data
	mat y;

	*zc_all = zeros(N,D);		// initialize matrix

	for(n = 0; n < N; n++)
	{
		for(d = 0; d < D; d++)
		{
			y =(*data)(n,n,(*segD)(d),(*segD)(d+1)-1);		// get the signal
			(*zc_all)(n,d) = 1.0*Zero_crossings_MM(y, zc_thr);	// calculate the # of ZCs
		}
	}

	// get statistics and return whether the covariance matrix is invertible or not
	return(get_Mu_and_Sigma(zc_all, zc_ave, zc_cov, BIAS_OPT));
}


// Takes a matrix with all training signals of a particular gesture and calculates the
// mean vector and covariance matrix of the GUSSS Ratio (RG) feature vectors.
// Input: data - ptr to the matrix with the training signals as row vectors
//		  Sigs - ptr to the matrix with the signatures of all gestures (classes)
//		gr_ave - ptr to a matrix where the mean vector of the GRs is to be stored
//				 (D-dimensional vector)
//		gr_cov - ptr to a matrix where the cov matrix of the GRs is to be stored (D-by-D).
//		gr_all - ptr to an N-by-D matrix to store all the GR values
// Output: an integer indicating if the covariance matrix is invertible (1) or not (0)
int get_GR_features_MM(mat *data, mat *Sigs, mat *gr_ave, mat *gr_cov, mat *gr_all)
{
	int i, n;
	int N = (*data).rows();		// total number of signals in data
	int S = (*Sigs).rows();		// total number of signatures
	//double gr_max, gr_min;		// for the change of scale
	vec temp;
    double tmp_gr;
	*gr_all = zeros(N,S);		// initialize matrix

	// Analysis and ratios for all signatures, for all training signals
	for(n = 0; n < N; n++)
	{
		for(i = 0; i < S; i++)
		{	// get the GUSSS ratio of the nth signal (Y), for al S signatures (Sp)	
            tmp_gr = GUSSS_ratio_EMG_MM((*data).get_rows(n,n),(*Sigs).get_rows(i,i));
//            tmp_gr = GUSSS_ratio_EMG_v2((*data).get_rows(n,n),(*Sigs).get_rows(i,i));
            if( isnan(tmp_gr) )
            {
                (*gr_all)(n,i) = 0.01;
                cout <<"not a number detected! n = "<< n <<" i = " << i << endl; 
            }
            else
                (*gr_all)(n,i) = tmp_gr;

//            (*gr_all)(n,i)=GUSSS_ratio_EMG_MM((*data).get_rows(n,n),(*Sigs).get_rows(i,i));
		}
		temp = (*gr_all).get_row(n);
		//gr_max = max(temp);
		//gr_min = min(temp);
		//(*gr_all).set_row(n,(temp - gr_min*ones(S))/(gr_max - gr_min));
		(*gr_all).set_row(n,temp/max(temp));	// normalize

	}

    cout << *gr_all << endl;
//    //print the GR for check
//    for(n = 0; n < N; n++)
//    {
//        for(i = 0; i < S; i++)
//            printf("%1.7e ", gr_all(n,i));
//        printf("\n");
//    } 
	// get statistics and return whether the covariance matrix is invertible or not
	return(get_Mu_and_Sigma(gr_all, gr_ave, gr_cov, BIAS_OPT));
}

// The next function does all the saving that are done in the last part of
// get_signatures_v4. It is called after getting all the training parameters in
// get_training_parameters_MM (ABOVE).
// Inputs: see the description in get_training_parameters_MM()
//	 cov_errors - indicates if the covariance matrices were not calculated properly
//				  (i.e. singular matrices). Don't save them if that was the case.
//				  Least significant bit is for MAV, next one for ZC, and next one for GR.
// Output: -1 if there was an error opening a file
//			0 if everything was saved successfully
//		   >0 if at least one of the covariance matrices was not saved (because it was
//			  a singular matrix)
int save_to_files_MM(char directory[], int S, int D, int L, int N, double zc_thr,
					 mat *Sigs, mat *mav_aves, mat *mav_covs, mat *zc_aves, mat *zc_covs,
					 mat *gr_aves, mat *gr_covs, vec *feat_wgts, mat *data_all,
					 int cov_errors)
{
	int i, j, k;
	char fullname[FNBUFFER], filename[50];
	FILE *fnr = NULL, *fsign = NULL, *ffeat_wgts = NULL, *fdata_all = NULL;
	FILE *fmav_aves = NULL, *fmav_covs = NULL, *fzc_aves = NULL, *fzc_covs = NULL;
	FILE *fgr_aves = NULL, *fgr_covs = NULL;

	char *gesture_names[5] = {(char*)"Stop", (char*)"Left", (char*)"Right",
							  (char*)"Forward", (char*)"Backward"};

	// Creates the directory if it doesn't exist already (if if does, a message saying
	// "mkdir: cannot create directory `../../Training/directory_name': File exists"
	// will be displayed.
	sprintf(fullname, "mkdir -m o-w /home/pi/EMG_MICO/Training/%s\n", directory);
	k = system(fullname);	// creates directory if it does not exist

	// First, remove old training files (if they exist)
	for(i = 0; i < S; i++)	// gestures' loop (0)
	{
		strcpy(filename, gesture_names[i]);
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname, "rm /home/pi/EMG_MICO/Training/%s/*\n", directory);
		k = system(fullname);
	}

	// Write the number of training signals to a file -----------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_training.txt", directory);
	fnr = fopen(fullname, "w");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_training file for writing" << endl;
		return -1;
	}
	fprintf(fnr, "%d", N);
	fclose(fnr);

	// Write the number of segments to a file -------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_segments.txt", directory);
	fnr = fopen(fullname, "w");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_segments file for writing" << endl;
		return -1;
	}
	fprintf(fnr, "%d", D);
	fclose(fnr);

	// Write the number of data-points to a file. Needed for "use" and "test" modes -----
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_signalpoints.txt", directory);
	fnr = fopen(fullname, "w");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_signalpoints file for writing" << endl;
		return -1;
	}
	fprintf(fnr, "%d", L);
	fclose(fnr);

	// Write the number of signatures to a file. Needed for "use" and "test" modes ------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_signatures.txt", directory);
	fnr = fopen(fullname, "w");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_signatures file for writing" << endl;
		return -1;
	}
	fprintf(fnr, "%d", S);
	fclose(fnr);

	// Write the ZC threshold to a file. Needed for "use" and "test" modes --------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/zc_threshold.txt", directory);
	fnr = fopen(fullname, "w");
	if(fnr == 0)
	{
		cerr << "Error: Could not open zc_threshold file for writing" << endl;
		return -1;
	}
	fprintf(fnr, "%+1.7e", zc_thr);
	fclose(fnr);

	// Open file to store signatures ----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/signatures1.txt", directory);
	fsign = fopen(fullname, "w");
	if(fsign == 0)
	{
		cerr << "Error: Could not open signatures1 file for writing" << endl;
		return -1;
	}
	// Open file to store mav_aves-------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_aves.txt", directory);
	fmav_aves = fopen(fullname, "w");
	if(fmav_aves == 0)
	{
		cerr << "Error: Could not open mav_aves file for writing" << endl;
		return -1;
	}
	// Open file to store mavs_covs -----------------------------------------------------
	if((cov_errors & MAV_COV_ERR) == 0)	// if no error when getting MAV cov. matrix
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_covs.txt", directory);
		fmav_covs = fopen(fullname, "w");
		if(fmav_covs == 0)
		{
			cerr << "Error: Could not open mav_covs file for writing" << endl;
			return -1;
		}
	}
	// Open file to store zc_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_aves.txt", directory);
	fzc_aves = fopen(fullname, "w");
	if(fzc_aves == 0)
	{
		cerr << "Error: Could not open zc_aves file for writing" << endl;
		return -1;
	}
	// Open file to store zc_covs -------------------------------------------------------
	if((cov_errors & ZC_COV_ERR) == 0)	// if no error when getting ZC cov. matrix
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_covs.txt", directory);
		fzc_covs = fopen(fullname, "w");
		if(fzc_covs == 0)
		{
			cerr << "Error: Could not open zc_covs file for writing" << endl;
			return -1;
		}
	}
	// Open file to store gr_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/gr_aves.txt", directory);
	fgr_aves = fopen(fullname, "w");
	if(fgr_aves == 0)
	{
		cerr << "Error: Could not open gr_aves file for writing" << endl;
		return -1;
	}
	// Open file to store gr_covs -------------------------------------------------------
	if((cov_errors & GR_COV_ERR) == 0)	// if no error when getting GR cov. matrix
	{
		memset(fullname, 0, FNBUFFER);
		sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/gr_covs.txt", directory);
		fgr_covs = fopen(fullname, "w");
		if(fgr_covs == 0)
		{
			cerr << "Error: Could not open gr_covs file for writing" << endl;
			return -1;
		}
	}
	// Open file to store feat_wgts -----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/feat_weights.txt", directory);
	ffeat_wgts = fopen(fullname, "w");
	if(ffeat_wgts == 0)
	{
		cerr << "Error: Could not open feat_weights file for writing" << endl;
		return -1;
	}

	// Write to the files ---------------------------------------------------------------
	// Signatures, aves and covs
	for(i = 0; i < S; i++)	// gestures' loop (0)
	{
		for(j = 0; j < L; j++)	// samples' loop
		{
			fprintf(fsign, "%+1.7e  ", (*Sigs)(i,j));
		}
		fprintf(fsign, "\n");

		for(j = 0; j < D; j++)	// segments' loop (1)
		{
			fprintf(fmav_aves, "%1.7e  \t", mav_aves[i](0,j));	// MAV mean vector
			fprintf(fzc_aves, "%1.7e  \t", zc_aves[i](0,j));	// ZC mean vector

			for(k = 0; k < D; k++)	// segments' loop (2, for covariance matrices)
			{
				if((cov_errors & MAV_COV_ERR) == 0)
					fprintf(fmav_covs, "%+1.7e  \t", mav_covs[i](j,k));	// MAV cov matrix
				if((cov_errors & ZC_COV_ERR) == 0)
					fprintf(fzc_covs, "%+1.7e  \t", zc_covs[i](j,k));	// ZC cov matrix
			}
			if((cov_errors & MAV_COV_ERR) == 0) fprintf(fmav_covs, "\n");
			if((cov_errors & ZC_COV_ERR) == 0) fprintf(fzc_covs, "\n");
		}
		fprintf(fmav_aves, "\n");
		fprintf(fzc_aves, "\n");
		if((cov_errors & MAV_COV_ERR) == 0) fprintf(fmav_covs, "\n");
		if((cov_errors & ZC_COV_ERR) == 0) fprintf(fzc_covs, "\n");

		for(j = 0; j < S; j++)	// signatures' loop (1)
		{
			fprintf(fgr_aves, "%1.7e  \t", gr_aves[i](0,j));	// GR mean vector

			if((cov_errors & GR_COV_ERR) == 0)
			{
				for(k = 0; k < S; k++)	// signatures' loop (2, for GR covariance matrix)
				{
					fprintf(fgr_covs, "%+1.7e  \t", gr_covs[i](j,k));	// GR cov matrix
				}
				fprintf(fgr_covs, "\n");
			}
		}
		fprintf(fgr_aves, "\n");
		if((cov_errors & GR_COV_ERR) == 0) fprintf(fgr_covs, "\n");
	}

	// feature weights
	for(i = 0; i < (*feat_wgts).size(); i++)
	{
		fprintf(ffeat_wgts, "%1.2f  ", (*feat_wgts)(i));
	}

	// All training samples
	for(i = 0; i < S; i++)	// gestures' loop
	{
		for(j = 0; j < N; j++)	// signals' loop
		{
			memset(fullname, 0, FNBUFFER);
			sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/%s_%03d.txt",
							   directory, gesture_names[i], j+1);
			fdata_all = fopen(fullname, "w");

			for(k = 0; k < L; k++)	// samples' loop
			{
				fprintf(fdata_all, "%+1.7e\n", data_all[i](j,k));
			}
			fclose(fdata_all);
		}
	}

	// close all the files that remain open ---------------------------------------------
	fclose(fmav_aves);	fclose(fzc_aves);	fclose(fgr_aves);
	fclose(fsign);		fclose(ffeat_wgts);
	if(fmav_covs != 0)	fclose(fmav_covs);
	if(fzc_covs != 0)	fclose(fzc_covs);
	if(fgr_covs != 0)	fclose(fgr_covs);

	// This is just to allow the group to modify the files just created -----------------
	sprintf(fullname, "chmod g+w -R /home/pi/EMG_MICO/Training/%s\n", directory);
	k = system(fullname);
	k++;	// this is just to avoid an annoying warning...

	// 0 if everything went well, > 0 if at least one of the covariance matrices was not
	// saved (because it was singular). If there had been a problem opening the files,
	// we wouldn't get here (a -1 would have been returned).
	return(cov_errors);
}


// ----------------------------- Miscellaneous ------------------------------------------

// get_Mu_and_Sigma: Function to calculate the mean vector and covariance matrix from a
// given dataset. If the full covariance matrix is singular, the function tries to create
// a diagonal covariance matrix, for which any zero diagonal element is replaced by half
// of the smallest non-zero variance, if any.
// Inputs: samples - pointer to the mat containing the samples (assumed to be row vectors)
//				mu - pointer to a matrix to store the mean vector estimate
//			 Sigma - pointer to a matrix to store the covariance matrix.
//			   opt - option for using the bias_factor (1) or not (0)
// Output: Integer indicating if the covariance matrix is invertible (1) or not (0)
int get_Mu_and_Sigma(mat *samples, mat *mu, mat *Sigma, int opt)
{
	int j;
	int N = (*samples).rows(), D = (*samples).cols(); // number of samples and dimension
	double bias_factor;				// compensation for covariance matrix
	double temp_var = 1000000.0;	// initialize with big number
	vec diag_vars(D);

	*mu = zeros(1,D);		// initialize matrices
	*Sigma = zeros(D,D);

	if(opt == 1)
		bias_factor = 1.0*N/(1.0*(N-1));
	else
		bias_factor = 1.0;

	for(j = 0; j < D; j++)
	{
		(*mu)(0,j) = mean((*samples).get_col(j));	// mean vector
	}

	*Sigma = bias_factor*cov(*samples);		// covariance matrix

	if(det(*Sigma) != 0.0)
		return 1;		// if the matrix is a proper (non-singular) covariance matrix
	else	// singular matrix, try to fix it.
	{
		*Sigma = zeros(D,D);	// reset the matrix

		// get the D variances and determine the smallest non-zero element (if it exists)
		for(j = 0; j < D; j++)
		{
			diag_vars(j) = variance((*samples).get_col(j));
			if((diag_vars(j) != 0.0) && (temp_var > diag_vars(j)))
				temp_var = diag_vars(j);
		}

		for(j = 0; j < D; j++)
		{
			if(diag_vars(j) != 0.0)
				(*Sigma)(j,j) = diag_vars(j);
			else	// replace the zero elements with 1/2 of the minimum non-zero element
				(*Sigma)(j,j) = temp_var/2.0;
		}

		if(det(*Sigma) != 0.0)
			return 1;	// if now the matrix is a proper (non-singular) covariance matrix
		else
			return 0;	// definitely not able to get a proper covariance matrix.
	}
}

// Zero Crossing (ZC) counter
// This function counts how many times the sign of a particular signal changes.
// Assuming that the signal is continuous in time, the calculated number is a lower bound
// to the number of zero crossings of the signal. This function sets to zero all signal
// points with absolute value below the specified threshold. Then, it counts how many times
// the compensated signal changes sign. The function is not otpimized in the Matlab sense
// (matrix and vector operations), because it is to be translated into a C function.
// Inputs: y - matrix with the signal to be analyzed (row vector).
//		 zc_thr - threshold, to compensate ZC due to noise
// Outputs: estimated number of zero crossings.
int Zero_crossings_MM(mat y, double zc_thr)
{
	int i, zc = 0, pivot = 0, pivotsign = 0, L = y.cols();
	vec y_fixed(L);

	for(i = 0; i < L; i++)
	{
		if(abs(y(0,i)) < zc_thr)
			y_fixed(i) = 0.0;
		else
			y_fixed(i) = y(0,i);

		// determine where the first nonzero value is
		if(pivotsign == 0)
		{
			if(y_fixed(i) > 0.0)
			{
				pivot = i;
				pivotsign = 1;
			}
			else if(y_fixed(i) < 0.0)
			{
				pivot = i;
				pivotsign = -1;
			}
		}
	}

	// Count how many sign changes there are. A zero is considered the same sign
	// as the previous one.
	for(i = pivot+1; i < L; i++)
	{
		if((y_fixed(i) > 0.0) && (pivotsign == -1))
		{
			zc++;
			pivotsign = 1;
		}
		else if((y_fixed(i) < 0.0) && (pivotsign == 1))
		{
			zc++;
			pivotsign = -1;
		}
	}

	return zc;
}

// Calculates the GUSSS ratio for a given test signal and signature
// Inputs:	Y - Testing signal
//		   Sp - Signature to be injected
// Output: GUSSS ratio
double GUSSS_ratio_EMG_MM(mat Y, mat Sp)
{
	int nrIC, L = Y.cols();
	double ratio, normsp = 0.0, norm0 = 0.0, norm1 = 0.0, error0 = 0.0, error1 = 0.0;
	mat Mix = zeros(2,L);			// Mix matrix
	mat A;							// for the mixing matrix
	mat icasig;						// for the independent components
	mat A_init = ones(3,3); // for some reason, initializing as 2x2 matrix doesn't work

	// Initial guess for the mixing matrix
//	A_init(0,0) = 1.0;	A_init(0,1) = 0.0;
//	A_init(1,0) = 0.0;	A_init(1,1) = 1.0;

	A_init(0,0) = 0.0;	A_init(0,1) = 1.0;
	A_init(1,0) = 1.0;	A_init(1,1) = 0.0;

	// constructs Mix matrices to be ICA-analyzed [Y; Sp + Y]
	Mix.set_rows(0,Y);
	Mix.set_rows(1,Sp+Y);

//--------------------------- ICA analysis and GUSSS ratios -----------------------------
	Fast_ICA Test(Mix);		// ICA object

	// Analysis and ratios for all signatures
	Test.set_non_linearity(FICA_NONLIN_GAUSS);	// Set GAUSS non-linearity
	Test.set_approach(FICA_APPROACH_DEFL);	// Use deflation approach: IC computed 1 by 1
	Test.set_max_num_iterations(MAX_ITER);	// 100
//	Test.set_max_num_iterations(200);	// 100
	Test.set_epsilon(EPSILON);				// 0.01
//	Test.set_epsilon(0.0001);				// 0.01
	Test.set_init_guess(A_init);			// there is a bug with this function...

	Test.separate();	// Perform ICA

	A = Test.get_mixing_matrix();		// Mixing matrix
	icasig = Test.get_independent_components();
	nrIC = Test.get_nrof_independent_components();

	if(nrIC == 0)
	{
//		ratio = 100.0;
		ratio = 0.25 * randu();
	}
	else if(nrIC == 1)
	{
//		ratio = 0.0;
		ratio = 0.25 * randu();
	}
	else
	{	// get normalization values
		normsp = max(abs(Sp.get_row(0)));
		norm0 = max(abs(icasig.get_row(0)));
		norm1 = max(abs(icasig.get_row(1)));

		Sp = (1/normsp)*Sp;
		icasig.set_row(0,(1/norm0)*icasig.get_row(0));
		icasig.set_row(1,(1/norm1)*icasig.get_row(1));

		// To determine which column corresponds to the coefficient cp (by looking at the
		// recovered, separated signals. One has to be equal (or very close) to the
		// injected signature. We calculate how different is icasig0 and icasig1 wrt the
		// signature Sp. We can then infer if cp is in A(0,0) or A(0,1).
		error0 = sum(abs(abs(Sp.get_row(0)) - abs(icasig.get_row(0))));
		error1 = sum(abs(abs(Sp.get_row(0)) - abs(icasig.get_row(1))));

        if(A(0,0) == 0 || A(0,1) == 0)
        {
            ratio = 25 * randu();
            cout << "Divided by zero occurs!" << endl;
        }
        else if(error0 <= error1)
			ratio = abs(A(0,1)/A(0,0));   // Sp matches icasig0, cp is in the 1st column
		else
			ratio = abs(A(0,0)/A(0,1));   // Sp matches icasig1, cp is in the 2nd column
	}

	return(ratio);
}


// This function reads the extracted signal and classifies it according to the distance
// criteria. It also calculates the confidence that the signal belongs to each class.
// Inputs:	Y - matrix with the testing signal as a row vector
// 		 Sigs - ptr to a matrix with the signatures
//	 mav_aves - array of matrices with the MAV mean vectors
//	 mav_covs - array of matrices with the MAV covariance matrices
//	  zc_aves - array of matrices with the ZC mean vectors
//	  zc_covs - array of matrices with the ZC covariance matrices
//	  gr_aves - array of matrices with the GR mean vectors
//	  gr_covs - array of matrices with the GR covariance matrices
//		 segD - ptr to a vector with the D + 1 positions where the signals are to
//				be segmented from, including 0 and L = used_datapts:
//				0 = segD(0) < segD(1) < ... < segD(D-2) < segD(D-1) = L
//	feat_wgts - ptr to a vector with the weights for the features
//	   zc_thr - threshold for the ZC feature
//		 conf - ptr to a matrix to store the confidence values
//	   cl_opt - Classification option (0 for distances, 1 for confidences)
// Output: the assigned class, based on the smallest distance.
int EMG_classify_MM(mat Y, mat *Sigs, mat *mav_aves, mat *mav_covs,
					mat *zc_aves, mat *zc_covs, mat *gr_aves, mat *gr_covs,
					ivec *segD, vec *feat_wgts, double zc_thr, mat *conf, int cl_opt)
{
	int i, d;
	int D = (*segD).size() - 1, S = (*Sigs).rows();
	mat mav(1,D), zc(1,D), gr(1,S), temp_dist;
	vec distMAV(S), distZC(S), distGR(S), class_dist_vec, erfc_vec(3);
	ivec sort_ind;			// vector of integers to store the sorted indexes
	Sort <double> mysort;	// Sorting object
	//changedMovements
	//int Movements[5] = {STOP_COM, LEFT_COM, RIGHT_COM, FORWARD_COM, BACKWARD_COM};
	int Movements[5] = {STOP_COM, LEFT_COM, RIGHT_COM};
	//double gr_max, gr_min;		// for the change of scale
	vec temp;

	// --- GUSSS ratios ---
	for(i = 0; i < S; i++)
	{
		gr(0,i) = GUSSS_ratio_EMG_MM(Y,(*Sigs).get_rows(i,i));
	}
	temp = gr.get_row(0);
	//gr_max = max(temp);
	//gr_min = min(temp);
	//gr.set_row(0,(temp - gr_min*ones(S))/(gr_max - gr_min));
	gr.set_row(0,temp/max(temp));	// normalize

	// --- MAVs and ZCs ---
	for(d = 0; d < D; d++)
	{
		mav(0,d) = mean(abs(Y(0,0,(*segD)(d),(*segD)(d+1)-1)));
		zc(0,d) = 1.0*Zero_crossings_MM(Y(0,0,(*segD)(d),(*segD)(d+1)-1), zc_thr);
	}

	// --- Mahalanobis distances ---
	for(i = 0; i < S; i++)
	{
		temp_dist = (gr-gr_aves[i])*inv(gr_covs[i])*(gr.transpose()-gr_aves[i].transpose());
		distGR(i) = sqrt((long double)temp_dist(0,0));
		temp_dist = (mav-mav_aves[i])*inv(mav_covs[i])*(mav.transpose()-mav_aves[i].transpose());
		distMAV(i) = sqrt((long double)temp_dist(0,0));
		temp_dist = (zc-zc_aves[i])*inv(zc_covs[i])*(zc.transpose()-zc_aves[i].transpose());
		distZC(i) = sqrt((long double)temp_dist(0,0));
	}

	// --- Confidences ---
	*conf = zeros(S,1);
	for(i = 0; i < S; i++)
	{
		erfc_vec(0) = distGR(i); erfc_vec(1) = distMAV(i); erfc_vec(2) = distZC(i);
		(*conf)(i,0) = mean(erfc(erfc_vec/pow2(1.0*D/2.0)));
	}

	// --- classification ---
	if(cl_opt == 0)
	{
		class_dist_vec=(*feat_wgts)(0)*distGR+(*feat_wgts)(1)*distMAV+(*feat_wgts)(2)*distZC;
		// sort the distances and get the indexes (ascending order)
		sort_ind = mysort.sort_index(0, S-1, class_dist_vec);
		return(Movements[sort_ind(0)]);
	}
	else
	{	// sort the confidences and get the indexes (ascending order)
		sort_ind = mysort.sort_index(0, S-1, (*conf).get_col(0));
		return(Movements[sort_ind(sort_ind.length()-1)]);
	}
}



// The next function retrieves all the training parameters from saved files, which were
// written after a training session in the function save_to_files_MM (ABOVE).
// Inputs: see the description in get_training_parameters_MM().
// Output: 1 if everything was saved successfully
// Output: 1 if everything was retrieved properly
//		   0 if there was a problem opening any of the files that are expected
int get_parameters_from_files_MM(char directory[], int *S, int *D, int *L, double *zc_thr,
							  mat *Sigs, mat *mav_aves, mat *mav_covs, mat *zc_aves,
							  mat *zc_covs, mat *gr_aves, mat *gr_covs, vec *feat_wgts)
{
	char fullname[FNBUFFER];
	FILE *fnr = NULL, *fsign = NULL;
	FILE *fmav_aves = NULL, *fmav_covs = NULL, *fzc_aves = NULL, *fzc_covs = NULL;
	FILE *fgr_aves = NULL, *fgr_covs = NULL, *ffeat_wgts = NULL;
	float temp = 0.0;
	int i, j, k, dummy;

	// get the number of segments D -----------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_segments.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_segments file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	else
	{
		dummy = fscanf(fnr, "%d", D);
		fclose(fnr);
	}
	// get the number of data points used for each signal -------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_signalpoints.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_signalpoints file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	else
	{
		dummy = fscanf(fnr, "%d", L);
		fclose(fnr);
	}
	// get the number of different signatures to be used/compared -----------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/nr_signatures.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open nr_signatures file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	else
	{
		dummy = fscanf(fnr, "%d", S);
		fclose(fnr);
	}
	// get the ZC threshold -------------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname, "/home/pi/EMG_MICO/Training/%s/zc_threshold.txt", directory);
	fnr = fopen(fullname, "r");
	if(fnr == 0)
	{
		cerr << "Error: Could not open zc_threshold file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	else
	{
		dummy = fscanf(fnr, "%f", &temp);
		*zc_thr = (double)temp;
		fclose(fnr);
	}

	// opens file with signatures -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/signatures1.txt", directory);
	fsign = fopen(fullname, "r");
	if(fsign == 0)
	{
		cerr << "Error: Could not open signature signal file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read mav_aves ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_aves.txt", directory);
	fmav_aves = fopen(fullname, "r");
	if(fmav_aves == 0)
	{
		cerr << "Error: Could not open signature mav_aves' file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read mav_covs ------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/mav_covs.txt", directory);
	fmav_covs = fopen(fullname, "r");
	if(fmav_covs == 0)
	{
		cerr << "Error: Could not open mav_covs' file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read zc_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_aves.txt", directory);
	fzc_aves = fopen(fullname, "r");
	if(fzc_aves == 0)
	{
		cerr << "Error: Could not open signature zc_ave file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read zc_covs -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/zc_covs.txt", directory);
	fzc_covs = fopen(fullname, "r");
	if(fzc_covs == 0)
	{
		cerr << "Error: Could not open zc_covs file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read gr_aves -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/gr_aves.txt", directory);
	fgr_aves = fopen(fullname, "r");
	if(fgr_aves == 0)
	{
		cerr << "Error: Could not open signature gr_ave file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read gr_covs -------------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/gr_covs.txt", directory);
	fgr_covs = fopen(fullname, "r");
	if(fgr_covs == 0)
	{
		cerr << "Error: Could not open gr_covs file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}
	// opens file to read feat_weights --------------------------------------------------
	memset(fullname, 0, FNBUFFER);
	sprintf(fullname,"/home/pi/EMG_MICO/Training/%s/feat_weights.txt", directory);
	ffeat_wgts = fopen(fullname, "r");
	if(ffeat_wgts == 0)
	{
		cerr << "Error: Could not open feat_weights file for reading" << endl;
		return 0; // indicates a problem with the file, so we need to train again.
	}

	*Sigs = zeros(*S,*L);	// initialize Signatures' matrix
	*feat_wgts = zeros(3);	// initialize feature weights vector

	// get feature weights
	for(i = 0; i < (*feat_wgts).size(); i++)
	{
		dummy = fscanf(ffeat_wgts, "%f", &temp);
		(*feat_wgts)(i) = (double)temp;
	}

	// get Signatures, aves and covs ----------------------------------------------------
	for(i = 0; i < *S; i++)	// gestures' loop (0)
	{
		// initialize remaining matrices
		mav_aves[i] = zeros(1,*D);
		mav_covs[i] = zeros(*D,*D);
		zc_aves[i] = zeros(1,*D);
		zc_covs[i] = zeros(*D,*D);
		gr_aves[i] = zeros(1,*S);
		gr_covs[i] = zeros(*S,*S);

		for(j = 0; j < *L; j++)	// fills matrix Sigs (signatures)
		{
			dummy = fscanf(fsign, "%f", &temp);
			(*Sigs)(i,j) = (double)temp;
		}

		for(j = 0; j < *D; j++)	// segments' loop (1)
		{
			// fills MAV and ZC aves matrices
			dummy = fscanf(fmav_aves, "%f", &temp);
			mav_aves[i](0,j) = (double)temp;
			dummy = fscanf(fzc_aves, "%f", &temp);
			zc_aves[i](0,j) = (double)temp;

			for(k = 0; k < *D; k++)	// fills MAV and ZC covariance matrices
			{
				dummy = fscanf(fmav_covs, "%f", &temp);
				mav_covs[i](j,k) = (double)temp;
				dummy = fscanf(fzc_covs, "%f", &temp);
				zc_covs[i](j,k) = (double)temp;
			}
		}

		for(j = 0; j < *S; j++)	// signatures' loop (1)
		{
			// fills GR aves matrices
			dummy = fscanf(fgr_aves, "%f", &temp);
			gr_aves[i](0,j) = (double)temp;

			for(k = 0; k < *S; k++)	// fills GR covariance matrix
			{
				dummy = fscanf(fgr_covs, "%f", &temp);
				gr_covs[i](j,k) = (double)temp;
			}
		}
	}

	// close all the files that remain open ---------------------------------------------
	fclose(fmav_aves);	fclose(fmav_covs);	fclose(fzc_aves);	fclose(fzc_covs);
	fclose(fgr_aves);	fclose(fgr_covs);	fclose(fsign);		fclose(ffeat_wgts);

	return(1 + dummy - dummy);	// just to get rid of the annoying warning...
}

// ------------------------------- Device Driver functions -----------------------------------------

//global variable for the ADC_BT
int read_flag = 1;
int fd_bluetooth;	 

// Function that starts the connection between two devices via bluetooth

void startADC()
{
	if((fd_bluetooth = serialOpen ("/dev/ttyAMA0",115200)) < 0)
	{
		printf("Unable to open serial device!\n");
		fflush(stdout);
		exit(1);
	}
//	else
//	{
//		printf("Open Bluetooth Serial device!\n");
//		fflush(stdout);
//	}
	
	serialFlush(fd_bluetooth);	// clear the serial port data	
	
}


void ADC_Init_P2()
{
	// setup GPIO
	if (wiringPiSetup() == -1)
	{
		printf("wiringPi Error!\n");
		exit(1);
	}
	
	// digital pins initialization
	pinMode(8,  INPUT);
	pullUpDnControl(8, PUD_UP);
	
	pinMode(9,  INPUT);
	pullUpDnControl(9, PUD_UP);

	pinMode(5,  INPUT);					// reads the READY from AVR
	pullUpDnControl(5, PUD_UP);

	pinMode(12, OUTPUT);				// sends the REQ to AVR	
	
	digitalWrite(12, 0);				// clear the REQ
	sleep(1);	

}

/*
// Function that reads a value from the ADC module. Bluetooth version
uint16_t readADC()
{
	int S = 1;			// signal to start the bluetooth transmission

	uint8_t buf[2];
	uint16_t data;	
	
	if(read_flag == 1)
	{
		serialPutchar(fd_bluetooth,S);		// send the "start" signal
		read_flag = 0;					// clear the read_flag so that it only sends the "start" once	
	}
	
	
	while( serialDataAvail(fd_bluetooth) < 1 );	// wait for the first byte
	buf[0] = serialGetchar(fd_bluetooth);
	while( serialDataAvail(fd_bluetooth) < 1 ); // wait for the second byte
	buf[1] = serialGetchar(fd_bluetooth);
	
	
	data = 	(uint16_t)buf[0];
	data |=	(uint16_t)buf[1] << 8;			
	
	return data;	
}
*/

// 4 channels AVR
int readADC()
{
	int data = 0;
		
	digitalWrite(12, 1);							// send the REQUEST to AVR
	while( digitalRead(5) == 0);					// wait READY to be 1
	data = data | digitalRead(8);					// read data
	data = data | digitalRead(9) << 1;
	digitalWrite(12, 0);							// clear the REQUEST
	while( digitalRead(5) == 1);					// wait READY to be 0

	digitalWrite(12, 1);
	while( digitalRead(5) == 0);
	data = data | digitalRead(8) << 2;
	data = data | digitalRead(9) << 3;
	digitalWrite(12, 0);
	while( digitalRead(5) == 1);

	digitalWrite(12, 1);
	while( digitalRead(5) == 0);
	data = data | digitalRead(8) << 4;
	data = data | digitalRead(9) << 5;
	digitalWrite(12, 0);
	while( digitalRead(5) == 1);

	digitalWrite(12, 1);
	while( digitalRead(5) == 0);
	data = data | digitalRead(8) << 6;
	data = data | digitalRead(9) << 7;
	digitalWrite(12, 0);
	while( digitalRead(5) == 1);

	digitalWrite(12, 1);
	while( digitalRead(5) == 0);
	data = data | digitalRead(8) << 8;
	data = data | digitalRead(9) << 9;
	digitalWrite(12, 0);
	while( digitalRead(5) == 1);
	
	return data;
} 

// function for ADS1256
float readADS1256()
{
	float data;
	uint8_t 	buffer[3];
	uint32_t 	read = 0;
	uint8_t 	del = 10;	
	
	waitDRDY();
	buffer[0] = recieve8bit();
	buffer[1] = recieve8bit();
	buffer[2] = recieve8bit();

	// construct 24 bit value
	read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buffer[1] << 8);
	read |= buffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;
	}

	data = (float)read/1670000;
	delayus(del);
	
	if( data <= 5 )     // When an error occurs (not very frequent) the ADC would return
		return data;    // a value greater than 5, in that case, we simple throw away that 
	else                // value and send a 5 (maximum number) as a indicator. 
		return 0;
}



/*
// Function that reads a value from the ADC module. Original version
int readADC_sample()
{
	int data_in = 0, j;
	while(digitalRead(10) == 1)	  // wait STB == 0
	{
		prevent_opt();
	}
	data_in = digitalReadLR(10);

	digitalWrite(11, 0);

	for(j = 0; j < ADC_WAIT_CONST; j++);
	digitalWrite(11, 1);
	for(j = 0; j < ADC_WAIT_CONST; j++);

	return data_in;
}


// Function that reads a value from the ADC module. Improved version
int readADC_sample_v2()
{
	int data_in = 0, j;
	digitalWrite(11, 0);	//activate REQ
	while(digitalRead(10) == 1);  // wait READY == 0
	data_in = digitalReadLR(10);

	digitalWrite(11, 1);	//remove REQ

	while(digitalRead(10)==0);  // wait READY == 1

	return data_in;
}

*/

