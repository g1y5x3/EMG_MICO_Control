/*
 * EMG_Voice.h
 *
 *  Created on: Feb 11, 2014 (Originally: Apr 15, 2013)
 *      Author: lulorivera
 *
 *  Copyright (C) 2013 ViGIR - Vision Guided and Intelligent Robotics Lab
 *                             (http://vigir.missouri.edu)
 *  Written by Luis Alberto Rivera <larmbc@mail.missouri.edu>
 *  Modified by G. N. DeSouza <DeSouzaG@missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 *
 * Header file for the unified EMG system program. It has the includes, definitions and
 * function prototypes that are required for EMG_unified.cpp and EMG_unified_functions.cpp.
 * These were taken from EMG_client_demo_small_changes_v2.cpp and
 * EMG_server_demo_small_changes_v3.cpp.
 */

// for the extraction and classification
#include <math.h>
#include <itpp/itsignal.h>
#include <itpp/stat/misc_stat.h>
#include <sys/time.h>


#ifndef EMG_REHAB_H_
#define EMG_REHAB_H_


// ---------- ----------
#define FNBUFFER		    300
#define MAV_COV_ERR			0x01			// signal of error when calculating MAV cov. matrix
#define ZC_COV_ERR			0x02			// signal of error when calculating ZC cov. matrix
#define GR_COV_ERR			0x04			// signal of error when calculating GR cov. matrix
#define BIAS_OPT			1				// bias term for covariance matrices (1/yes, 0/no)
#define USED_DATAPTS		1500			// Signals will be down-sampled to have this # of points

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Numbers associated to the classes. They have to be > 0
#define STOP_COM			1
#define LEFT_COM			2
#define RIGHT_COM			3
#define FORWARD_COM			4
#define BACKWARD_COM		5

#define nrS 				3				// number of signatures (keep for compatibility with prev. ver.)
#define MAX_ITER		    50
#define EPSILON				0.01			// for ICA
#define WA					0.2		// zero-weight threshold (for the features' weights)
#define WB					0.8		// one-weight threshold (for the features' weights)
#define round_i(x)		(int)(x + 0.5)

//--------- Not sure about the functionality ----------
// constants in terms of time
#define MOV_AVE_T			0.125			// Duration of the moving average window, also used
											// for the DC offset of every signal (in seconds)
#define THR_WIN_T			3.0				// Threshold window, in seconds
#define SIGNAL_T			0.4				// Signal window, in seconds
#define WC_PORT			 	3000			// Port for communication with the wheelchair server
#define TIME_TO_STOP		2500			// Time to wait after the 1st stop command before act
#define SEG					3				// default number of segments to divide the signal into
//#define DT_MIN				0.5		// Delta times, in seconds
#define DT_MIN				0.1				// Delta times, in seconds
#define DT_MAX				1.5				// 2.0   in seconds
#define DT_TRAIN			1.0				// 2.0   in seconds
#define DT_CLAS				0.75
#define CLASSIF_THR			0.01			// minimum difference that the classification distances
											// should have to consider a classification "not
											// ambiguous"

// ------ Directory of the workspace
//#define DIRECTORY			"/home/pi/EMG_ADS1256/Training"

using namespace std;
using namespace itpp;

// Functions for reading from ADS1256
float readADS1256();

// Functions for 4 channel AVR
int readADC();
void ADC_Init_P2();

// Functions for reading ADC for bluetooth moduel
void startADC();

// Functions for car control to arduino
void controlCar(int movement);


// ------------------------------- GUSSS functions -----------------------------------------

int get_MAV_features_MM(mat *data, ivec *segD, mat *mav_ave, mat *mav_cov, mat *mav_all);

int get_ZC_features_MM(mat *data, ivec *segD, double zc_thr, mat *zc_ave, mat *zc_cov, mat *zc_all);

int get_GR_features_MM(mat *data, mat *Sigs, mat *gr_ave, mat *gr_cov, mat *gr_all);

int get_training_parameters_MM(mat *data_all, ivec *segD, mat *Sigs, int S,
							   mat *mav_aves, mat *mav_covs, mat *zc_aves, mat *zc_covs,
							   mat *gr_aves, mat *gr_covs, double *zc_thr,
							   vec *feat_wgts, int wgts_opt, char directory[]);

int save_to_files_MM(char directory[], int S, int D, int L, int N, double zc_thr,
					 mat *Sigs, mat *mav_aves, mat *mav_covs, mat *zc_aves, mat *zc_covs,
					 mat *gr_aves, mat *gr_covs, vec *feat_wgts, mat *data_all,
					 int cov_errors);
int get_Mu_and_Sigma(mat *samples, mat *mu, mat *Sigma, int opt);

int Zero_crossings_MM(mat y, double zc_thr);

double GUSSS_ratio_EMG_MM(mat Y, mat Sp);

int EMG_classify_MM(mat Y, mat *Sigs, mat *mav_aves, mat *mav_covs,
					mat *zc_aves, mat *zc_covs, mat *gr_aves, mat *gr_covs,
					ivec *segD, vec *feat_wgts, double zc_thr, mat *conf, int cl_opt);

int get_parameters_from_files_MM(char directory[], int *S, int *D, int *L, double *zc_thr,
							  mat *Sigs, mat *mav_aves, mat *mav_covs, mat *zc_aves,
							  mat *zc_covs, mat *gr_aves, mat *gr_covs, vec *feat_wgts);
//-----------------------------------------------------------------------------------------
int Zero_crossings(float *y, int L, float thr);

void get_MAV_features(float **data, int N, int *segD, int D, float *mav_ave, float *mav_std);

void get_ZC_features(float **data, int N, int *segD, int D, float thr, float *zc_ave,
					 float *zc_std);

float GUSSS_ratio_EMG_v2(mat Y, mat Sp, int L);

int get_signatures_v2(char directory[], int *segD, int D);

int get_signatures_v3(char directory[], int *segD, int D, int nrs);

int get_signatures_v4(char directory[], int *segD, int D, int nrs, int weightOpt);

int EMG_classify_v2(mat Y, int L, mat S, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D);

int EMG_classify_v3(mat Y, int L, mat S, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr);

int EMG_classify_v4(mat Y, int L, mat S, int nrs, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr);

int EMG_classify_v5(mat Y, int L, mat S, int nrs, float **mav_aves, float **mav_stds,
					float **zc_aves, float **zc_stds, int *segD, int D, float cl_thr,
					float *mav_wgts, float *zc_wgts);

void prevent_opt();

void sub_sample(float *yc, int C, double *yn, int N);

void push_buttons_training(unsigned char pb_data);

float Rand_index(int *LC, int *LP, int N);

float get_feature_weight(float *values, int N, float *mus, float *stds, int nrs, int *TL,
						 float a, float b);

#endif /* EMG_REHAB_H_ */
