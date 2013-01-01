/*
 * imageTrans.h
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#ifndef IMAGETRANS_H_
#define IMAGETRANS_H_

#include "CharacterRecognition.h"

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

//state
#define SG_LOW_CHECK		"G_LC"	//0
#define SG_MID_CHECK		"G_MC"	//1
#define SU_UP1				"U_UP1"	//2
#define SU_HOLD				"U_HOLD"	//3
#define SD_RETRY_UP			"R_UP"	//4
#define SD_RETRY_UP_HOLD	"R_HOLD"	//5
#define SD_CHECK_TARGET		"CHECK_TARGET"	//6
#define SD_FLY_TARGET		"FLY_TARGET"	//7

#define SD_HOLD				"D_HOLD"	//13
#define SD_MISS_SEARCH		"D_MISS"	//14
#define SD_HOLD2			"D_HOLD2"	//15
#define SD_HIGH_FAST_DOWN	"D_FastD"	//16
#define SD_CIRCLE_SLOW_DOWN	"D_CSD"	//17
#define SD_CIRCLE_HOLD		"D_CHOLD"	//18
#define SD_CIRCLE_MID_DOWN	"D_CMD"	//19
#define SD_CHECK_G			"D_GC"	//20
#define SD_SHUT_DOWN		"D_SHUT"	//21
#define SD_SAFE				"D_SAFE"	//22
#define HOLD_BACK			"HOLD_BACK"	//23
#define BREAK				"BREAK"	//24
#define SHUT				"SHUT"	//25

cv::Rect dlibRect2CVRect(dlib::rectangle r);
dlib::rectangle getInitPosition(cv::Rect r);

void init();
void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle);
std::string get_time();

void drawImage(cv::Mat &image);

void startWriteVideo(std::ifstream &video_num_read,
		cv::VideoWriter &video_writer);

#endif /* IMAGETRANS_H_ */
