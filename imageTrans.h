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
#define SG_LOW_CHECK		0	//"G_LC"
#define SG_MID_CHECK		1	//"G_MC"
#define SU_UP1				2	//"U_UP1"
#define SU_HOLD				3	//"U_HOLD"
#define SD_RETRY_UP			4	//"R_UP"
#define SD_RETRY_UP_HOLD	5	//"R_HOLD"
#define SD_FLY_TARGET		6	//"FLY_TARGET"
#define SD_CHECK_TARGET		7	//"CHK_TARGET"
#define TO_START			8	//"TO_START"

#define SD_HOLD				13	//"D_HOLD"
#define SD_MISS_SEARCH		14	//"D_MISS"
#define SD_HOLD2			15	//"D_HOLD2"
#define SD_HIGH_FAST_DOWN	16	//"D_FastD"
#define SD_CIRCLE_SLOW_DOWN	17	//"D_CSD"
#define SD_CIRCLE_HOLD		18	//"D_CHOLD"
#define SD_CIRCLE_MID_DOWN	19	//"D_CMD"
#define SD_CHECK_G			20	//"D_GC"
#define SD_SHUT_DOWN		21	//"D_SHUT"
#define SD_SAFE				22	//"D_SAFE"
#define HOLD_BACK			23	//"HOLD_BACK"
#define BREAK				24	//"BREAK"
#define SHUT				25	//"SHUT"
#define GO_HOME				26	//"GO_HOME"

cv::Rect dlibRect2CVRect(dlib::rectangle r);
dlib::rectangle getInitPosition(cv::Rect r);

void init();
void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle);
std::string get_time();

void drawImage(cv::Mat &image);

void startWriteVideo(std::ifstream &video_num_read,
		cv::VideoWriter &video_writer);

#endif /* IMAGETRANS_H_ */
