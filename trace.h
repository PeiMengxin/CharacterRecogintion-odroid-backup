/*
 * trace.h
 *
 *  Created on: May 5, 2016
 *      Author: odroid
 */

#ifndef TRACE_H_
#define TRACE_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <ctype.h>
#include "CharacterRecognition.h"
#include <serial/serial.h>
#include <unistd.h>

extern int circle_check, circles_x, circles_y, circles_r, track_check, state_v;
extern int mouse_x, mouse_y;
extern unsigned char circle_control[4];
extern int target_num, state_num, delay_ms;
extern float Pitch, Yaw, Roll;
extern NumberPosition number_position_send;
extern bool is_print_character, isTerminal;
extern int flow_pix[2], uart_good, fd;
extern int char_num;
extern bool LBtnDown;
extern serial::Serial my_serial;
extern std::ifstream video_num_read;

#endif /* TRACE_H_ */
