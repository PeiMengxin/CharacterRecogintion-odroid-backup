#include <fstream>
#include <sstream>
#include <thread>
#include "trace.h"
#include "imageTrans.h"
#include "my_serial.h"

using namespace std;
using namespace cv;

bool LBtnDown = false;

void onMouse(int event, int x, int y, int, void* param)
{
	if (LBtnDown)
	{
		mouse_x = x;
		mouse_y = y;
		//uart_mouse(x, y);
		//uartSent(UART_SENT_TYPE_MOUSE);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		LBtnDown = true; 
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		LBtnDown = false;
	}
}

int char_num = 5;

int target_num = -1;
int state_num = 0;
bool isTerminal = false;
int delay_ms = 5;
int flag_LX_target = 0;
bool have_target = false;

NumberPosition number_position_send;

int main()
{
	init();
	//my_serial.open();

	std::thread uart_read_thread(uartReadThread);

	string imagename("NumberBlock-2.bmp");

	tesseract::TessBaseAPI tess;

	tess.Init(NULL, "eng");

	tess.SetPageSegMode(tesseract::PageSegMode::PSM_SINGLE_CHAR);

	VideoCapture cap;

	ofstream log_out;
	log_out.open("log.txt");

	string videoName("/home/odroid/workspace/characterRecognition/video/5.avi");

	dlib::correlation_tracker tracker;

	cv::namedWindow("bar");
	int psr_threshold = 20;
	int flag_writevideo = 0;
	int flag_writing = 0;
	int flag_writevideo_src = 0;
	int flag_writing_src = 0;
	int check_count_thres = 5;

	cv::VideoWriter video_writer;
	cv::VideoWriter video_writer_src;

	cv::createTrackbar("character", "bar", &char_num, 9);
	cv::createTrackbar("LX_target", "bar", &flag_LX_target, 1);
	cv::createTrackbar("psr_threshold", "bar", &psr_threshold, 60);
	cv::createTrackbar("writevideo", "bar", &flag_writevideo, 1);
	cv::createTrackbar("writevideo_src", "bar", &flag_writevideo_src, 1);
	cv::createTrackbar("delay_ms", "bar", &delay_ms, 50);
	cv::createTrackbar("check_count_thres", "bar", &check_count_thres, 10);

#define USE_CAMERA 1

#if USE_CAMERA
	cap.open(0);
	waitKey(1000);

	if (!cap.isOpened())
	{
		cout << "camera open failed!" << endl;
		return -1;
	}

#else
	cap.open(videoName);
	waitKey(2000);

#endif

	Mat src, src_temp, src_save;
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	cap >> src;
	imshow("cap", src);
	setMouseCallback("cap", onMouse, NULL); //mouse interface

#if USE_CAMERA

	while (true)
	{
		cap >> src_temp;
		warpFfine(src_temp, src, 180);
		imshow("cap", src);

		if (LBtnDown)
		{
			imshow("cap", src);
			cout << "LBTN:" << LBtnDown << endl;
			uartSent(UART_SENT_TYPE_MOUSE);
			waitKey(10);
			continue;
		}
		//uart_mouse(0, 0);
		uartSent(UART_SENT_TYPE_MOUSE);

		char c = waitKey(10);

		if (c == 32)
		{
			break;
		}
		if (c == 27)
		{
			return 0;
		}
	}

#endif

	if (src.data == NULL)
	{
		cout << "camera open failed!" << endl;
		return -1;
	}

	//TickMeter tm;

	int frame = 0;

	Size _size(300, 400);

	vector<NumberPosition> result;

	float tracker_psr = 0.0;

	bool start_track = false;

	Mat imgGray;
	cvtColor(src, imgGray, CV_BGR2GRAY);

	dlib::cv_image<unsigned char> dlib_img(imgGray);

	//int count_frame = 0;
	char temp_text[50];
	char character_temp = 0;
	Point pt_src_center(src.cols / 2, src.rows / 2);
	int check_count = 0;
	char check_character = 0;

	while (true)
	{
		frame++;
		log_out << "frame: " << frame << endl;

#if USE_CAMERA
		cap >> src_temp;
		warpFfine(src_temp, src, 180);
		src.copyTo(src_save);
#else
		cap >> src;
#endif

		if (LBtnDown)
		{
			imshow("cap", src);
			cout << "LBTN:" << LBtnDown << endl;
			uartSent(UART_SENT_TYPE_MOUSE);
			waitKey(10);
			continue;
		}

		cvtColor(src, imgGray, CV_BGR2GRAY);

		if (src.data == NULL)
		{
			cout << "src.data = NULL...try again..." << endl;
			continue;
		}

		if (flag_LX_target == 1)
		{
			if (state_num == SD_FLY_TARGET)
			{
				check_count = 0;
				have_target = false;
				target_num = -1;
				//continue;
			}

			if (state_num == SD_CHECK_TARGET)
			{
				detectNumber(src, tess, result);
				if (result.size() == 1)
				{
					if (check_character == result[0].number_[0])
					{
						check_count++;
					}
					else
					{
						check_count = 0;
					}

					check_character = result[0].number_[0];
				}
				if (check_count >= check_count_thres)
				{
					have_target = true;
					target_num = check_character - 48;
				}

				uartSent(UART_SENT_TYPE_TARGET);

				//continue;
			}
			else
			{
				check_count = 0;
			}
		}
		else
		{
			have_target = true;
			target_num = char_num;
		}

		if (state_num > 10)	//fly to workspace, detect, track, print and so on  
		{
			if (start_track)
			{
				tracker_psr = tracker.update(dlib_img) * 2;
				static int cnt_loss_track = 0, cnt_loss_track5 = 0;
				if (tracker_psr < 12)
					cnt_loss_track++;
				if (tracker_psr < 7)
					cnt_loss_track5++;
				if (cnt_loss_track > 20 || cnt_loss_track5 > 2)
				{
					cnt_loss_track = 0;
					cnt_loss_track5 = 0;
					start_track = false;
				}

				if (tracker_psr < psr_threshold || frame % 50 == 0)
				{
					start_track = false;
				}

				dlib::rectangle rect = tracker.get_position();
				Rect rect_temp = dlibRect2CVRect(tracker.get_position());
				number_position_send.position_ = Point(
						rect_temp.x + rect_temp.width / 2,
						rect_temp.y + rect_temp.height / 2);
				cv::rectangle(src, dlibRect2CVRect(tracker.get_position()),
						CV_RGB(255, 0, 0), 2, 8, 0);
				putText(src, number_position_send.number_,
						number_position_send.position_, FONT_HERSHEY_SIMPLEX, 1,
						CV_RGB(255, 0, 0), 2);
				sprintf(temp_text, "psr=%d", int(tracker_psr));
				putText(src, temp_text, Point(10, 20), FONT_HERSHEY_SIMPLEX,
						0.6, CV_RGB(255, 0, 0), 2);
				cv::circle(src,
						Point(rect.bl_corner().x() + rect.width() / 2,
								rect.bl_corner().y() - rect.height() / 2), 2,
						CV_RGB(255, 0, 0), 2, 8, 0);
			}
			else
			{
				number_position_send.init();
				detectNumber(src, tess, result);

				for (size_t i = 0; i < result.size(); i++)
				{
					if (result[i].number_[0] == target_num + 48)
					{
						number_position_send.number_ = result[i].number_;
						number_position_send.position_ = result[i].position_;
						number_position_send.boundRect = result[i].boundRect;

						tracker.start_track(dlib_img,
								getInitPosition(
										number_position_send.boundRect));
						start_track = true;
					}
					putText(src, result[i].number_, result[i].position_,
							FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 2);
				}
			}
		}

		else
		{
			number_position_send.init();
			start_track = false;
		}

		if (number_position_send.position_.x >= 0)
		{
			sprintf(temp_text, "position=%d,%d",
					int(number_position_send.position_.x),
					number_position_send.position_.y);
			putText(src, temp_text, Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.6,
					CV_RGB(255, 0, 0), 2);
		}

		drawImage(src);

		uartSent(UART_SENT_TYPE_CHARACTER);
		uartSent(UART_SENT_TYPE_TARGET);
		uartSent(UART_SENT_TYPE_MOUSE);

		imshow("cap", src);

		if (flag_writing == 1)
		{
			video_writer << src;
			if (flag_writevideo == 0)
			{
				flag_writing = 0;
				video_writer.release();
			}
		}
		else
		{
			if (flag_writevideo == 1)
			{
				startWriteVideo(video_num_read, video_writer);
				flag_writing = 1;
			}
		}

		if (flag_writing_src == 1)
		{
			video_writer_src << src_save;
			if (flag_writevideo_src == 0)
			{
				flag_writing_src = 0;
				video_writer_src.release();
			}
		}
		else
		{
			if (flag_writevideo_src == 1)
			{
				startWriteVideo(video_num_read, video_writer_src);
				flag_writing_src = 1;
			}
		}

		char c = waitKey(10);
		if (c == 27)
		{
			break;
		}

	}

	log_out.close();
	if (video_writer.isOpened())
	{
		video_writer.release();
	}
	if (video_writer_src.isOpened())
	{
		video_writer_src.release();
	}

	isTerminal = true;
	uart_read_thread.join();

	return 0;
}
