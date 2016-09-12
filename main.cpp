#include "CharacterRecognition.h"
#include <fstream>
#include <sstream>
#include <thread>
#include "trace.h"
#include "uart.h"

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include "percipio_camport.h"
#include "depth_render.h"

using namespace std;
using namespace cv;

//state
#define SG_LOW_CHECK		"G_LC"	//0
#define SG_MID_CHECK		"G_MC"	//1
#define SU_UP1				"U_UP1"	//2
#define SU_HOLD				"U_HOLD"	//3
#define SD_RETRY_UP			"R_UP"	//4
#define SD_RETRY_UP_HOLD	"R_HOLD"	//5


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

cv::Rect dlibRect2CVRect(dlib::rectangle r) {
	return cv::Rect(r.left(), r.top(), r.width(), r.height());
}

dlib::rectangle getInitPosition(cv::Rect r) {
	//dlib::rectangle initPos(r.x, r.y, r.x + r.width, r.y + r.height);
	return dlib::rectangle(r.x, r.y, r.x + r.width, r.y + r.height);
}

bool LBtnDown = false;

void onMouse(int event, int x, int y, int, void* param)
{
	//Point origin;//������������������������������������������������������������������������������������������������������������������������origin����������������������������������������������������

	if(LBtnDown)
	{
		uart_mouse(x, y);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		LBtnDown = true;//����������������������������������������

	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		LBtnDown = false;
	}
}

void warpFfine(Mat &inputIm, Mat &tempImg, float angle) {
	CV_Assert(!inputIm.empty());
	Mat inputImg;
	inputIm.copyTo(inputImg);
	float radian = (float) (angle / 180.0 * CV_PI);
	int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
	int dx = (int) (uniSize - inputImg.cols) / 2;
	int dy = (int) (uniSize - inputImg.rows) / 2;
	copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
	Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
	Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
	warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
	float sinVal = fabs(sin(radian));
	float cosVal = fabs(cos(radian));
	Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
			(int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
	int x = (tempImg.cols - targetSize.width) / 2;
	int y = (tempImg.rows - targetSize.height) / 2;
	Rect rect(x, y, targetSize.width, targetSize.height);
	tempImg = Mat(tempImg, rect);
}

string  get_time()
{
	time_t time_ptr;
	time(&time_ptr);
	tm *ptm = gmtime(&time_ptr);
	char date[60] = { 0 };
	sprintf(date, "%d-%02d-%02d--%02d:%02d:%02d",
			(int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
			(int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
	return std::string(date);

}

int char_num = 5;

static cv::Mat depth, depth_show, point_cloud;
static DepthRender render;
static int fps_counter = 0;
static clock_t fps_tm = 0;

void process_frames(percipio::DepthCameraDevice &port);
void save_frame_to_file();
int get_fps();
void CopyBuffer(percipio::ImageBuffer *pbuf, cv::Mat &img);

void frame_arrived_callback(void *user_data) {
  // call port.FramePackageGet to update internal buffer
  // call port.FrameGet to get frame data here
  // To avoid performance problem ,time consuming task in callback function is not recommended.
}

int initPercipio()
{
	percipio::DepthCameraDevice port(percipio::MODEL_DPB04GN);
	  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;
	  render.color_type = DepthRender::COLORTYPE_BLUERED;
	  render.invalid_label = 0;
	  render.Init();
	  percipio::SetLogLevel(percipio::LOG_LEVEL_INFO);
	  port.SetCallbackUserData(NULL);
	  port.SetFrameReadyCallback(frame_arrived_callback);

	  int ver = percipio::LibVersion();
	  printf("Sdk version is %d\n", ver);

	  int ret = port.OpenDevice();
	  if (percipio::CAMSTATUS_SUCCESS != ret) {
	    printf("open device failed\n");
	    return -1;
	  }
	  else
		  return 0;
}

std::string video_num_path("/home/odroid/workspace/characterRecognition/video/video_num.txt");

int video_num = 0;
std::ifstream video_num_read;
std::ofstream video_num_write;

void startWriteVideo(std::ifstream &video_num_read, cv::VideoWriter &video_writer)
{
	video_num_read.open(video_num_path.c_str());
	video_num_read >> video_num;
	video_num_read.close();

	cout << video_num << endl;

	video_num_write.open(video_num_path.c_str());
	video_num_write << (video_num + 1);
	video_num_write.close();

	if (video_writer.isOpened())
	{
		video_writer.release();
	}

	std::stringstream ss;
	string video_name;

	ss << video_num;
	ss >> video_name;
	video_name += ".avi";

	video_writer.open("/home/odroid/workspace/characterRecognition/video/"+video_name,
			CV_FOURCC('D', 'I', 'V', 'X'), 15,Size(320, 240));

}

int target_num = -1;
int state_num = 0;

int main()
{
//	percipio::DepthCameraDevice port(percipio::MODEL_DPB04GN);
//		  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;
//		  render.color_type = DepthRender::COLORTYPE_BLUERED;
//		  render.invalid_label = 0;
//		  render.Init();
//		  percipio::SetLogLevel(percipio::LOG_LEVEL_INFO);
//		  port.SetCallbackUserData(NULL);
//		  port.SetFrameReadyCallback(frame_arrived_callback);
//
//		  int ver = percipio::LibVersion();
//		  printf("Sdk version is %d\n", ver);
//
//		  int ret = port.OpenDevice();
//		  if (percipio::CAMSTATUS_SUCCESS != ret) {
//		    printf("open device failed\n");
//		    return -1;
//		  }
//
//	int wait_time;
//	  port.GetProperty(percipio::PROP_WAIT_NEXTFRAME_TIMEOUT, (char *)&wait_time, sizeof(int));
//	  printf("get property PROP_WAIT_NEXTFRAME_TIMEOUT %d\n", wait_time);
//
//	  int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_DEPTH);
//	  //int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR);
//	  //int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR_DEPTH);
//	  if (reti < 0) {
//	    printf("set mode failed,error code:%d\n", reti);
//	    return -1;
//	  }

	string imagename("NumberBlock-2.bmp");

	tesseract::TessBaseAPI tess;

	tess.Init(NULL, "eng");

	tess.SetPageSegMode(tesseract::PageSegMode::PSM_SINGLE_BLOCK);

	VideoCapture cap;

	ofstream log_out;
	log_out.open("log.txt");

    string videoName("/home/odroid/workspace/characterRecognition/video/src2.avi");

    dlib::correlation_tracker tracker;

	cv::namedWindow("bar");
	int psr_threshold = 20;
	int flag_writevideo = 0;
	int flag_writing = 0;
	int flag_writevideo_src = 0;
	int flag_writing_src = 0;

	cv::VideoWriter video_writer;
	cv::VideoWriter video_writer_src;

	string time;
	time = get_time();

	cv::createTrackbar("character", "bar", &char_num, 9);
	cv::createTrackbar("psr_threshold", "bar", &psr_threshold, 60);
	cv::createTrackbar("writevideo", "bar", &flag_writevideo, 1);
	cv::createTrackbar("writevideo_src", "bar", &flag_writevideo_src, 1);

	std::vector<std::string> state_str;
	state_str.resize(30,"none");
		state_str[0] = SG_LOW_CHECK;
		state_str[1] = SG_MID_CHECK;
		state_str[2] = SU_UP1;
		state_str[3] = SU_HOLD;
		state_str[4] = SD_RETRY_UP;
		state_str[5] = SD_RETRY_UP_HOLD;

		state_str[13] = SD_HOLD;
		state_str[14] = SD_MISS_SEARCH;
		state_str[15] = SD_HOLD2;
		state_str[16] = SD_HIGH_FAST_DOWN;
		state_str[17] = SD_CIRCLE_SLOW_DOWN;
		state_str[18] = SD_CIRCLE_HOLD;
		state_str[19] = SD_CIRCLE_MID_DOWN;
		state_str[20] = SD_CHECK_G;
		state_str[21] = SD_SHUT_DOWN;
		state_str[22] = SD_SAFE;

#define USE_CAMERA 0

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
    setMouseCallback("cap", onMouse, NULL);//mouse interface

#if USE_CAMERA

    while (true)
    {
//    	if (port.FramePackageGet() == percipio::CAMSTATUS_SUCCESS) {
//    	      process_frames(port);
//    	    }

        cap >> src_temp;
        warpFfine(src_temp, src, 180);
        imshow("cap", src);

        if(LBtnDown)
        {
        	imshow("cap", src);
        	cout<<"LBTN:"<<LBtnDown<<endl;
        	waitKey(10);
        	continue;
        }
        uart_mouse(0,0);

        char c = waitKey(10);

        if (c==32)
        {
            break;
        }
        if (c==27)
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
	Point pt_src_center(src.cols/2, src.rows/2);


	while (true)
	{
//		if (port.FramePackageGet() == percipio::CAMSTATUS_SUCCESS) {
//		    	      process_frames(port);
//		    	    }

		frame++;
		log_out << "frame: " << frame << endl;

		//tm.reset();
		//tm.start();
#if USE_CAMERA
		cap >> src_temp;
		warpFfine(src_temp, src, 180);
		src.copyTo(src_save);
#else
		cap >> src;
#endif

		if(LBtnDown)
		{
			imshow("cap", src);
			cout<<"LBTN:"<<LBtnDown<<endl;
			waitKey(10);
			continue;
		}

		cvtColor(src, imgGray, CV_BGR2GRAY);

		if (src.data==NULL)
		{
			break;
		}

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

			if (tracker_psr < psr_threshold || frame%50 == 0)
			{
				start_track = false;
			}

			dlib::rectangle rect = tracker.get_position();
			Rect rect_temp = dlibRect2CVRect(tracker.get_position());
			number_position_send[0].position_ = Point(rect_temp.x+rect_temp.width/2,
					rect_temp.y+rect_temp.height/2);
			cv::rectangle(src, dlibRect2CVRect(tracker.get_position()),CV_RGB(255, 0, 0), 2, 8, 0);
			putText(src, number_position_send[0].number_,
					number_position_send[0].position_,
					FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0), 2);
			sprintf(temp_text, "psr=%d", int(tracker_psr));
			putText(src, temp_text,Point(10,20),
								FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,0,0), 2);
			cv::circle(src,Point(rect.bl_corner().x() + rect.width() / 2,rect.bl_corner().y() - rect.height() / 2), 2,CV_RGB(255, 0, 0), 2, 8, 0);
		}
		else
		{
			detectNumber(src, tess, number_position_send);
			if(number_position_send.size()>0)
			{
				tracker.start_track(dlib_img, getInitPosition(number_position_send[0].boundRect));
				start_track = true;
			}
		}
		if (number_position_send.size() > 0)
		{
			sprintf(temp_text, "position=%d,%d",
					int(number_position_send[0].position_.x),
					number_position_send[0].position_.y);
			putText(src, temp_text, Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.6,
					CV_RGB(255, 0, 0), 2);
		}

		if(state_v >= 50)
		{
			state_num=state_v-50;
		}
		else
		{
			target_num = state_v;
		}

		sprintf(temp_text, "target=%d",target_num);
		putText(src, temp_text, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.6,
								CV_RGB(255, 0, 0), 2);

		putText(src, state_str[state_num], Point(160, 20),
				CV_FONT_HERSHEY_TRIPLEX, 0.6, CV_RGB(255, 0, 0), 1, 8);


		cv::line(src, pt_src_center-Point(10,0),pt_src_center+Point(10,0),CV_RGB(0,255,0),2);
		cv::line(src, pt_src_center-Point(0,10),pt_src_center+Point(0,10),CV_RGB(0,255,0),2);

		uart_send();
		uart_mouse(0,0);

//		for (size_t i = 0; i < number_position_send.size(); i++)
//		{
//			/**
//			 * Check the character to print
//			 *
//			 * TODO
//			 */
//
//			uart_send();
//
//			cout << "number: " << number_position_send[i].number_ << endl;
//			cout << "position: " << number_position_send[i].position_ << endl;
//			cout << "uart_good: " << uart_good << endl;
//			log_out << "number: " << number_position_send[i].number_ << " ";
//			log_out << "position: " << number_position_send[i].position_ << endl;
//		}

//		tm.stop();
//		cout << tm.getTimeMilli() << "ms" << endl;
//		cout <<endl;
        
		imshow("cap", src);

		//double alpha = 0.8;
		//double beta = 1-alpha;
		//Mat dst;
		//addWeighted(src, alpha, depth_show, beta, 0.0, dst);
		//imshow("dst", dst);

		if(flag_writing == 1)
		{
			video_writer<<src;
			if(flag_writevideo == 0)
			{
				flag_writing = 0;
				video_writer.release();
			}
		}
		else
		{
			if(flag_writevideo == 1)
			{
				startWriteVideo(video_num_read, video_writer);
				flag_writing = 1;
			}
		}

		if(flag_writing_src == 1)
				{
					video_writer_src<<src_save;
					if(flag_writevideo_src == 0)
					{
						flag_writing_src = 0;
						video_writer_src.release();
					}
				}
				else
				{
					if(flag_writevideo_src == 1)
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
	if(video_writer.isOpened())
	{
		video_writer.release();
	}
	if(video_writer_src.isOpened())
	{
		video_writer_src.release();
	}

	 //port.CloseDevice();

	  depth.release();
	  point_cloud.release();
	  render.Uninit();

	return 0;
}

void process_frames(percipio::DepthCameraDevice &port) {
  percipio::ImageBuffer pimage;
  int ret = port.FrameGet(percipio::CAMDATA_LEFT, &pimage);
//  if (percipio::CAMSTATUS_SUCCESS == ret) {
//    CopyBuffer(&pimage, left);
//    cv::imshow("left", left);
//  }
//  ret = port.FrameGet(percipio::CAMDATA_RIGHT, &pimage);
//  if (percipio::CAMSTATUS_SUCCESS == ret) {
//    CopyBuffer(&pimage, right);
//    cv::imshow("right", right);
//  }
  ret = port.FrameGet(percipio::CAMDATA_DEPTH, &pimage);
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    CopyBuffer(&pimage, depth);
    cv::Mat t;
    //int fps = get_fps();
    render.Compute(depth, t);

    int w = depth.size().width/5;
    int h = depth.size().height/5;

    cv::Rect rect(2*w,0,w,depth.size().height);
    //Mat t_m = t(rect);
    Mat depth_m = depth(rect);
    //imshow("depth_m", t_m);

    int mean_depth[5]={0};

    for(size_t i=0;i<5;i++)
    {
    	Rect rrr(0,i*h,w,h);
    	//Mat region = t_m(rrr);
    	Mat depth_region = depth_m(rrr);

    	int num = cv::countNonZero(depth_region);
    	Scalar sum_depth = sum(depth_region);
    	mean_depth[i] = sum_depth.val[0]/num/10;
    	//cout<<mean_depth[i]<<endl;
    	//imshow("region", region);
    	//cv::waitKey(0);
    }

    uart_depth(mean_depth,5);

    cv::resize(t, depth_show, Size(320,240));

    cv::imshow("depth", depth_show);

  }
//  ret = port.FrameGet(percipio::CAMDATA_POINT3D, &pimage);
//  if (percipio::CAMSTATUS_SUCCESS == ret) {
//    CopyBuffer(&pimage, point_cloud);
//  }
}
#ifdef _WIN32
int get_fps() {
  const int kMaxCounter = 20;
  fps_counter++;
  if (fps_counter < kMaxCounter) {
    return -1;
  }
  int elapse = (clock() - fps_tm);
  int v = (int)(((float)fps_counter) / elapse * CLOCKS_PER_SEC);
  fps_tm = clock();

  fps_counter = 0;
  return v;
}
#else
int get_fps() {
  const int kMaxCounter = 20;
  struct timeval start;
  fps_counter++;
  if (fps_counter < kMaxCounter) {
    return -1;
  }

  gettimeofday(&start, NULL);
  int elapse = start.tv_sec * 1000 + start.tv_usec / 1000 - fps_tm;
  int v = (int)(((float)fps_counter) / elapse * 1000);
  gettimeofday(&start, NULL);
  fps_tm = start.tv_sec * 1000 + start.tv_usec / 1000;

  fps_counter = 0;
  return v;
}
#endif

void save_frame_to_file() {
  static int idx = 0;
  char buff[100];
//  if (!left.empty()) {
//    sprintf(buff, "%d-left.png", idx);
//    cv::imwrite(buff, left);
//  }
//  if (!right.empty()) {
//    sprintf(buff, "%d-right.png", idx);
//    cv::imwrite(buff, right);
//  }
  if (!depth.empty()) {
    sprintf(buff, "%d-depth.txt", idx);
    std::ofstream ofs(buff);
    ofs << depth;
    ofs.close();
  }
  if (!point_cloud.empty()) {
    sprintf(buff, "%d-points.txt", idx);
    std::ofstream ofs(buff);
    percipio::Vect3f *ptr = point_cloud.ptr<percipio::Vect3f>();
    for (int i = 0; i < point_cloud.size().area(); i++) {
      ofs << ptr->x << "," << ptr->y << "," << ptr->z << std::endl;
      ptr++;
    }
    ofs.close();
  }
  idx++;
}

void CopyBuffer(percipio::ImageBuffer *pbuf, cv::Mat &img) {
  switch (pbuf->type) {
  case percipio::ImageBuffer::PIX_8C1:
    img.create(pbuf->height, pbuf->width, CV_8UC1);
    break;
  case percipio::ImageBuffer::PIX_16C1:
    img.create(pbuf->height, pbuf->width, CV_16UC1);
    break;
  case percipio::ImageBuffer::PIX_8C3:
    img.create(pbuf->height, pbuf->width, CV_8UC3);
    break;
  case percipio::ImageBuffer::PIX_32FC3:
    img.create(pbuf->height, pbuf->width, CV_32FC3);
    break;
  default:
    img.release();

    return;
  }
  memcpy(img.data, pbuf->data, pbuf->width * pbuf->height * pbuf->get_pixel_size());
}
