#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "cv-helpers.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <cmath>
#include <omp.h>
#include <fstream>
#include "iostream"
#include "time.h"
#include <chrono>
#include <thread>
#include "GetImage.h"
#include "kalmanfilter.h"

using namespace rs2;
using namespace std;
using namespace cv;
using namespace cv::dnn;

namespace {
	int iLowH = 0;
	int iHighH = 38;
	int iLowS = 71;
	int iHighS = 255;

	int iLowV = 203;
	int iHighV = 255;
	const size_t inWidth = 600;
	const size_t inHeight = 900;
	const float WHRatio = inWidth / (float)inHeight;
	const float inScaleFactor = 0.007843f;
	const float meanVal = 127.5;
	KalmanFilter1 kalman_filter;

}

GetImage::GetImage():color_frame(nullptr),depth_frame(nullptr){
    config = pipe.start();
	load_JSON();
	align_to = new rs2::align(RS2_STREAM_COLOR);
}

GetImage::~GetImage(){
	if (color_frame)
		delete color_frame;
	if (depth_frame)
		delete depth_frame;
}

int GetImage::check_camera(){
	cout << "check_camera" << endl;
	 context ctx;
    auto devices = ctx.query_devices();
    size_t device_count = devices.size();
    if (!device_count)
    {
        cout <<"No device detected. Is it plugged in?\n";
        return EXIT_SUCCESS;
    }

    // Get the first connected device
    auto dev = devices[0];

    if (dev.is<rs400::advanced_mode>())
    {
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
        // Check if advanced-mode is enabled
        if (!advanced_mode_dev.is_enabled())
        {
            // Enable advanced-mode
            advanced_mode_dev.toggle_advanced_mode(true);
        }
    }
    else
    {
        cout << "Current device doesn't support advanced-mode!\n";
        return EXIT_FAILURE;
    }
}

void GetImage::load_JSON(){
	cout << "JSON" << endl;
	std::ifstream t("../include/config/415highDensity.json");
    std::string str((std::istreambuf_iterator<char>(t)),
                std::istreambuf_iterator<char>());
    rs400::advanced_mode dev4json = config.get_device();
    dev4json.load_json(str);
    

     profile = new rs2::video_stream_profile(config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>());
}

void GetImage::temporalFilter(double a, double d, int h){
	cout << "Tfilter" << endl;
 	temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, float(100));   
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL,float(7)); 
}

void GetImage::hole_filling_filter(int h){
	cout << "Hfilter" << endl;
	rs2::hole_filling_filter hole_filling_filter;
	hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, float(2));
}

void GetImage::displayControl() {
	cout << "display_control" << endl;
	Size cropSize;
	if (profile->width() / (float)profile->height() > WHRatio)
	{
		cropSize = Size(static_cast<int>(profile->height() * WHRatio),
			profile->height());
	}
	else
	{
		cropSize = Size(profile->width(),
			static_cast<int>(profile->width() / WHRatio));
	}

	 crop = Rect(
		Point(
			(profile->width() - cropSize.width) / 2,
			(profile->height() - cropSize.height) / 2
		),
		cropSize
	);

	window_name = "Display Image";
	namedWindow(window_name, WINDOW_AUTOSIZE);


	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	

}




void GetImage::get_Frame() {


	auto start_timeFrame = clock();

	// Wait for the next set of frames
	data = pipe.wait_for_frames();
	auto end_timeFrame = clock();
	cout << "time in Frame  " << 1000.000*(end_timeFrame - start_timeFrame) / CLOCKS_PER_SEC << endl;
}
bool GetImage::get_RGBD_data() {
	auto start_timeRGB = clock();
	
	// Make sure the frames are spatially aligned
	data = align_to->process(data);
	
	auto&& color_frame_ = data.get_color_frame();
	auto&& depth_frame_ = data.get_depth_frame();


	if (color_frame)
		*color_frame = color_frame_;//data.get_color_frame();
	else
		color_frame = new rs2::video_frame(color_frame_);

	if (depth_frame)
		*depth_frame = depth_frame_;// data.get_depth_frame();
	else
		depth_frame = new rs2::depth_frame(depth_frame_);


	*depth_frame = temporal_filter.process(*depth_frame);

	// If we only received new depth frame, 
	// but the color did not update, continue
	static int last_frame_number = 0;
	if (color_frame->get_frame_number() == last_frame_number) return false;
	
	last_frame_number = color_frame->get_frame_number();

	auto end_timeRGB = clock();
	cout << "time in RGB  " << 1000.000*(end_timeRGB - start_timeRGB) / CLOCKS_PER_SEC << endl;
	return true;
}



void GetImage::convert_2_GMAT() {

	auto start_timeGMAT = clock();
	// Convert RealSense frame to OpenCV matrix:
	auto color_mat = frame_to_mat(*color_frame);
	// imshow ("image", color_mat);
	auto depth_mat = depth_frame_to_meters(pipe, *depth_frame);
	//imshow ("image_depth", depth_mat);
	Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
		Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images

	GaussianBlur(color_mat, Gcolor_mat, Size(11,11), 0);
	GaussianBlur(depth_mat, Gdepth_mat, Size(3,3), 0);

	// Crop both color and depth frames
	Gcolor_mat = Gcolor_mat(crop);
	Gdepth_mat = Gdepth_mat(crop);
	auto end_timeGMAT = clock();
	cout << "time in GMAT  " << 1000.000*(end_timeGMAT - start_timeGMAT) / CLOCKS_PER_SEC << endl;
}

void GetImage::rgb_2_HSV() {

	auto start_timeHSV = clock();
	vector<Mat> hsvSplit;
	cvtColor(Gcolor_mat, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
												 //?Ϊ????ȡ??ǲ??ͼ??ֱ??ͼ???�?Ҫ?HSV?ռ??

	split(imgHSV, hsvSplit);
	//cout << hsvSplit.size() << endl;

	equalizeHist(hsvSplit[2], hsvSplit[2]);

	merge(hsvSplit, imgHSV);

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

																																						//????? (ȥ??һЩ???
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	//?ղ?? (?????Щ??ͨ?)
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	imshow("Thresholded Image", imgThresholded);
	key = (char)cv::waitKey(1);
	auto end_timeHSV = clock();
	cout << "time in HSV  " << 1000.000*(end_timeHSV - start_timeHSV) / CLOCKS_PER_SEC << endl;
}


double GetImage::depth_length_coefficient(double depth) {
	double length;
	length = 0.48033*depth + 5.4556;
	return length;
}


void GetImage::find_Contour(bool KF) {


	auto start_timeContour = clock();

	vector<vector<cv::Point>> contours;
	cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	double maxArea = 0;
	vector<cv::Point> maxContour;
	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > maxArea)
		{
			maxArea = area;
			maxContour = contours[i];
		}
	}
	cv::Rect maxRect = cv::boundingRect(maxContour);

	// auto object =  maxRect & Rect (0,0,depth_mat.cols, depth_mat.rows );
	object = maxRect;
	moment = cv::moments(maxContour, true);

	Scalar depth_m;
	if (moment.m00 == 0) {
		moment.m00 = 1;
	}
	Point moment_center(moment.m10 / moment.m00, moment.m01 / moment.m00);
	depth_m = Gdepth_mat.at<double>((int)moment.m01 / moment.m00, (int)moment.m10 / moment.m00);
	magic_distance = depth_m[0] * 1.042 * 100;
	pixal_to_bottom = (480 - moment.m01 / moment.m00);



	if (KF) {
		if (count_for_while2 == 0) {
			length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320;
			lenght_to_midline_OFFSET = length_to_mid;
			length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320 - lenght_to_midline_OFFSET;
			first_magic_distance = magic_distance;
			magic_distance = kalman_filter.update(magic_distance, length_to_mid)(0, 0);
			length_to_mid = kalman_filter.update(magic_distance, length_to_mid)(1, 0);
			last_x_meter = magic_distance;
			last_y_meter = abs(length_to_mid);
			pixal_to_bottom = (480 - moment.m01 / moment.m00);
			count_for_while2 += 1;
			cout << "lenghtOFFSET = " << lenght_to_midline_OFFSET << endl;
		}
		else {
			length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320 - lenght_to_midline_OFFSET;
			//KalmanFilter
			magic_distance = kalman_filter.update(magic_distance, length_to_mid)(0, 0);
			length_to_mid = kalman_filter.update(magic_distance, length_to_mid)(1, 0);
		}
	}
	else {
		// calculate length to midline
		length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320;

	}

	auto end_timeContour = clock();
	cout << "time in Contour  " << 1000.000*(end_timeContour - start_timeContour) / CLOCKS_PER_SEC << endl;
}

void GetImage::show_window() {
	std::ostringstream ss;
	ss << " Ball Detected ";
	ss << std::setprecision(3) << magic_distance << " centimeters away";
	String conf(ss.str());
	// distance[i]=magic_distance;

	rectangle(Gcolor_mat, object, Scalar(0, 255, 0));
	int baseLine = 0;
	Size labelSize = getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	auto center = (object.br() + object.tl())*0.5;
	center.x = center.x - labelSize.width / 2;
	center.y = center.y + 30;
	rectangle(Gcolor_mat, Rect(Point(center.x, center.y - labelSize.height),
		Size(labelSize.width, labelSize.height + baseLine)),
		Scalar(255, 255, 255), CV_FILLED);
	putText(Gcolor_mat, ss.str(), center,
		FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));


	// velocity window
	ostringstream ss_v;
	ss_v << " The speed is ";
	ss_v << setprecision(3) << velocity << " m/s";
	String conf_v(ss_v.str());
	Size labelSize_v = getTextSize(ss_v.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

	auto center_v = (object.br() + object.tl())*0.5;
	center_v.x = 160 - labelSize_v.width / 2;
	center_v.y = 475;

	rectangle(Gcolor_mat, Rect(Point(center_v.x, center_v.y - labelSize_v.height),
		Size(labelSize_v.width, labelSize_v.height + baseLine)),
		Scalar(255, 255, 255), CV_FILLED);
	putText(Gcolor_mat, ss_v.str(), center_v,
		FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));

	/*
// move direction window

	ostringstream ss_move_direction;
	ss_move_direction << " Move to the ";
	ss_move_direction << move_direction;
	ss_move_direction << " for ";
	ss_move_direction << setprecision(3) << move_distance << " cm";
	String conf_move_direction(ss_v.str());
	Size labelSize_move_direction = getTextSize(ss_move_direction.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

	auto center_move_direction = (object.br() + object.tl())*0.5;
	center_move_direction.x = 160 - labelSize_move_direction.width / 2;
	center_move_direction.y = 450;

	rectangle(Gcolor_mat, Rect(Point(center_move_direction.x, center_move_direction.y - labelSize_move_direction.height),
		Size(labelSize_move_direction.width, labelSize_move_direction.height + baseLine)),
		Scalar(255, 255, 255), CV_FILLED);
	putText(Gcolor_mat, ss_move_direction.str(), center_move_direction,
		FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
		*/

	imshow(window_name, Gcolor_mat);
	key = (char)cv::waitKey(1);
	imshow("heatmap", Gdepth_mat);
	key = (char)cv::waitKey(1);
}