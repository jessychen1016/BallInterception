// This example is derived from the ssd_mobilenet_object_detection opencv demo
// and adapted to be used with Intel RealSense Cameras
// Please see https://github.com/opencv/opencv/blob/master/LICENSE
// under the Intel® Core™ i5-3570 CPU @ 3.40GHz × 4 

#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "example.hpp"
#include "cv-helpers.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <cmath>
#include <omp.h>
#include <fstream>
#include "iostream"
#include "time.h"
#include <chrono>
#include <thread>
#include "actionmodule.h"
#include "kalmanfilterdir.h"
#include "kalmanfilter.h"
#include "gotoposition.hpp"
#define MAX_THREADS 8


const size_t inWidth      = 600;
const size_t inHeight     = 900;
const float WHRatio       = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal       = 127.5;
int mat_columns;
int mat_rows;
int length_to_mid;
double alpha=0;
int full_time =0 ;
KalmanFilterDir kalman_filter_dir;
KalmanFilter1 kalman_filter;
double depth_length_coefficient(double depth){
    
    double length;
    length = 0.48033*depth+5.4556;
    return length;

}
    using namespace cv;
    using namespace cv::dnn;
    using namespace rs2;
    using namespace std;
    using namespace rs400;

int main(int argc, char** argv) try
{


// #ifdef _OPENMP

//     cout << "OpenMP supported !" << endl;

// # endif
    ZActionModule::instance();
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
   
    // Start streaming from Intel RealSense Camera
    pipeline pipe;

    //add filter

    // rs2::decimation_filter decimation_filter;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole_filling_filter;

    // decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, float(100));   
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL,float(7)); 
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, float(2));   
    // spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    auto config = pipe.start();
    std::ifstream t("../include/config/415highDensity.json");
    std::string str((std::istreambuf_iterator<char>(t)),
                std::istreambuf_iterator<char>());
    rs400::advanced_mode dev4json = config.get_device();
    dev4json.load_json(str);
    

    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);

    Size cropSize;
    if (profile.width() / (float)profile.height() > WHRatio)
    {
        cropSize = Size(static_cast<int>(profile.height() * WHRatio),
                        profile.height());
    }
    else
    {
        cropSize = Size(profile.width(),
                        static_cast<int>(profile.width() / WHRatio));
    }

    Rect crop(Point((profile.width() - cropSize.width) / 2,
                    (profile.height() - cropSize.height) / 2),
              cropSize);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);


    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
       
    int iLowH = 0;
    int iHighH = 38;
    int iLowS = 71; 
    int iHighS = 255;
    
    int iLowV = 203;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    // double distance[60] = {0};
    double last_x_meter = 0;
    double this_x_meter = 0;
    double last_y_meter = 0;
    double this_y_meter = 0;
    double y_vel = 0;
    double x_vel = 0;
    double velocity;
	double alphaset[2][10] = { 0 };
    double alpha_mean=0;
    double move_distance=0;
    double first_magic_distance=5;
    int count = 0;
    int magic_distance_flag = 1;
	int frame_number_before_decision = 3;
	double magic_distance;
	double lenght_to_midline_OFFSET;
    int moveDirection=1;
    string move_direction ;
    //action consts
    const double ACC_MAX = 900;
    const double VEL_MAX = 400;
    const double FRAME_PERIOD = 1 / 60.0;
	ofstream debug("../build/velocity.txt");
	// First while to aim at the ball

	while (cvGetWindowHandle(window_name))
		// for(int i = 0; i<60 && cvGetWindowHandle(window_name) ; i++)
	{
		auto start_time = clock();

		// Wait for the next set of frames
		auto data = pipe.wait_for_frames();
		// Make sure the frames are spatially aligned
		data = align_to.process(data);

		auto color_frame = data.get_color_frame();
		auto depth_frame = data.get_depth_frame();

		// If we only received new depth frame, 
		// but the color did not update, continue
		static int last_frame_number = 0;
		if (color_frame.get_frame_number() == last_frame_number) continue;
		last_frame_number = color_frame.get_frame_number();

		// Convert RealSense frame to OpenCV matrix:
		auto color_mat = frame_to_mat(color_frame);
		// imshow ("image", color_mat);
		auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
		// imshow ("image_depth", depth_mat);
		Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
			Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images


		Mat Gcolor_mat;
		Mat Gdepth_mat;


		GaussianBlur(color_mat, Gcolor_mat, Size(11, 11), 0);
		GaussianBlur(depth_mat, Gdepth_mat, Size(11,11), 0);


		// Crop both color and depth frames
		Gcolor_mat = Gcolor_mat(crop);
		Gdepth_mat = Gdepth_mat(crop);

		//start of mod
		mat_rows = Gcolor_mat.rows;
		mat_columns = Gcolor_mat.cols;
		// cout<< mat_columns<< endl;
		Mat imgHSV;
		vector<Mat> hsvSplit;
		cvtColor(Gcolor_mat, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
													 //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做

		split(imgHSV, hsvSplit);
		//cout << hsvSplit.size() << endl;
		
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		
		merge(hsvSplit, imgHSV);
		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

																									  //开操作 (去除一些噪点)
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

		//闭操作 (连接一些连通域)
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
													 //   imshow("Original", Gcolor_mat); //show the original image

		char key = (char)waitKey(1);
		//end of mod

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
		auto object = maxRect;
		auto moment = cv::moments(maxContour, true);

		Scalar depth_m;
		if (moment.m00 == 0) {
			moment.m00 = 1;
		}
		Point moment_center(moment.m10 / moment.m00, moment.m01 / moment.m00);
		depth_m = Gdepth_mat.at<double>((int)moment.m01 / moment.m00, (int)moment.m10 / moment.m00);
		magic_distance = depth_m[0] * 1.042 *100;
        // magic_distance = kalman_filter_dir.update(magic_distance);
		velocity = sqrt(y_vel*y_vel + x_vel*x_vel);

		// calculate length to midline
		length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320;
		last_x_meter = abs(length_to_mid);
		cout << "length_to_mid " << length_to_mid <<endl;
		cout << "Trying to locate the ball " << endl;
		if (abs(length_to_mid) <= 2) {
		
			ZActionModule::instance()->sendPacket(2, 0, 0, 0);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			count += 1;

			if (count == 10) {
				ZActionModule::instance()->sendPacket(2, 0, 0, 0, true );
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				cout << "go to next state" << endl;
				break;
				
			}
		
		}
		else {
			if (length_to_mid < -2) {
				
					ZActionModule::instance()->sendPacket(2, 0, 0, -10);
					std::this_thread::sleep_for(std::chrono::milliseconds(5));
				
			}
			else if (length_to_mid > 2) {
				
					ZActionModule::instance()->sendPacket(2, 0, 0, 10);
					std::this_thread::sleep_for(std::chrono::milliseconds(5));
				
			}
			else {
				ZActionModule::instance()->sendPacket(2, 0, 0, 0);
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
			
		
		}
		auto end_time = clock();
		cout << "time in a while1  " << 1000.000*(end_time - start_time) / CLOCKS_PER_SEC << endl;
	}

	ZActionModule::instance()->sendPacket(2, 0, 0, 0, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	count = 0;
	// 2nd While to intercept the ball
	int count_for_while2 = 0;
	last_x_meter = magic_distance *100;
	last_y_meter = length_to_mid;
	lenght_to_midline_OFFSET = length_to_mid;
	cout << "OFFSET =" << lenght_to_midline_OFFSET << endl;
    while (cvGetWindowHandle(window_name))
    // for(int i = 0; i<60 && cvGetWindowHandle(window_name) ; i++)
    {

        auto start_time = clock();

        // Wait for the next set of frames
        auto data = pipe.wait_for_frames();
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        depth_frame= temporal_filter.process(depth_frame);
        // depth_frame= hole_filling_filter.process(depth_frame);
        // depth_frame= spatial_filter.process(depth_frame);
        // If we only received new depth frame, 
        // but the color did not update, continue
        static int last_frame_number = 0;
        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = color_frame.get_frame_number();

        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = frame_to_mat(color_frame);
        // imshow ("image", color_mat);
        auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
        // imshow ("image_depth", depth_mat);
        Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
                                      Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images
        

        Mat Gcolor_mat;
        Mat Gdepth_mat;


        GaussianBlur(color_mat,Gcolor_mat,Size(9,9),0);
        GaussianBlur(depth_mat,Gdepth_mat,Size(3,3),0);
		//Gdepth_mat = depth_mat;

        // Crop both color and depth frames
        Gcolor_mat = Gcolor_mat(crop);
        Gdepth_mat = Gdepth_mat(crop);

         //start of mod
        mat_rows = Gcolor_mat.rows;
        mat_columns =Gcolor_mat.cols;
        // cout<< mat_columns<< endl;
		Mat imgHSV;
		vector<Mat> hsvSplit;
		cvtColor(Gcolor_mat, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
													 //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做

		split(imgHSV, hsvSplit);
		//cout << hsvSplit.size() << endl;
		
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		
		merge(hsvSplit, imgHSV);
        Mat imgThresholded;
    
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    
        //开操作 (去除一些噪点)
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    
        //闭操作 (连接一些连通域)
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
    
        imshow("Thresholded Image", imgThresholded); //show the thresholded image
    //   imshow("Original", Gcolor_mat); //show the original image
		
        char key = (char) waitKey(1);

        vector<vector<cv::Point>> contours ;
        cv::findContours(imgThresholded,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        double maxArea = 0;
        vector<cv::Point> maxContour;
        for(size_t i = 0; i < contours.size(); i++)
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
        auto object =  maxRect ;
        auto moment = cv::moments(maxContour,true);
        // std::cout<<"x="<< moment.m10 / moment.m00<<"  y="<<moment.m01/moment.m00<<std::endl;
        // std::cout<<"depth"<< depth_mat.at<double>((int)moment.m01 / moment.m00,(int)moment.m10/moment.m00)<<std::endl;
        
        // Calculate mean depth inside the detection region
        // This is a very naive way to estimate objects depth
        // but it is intended to demonstrate how one might 
        // use depht data in general
        Scalar depth_m;            
        if (moment.m00==0 ){
            moment.m00 = 1;
        }
        Point moment_center (moment.m10 / moment.m00, moment.m01/moment.m00);
        depth_m = Gdepth_mat.at<double>((int)moment.m01 / moment.m00,(int)moment.m10/moment.m00);
        magic_distance = depth_m[0] * 0.98* 100;
        // calculate length to midline
        if (count_for_while2 == 0) {
            length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320;
            lenght_to_midline_OFFSET = length_to_mid;
            first_magic_distance = magic_distance;
			last_x_meter = magic_distance;
            count_for_while2 += 1;
            cout << "lenghtOFFSET　= " << lenght_to_midline_OFFSET << endl;
        }
        length_to_mid = (moment.m10 / moment.m00-200)*depth_length_coefficient(magic_distance)/320 - lenght_to_midline_OFFSET;
        
		

        //KalmanFilter
        magic_distance = kalman_filter.update(magic_distance, length_to_mid)(0,0);
        length_to_mid  = kalman_filter.update(magic_distance, length_to_mid)(1,0);
		cout << endl << "length to midline =" << length_to_mid << "    ";
        // imshow
        std::ostringstream ss;
        ss << " Ball Detected ";
        ss << std::setprecision(3) << magic_distance << " centimeters away" ;
        String conf(ss.str());
        // distance[i]=magic_distance;

        rectangle(Gcolor_mat, object, Scalar(0, 255, 0));
        int baseLine = 0;
        Size labelSize = getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        auto center = (object.br() + object.tl())*0.5;
        center.x = center.x - labelSize.width / 2;
        center.y = center.y + 30 ;
        rectangle(Gcolor_mat, Rect(Point(center.x, center.y - labelSize.height),
        Size(labelSize.width, labelSize.height + baseLine)),
        Scalar(255, 255, 255), CV_FILLED);
        putText(Gcolor_mat, ss.str(), center,
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
		/*if (count_for_while2 <= 1) {
			count_for_while2 += 1;
			velocity = 0;
		}
		else {
			velocity = sqrt(y_vel*y_vel + x_vel*x_vel);
		}*/
		
        // velocity window
        ostringstream ss_v;
        ss_v << " The speed is ";
        ss_v <<setprecision(3) << velocity << " m/s" ;
        String conf_v(ss_v.str());
        Size labelSize_v = getTextSize(ss_v.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        auto center_v = (object.br() + object.tl())*0.5;
        center_v.x = 160- labelSize_v.width / 2;
        center_v.y = 475;

        rectangle(Gcolor_mat, Rect(Point(center_v.x, center_v.y - labelSize_v.height),
            Size(labelSize_v.width, labelSize_v.height + baseLine)),
            Scalar(255, 255, 255), CV_FILLED);
        putText(Gcolor_mat, ss_v.str(), center_v,
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));

        // move direction window
        
        ostringstream ss_move_direction;
        ss_move_direction << " Move to the ";
        ss_move_direction << move_direction;
        ss_move_direction << " for ";
        ss_move_direction << setprecision(3) << move_distance << " cm" ;
        String conf_move_direction(ss_v.str());
        Size labelSize_move_direction = getTextSize(ss_move_direction.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        auto center_move_direction = (object.br() + object.tl())*0.5;
        center_move_direction.x = 160- labelSize_move_direction.width / 2;
        center_move_direction.y = 450;

        rectangle(Gcolor_mat, Rect(Point(center_move_direction.x, center_move_direction.y - labelSize_move_direction.height),
            Size(labelSize_move_direction.width, labelSize_move_direction.height + baseLine)),
            Scalar(255, 255, 255), CV_FILLED);
        putText(Gcolor_mat, ss_move_direction.str(), center_move_direction,
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
        
        imshow(window_name, Gcolor_mat);
        if (waitKey(1) >= 0) break;
        imshow("heatmap", depth_mat);
        this_x_meter = magic_distance;
        this_y_meter = abs(length_to_mid);
        auto end_time = clock();

        x_vel = (this_x_meter-last_x_meter)/(end_time-start_time)*CLOCKS_PER_SEC;
        y_vel = (last_y_meter-this_y_meter)/(end_time-start_time)*CLOCKS_PER_SEC;
        velocity = sqrt(y_vel*y_vel + x_vel*x_vel);


		if (x_vel<= -150) {
			cout << "x_velocity = " << x_vel << "       ";
			cout << "y_velocity = " << y_vel << "       ";
			cout << "velocity = " << velocity << endl;
		}
		else {
			cout << "Yvelocity = " << y_vel << endl;
		}

        if(x_vel<= -150){
            count += 1;
            //alpha = atan(abs(last_y_meter - this_y_meter)/abs(this_x_meter-last_x_meter)/100);
            //cout<<"alpha  ="<<alpha<<"      ";
            if( count <= frame_number_before_decision){
				alphaset[0][count - 1] = this_x_meter;
				alphaset[1][count - 1] = this_y_meter;
				cout << "X"<<count<<" = "<< this_x_meter<< "        ";
				cout << "Y" << count << " = " << this_y_meter<< endl;
                cout << "time in a while" << 1000.000*(end_time - start_time) / CLOCKS_PER_SEC << endl;
                full_time += 1000.000*(end_time - start_time) / CLOCKS_PER_SEC;
            //alphaset[count-1]=alpha;
            //alpha_mean+= alphaset[count-1];
            }
        }
        
        if(count ==frame_number_before_decision ){
            //alpha_mean /= 3;
			// alpha_mean += atan(abs(alphaset[1][5] - alphaset[1][0]) / abs(alphaset[0][5] - alphaset[0][0]));
			// alpha_mean += atan(abs(alphaset[1][6] - alphaset[1][1]) / abs(alphaset[0][6] - alphaset[0][1]));
			// alpha_mean += atan(abs(alphaset[1][7] - alphaset[1][2]) / abs(alphaset[0][7] - alphaset[0][2]));
			// alpha_mean += atan(abs(alphaset[1][8] - alphaset[1][3]) / abs(alphaset[0][8] - alphaset[0][3]));
			// alpha_mean += atan(abs(alphaset[1][9] - alphaset[1][4]) / abs(alphaset[0][9] - alphaset[0][4]));
			alpha_mean += atan(abs(alphaset[1][1] - 0) / abs(alphaset[0][1] - first_magic_distance));
			alpha_mean += atan(abs(alphaset[1][2] - alphaset[1][0]) / abs(alphaset[0][2] - alphaset[0][0]));
			//alpha_mean += atan(abs(alphaset[1][4] - alphaset[1][1]) / abs(alphaset[0][4] - alphaset[0][1]));
			alpha_mean /= 2;
            cout<<"alpha mean=  "<<alpha_mean;
            move_distance = alpha_mean*first_magic_distance;
            cout<<endl<<"the depth for you to react ="<<first_magic_distance-magic_distance<<endl;
            count = frame_number_before_decision+1;
            moveDirection = length_to_mid/abs(length_to_mid);
			last_x_meter = this_x_meter;
			last_y_meter = this_y_meter;
			cout << "  first_magic_distance =" << first_magic_distance << endl;
			cout << "  move distance = " << move_distance << endl;
            cout << "FULL while = " << full_time << endl;
            break;

        }

    }
    if(move_distance >= 100){
        cout<< "Out Of RANGE DAMN IT!!!!!!!!"<< endl;
        return EXIT_SUCCESS;
    }
    // claculate movement
    double start = 0;  
    double end = move_distance;
    double dist = abs(end - start);
    int tAcc, tFlat;    
    double vMax = sqrt(ACC_MAX * dist);
    if(vMax < VEL_MAX) {
        tAcc = (int)(vMax / ACC_MAX / FRAME_PERIOD);
        tFlat = 0;
    }
    else {
        tAcc = (int)(VEL_MAX / ACC_MAX / FRAME_PERIOD);
        double distAcc = VEL_MAX * VEL_MAX / (2 * ACC_MAX);
        tFlat = (int)((dist - distAcc * 2) / VEL_MAX / FRAME_PERIOD);
    }
    int numFrame = 2 * tAcc + tFlat;    
    double *velList = new double[numFrame+1];
    for (int i = 1; i <= tAcc; i++) {
        velList[i - 1] = i * ACC_MAX * FRAME_PERIOD;
        velList[numFrame - i] = i * ACC_MAX * FRAME_PERIOD;
    }
    for(int i = tAcc; i < tAcc + tFlat; i++) {
        velList[i] = VEL_MAX;
    }
    velList[numFrame]=0;
    //move
	cout << "moveDirection" << moveDirection<< endl;
	cout << "Vellist[3]" << velList[3] << endl;

    for (int i = 0; i<= numFrame; i++){
        ZActionModule::instance()->sendPacket(2, 0, moveDirection*velList[i]*2, 0, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
	// std::this_thread::sleep_for(std::chrono::milliseconds(500));

	for (int i = 1; i <= 1000; i++) {
		ZActionModule::instance()->sendPacket(2, 0, 0, 0, true);
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	
	}

    for (int i =0; i<= numFrame; i++){
        debug << "i = "<< i<< "      ";
        debug << "velocity = "<< velList[i]<< endl;
    }

    // double dmean = 0;
    // double dvariance = 0; 

    // for(int i=30 ; i<60; i++){

    //     dmean += distance[i];
    // }
    // dmean /= 30.0;

    // for (int i=30; i<60; i++ ){
    //     dvariance += (distance[i]-dmean) * (distance[i]-dmean);
    // }
    
    // dvariance /= 29;

    // std::cout << dmean << ",      "<< sqrt(dvariance) << std::endl;  
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

