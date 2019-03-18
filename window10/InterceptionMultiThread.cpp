// This example is derived from the ssd_mobilenet_object_detection opencv demo
// and adapted to be used with Intel RealSense Cameras
// Please see https://github.com/opencv/opencv/blob/master/LICENSE
// under the Intel® Core™ i5-3570 CPU @ 3.40GHz × 4 

//#include <librealsense2/rs.hpp>
//#include <librealsense2/rs_advanced_mode.hpp>
//#include "example.hpp"
//#include "cv-helpers.hpp"
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
#include "GetImage.h"
#include "actionmodule.h"
#include "kalmanfilterdir.h"
#include "kalmanfilter.h"
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


    using namespace cv;
    using namespace rs2;
    using namespace std;
    using namespace rs400;

int main(int argc, char** argv) try
{


// # endif
    ZActionModule::instance();
    GetImage getImage;
	getImage.check_camera();

   
    // Start streaming from Intel RealSense Camera

    //add filter
	getImage.temporalFilter();
	getImage.hole_filling_filter();

	//load JSON


    rs2::align align_to(RS2_STREAM_COLOR);

	getImage.displayControl();
    

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
    double pixal_to_bottom;
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

	while (cvGetWindowHandle(getImage.window_name))
		// for(int i = 0; i<60 && cvGetWindowHandle(getImage.window_name) ; i++)
	{

		bool result = getImage.get_RGBD_data();

		if (!result) {
			continue;
		}


		getImage.convert_2_GMAT();
		getImage.rgb_2_HSV();
		getImage.find_Contour(false);


		
		length_to_mid = getImage.length_to_mid;
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

	}

	ZActionModule::instance()->sendPacket(2, 0, 0, 0, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	count = 0;
	// 2nd While to intercept the ball
	int count_for_while2 = 0;
	last_x_meter = getImage.magic_distance;
	last_y_meter = getImage.length_to_mid;

    while (cvGetWindowHandle(getImage.window_name))
    // for(int i = 0; i<60 && cvGetWindowHandle(getImage.window_name) ; i++)
    {

        auto start_time = clock();

		bool result = getImage.get_RGBD_data();

		if (!result) {
			continue;
		}


		getImage.convert_2_GMAT();
		getImage.rgb_2_HSV();
		getImage.find_Contour(true);
        

        this_x_meter = getImage.magic_distance;
        this_y_meter = abs(getImage.length_to_mid);
        auto end_time = clock();
		cout << "time in a While  " << 1000.000*(end_time - start_time) / CLOCKS_PER_SEC << endl;
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
			alpha_mean += atan(abs(alphaset[1][1] - 0			  ) / abs(alphaset[0][1] - getImage.first_magic_distance));
			alpha_mean += atan(abs(alphaset[1][2] - alphaset[1][0]) / abs(alphaset[0][2] - alphaset[0][0]));
			//alpha_mean += atan(abs(alphaset[1][4] - alphaset[1][1]) / abs(alphaset[0][4] - alphaset[0][1]));
			alpha_mean /= 2;
            cout<<"alpha mean=  "<<alpha_mean;
			cout << "  first_magic_distance =" << getImage.first_magic_distance << endl;
            move_distance = alpha_mean*getImage.first_magic_distance;
			cout << "  move distance = " << move_distance << endl;
            cout<<endl<<"the depth for you to react ="<< this_x_meter <<endl;
			cout << endl << "length2MID =" << getImage.length_to_mid << endl;
            moveDirection = getImage.length_to_mid/abs(getImage.length_to_mid);
            cout << "FULL while = " << full_time << endl;
            //pixal_to_bottom = (480 - getImage.moment.m01 / getImage.moment.m00);
            break;
        }
		last_x_meter = this_x_meter;
		last_y_meter = this_y_meter;
    }
	cout << "out of while2" << endl;
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


    //third while to adjust the locate base on vision


    // while (cvGetWindowHandle(getImage.window_name))
    //     // for(int i = 0; i<60 && cvGetWindowHandle(getImage.window_name) ; i++)
    // {
    //     auto start_time = clock();

    //     // Wait for the next set of frames
    //     auto data = pipe.wait_for_frames();
    //     // Make sure the frames are spatially aligned
    //     data = align_to.process(data);

    //     auto color_frame = data.get_color_frame();
    //     auto depth_frame = data.get_depth_frame();

    //     // If we only received new depth frame, 
    //     // but the color did not update, continue
    //     static int last_frame_number = 0;
    //     if (color_frame.get_frame_number() == last_frame_number) continue;
    //     last_frame_number = color_frame.get_frame_number();

    //     // Convert RealSense frame to OpenCV matrix:
    //     auto color_mat = frame_to_mat(color_frame);
    //     // imshow ("image", color_mat);
    //     auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
    //     // imshow ("image_depth", depth_mat);
    //     Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
    //         Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images


    //     Mat Gcolor_mat;
    //     Mat Gdepth_mat;


    //     GaussianBlur(color_mat, Gcolor_mat, Size(11, 11), 0);
    //     GaussianBlur(depth_mat, Gdepth_mat, Size(11,11), 0);


    //     // Crop both color and depth frames
    //     Gcolor_mat = Gcolor_mat(crop);
    //     Gdepth_mat = Gdepth_mat(crop);

    //     //start of mod
    //     mat_rows = Gcolor_mat.rows;
    //     mat_columns = Gcolor_mat.cols;
    //     // cout<< mat_columns<< endl;
    //     Mat imgHSV;
    //     vector<Mat> hsvSplit;
    //     cvtColor(Gcolor_mat, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    //                                                  //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做

    //     split(imgHSV, hsvSplit);
    //     //cout << hsvSplit.size() << endl;
        
    //     equalizeHist(hsvSplit[2], hsvSplit[2]);
        
    //     merge(hsvSplit, imgHSV);
    //     Mat imgThresholded;

    //     inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //                                                                                                   //开操作 (去除一些噪点)
    //     Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //     morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    //     //闭操作 (连接一些连通域)
    //     morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    //     imshow("Thresholded Image", imgThresholded); //show the thresholded image
    //                                                  //   imshow("Original", Gcolor_mat); //show the original image

    //     char key = (char)waitKey(1);
    //     //end of mod

    //     vector<vector<cv::Point>> contours;
    //     cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //     double maxArea = 0;
    //     vector<cv::Point> maxContour;
    //     for (size_t i = 0; i < contours.size(); i++)
    //     {
    //         double area = cv::contourArea(contours[i]);
    //         if (area > maxArea)
    //         {
    //             maxArea = area;
    //             maxContour = contours[i];
    //         }
    //     }
    //     cv::Rect maxRect = cv::boundingRect(maxContour);

    //     // auto object =  maxRect & Rect (0,0,depth_mat.cols, depth_mat.rows );
    //     auto object = maxRect;
    //     auto moment = cv::moments(maxContour, true);

    //     Scalar depth_m;
    //     if (moment.m00 == 0) {
    //         moment.m00 = 1;
    //     }
    //     Point moment_center(moment.m10 / moment.m00, moment.m01 / moment.m00);
    //     depth_m = Gdepth_mat.at<double>((int)moment.m01 / moment.m00, (int)moment.m10 / moment.m00);
    //     magic_distance = depth_m[0] * 1.042 *100;
    //     // magic_distance = kalman_filter_dir.update(magic_distance);
    //     velocity = sqrt(y_vel*y_vel + x_vel*x_vel);

    //     // calculate length to midline
    //     length_to_mid = (moment.m10 / moment.m00 - 200)*depth_length_coefficient(magic_distance) / 320;
    //     last_x_meter = abs(length_to_mid);
    //     cout << "length_to_mid " << length_to_mid <<endl;

            
    //     if(pixal_to_bottom != 480){
    //         if (length_to_mid < -2) {
                
    //                 ZActionModule::instance()->sendPacket(2, 0, -10, 0, true);
                
    //         }
    //         else if (length_to_mid > 2) {
                
    //                 ZActionModule::instance()->sendPacket(2, 0, 10, 0, true);
                
    //         }
    //         else {
    //             ZActionModule::instance()->sendPacket(2, 0, 0, 0, true);
    //         }
    //     }
    //     else{
    //         ZActionModule::instance()->sendPacket(2, 0, 0, 0, true);
    //                 std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     }
        
        

    // }*/
   
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

