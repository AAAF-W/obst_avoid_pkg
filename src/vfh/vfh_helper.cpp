#define CV_WIN

#include <vfh_helper.h>
#include <string.h>
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>

#define MAP_WIDTH 600
#define MAP_HEIGHT 400

#ifdef CV_WIN
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
cv::Mat vfh_img( MAP_HEIGHT, MAP_WIDTH, CV_8UC3, Scalar(0,0,0) );
#endif

static float lidar_ranges[180];
static float vfh_th = 1.5;

void InitHelper()
{
	for(int i=0;i<180;i++)
	{
		lidar_ranges[i] = i*0.02;
	}
	
#ifdef CV_WIN
	cv::namedWindow("vfh",CV_WINDOW_NORMAL);
	cvResizeWindow("vfh",MAP_WIDTH, MAP_HEIGHT); 
#endif
}

void SetRanges(float* inData)
{
	
	for(int i=0;i<180;i++)
	{
    	lidar_ranges[i] = inData[90+i];
		// printf("lidar_ranges[%d] = %.2f\n",i,lidar_ranges[i]);
	}
	ShowVFH();
}

void SetObjectTh(float inTh)
{
	vfh_th = inTh;
}

float CalTargetAngle(float inX, float inY)
{
	float retAngle = 0;
	if(inY == 0 )
	{
		if(inX >= 0)
			retAngle = M_PI/2;
		else
			retAngle = M_PI*3/2;
		return retAngle;
	}
	if(inY >0)
		retAngle = M_PI-atan(inX/inY);
	else
		retAngle = -atan(inX/inY);
	return retAngle;
}

void ShowVFH()
{
#ifdef CV_WIN
	vfh_img = {Scalar(0,0,0)};
	Scalar lidar_clr = Scalar(255, 255, 0);
	int y_org = MAP_HEIGHT-30;
	int x_org = MAP_WIDTH - 40;
	for(int i=0;i<180;i++)
	{
		int nHeight = lidar_ranges[i]*100;
		int x = x_org - i*3;
		line(vfh_img, Point(x,y_org),Point(x,y_org-nHeight), lidar_clr , 1 ,8);
	}
	// 坐标轴
	Scalar axis_clr = Scalar(155, 155, 155);
	line(vfh_img, Point(x_org,y_org+1),Point(0,y_org+1), axis_clr , 1 ,8);
	line(vfh_img, Point(x_org,y_org+1),Point(x_org,0), axis_clr , 1 ,8);

	// 刻度
	int txt_x = x_org -15;
	int txt_y = y_org +20;
	int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 1;
	
	putText(vfh_img, "0", Point(txt_x,txt_y), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
	putText(vfh_img, "90", Point(txt_x - 90*3,txt_y), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
	putText(vfh_img, "180", Point(txt_x - 180*3,txt_y), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
	
	putText(vfh_img, "0.0", Point(x_org+3,y_org), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
	putText(vfh_img, "1.0", Point(x_org+3,y_org - 100), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
	putText(vfh_img, "2.0", Point(x_org+3,y_org - 200), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
	putText(vfh_img, "3.0", Point(x_org+3,y_org - 300), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

	// 阈值线
	int th_y = y_org-(vfh_th * 100);
	Scalar th_clr = Scalar(0, 0, 250);
	line(vfh_img, Point(x_org,th_y),Point(0,th_y), th_clr , 1 ,8);

	cv::imshow("vfh", vfh_img);
    cv::waitKey(1);
#endif
}
