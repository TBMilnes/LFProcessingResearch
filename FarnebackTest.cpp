#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

void help()
{
	cout <<
			"\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
			"Mainly the function: calcOpticalFlowFarneback()\n"
			"Call:\n"
			"./fback\n"
			"This reads from video camera 0\n" << endl;
}
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
					double, const Scalar& color)
{
	for(int y = 0; y < cflowmap.rows; y += step)
		for(int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);
			line(cflowmap, Point(x,y), Point(cvRound(x+5*fxy.x), cvRound(y+5*fxy.y)),
				 color);
			circle(cflowmap, Point(x,y), 1, color, -1);
		}
}

int main(int, char**)
{
//	VideoCapture cap(0);
//	help();
//	if( !cap.isOpened() )
//		return -1;
		
	// Declare variables
	Mat flow, cflow, view1, view2;
	namedWindow("flow", 1);

	// Read image data
	view1 = imread("LightField5_5.jpg", 1);
	view2 = imread("LightField5_6.jpg", 1);

	// Test pixel access
	Vec3b pixelColor = view1.at<Vec3b>(250,200);
	printf("%i %i %i\n",pixelColor.val[2],pixelColor.val[1],pixelColor.val[0]);
	//printf("%f %f %f\n",view1.at<float>(200,200,2),view1.at<float>(200,200,1),view1.at<float>(200,200,3));

	cvtColor(view1, view1, CV_BGR2GRAY);
	cvtColor(view2, view2, CV_BGR2GRAY);

	// Calculate and display flow
	calcOpticalFlowFarneback(view1, view2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	cvtColor(view1, cflow, CV_GRAY2BGR);
	drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
	imshow("flow", cflow);
	int key = waitKey(0);
	

//	for(;;)
//	{
//		cap >> frame;
//		cvtColor(frame, gray, CV_BGR2GRAY);
//		
//		if( prevgray.data )
//		{
//			calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
//			cvtColor(prevgray, cflow, CV_GRAY2BGR);
//			drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
//			imshow("flow", cflow);
//		}
//		if(waitKey(30)>=0)
//			break;
//		std::swap(prevgray, gray);
//	}
	return 0;
}
