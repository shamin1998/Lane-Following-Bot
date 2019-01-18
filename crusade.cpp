#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include<bits/stdc++.h> 

using namespace std;
using namespace cv;

int blur_thresh = 11;
int canny_thresh = 13;
Mat src = imread("Lane.jpg",0);

typedef struct
{
	float m, b;
}hough_line;

void GaussianThreshold(int, void*)
{
	Mat b(src.rows, src.cols, CV_8UC1, Scalar(0));
	Size ksize;
	ksize.width = 23;
	ksize.height = ksize.width;
	GaussianBlur(src, b, ksize, blur_thresh);
	imshow("Gaussian Blur",b);
	waitKey(10);
	return;
}

void CannyThreshold(int, void*)
{
	Mat c(src.rows, src.cols, CV_8UC1, Scalar(0));

	Canny(src, c, canny_thresh, canny_thresh*3, 3);
	imshow("Canny", c);
	waitKey(10);
	return;
}

vector<hough_line> hough_transform(vector<Vec4i> lines, Mat hough)
{
	vector<hough_line> houghedlines;
	for( size_t i = 0; i < lines.size(); i++ )
    {
      float x1=lines[i][1], y1=hough.rows-lines[i][0], x2=lines[i][3], y2=hough.rows-lines[i][2];
      hough_line h_l;
      if(x1 == x2) h_l.m = INT_MAX;
      else h_l.m = (y2 - y1)/(x2 - x1);
      //y-y1 = m*(x-x1) b = y1-m*x1
      h_l.b = y1 - h_l.m*x1;
      houghedlines.push_back(h_l);
    }
    return houghedlines;
}

int main()
{
	
	imshow("Lane",src);
	while(waitKey(10)!=27){}

	// namedWindow("Gaussian Blur", WINDOW_AUTOSIZE);
	// createTrackbar("Gaussian_Blur", "Gaussian Blur", &blur_thresh, 50, GaussianThreshold);

	// GaussianThreshold(0,0);
	// waitKey(0);

	Mat blurred(src.rows, src.cols, CV_8UC1, Scalar(0));
	Size ksize;
	ksize.width = 2*blur_thresh + 1;
	ksize.height = ksize.width;
	GaussianBlur(src, blurred, ksize, 7);
	imshow("Gaussian Blur",blurred);
	while(waitKey(10)!=27){}

	// a = b.clone();

	// namedWindow("Canny",WINDOW_AUTOSIZE);
	// createTrackbar("Canny_Threshold", "Canny", &canny_thresh, 50, CannyThreshold);
	// CannyThreshold(0,0);
	// waitKey(0);

	Mat canny(blurred.rows, blurred.cols, CV_8UC1, Scalar(0));
	Canny(blurred, canny, canny_thresh, canny_thresh*3, 3);
	imshow("Canny", canny);
	while(waitKey(10)!=27){}	

	Mat hough(canny.rows, canny.cols, CV_8UC1, Scalar(0));
	vector<Vec4i> lines;
	HoughLinesP(canny, lines, 1, CV_PI/180, 50, 50, 10);
    printf("%d\n",(int)lines.size());
    size_t i;
    for(i=0; i<lines.size(); i++)
    {
      Vec4i l = lines[i];
      line(hough, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255), 3, CV_AA);
    }

    imshow("Hough",hough);
	while(waitKey(10)!=27){}

	vector<hough_line> houghedlines = hough_transform(lines, hough);    
	for(i=0; i<houghedlines.size(); i++)
		printf("line %d: m=%f, b=%f\n", (int)i, houghedlines[i].m*180/3.14159265, houghedlines[i].b);

	int n_positive=0, n_negative=0;
	hough_line positive_line, negative_line;
	positive_line.m = 0;
	positive_line.b = 0;
	negative_line.m = 0;
	negative_line.b = 0;
	for(i=0; i<houghedlines.size(); i++)
	{
		if(houghedlines[i].m >= 0)
		{
			positive_line.m += houghedlines[i].m;
			positive_line.b += houghedlines[i].b;
			n_positive++;
		}

		else
		{
			negative_line.m += houghedlines[i].m;
			negative_line.b += houghedlines[i].b;	
			n_negative++;
		}
	}

	if(n_positive==0 || n_negative==0) {printf("error\n"); return 0;}

	positive_line.m /= n_positive;
	positive_line.b /= n_positive;
	negative_line.m /= n_negative;
	negative_line.b /= n_negative;

	Mat lane(hough.rows, hough.cols, CV_8UC1, Scalar(0));
	line(lane, Point(0,(lane.rows-positive_line.b)/positive_line.m), Point(lane.rows-positive_line.b,0), Scalar(255), 3, CV_AA);
	line(lane, Point(0,(lane.rows-negative_line.b)/negative_line.m), Point(lane.rows-(negative_line.m*lane.cols+negative_line.b),lane.cols), Scalar(255), 3, CV_AA);

	imshow("Final Lane", lane);
	while(waitKey(10)!=27){}
		
	return 0;

}