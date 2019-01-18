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
int canny_thresh = 10;
int hough_thresh = 72;
Mat src = imread("Lane.jpg",0);

typedef struct
{
	float m, b;
}hough_line;

void GaussianThreshold(int, void*)
{
	Mat b(src.rows, src.cols, CV_8UC1, Scalar(0));		//to threshold Gaussian Blur
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

	Canny(src, c, canny_thresh, canny_thresh*3, 3);		//to threshold Canny Edge Detector
	imshow("Canny", c);
	waitKey(10);
	return;
}

vector<hough_line> hough_transform(vector<Vec4i> lines, Mat hough)		//to convert lines from 2-point form to y=m*x+b form
{																		//where origin is at bottom left of image
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

void HoughThreshold(int, void *)
{
	Mat h(src.rows, src.cols, CV_8UC1, Scalar(0));
	vector<Vec4i> lines;
	HoughLinesP(src, lines, 1, CV_PI/180, hough_thresh, 5, 500);
	size_t i;
    for(i=0; i<lines.size(); i++)
    {
      Vec4i l = lines[i];
      line(h, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255), 3, CV_AA);
    }
    imshow("Hough",h);
    waitKey(10);
    return;
}

int main()
{
	
	imshow("Lane",src);
	while(waitKey(10)!=27){}

	// namedWindow("Gaussian Blur", WINDOW_AUTOSIZE);
	// createTrackbar("Gaussian_Blur", "Gaussian Blur", &blur_thresh, 50, GaussianThreshold);

	// GaussianThreshold(0,0);
	// waitKey(0);

	Mat blurred(src.rows, src.cols, CV_8UC1, Scalar(0));		//blur image
	Size ksize;
	ksize.width = 2*blur_thresh + 1;
	ksize.height = ksize.width;
	GaussianBlur(src, blurred, ksize, 7);
	imshow("Gaussian Blur",blurred);
	while(waitKey(10)!=27){}


	// src = blurred.clone();

	// namedWindow("Canny",WINDOW_AUTOSIZE);
	// createTrackbar("Canny_Threshold", "Canny", &canny_thresh, 50, CannyThreshold);
	// CannyThreshold(0,0);
	// waitKey(0);
	// return 0;


	Mat canny(blurred.rows, blurred.cols, CV_8UC1, Scalar(0));		//apply canny edge setector
	Canny(blurred, canny, canny_thresh, canny_thresh*3, 3);
	imshow("Canny", canny);
	while(waitKey(10)!=27){}

	// src = canny.clone();
	// namedWindow("Hough",WINDOW_AUTOSIZE);
	// createTrackbar("Hough_Threshold", "Hough", &hough_thresh, 200, HoughThreshold);
	// HoughThreshold(0,0);
	// waitKey(0);
	// return 0;

	Mat hough(canny.rows, canny.cols, CV_8UC1, Scalar(0));		//apply hough line detector
	vector<Vec4i> lines;
	HoughLinesP(canny, lines, 1, CV_PI/180, hough_thresh, 5, 500);
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

	int n_positive=0, n_negative=0, n_top=0;
	map<int, pair<hough_line,int> > positive_line, negative_line, top_line;		//average out all lines with positive slope to one line, and negative slope to another
	
	for(i=0; i<houghedlines.size(); i++)
	{
		if(houghedlines[i].m >= tan(15*M_PI/180))
		{
			int slope=(int)(atan(houghedlines[i].m/3));
			if(positive_line.find(slope) != positive_line.end())
			{
				positive_line[slope].first.m += houghedlines[i].m;
				positive_line[slope].first.b += houghedlines[i].b;
				positive_line[slope].second++;
			}
			else
			{
				positive_line[slope].first = houghedlines[i];
				positive_line[slope].second = 1;				
			}
			n_positive++;
		}

		else if(houghedlines[i].m <= tan(-15*M_PI/180))
		{
			int slope=(int)(atan(houghedlines[i].m/3));
			if(negative_line.find(slope) != negative_line.end())
			{
				negative_line[slope].first.m += houghedlines[i].m;
				negative_line[slope].first.b += houghedlines[i].b;
				negative_line[slope].second++;
			}
			else
			{
				negative_line[slope].first = houghedlines[i];
				negative_line[slope].second = 1;				
			}
			n_negative++;
		}

		else
		{
			int slope=(int)(atan(houghedlines[i].m/3));
			if(top_line.find(slope) != top_line.end())
			{
				top_line[slope].first.m += houghedlines[i].m;
				top_line[slope].first.b += houghedlines[i].b;
				top_line[slope].second++;
			}
			else
			{
				top_line[slope].first = houghedlines[i];
				top_line[slope].second = 1;				
			}
			n_top++;
		}
	}

	// if(n_positive==0 || n_negative==0) {printf("error\n"); return 0;}

	//Positive
	int te_sl,te_cn=0;
	for(auto it:positive_line)
	{
		if(te_cn > it.second.second)
		{
			te_cn=it.second.second;
			te_sl=it.first;
		}
	}
	hough_line pos_line;
	pos_line.m = positive_line[te_sl].first.m/positive_line[te_sl].second;
	pos_line.b = positive_line[te_sl].first.b/positive_line[te_sl].second; 
	
	// Negative
	te_cn=0;
	for(auto it:negative_line)
	{
		if(te_cn > it.second.second)
		{
			te_cn=it.second.second;
			te_sl=it.first;
		}
	}
	hough_line neg_line;
	neg_line.m = negative_line[te_sl].first.m/negative_line[te_sl].second;
	neg_line.b = negative_line[te_sl].first.b/negative_line[te_sl].second; 

	// Top
	te_cn=0;
	for(auto it:top_line)
	{
		if(te_cn > it.second.second)
		{
			te_cn=it.second.second;
			te_sl=it.first;
		}
	}
	hough_line to_line;
	to_line.m = top_line[te_sl].first.m/top_line[te_sl].second;
	to_line.b = top_line[te_sl].first.b/top_line[te_sl].second; 

	Mat lane(hough.rows, hough.cols, CV_8UC1, Scalar(0));
	line(lane, Point(0,(lane.rows-pos_line.b)/pos_line.m), Point(lane.rows-pos_line.b,0), Scalar(255), 3, CV_AA);
	line(lane, Point(0,(lane.rows-neg_line.b)/neg_line.m), Point(lane.rows-(neg_line.m*lane.cols+neg_line.b),lane.cols), Scalar(255), 3, CV_AA);

	imshow("Final Lane", lane);
	while(waitKey(10)!=27){}
		
	return 0;

}