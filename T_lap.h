#ifndef T_LAP_H
#define T_LAP_H

#include <opencv2\opencv.hpp>
#include <iostream>
#include "T_Common.h"


using namespace std;
using namespace cv;

class Lap
{
public:
	Lap();
	~Lap();

	static Point laser1_Extract(Mat& img);
	static Point laser2_Extract(Mat& img);

private:
	string savePath;
};

#endif	// T_LAP_H
