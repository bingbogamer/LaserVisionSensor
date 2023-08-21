#include"T_Lap.h"

Lap::Lap()
{
}

Lap::~Lap()
{

}

// 需要提供信息：laser1_OnWorkpiece，而SingleBevel类型 还需要 edgeP1、startP1
Point Lap::laser1_Extract(Mat& img)
{
	// cout << "********************** Lap :: laser1_OnWorkpiece 特征点P1提取 **********************" << endl;
	vector<Mat> mv; split(img, mv); Mat red = mv[2];

	Mat threshold, laser1_threshold;
	cv::threshold(red, threshold, 190, 255, THRESH_BINARY);
	medianBlur(threshold, laser1_threshold, 3);

	imshow("laser1_threshold", laser1_threshold);
	// waitKey(0);
	imwrite("F:\\MVS_Data\\laser1_threshold.png", laser1_threshold);

	int height = laser1_threshold.rows;
	int width = laser1_threshold.cols;
	int channels = laser1_threshold.channels();
	// printf("height=%d width=%d channels=%d \n", height, width, channels);


	// 将上面的重叠光条去掉
	for (int col = 0; col < width; ++col) {	// 遍历每一列
		int n = 0;
		for (int row = height - 1; row > 0; --row) {	// 反向遍历每一行()
			int diff = laser1_threshold.at<uchar>(row - 1, col) - laser1_threshold.at<uchar>(row, col);
			if (diff > 0)
				n += 1;
			if (n > 1 && laser1_threshold.at<uchar>(row - 1, col) != 0)
				laser1_threshold.at<uchar>(row - 1, col) = 0;
		}
	}
	imshow("remove", laser1_threshold);
	waitKey(0);

	// 求laser1光条的下间断点行坐标
	vector<int> lightRowIdx;
	WeldCom::findLight(laser1_threshold, lightRowIdx, "row");

	// 最大值在的索引
	vector<int>::iterator it;
	int max = 0, row_gapDn = -1;
	for (it = lightRowIdx.begin(); it != lightRowIdx.end(); it++) {
		int diff;
		if (it == lightRowIdx.begin())
			diff = *it - *(lightRowIdx.end() - 1);	// 首元素-最后一个元素
		else
			diff = *it - *(it - 1);	// 后面元素减去前面元素
		if (diff >= max) {
			max = diff;
			row_gapDn = *it;
		}
	}

	// 计算这一行的特征点坐标
	int n = 0, sumRow = 0;
	for (int col = 0; col < width; col++) {
		if (laser1_threshold.at<uchar>(row_gapDn, col) != 0) {
			n += 1;
			sumRow += (col + 1);
		}
	}
	int indexCol = sumRow / n - 1;
	Point P1(indexCol, row_gapDn);
	cout << "laser1_OnWorkpiece的焊缝特征点P1：" << P1 << endl;

	circle(img, P1, 1, Scalar(255, 0, 255), -1, 8);
	circle(img, P1, 5, Scalar(255, 0, 255), 1, 8);

	imshow("P1_L1img", img);
	imwrite("F:\\MVS_Data\\P1_L1img.png", img);
	return P1;
}

Point Lap::laser2_Extract(Mat& img)
{
	// cout << "********************** Lap :: laser2_OnWorkpiece 特征点P2提取 **********************" << endl;
	vector<Mat> mv; split(img, mv); Mat red = mv[2];

	Mat threshold, laser2_threshold;
	cv::threshold(red, threshold, 190, 255, THRESH_BINARY);
	medianBlur(threshold, laser2_threshold, 3);

	imshow("laser2_threshold", laser2_threshold);
	waitKey(0);
	imwrite("F:\\MVS_Data\\laser2_threshold.jpg", laser2_threshold);

	int height = laser2_threshold.rows;
	int width = laser2_threshold.cols;
	int channels = laser2_threshold.channels();


	// 求laser2光条的左间断点列坐标
	vector<int> lightColIdx;	// 记录有光条的行索引
	WeldCom::findLight(laser2_threshold, lightColIdx, "col");

	// 最大值在的索引
	vector<int>::iterator it;
	int max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// 首元素-最后一个元素
		else
			diff = *(it + 1) - *it;	// 后面元素减去前面元素
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	//再将间断点右边的 光条排除
	// ROI  左上角坐标、宽度和高度()
	Rect roiRange(0, 0, col_gapLeft + 1, height);	// 刚好截止到 左间断点
	Mat roi = laser2_threshold(roiRange);	// 注意：截取的图像改变的话,是会改变载入的原图像src（内存数据，不是磁盘文件）的, 因为两者指向的是同一个内存对象
	//Mat sub = src(roi).clone();
	imshow("roi", roi);

	height = roi.rows;
	width = roi.cols;
	// 找到光条最上点的行索引
	int rowIdx = -1;
	uchar* p;
	for (int row = 0; row < height; ++row) {
		p = roi.ptr<uchar>(row);   // 获取每一行开始处的指针，即每一行数组的首地址
		for (int col = 0; col < width; ++col) {
			if (p[col] != 0) {
				rowIdx = row;
				row = height;	// 退出外层循环
				break;
			}
		}
	}

	// 计算这一行的特征点坐标
	int n = 0, sumCol = 0;
	for (int col = 0; col < width; col++) {
		if (roi.at<uchar>(rowIdx, col) != 0) {
			n += 1;
			sumCol += (col + 1);
		}
	}
	int indexCol = sumCol / n - 1;

	Point P2(indexCol, rowIdx);
	cout << "laser2_OnWorkpiece的焊缝特征点P2：" << P2 << endl;

	circle(img, P2, 1, Scalar(255, 0, 255), -1, 8);
	circle(img, P2, 5, Scalar(255, 0, 255), 1, 8);


	imshow("P2_L2img", img);
	imwrite("F:\\MVS_Data\\P1_L2img.png", img);
	return P2;
}