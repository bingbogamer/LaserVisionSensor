#include"T_Lap.h"

Lap::Lap()
{
}

Lap::~Lap()
{

}

// ��Ҫ�ṩ��Ϣ��laser1_OnWorkpiece����SingleBevel���� ����Ҫ edgeP1��startP1
Point Lap::laser1_Extract(Mat& img)
{
	// cout << "********************** Lap :: laser1_OnWorkpiece ������P1��ȡ **********************" << endl;
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


	// ��������ص�����ȥ��
	for (int col = 0; col < width; ++col) {	// ����ÿһ��
		int n = 0;
		for (int row = height - 1; row > 0; --row) {	// �������ÿһ��()
			int diff = laser1_threshold.at<uchar>(row - 1, col) - laser1_threshold.at<uchar>(row, col);
			if (diff > 0)
				n += 1;
			if (n > 1 && laser1_threshold.at<uchar>(row - 1, col) != 0)
				laser1_threshold.at<uchar>(row - 1, col) = 0;
		}
	}
	imshow("remove", laser1_threshold);
	waitKey(0);

	// ��laser1�������¼�ϵ�������
	vector<int> lightRowIdx;
	WeldCom::findLight(laser1_threshold, lightRowIdx, "row");

	// ���ֵ�ڵ�����
	vector<int>::iterator it;
	int max = 0, row_gapDn = -1;
	for (it = lightRowIdx.begin(); it != lightRowIdx.end(); it++) {
		int diff;
		if (it == lightRowIdx.begin())
			diff = *it - *(lightRowIdx.end() - 1);	// ��Ԫ��-���һ��Ԫ��
		else
			diff = *it - *(it - 1);	// ����Ԫ�ؼ�ȥǰ��Ԫ��
		if (diff >= max) {
			max = diff;
			row_gapDn = *it;
		}
	}

	// ������һ�е�����������
	int n = 0, sumRow = 0;
	for (int col = 0; col < width; col++) {
		if (laser1_threshold.at<uchar>(row_gapDn, col) != 0) {
			n += 1;
			sumRow += (col + 1);
		}
	}
	int indexCol = sumRow / n - 1;
	Point P1(indexCol, row_gapDn);
	cout << "laser1_OnWorkpiece�ĺ���������P1��" << P1 << endl;

	circle(img, P1, 1, Scalar(255, 0, 255), -1, 8);
	circle(img, P1, 5, Scalar(255, 0, 255), 1, 8);

	imshow("P1_L1img", img);
	imwrite("F:\\MVS_Data\\P1_L1img.png", img);
	return P1;
}

Point Lap::laser2_Extract(Mat& img)
{
	// cout << "********************** Lap :: laser2_OnWorkpiece ������P2��ȡ **********************" << endl;
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


	// ��laser2���������ϵ�������
	vector<int> lightColIdx;	// ��¼�й�����������
	WeldCom::findLight(laser2_threshold, lightColIdx, "col");

	// ���ֵ�ڵ�����
	vector<int>::iterator it;
	int max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// ��Ԫ��-���һ��Ԫ��
		else
			diff = *(it + 1) - *it;	// ����Ԫ�ؼ�ȥǰ��Ԫ��
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	//�ٽ���ϵ��ұߵ� �����ų�
	// ROI  ���Ͻ����ꡢ��Ⱥ͸߶�()
	Rect roiRange(0, 0, col_gapLeft + 1, height);	// �պý�ֹ�� ���ϵ�
	Mat roi = laser2_threshold(roiRange);	// ע�⣺��ȡ��ͼ��ı�Ļ�,�ǻ�ı������ԭͼ��src���ڴ����ݣ����Ǵ����ļ�����, ��Ϊ����ָ�����ͬһ���ڴ����
	//Mat sub = src(roi).clone();
	imshow("roi", roi);

	height = roi.rows;
	width = roi.cols;
	// �ҵ��������ϵ��������
	int rowIdx = -1;
	uchar* p;
	for (int row = 0; row < height; ++row) {
		p = roi.ptr<uchar>(row);   // ��ȡÿһ�п�ʼ����ָ�룬��ÿһ��������׵�ַ
		for (int col = 0; col < width; ++col) {
			if (p[col] != 0) {
				rowIdx = row;
				row = height;	// �˳����ѭ��
				break;
			}
		}
	}

	// ������һ�е�����������
	int n = 0, sumCol = 0;
	for (int col = 0; col < width; col++) {
		if (roi.at<uchar>(rowIdx, col) != 0) {
			n += 1;
			sumCol += (col + 1);
		}
	}
	int indexCol = sumCol / n - 1;

	Point P2(indexCol, rowIdx);
	cout << "laser2_OnWorkpiece�ĺ���������P2��" << P2 << endl;

	circle(img, P2, 1, Scalar(255, 0, 255), -1, 8);
	circle(img, P2, 5, Scalar(255, 0, 255), 1, 8);


	imshow("P2_L2img", img);
	imwrite("F:\\MVS_Data\\P1_L2img.png", img);
	return P2;
}