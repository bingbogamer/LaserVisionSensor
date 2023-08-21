#include "T_Common.h"


WeldCom::WeldCom(Mat& src)
{
	CrossPointlocation_step2(src, 6);			/* ��ȷ��λ����㣬��ʼ�� crossPoint��startP1��startP2��Ա*/
	edgePointlocation(src);						/* ����Ѱ�ұ�Ե�㣬��ʼ�� edgeP1��edgeP2��Ա*/
	regionSegment(src);							/* ��ʼ��regionROI��laser1��laser2*/

	typeArray();								/* ʶ�𺸷����ͣ���ʼ�� wledType��Ա */
}

WeldCom::~WeldCom()
{

}

Mat WeldCom::Multidiagonal(int lw = 6) {
	Mat dia = Mat::eye(lw * 2 + 1, lw * 2 + 1, CV_8UC1);
	for (int row = 0, col = 0; row < lw * 2 + 1 && col < lw * 2 + 1; row++, col++) {
		for (int k = 1; k <= lw / 2 - 1; k++) {
			if (col + k < lw * 2 + 1) 				// �ұ߶Խ�
				dia.at<uchar>(row, col + k) = 1;
			if (0 <= col - k) 						// ��߶Խ�
				dia.at<uchar>(row, col - k) = 1;
		}
	}
	// cout << "�޸� dia = " << endl << " " << dia << endl << endl;
	Mat dst;
	flip(dia, dst, 1);
	// cout << "y�ᷭת dia = " << endl << " " << dst << endl << endl;
	Mat kernel = dia + dst;
	// cout << " kernel = " << endl << " " << kernel << endl << endl;
	return kernel;
}

// ���������ڵ����������� �������ڵ����������飬���������飩
// �жϵ�ǰ����ͼ��  �Ƿ���ڹ���
bool WeldCom::findLight(Mat& binary, vector<int>& lightIdx, string mode = "row") {
	lightIdx.clear();	// �����
	int height = binary.rows, width = binary.cols;
	if (mode == "row") {
		uchar* p;
		for (int row = 0; row < height; ++row) {
			p = binary.ptr<uchar>(row);   // ��ȡÿһ�п�ʼ����ָ�룬��ÿһ��������׵�ַ
			for (int col = 0; col < width; ++col) {
				if (p[col] != 0) {
					lightIdx.push_back(row);
					break;
				}
			}
		}
	}
	else if (mode == "col") {
		for (int col = 0; col < width; ++col) {
			for (int row = 0; row < height; ++row) {
				if (binary.at<uchar>(row, col) != 0) {
					lightIdx.push_back(col);
					break;
				}
			}
		}
	}
	if (lightIdx.size() != 0)
		return true;
	else
		return false;
}

// ��ֱ�Ҷ����ķ�
// �������RGB ��ȡͼ��
// upperRowIndex �� �����ϱ߽���ͼ���е�������
vector<Point>	WeldCom::wds_Roi(Mat& roi, int upperRowIndex) {
	int height = roi.rows, width = roi.cols, channels = roi.channels();
	vector<Mat> mv;	split(roi, mv);	Mat red = mv[2];
	Mat thrRed;
	cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
	// �����飬��¼���ĵ�������б�
	vector<Point>	gravityCoorList;
	// ����ͼ����������
	for (int col = 0; col < width; ++col) {
		int n = 0, sum_RowIdx = 0;
		for (int row = 0; row < height; ++row) {
			if (thrRed.at<uchar>(row, col) != 0) {
				n += 1;
				sum_RowIdx += row + 1;
			}
		}
		if (n > 0) {
			int gravityRow = int(round(sum_RowIdx / n)) - 1;
			int realRow = upperRowIndex + gravityRow;
			Point p(col, realRow);	// (x, y) �У���
			gravityCoorList.push_back(p);
		}
	}
	return gravityCoorList;
}

/* ����ָ� */
void WeldCom::regionSegment(Mat& img) {
	// �����䣬������ ������ҿ���
	Mat LeftUpRoi = img(Range(0, crossPoint.y), Range(0, crossPoint.x));						/* roi1*/
	Mat RightUpRoi = img(Range(0, crossPoint.y), Range(crossPoint.x, edgeP2.x + 1));			/* roi2*/
	Mat LeftDownRoi = img(Range(crossPoint.y, img.rows), Range(0, crossPoint.x));				/* roi3*/
	Mat RightDownRoi = img(Range(crossPoint.y, img.rows), Range(crossPoint.x, edgeP1.x + 1));	/* roi4*/

	// imwrite("F:\\MVS_Data\\roi1.png", LeftUpRoi);
	// imwrite("F:\\MVS_Data\\roi2.png", RightUpRoi);
	// imwrite("F:\\MVS_Data\\roi3.png", LeftDownRoi);
	// imwrite("F:\\MVS_Data\\roi4.png", RightDownRoi);

	regionROI.push_back(LeftUpRoi);		/* roi1*/
	regionROI.push_back(RightUpRoi);	/* roi2*/
	regionROI.push_back(RightDownRoi);	/* roi3*/
	regionROI.push_back(LeftDownRoi);	/* roi4*/


	laser1 = Mat::zeros(img.rows, img.cols, img.type());
	Mat RightUpRoi_All = img(Range(0, crossPoint.y), Range(crossPoint.x, img.cols));			/* roi2	�й���̨*/
	Mat RightDownRoi_All = img(Range(crossPoint.y, img.rows), Range(crossPoint.x, img.cols));	/* roi4	�й���̨	*/
	//���û����������򲢸���
	Rect roi1_rect = cv::Rect(0, 0, LeftUpRoi.cols, LeftUpRoi.rows);	// �У��У�����
	LeftUpRoi.copyTo(laser1(roi1_rect));
	Rect roi3_rect = cv::Rect(crossPoint.x, crossPoint.y, RightDownRoi_All.cols, RightDownRoi_All.rows);	// ��x����y������
	RightDownRoi_All.copyTo(laser1(roi3_rect));

	laser2 = Mat::zeros(img.rows, img.cols, img.type());
	//���û����������򲢸���
	Rect roi2_rect = cv::Rect(crossPoint.x, 0, RightUpRoi_All.cols, RightUpRoi_All.rows);	// �У��У�����
	RightUpRoi_All.copyTo(laser2(roi2_rect));
	Rect roi4_rect = cv::Rect(0, crossPoint.y, LeftDownRoi.cols, LeftDownRoi.rows);	// ��x����y������
	LeftDownRoi.copyTo(laser2(roi4_rect));

	// imshow("laser1", laser1);
	// imshow("laser2", laser2);
	imwrite("F:\\MVS_Data\\laser1.png", laser1);
	imwrite("F:\\MVS_Data\\laser2.png", laser2);
	waitKey(0);
	destroyAllWindows();

	/* ��ʼ�� laser1_OnWorkpiece��laser2_OnWorkpiece ���ݳ�Ա*/
	laser1_OnWorkpiece = laser1(Range(0, laser1.rows), Range(0, edgeP1.x));	// �����䣬������ ������ҿ���
	laser2_OnWorkpiece = laser2(Range(0, laser2.rows), Range(0, edgeP2.x));
	/*imshow("laser1_OnWorkpiece", laser1_OnWorkpiece);
	imshow("laser2_OnWorkpiece", laser2_OnWorkpiece);*/
	imwrite("F:\\MVS_Data\\laser1_OnWorkpiece.png", laser1_OnWorkpiece);
	imwrite("F:\\MVS_Data\\laser2_OnWorkpiece.png", laser2_OnWorkpiece);
	waitKey(0);
	destroyAllWindows();
}

/* ����������λ */
/* ���2����λ���ڵĽ����ߣ����㲢���� ���Խ����*/
Point WeldCom::CrossPointlocation_step1(Mat& img)
{
	// img ������ʾԭʼͼ��imgProcess�����������ᷢ���ı�
	Mat imgProcess = img.clone();
	Mat fitWdsUp, fitWdsDn;			// ���� ����ͷ

	int height = img.rows, width = img.cols, channels = img.channels();
	int h = 40, w = 20;				// �������ó� Ĭ�ϳ�Ա����
	// �����������ڵ��ϱ߽��������
	int rowIndexUp = 0;
	int rowIndexDn = height - h;
	// ���¶�λ���� ���ϱ߽���ȫͼ�е�������
	int wdsUp_row, wdsDn_row;

	// Laser1 �ϴ�������
	int wdsnum = 1;
	while (true) 
	{
		Mat wdsUp = img(Range(rowIndexUp, rowIndexUp + h), Range(0, w));	// �����䣬������ ������ҿ���
		// ���������ԽǶ���Ϊ Point(x, y) �� Point(x, y)	�ߴ�Ϊ -1, �˾��ν������
		rectangle(imgProcess, Point(0, rowIndexUp), Point(w - 1, rowIndexUp + h - 1), Scalar(0, 255, 0), 1);	// ��ͼ���ڲ� ����Ȧ

		vector<Mat> mv;	split(wdsUp, mv);	Mat red = mv[2];
		Mat threshold, thrRed;
		cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
		medianBlur(threshold, thrRed, 3);

		// �жϵ�ǰ����ͼ��  �Ƿ���ڹ���
		vector<int> lightRowIdx;	// ��¼�й�����������
		bool exist = findLight(thrRed, lightRowIdx, "row");
		// �����ڹ���
		if (exist) 
		{
			int rowUpIdx = lightRowIdx[0];			// ����������ĵ���roi�е�������
			int findRow = rowIndexUp + rowUpIdx;	// ����������ĵ���ȫͼ������
			cout << "����㶨λ����ǰ���ڼ�⵽������������������" << findRow << endl;
			// �϶�λ����fitWdsUp ���		��ͼ���ڲ�����Ȧ����20 ��60��19 + 1 + 40��
			rectangle(imgProcess, Point(0, findRow - 19), Point(w - 1, findRow + 40), Scalar(255, 0, 255), 1);
			fitWdsUp = img(Range(findRow - 19, findRow + 41), Range(0, w));
			wdsUp_row = findRow - 19;
			break;
		}
		else 
		{
			// printf("����㶨λ����ǰ����%dδ��⵽����\n", wdsnum);
			rowIndexUp += h;
			wdsnum += 1;
		}
	}

	// Laser2 �´�������
	wdsnum = 1;
	while (true) {
		Mat wdsDn = img(Range(rowIndexDn, rowIndexDn + h), Range(0, w));	// �����䣬������ ������ҿ���
		// ���������ԽǶ���Ϊ Point(x, y) �� Point(x, y)	�ߴ�Ϊ -1, �˾��ν������
		rectangle(imgProcess, Point(0, rowIndexDn), Point(w - 1, rowIndexDn + h - 1), Scalar(0, 255, 0), 1);	// ��ͼ���ڲ� ����Ȧ

		vector<Mat> mv;	split(wdsDn, mv);	Mat red = mv[2];
		Mat threshold, thrRed;
		cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
		medianBlur(threshold, thrRed, 3);

		// �жϵ�ǰ����ͼ��  �Ƿ���ڹ���
		vector<int> lightRowIdx;	// ��¼�й�����������
		bool exist = findLight(thrRed, lightRowIdx, "row");
		// �����ڹ���
		if (exist) 
		{
			int rowDnIdx = lightRowIdx.back();	// ����������ĵ���roi�е�������
			int findRow = rowIndexDn + rowDnIdx;	// ����������ĵ���ȫͼ������
			cout << "����㶨λ����ǰ���ڼ�⵽������������������" << findRow << endl;
			// �¶�λ����fitWdsDn ���		��ͼ���ڲ�����Ȧ����20 ��60��19 + 1 + 40��
			rectangle(imgProcess, Point(0, findRow - 40), Point(w - 1, findRow + 19), Scalar(255, 0, 255), 1);
			fitWdsDn = img(Range(findRow - 40, findRow + 20), Range(0, w));
			wdsDn_row = findRow - 40;
			break;
		}
		else 
		{
			// printf("����㶨λ����ǰ����%dδ��⵽����\n", wdsnum);
			rowIndexDn -= h;
			wdsnum += 1;
		}
	}
	//imshow("fitWdsUp", fitWdsUp);
	//imshow("fitWdsDn", fitWdsDn);

	// �Դ��ڴ�����ϳ�ֱ��б��
	// ��ֱcog���������ĵ�����
	vector<Point> gravityCoorUp = wds_Roi(fitWdsUp, wdsUp_row);
	vector<Point> gravityCoorDn = wds_Roi(fitWdsDn, wdsDn_row);

	//�����ĵ�1 ��� 
	Vec4f line1_para;	// (vx, vy, x0, y0)������(vx, vy)����ֱ�߹��ߵĹ�һ��������(x0, y0)��ֱ���ϵ�һ����
	cv::fitLine(gravityCoorUp, line1_para, cv::DIST_L2, 0, 1e-2, 1e-2);	// DIST_L2:��С���˷�
	cout << "line1_para = " << line1_para << endl;
	double k1 = line1_para[1] / line1_para[0];
	double b1 = line1_para[3] - k1 * line1_para[2]; //  A, B, C = k, -1, b

	Point start(gravityCoorUp[0].x, int(round(gravityCoorUp[0].x * k1 + b1)));
	Point end(int(width * 3 / 8), int(round((width * 3 / 8) * k1 + b1)));
	line(imgProcess, start, end, Scalar(255, 255, 0), 1, 8);
	startP1 = gravityCoorUp.back();	// ȡ���һ����, ��Ϊ֮��V�͹����� ��Ͼ���� �����ߵ���ʼ��

	//�����ĵ�2 ��� 
	Vec4f line2_para;
	cv::fitLine(gravityCoorDn, line2_para, cv::DIST_L2, 0, 1e-2, 1e-2);	// DIST_L2:��С���˷�
	cout << "line2_para = " << line2_para << endl;
	double k2 = line2_para[1] / line2_para[0];
	double b2 = line2_para[3] - k2 * line2_para[2]; //  A, B, C = k, -1, b

	Point start2(gravityCoorDn[0].x, int(round(gravityCoorDn[0].x * k2 + b2)));
	Point end2(int(width * 3 / 8), int(round((width * 3 / 8) * k2 + b2)));
	line(imgProcess, start2, end2, Scalar(255, 255, 0), 1, 8);
	startP2 = gravityCoorDn.back();	// ȡ���һ����, ��Ϊ֮��V�͹����� ��Ͼ���� �����ߵ���ʼ��

	// 3. ���㽻�����꣬�����ý����ROI
	Point cp;
	cp.x = int(round((b2 - b1) / (k1 - k2)));
	cp.y = int(round(k1 * cp.x + b1));

	circle(imgProcess, cp, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, cp, 5, Scalar(255, 0, 255), 1, 8);
	// ���ƾ�ȷ��λ����
	int roiH = 80, roiW = 160;
	rectangle(imgProcess, Point(cp.x - roiW / 2, cp.y - roiH / 2), Point(cp.x + roiW / 2, cp.y + roiH / 2), Scalar(0, 255, 0), 1);

	imshow("edgeSearch", imgProcess);
	imwrite("F:\\MVS_Data\\edgeSearch.png", imgProcess);

	return cp;
}

/* ����� ��ȷ��λ*/
void WeldCom::CrossPointlocation_step2(Mat& img, int lw = 6) {
	// img ������ȡԭʼͼ��û�л�ͼ����imgProcess�����������ᷢ���ı�
	Mat imgProcess = img.clone();
	Point cp = CrossPointlocation_step1(img);
	int roiH = 80, roiW = 160;
	// ��ȡ�����Ķ�λ����
	Mat roi_row = imgProcess(Range(cp.y - roiH / 2, cp.y + roiH / 2), Range(cp.x - roiW / 2, cp.x + roiW / 2));	// �����䣬������ ������ҿ���
	// imshow("roi", roi_row);
	rectangle(imgProcess, Point(cp.x - roiW / 2, cp.y - roiH / 2), Point(cp.x + roiW / 2, cp.y + roiH / 2), Scalar(0, 255, 0), 1);

	int height = roi_row.rows, width = roi_row.cols;
	vector<Mat> mv;	split(roi_row, mv);	Mat red = mv[2];
	Mat threshold, thrRed;
	cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
	medianBlur(threshold, thrRed, 3);
	imshow("roi_threshold", thrRed);
	imwrite("F:\\MVS_Data\\xxxxxxxxxxxx.png", thrRed);

	// ��������ˣ���ȷ��λCrossPoint
	Mat norm = thrRed / 255;	// ��һ����ͨ��ͼ��
	Mat kernel = Multidiagonal(lw);	// �����
	Mat dst;
	dst.create(thrRed.size(), thrRed.type());
	dst = Scalar(0);
	filter2D(norm, dst, norm.depth(), kernel);
	imshow("dst", dst);
	double minValue, maxValue;    // ���ֵ����Сֵ
	Point  minIdx, maxIdx;    // ��Сֵ���꣬���ֵ����

	cv::minMaxLoc(dst, &minValue, &maxValue, &minIdx, &maxIdx);

	cout << "CrossPoint��roi�еľ�ȷ����: " << maxIdx << endl;
	circle(roi_row, maxIdx, 1, Scalar(255, 0, 255), -1, 8);
	circle(roi_row, maxIdx, 5, Scalar(255, 0, 255), 1, 8);

	imshow("CrossPoint", roi_row);
	imwrite("F:\\MVS_Data\\CrossPoint.png", roi_row);

	crossPoint.x = cp.x - int(roiW / 2) + maxIdx.x;
	crossPoint.y = cp.y - int(roiH / 2) + maxIdx.y;

	cout << "CrossPoint��ȫͼ�еľ�ȷ����: " << crossPoint << endl;
	// regionSegment(img, CrossPoint);
}

void WeldCom::edgePointlocation(Mat& src) 
{
	cout << "------------��Ե���---------------" << endl;
	// src ������ȡԭʼͼ��û�л�ͼ����imgProcess�����������ᷢ���ı�
	Mat imgProcess = src.clone();
	// ��Ե���� �������ڵĴ�С
	int h = 20, w = 40;		// �������ó� Ĭ�ϳ�Ա���ԣ��ͽ���������Ĵ��ڴ�С��һ��

	int hight = imgProcess.rows, width = imgProcess.cols;
	int wdsnum = 1;
	Mat egdeWdsUp, egdeWdsDn;	// ��λ����
	int lightUpRight_inImg, lightDnRight_inImg;		// ���������ҵ���ȫͼ�е�������
	//1. �������ڵ���
	//��һ��������߽��������
	int edgeUpLeft = width - w;  //���Ǵ��ڵ���߽��������
	int lightWidth_Up = 0;
	while (true)
	{
		Mat wdsUp = imgProcess(Range(0, h), Range(edgeUpLeft, edgeUpLeft + w));
		rectangle(imgProcess, Point(edgeUpLeft, 0), Point(edgeUpLeft + w - 1, h - 1), Scalar(0, 255, 0), 1);
		vector<Mat> mv;	split(wdsUp, mv);	Mat red = mv[2];
		Mat thrRed;
		cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
		//�жϵ�ǰ����ͼ��  �Ƿ���ڹ���
		vector<int> lightColIdx;
		bool exist = findLight(thrRed, lightColIdx, "col");
		if (exist) 
		{
			int lightUpRight_inRoi = lightColIdx.back();	// ���һ�У��������ֵ
			lightUpRight_inImg = edgeUpLeft + lightUpRight_inRoi;	// ���������ҵ���ȫͼ�е�������
			cout << "��Ե���������ϴ��ڼ�⵽������������������" << lightUpRight_inImg << endl;
			// ��� ��λ���� ���� ��ͼ���ڲ�����Ȧ����60 ��20��
			rectangle(imgProcess, Point(lightUpRight_inImg - 40, 0), Point(lightUpRight_inImg + 19, 19), Scalar(255, 0, 255), 1);
			circle(imgProcess, Point(lightUpRight_inImg, 0), 1, Scalar(255, 0, 255), -1, 8);
			// ��λ���� ���ͼ���������wdsUp�� ��ͬ�ģ�
			egdeWdsUp = src(Range(0, 20), Range(lightUpRight_inImg - 40, lightUpRight_inImg + 20));
			// ���� ��λ���ڣ�������ȫ�Ĺ����Σ��� ������ȣ�Ĭ��û��������
			vector<Mat> mv;	split(egdeWdsUp, mv);	Mat red = mv[2];
			Mat thrRed;
			cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
			vector<int> lightColIdx;
			findLight(thrRed, lightColIdx, "col");
			lightWidth_Up = lightColIdx.back() - lightColIdx.front();	// ��λ�����ڵĹ��������������
			break;
		}
		edgeUpLeft -= w;
		wdsnum += 1;
	}
	wdsnum = 1;
	//��һ��������߽��������
	int edgeDnLeft = width - w;  //���д��ڵ�����ߵ�������
	int lightWidth_Dn = 0;
	while (true)
	{
		Mat wdsDn = imgProcess(Range(hight - h, hight), Range(edgeDnLeft, edgeDnLeft + w));
		rectangle(imgProcess, Point(edgeDnLeft, hight - h), Point(edgeDnLeft + w - 1, hight - 1), Scalar(0, 255, 0), 1);
		vector<Mat> mv;	split(wdsDn, mv);	Mat red = mv[2];
		Mat thrRed;
		cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
		//�жϵ�ǰ����ͼ��  �Ƿ���ڹ���
		vector<int> lightColIdx;
		bool exist = findLight(thrRed, lightColIdx, "col");
		if (exist) 
		{
			int lightDnRight_inRoi = lightColIdx.back();	// ���һ�У��������ֵ
			lightDnRight_inImg = edgeDnLeft + lightDnRight_inRoi; // edgeDnLeft�����б�Ե���ڵ�����ߵ�������
			cout << "��Ե���������´��ڼ�⵽������������������������" << lightDnRight_inImg << endl;
			// ��� ��λ���� ���� ��ͼ���ڲ�����Ȧ����60 ��20��
			rectangle(imgProcess, Point(lightDnRight_inImg - 40, hight - h), Point(lightDnRight_inImg + 19, hight - 1), Scalar(255, 0, 255), 1);
			circle(imgProcess, Point(lightDnRight_inImg, hight - 1), 1, Scalar(255, 0, 255), -1, 8);
			// ��λ���� ���ͼ���������wdsUp�� ��ͬ�ģ�
			egdeWdsDn = src(Range(hight - 20, hight), Range(lightDnRight_inImg - 40, lightDnRight_inImg + 20));
			// ���� ��λ���ڣ�������ȫ�Ĺ����Σ��� ������ȣ�Ĭ��û��������
			vector<Mat> mv;	split(egdeWdsDn, mv);	Mat red = mv[2];
			Mat thrRed;
			cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
			vector<int> lightColIdx;
			findLight(thrRed, lightColIdx, "col");
			lightWidth_Dn = lightColIdx.back() - lightColIdx.front();	// ��λ�����ڵĹ��������������
			break;
		}
		edgeDnLeft -= w;
		wdsnum += 1;
	}
	//���������һ�£������Ϸ�����ߵ���Ϊ��㣬���õ�������
	imshow("egdeWdsUp", egdeWdsUp);
	//Up_curWdsLeft_inImg :egdeWdsUp ��λ���ڵ� ���ȫͼ������
	int Up_curWdsLeft_inImg = lightUpRight_inImg - 40;	//  lightUpRight_inImg: ���������ҵ���ȫͼ�е�������

	imshow("egdeWdsDn", egdeWdsDn);
	//Dn_curWdsLeft_inImg :egdeWdsDn ��λ���ڵ� ���ȫͼ������
	int Dn_curWdsLeft_inImg = lightDnRight_inImg - 40;

	cout << "------------------��ͼ ��ϼ�� roi ����------------------" << endl;
	int num = 1;
	Mat cur_IterRoi = egdeWdsUp;	// ��ʼ�� ��ǰ������ cur_IterRoi�� �϶�λ���� egdeWdsUp
	int Previous_wds_up_row = 0;
	int Previous_wds_right_col = 0;
	cout << "lightWidth_Up:" << lightWidth_Up << endl;	// ��λ�����ڵĹ��������������
	// ��һ�ο϶����ᷢ����ϣ������ò���Up_startCol_inImg�����ó�ʼ��
	// Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();
	int Up_startCol_inImg = 0;	// Up_startCol_inImg������֮ǰ���ڵ� ��������ߵ� ��ȫͼ������
	while (true)
	{
		vector<Mat> mv;	split(cur_IterRoi, mv);	Mat cur_IterRoi_Red = mv[2];
		Mat cur_IterRoi_threshold;
		cv::threshold(cur_IterRoi_Red, cur_IterRoi_threshold, 140, 255, THRESH_BINARY);
		// �����Ĺ��������� 
		vector<int> lightColIdx, lightRowIdx;
		lightColIdx.clear();
		bool exist = findLight(cur_IterRoi_threshold, lightRowIdx, "row");
		findLight(cur_IterRoi_threshold, lightColIdx, "col");
		// ������ϵĴ����� �϶����й�����
		if (true == exist) {
			// ע�⣺��ʼ��λ���ڣ�cur_IterRoi = egdeWdsUp���϶����ᷢ�����
			int lightWidth = lightColIdx.back() - lightColIdx.front();
			// ��� ������ �����Ŀ��
			if (lightWidth > lightWidth_Up + 5 || lightWidth < lightWidth_Up - 5) {
				cout << "��ǰ���ڿ�Ƚϴ�|��С:" << lightWidth << "   �ж�Ϊ���" << endl;
				// ��ǵ�ǰ������ϵĴ���
				rectangle(imgProcess, Point(Up_startCol_inImg - 40, (num - 1) * 20), Point(Up_startCol_inImg + 19, num * 20 - 1), Scalar(0, 0, 255), 1);
				break;
			}
			//��ⴰ���� ������ �߶�
			if (lightRowIdx.back() < 18) {
				cout << "��ǰ���ڹ����߶� ���ڿ�϶���ж�Ϊ���" << endl;
				// ��ǵ�ǰ������ϵĴ���
				rectangle(imgProcess, Point(Up_startCol_inImg - 40, (num - 1) * 20), Point(Up_startCol_inImg + 19, num * 20 - 1), Scalar(0, 0, 255), 1);
				break;
			}
		}
		else {
			cout << "��ǰ���ڲ����ڹ������ж���һ����Ϊ��ϣ�����ĩ�� �պ�ռ����һ�����ڣ�" << endl;
			break;
		}
		// ------------ ��ǰ���� û�м�⵽������ϣ������ --------------------
		// ����ǰ �ȼ�¼ ��ǰ�������ڵ� ������Ϣ���������һ���󷽿�Ҫ�õ���
		Previous_wds_up_row = (num - 1) * 20;
		Previous_wds_right_col = Up_curWdsLeft_inImg + 59;	// �����������ұ�������
		//Up_startCol_inImg������֮ǰ���ڵ� ��������ߵ� ��ȫͼ������
		Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();

		// ���´��� ��ز���
		Up_curWdsLeft_inImg = Up_startCol_inImg - 40;	// ���´��� ��߽���ȫͼ�е� ������
		// ���µĴ��ڣ� ������������� Up_startCol_inImg - 40�� ���ұ���������Up_startCol_inImg + 19��20�ǿ����䣩
		cur_IterRoi = src(Range(num * 20, (num + 1) * 20), Range(Up_curWdsLeft_inImg, Up_startCol_inImg + 20));	// �����䣬������ ������ҿ���
		rectangle(imgProcess, Point(Up_curWdsLeft_inImg, num * 20), Point(Up_startCol_inImg + 19, (num + 1) * 20 - 1), Scalar(0, 255, 0), 1);
		num += 1;
	}
	// �˳�������������ļ�ϴ���
	Mat finalRoiUp = src(Range(Previous_wds_up_row, Previous_wds_up_row + 40), Range(Up_curWdsLeft_inImg - 60, Up_curWdsLeft_inImg + 60));
	int oriUp_row = Previous_wds_up_row;		// ��λ���ڵ� ��������
	int oriUp_col = Up_curWdsLeft_inImg - 60;	// ��λ���ڵ� ��������
	// ���
	rectangle(imgProcess, Point(Previous_wds_right_col - 119, Previous_wds_up_row), Point(Previous_wds_right_col, Previous_wds_up_row + 39),
		Scalar(0, 255, 255), 1);

	cout << "------------------��ͼ ��ϼ�� roi ����------------------" << endl;
	num = 1;
	cur_IterRoi = egdeWdsDn;	// ��ʼ�� ��ǰ������ cur_IterRoi�� �϶�λ���� egdeWdsUp
	int Previous_wds_dn_row = 0;
	//int Previous_wds_right_col = 0;
	cout << "lightWidth_Dn:" << lightWidth_Dn << endl;	// ��λ�����ڵĹ��������������
	// ��һ�ο϶����ᷢ����ϣ������ò���Up_startCol_inImg�����ó�ʼ��
	// Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();
	int Dn_startCol_inImg = 0;	// Up_startCol_inImg������֮ǰ���ڵ� ��������ߵ� ��ȫͼ������
	while (true)
	{
		vector<Mat> mv;	split(cur_IterRoi, mv);	Mat cur_IterRoi_Red = mv[2];
		Mat cur_IterRoi_threshold;
		cv::threshold(cur_IterRoi_Red, cur_IterRoi_threshold, 140, 255, THRESH_BINARY);
		// �����Ĺ���������
		vector<int> lightColIdx, lightRowIdx;
		bool exist = findLight(cur_IterRoi_threshold, lightRowIdx, "row");
		findLight(cur_IterRoi_threshold, lightColIdx, "col");
		// ������ϵĴ����� �϶����й�����
		if (true == exist) {
			// ע�⣺��ʼ��λ���ڣ�cur_IterRoi = egdeWdsUp���϶����ᷢ�����
			int lightWidth = lightColIdx.back() - lightColIdx.front();
			// ��� ������ �����Ŀ��
			if (lightWidth > lightWidth_Dn + 5 || lightWidth < lightWidth_Dn - 5) {
				cout << "��ǰ���ڿ�Ƚϴ�|��С:" << lightWidth << "   �ж�Ϊ���" << endl;
				rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - num * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 + 19), Scalar(0, 0, 255), 1);
				break;
			}
			//��ⴰ���� ������ �߶�
			if (lightRowIdx.back() < 18) {
				cout << "��ǰ���ڹ����߶� ���ڿ�϶���ж�Ϊ���" << endl;
				rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - num * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 + 19), Scalar(0, 0, 255), 1);
				break;
			}
		}
		else {
			cout << "��ǰ���ڲ����ڹ������ж���һ����Ϊ��ϣ�����ĩ�� �պ�ռ����һ�����ڣ�" << endl;
			break;
		}
		// ------------ ��ǰ���� û�м�⵽������ϣ������ --------------------
		// ����ǰ �ȼ�¼ ��ǰ�������ڵ� ������Ϣ���������һ���󷽿�Ҫ�õ���
		Previous_wds_dn_row = hight - 1 - (num - 1) * 20;
		Previous_wds_right_col = Dn_curWdsLeft_inImg + 59;	// �����������ұ�������
		//Up_startCol_inImg������֮ǰ���ڵ� ��������ߵ� ��ȫͼ������
		Dn_startCol_inImg = Dn_curWdsLeft_inImg + lightColIdx.front();

		// ���´��� ��ز���
		Dn_curWdsLeft_inImg = Dn_startCol_inImg - 40;	// ���´��� ��߽���ȫͼ�е� ������
		// ���µĴ��ڣ� ������������� Up_startCol_inImg - 40�� ���ұ���������Up_startCol_inImg + 19��20�ǿ����䣩
		cur_IterRoi = src(Range(hight - (num + 1) * 20, hight - num * 20), Range(Dn_startCol_inImg - 40, Dn_startCol_inImg + 20));	// �����䣬������ ������ҿ���
		rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - (num + 1) * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 - 1), Scalar(0, 255, 0), 1);
		num += 1;
	}
	// �˳�������������ļ�ϴ���
	Mat finalRoiDn = src(Range(Previous_wds_dn_row - 39, Previous_wds_dn_row + 1), Range(Previous_wds_right_col - 119, Previous_wds_right_col + 1));
	int oriDn_row = Previous_wds_dn_row - 39;		// ��λ���ڵ� ��������
	int oriDn_col = Previous_wds_right_col - 119;	// ��λ���ڵ� ��������
	// ���
	rectangle(imgProcess, Point(Previous_wds_right_col - 119, Previous_wds_dn_row - 39), Point(Previous_wds_right_col, Previous_wds_dn_row),
		Scalar(0, 255, 255), 1);


	cout << "------------------�� ��ͼ��󴰿� finalRoiUp ���б�Ե�㶨λ------------------" << endl;
	vector<Mat> mv;	split(finalRoiUp, mv);	Mat finRoiUp_Red = mv[2];
	Mat finRoiUp;
	cv::threshold(finRoiUp_Red, finRoiUp, 140, 255, THRESH_BINARY);

	vector<int> lightColIdx;
	findLight(finRoiUp, lightColIdx, "col");
	// ���ֵ�ڵ�����
	vector<int>::iterator it;
	int max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// ��Ԫ��-���һ��Ԫ��
		else
			diff = *(it + 1) - *it;	// ����Ԫ�� - ǰ��Ԫ��
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	int colUp = col_gapLeft;
	// �����Ե�� ������
	int n = 0;
	int sum_RowIdx = 0;
	for (int row = 0; row < finRoiUp.rows; row++) {
		if (finRoiUp.at<uchar>(row, colUp) != 0) {
			n += 1;
			sum_RowIdx += row + 1;
		}
	}
	int RowUp = sum_RowIdx / n - 1;
	// ��ȫͼ�е�����
	edgeP2.y = oriUp_row + RowUp;  /* ��ԭʼͼ���е�������*/
	edgeP2.x = oriUp_col + colUp;
	cout << "�±�Ե��P2���꣺" << edgeP2 << endl;

	circle(imgProcess, edgeP2, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, edgeP2, 5, Scalar(255, 0, 255), 1, 8);


	cout << "------------------�� ��ͼ��󴰿� finalRoiDn ���б�Ե�㶨λ------------------" << endl;
	split(finalRoiDn, mv);	Mat finRoiDn_Red = mv[2];
	Mat finRoiDn;
	cv::threshold(finRoiDn_Red, finRoiDn, 140, 255, THRESH_BINARY);
	lightColIdx.clear();
	findLight(finRoiDn, lightColIdx, "col");
	// ���ֵ�ڵ�����
	max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// ��Ԫ��-���һ��Ԫ��
		else
			diff = *(it + 1) - *it;	// ����Ԫ�� - ǰ��Ԫ��
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	int colDn = col_gapLeft;
	cout << "colDn" << colDn << endl;
	// �����Ե�� ������
	n = 0;
	sum_RowIdx = 0;
	for (int row = 0; row < finRoiDn.rows; row++) {
		if (finRoiDn.at<uchar>(row, colDn) != 0) {
			n += 1;
			sum_RowIdx += row + 1;
		}
	}
	int RowDn = sum_RowIdx / n - 1;
	// ��ȫͼ�е�����
	edgeP1.y = oriDn_row + RowDn;  /* ��ԭʼͼ���е�������*/
	edgeP1.x = oriDn_col + colDn;

	circle(imgProcess, edgeP1, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, edgeP1, 5, Scalar(255, 0, 255), 1, 8);

	cout << "�ϱ�Ե�� P1���꣺" << edgeP1 << endl;
	cout << "�±�Ե�� P2���꣺" << edgeP2 << endl;
	imshow("edgePointlocation", imgProcess);
	imwrite("F:\\MVS_Data\\edgePointlocation.png", imgProcess);
	waitKey(0);
	destroyAllWindows();
}


// �жϹ����Ƿ���ڼ򵥣�TΪ������ֵ
bool WeldCom::gapNum(Mat& roi, int T = 2)
{
	vector<Mat> mv;	split(roi, mv);	Mat red = mv[2];
	Mat threshold, thrRed;
	cv::threshold(red, threshold, 160, 255, THRESH_BINARY);
	medianBlur(threshold, thrRed, 3);

	vector<int> lightColIdx;	// ��¼�й�����������
	bool exist = findLight(thrRed, lightColIdx, "col");

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
		}
	}
	int gapPix = max - 1;
	if (gapPix >= T) 
		return true;	// ���ڼ��
	else
		return false;	// �����ڼ��
}

/* ȷ���������� */
void WeldCom::typeArray() {
	string type;
	for (int i = 0; i < 4; i++) {
		if (gapNum(regionROI[i])) {
			type.append("1");
		}
		else
			type.append("0");
	}
	cout << "Recognition Type Array��" << type << endl;

	if (type == "0000") {
		wledType = "V";
	}
	else if (type == "0100" || type == "0001") {
		wledType = "Lap";
	}
	else if (type == "1000" || type == "0010") {
		wledType = "SingleBevel";
	}
	else if (type == "1100" || type == "0011") {
		wledType = "Square";
	}
	cout << "Recognition Weld Type��" << wledType << endl;
}


void WeldCom::weldLine(Mat& src) {
	cout << endl;
	float weld_k = (weldP1.y - weldP2.y) / (weldP1.x - weldP2.x);		// б��
	float degree = atan(weld_k) / 3.1415926 * 180;					// �Ƕ�
	float missDistance = sqrt(pow((initP1.y - initP2.y), 2) + pow((initP1.x - initP2.x), 2));	// 

	// �Ϲ��� ������Ե�ͺ����� �н�
	float CornerUp = 90.2;
	float radUp = (degree + CornerUp) * 3.1415926 / 180;	// ����
	float boundUp_k = tan(radUp);	// �Ϲ����ı�Ե ��б��

	// ���ƺ��������ߵ� ��β2���������
	Point weldSrart, weldEnd;
	weldSrart.x = min(weldP1.x, weldP2.x) - 100;
	weldEnd.x = max(edgeP1.x, edgeP2.x) + 100;

	// �������ǶȲ���ˮƽ
	if (weld_k != 0) {
		float weld_b = weldP1.y - weld_k * weldP1.x;
		weldSrart.y = weld_k * weldSrart.x + weld_b;
		weldEnd.y = weld_k * weldEnd.x + weld_b;
		// ���ƺ�����
		line(src, weldSrart, weldEnd, Scalar(255, 255, 0), 1, 8);	/* ����*/
		// ���ƴ���
		float boundK = -1 / weld_k;	// �ͺ���б�ʴ�ֱ�� б��

		float bound_b1 = edgeP1.y - boundK * edgeP1.x;
		// ���㺸���� �� ��Ե��  �� �����
		initP1.x = (weld_b - bound_b1) / (boundK - weld_k);
		initP1.y = weld_k * initP1.x + weld_b;

		float bound_b2 = edgeP2.y - boundUp_k * edgeP2.x;	// ʹ��boundUp_k��������
		// ���㺸���� �� ��Ե��  �� �����
		initP2.x = (weld_b - bound_b2) / (boundUp_k - weld_k);
		initP2.y = weld_k * initP2.x + weld_b;
		line(src, edgeP1, initP1, Scalar(0, 255, 255), 1, 8);	/* */
		line(src, edgeP2, initP2, Scalar(0, 255, 255), 1, 8);	/* */
	}
	else {	// ����ˮƽ
		// ���ƺ�����
		weldSrart.y = weldP1.y;
		weldEnd.y = weldP1.y;
		line(src, weldSrart, weldEnd, Scalar(255, 255, 0), 1, 8);	/* ����*/
		// ���ƴ���
		initP1.x = edgeP1.x;
		initP1.y = weldP1.y;
		initP2.x = edgeP2.x;
		initP2.y = weldP2.y;
		line(src, edgeP1, initP1, Scalar(0, 255, 255), 1, 8);	/* */
		line(src, edgeP2, initP2, Scalar(0, 255, 255), 1, 8);	/* */
	}
	/* �������������   ��ʼ�㡢���������㡢��Ե��*/
	circle(src, initP1, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, initP1, 5, Scalar(255, 0, 255), 1, 8);
	circle(src, initP2, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, initP2, 5, Scalar(255, 0, 255), 1, 8);

	circle(src, weldP1, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, weldP1, 5, Scalar(255, 0, 255), 1, 8);
	circle(src, weldP2, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, weldP2, 5, Scalar(255, 0, 255), 1, 8);

	circle(src, edgeP1, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, edgeP1, 5, Scalar(255, 0, 255), 1, 8);
	circle(src, edgeP2, 1, Scalar(255, 0, 255), -1, 8);
	circle(src, edgeP2, 5, Scalar(255, 0, 255), 1, 8);
	//��ֹ�ڷ�����
	circle(src, crossPoint, 40, Scalar(0, 0, 255), 1, 8);
	circle(src, crossPoint, 2, Scalar(255, 0, 255), -1, 8);

	cout << "------------ ���Ӳ��������ܣ�---------------" << endl;
	cout << " 1. ��ʼ������ " << endl;
	cout << "		initP1: " << initP1 << endl;
	cout << "		initP2: " << initP2 << endl;

	cout << " 2. �¿ڿ�� " << endl;
	if (wledType == "Lap")
		cout << "		Lap û���¿ڿ��" << endl;
	else if (wledType == "V" || wledType == "Square") {
		/*float laser1Width = weldWidth(p1, p2);
		printf("		laser1 �¿ڿ��: %0.2f \n", laser1Width);
		float laser2Width = weldWidth(p1, p2);
		printf("		laser2 �¿ڿ��: %0.2f \n", laser2Width);*/
	}
	else if (wledType == "SingleBevel") {
		/*float laser1Width = weldWidth(p1, weldP1);
		printf("		laser1 �¿ڿ��: %0.2f \n", laser1Width);
		float laser2Width = weldWidth(p1, weldP2);
		printf("		laser2 �¿ڿ��: %0.2f \n", laser2Width);*/
	}

	cout << " 3. ������б�Ƕ� " << endl;
	printf("		������б %0.2f   б��:= %0.2f \n", degree, weld_k);

	cout << " 4. ��ʼ���λ���� " << endl;
	if (missDistance > 20)
		printf("		��λ���룺 %0.2f   ����δ���� \n", missDistance);
	else
		printf("		��λ���룺 %0.2f   �������� \n", missDistance);


	if (weldP1.x < weldP2.x) {
		cout << "�����ڽ�����Ϸ�" << endl;
	}
	else
		cout << "�����ڽ�����·�" << endl;

	imshow("Results", src);
	imwrite("F:\\MVS_Data\\Results.png", src);
	waitKey(0);
	destroyAllWindows();
}

// ���㺸����
float WeldCom::weldWidth(Point p1, Point p2) {
	float weld_k = (weldP1.y - weldP2.y) / (weldP1.x - weldP2.x);		// б��
	//���������ߵ�ֱ�߷��̲���
	float A = weld_k, B = -1, C = weldP1.y - weld_k * weldP1.y;
	float d1 = fabs(A * p1.x + B * p1.y + C) / sqrt(pow(A, 2) + pow(B, 2));
	float d2 = fabs(A * p2.x + B * p2.y + C) / sqrt(pow(A, 2) + pow(B, 2));
	return d1 + d2;
}
