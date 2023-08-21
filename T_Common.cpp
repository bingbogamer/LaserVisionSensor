#include "T_Common.h"


WeldCom::WeldCom(Mat& src)
{
	CrossPointlocation_step2(src, 6);			/* 精确定位交叉点，初始化 crossPoint、startP1、startP2成员*/
	edgePointlocation(src);						/* 迭代寻找边缘点，初始化 edgeP1、edgeP2成员*/
	regionSegment(src);							/* 初始化regionROI、laser1、laser2*/

	typeArray();								/* 识别焊缝类型，初始化 wledType成员 */
}

WeldCom::~WeldCom()
{

}

Mat WeldCom::Multidiagonal(int lw = 6) {
	Mat dia = Mat::eye(lw * 2 + 1, lw * 2 + 1, CV_8UC1);
	for (int row = 0, col = 0; row < lw * 2 + 1 && col < lw * 2 + 1; row++, col++) {
		for (int k = 1; k <= lw / 2 - 1; k++) {
			if (col + k < lw * 2 + 1) 				// 右边对角
				dia.at<uchar>(row, col + k) = 1;
			if (0 <= col - k) 						// 左边对角
				dia.at<uchar>(row, col - k) = 1;
		}
	}
	// cout << "修改 dia = " << endl << " " << dia << endl << endl;
	Mat dst;
	flip(dia, dst, 1);
	// cout << "y轴翻转 dia = " << endl << " " << dst << endl << endl;
	Mat kernel = dia + dst;
	// cout << " kernel = " << endl << " " << kernel << endl << endl;
	return kernel;
}

// 检测光条存在的索引（返回 光条存在的行索引数组，列索引数组）
// 判断当前窗口图像  是否存在光条
bool WeldCom::findLight(Mat& binary, vector<int>& lightIdx, string mode = "row") {
	lightIdx.clear();	// 先清空
	int height = binary.rows, width = binary.cols;
	if (mode == "row") {
		uchar* p;
		for (int row = 0; row < height; ++row) {
			p = binary.ptr<uchar>(row);   // 获取每一行开始处的指针，即每一行数组的首地址
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

// 垂直灰度重心法
// 传入的是RGB 截取图像
// upperRowIndex ： 窗口上边界在图像中的行索引
vector<Point>	WeldCom::wds_Roi(Mat& roi, int upperRowIndex) {
	int height = roi.rows, width = roi.cols, channels = roi.channels();
	vector<Mat> mv;	split(roi, mv);	Mat red = mv[2];
	Mat thrRed;
	cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
	// 空数组，记录中心点的坐标列表
	vector<Point>	gravityCoorList;
	// 遍历图像所有像素
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
			Point p(col, realRow);	// (x, y) 列，行
			gravityCoorList.push_back(p);
		}
	}
	return gravityCoorList;
}

/* 区域分割 */
void WeldCom::regionSegment(Mat& img) {
	// 行区间，列区间 （左闭右开）
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
	Mat RightUpRoi_All = img(Range(0, crossPoint.y), Range(crossPoint.x, img.cols));			/* roi2	有工作台*/
	Mat RightDownRoi_All = img(Range(crossPoint.y, img.rows), Range(crossPoint.x, img.cols));	/* roi4	有工作台	*/
	//设置画布绘制区域并复制
	Rect roi1_rect = cv::Rect(0, 0, LeftUpRoi.cols, LeftUpRoi.rows);	// 列，行，宽，高
	LeftUpRoi.copyTo(laser1(roi1_rect));
	Rect roi3_rect = cv::Rect(crossPoint.x, crossPoint.y, RightDownRoi_All.cols, RightDownRoi_All.rows);	// 列x，行y，宽，高
	RightDownRoi_All.copyTo(laser1(roi3_rect));

	laser2 = Mat::zeros(img.rows, img.cols, img.type());
	//设置画布绘制区域并复制
	Rect roi2_rect = cv::Rect(crossPoint.x, 0, RightUpRoi_All.cols, RightUpRoi_All.rows);	// 列，行，宽，高
	RightUpRoi_All.copyTo(laser2(roi2_rect));
	Rect roi4_rect = cv::Rect(0, crossPoint.y, LeftDownRoi.cols, LeftDownRoi.rows);	// 列x，行y，宽，高
	LeftDownRoi.copyTo(laser2(roi4_rect));

	// imshow("laser1", laser1);
	// imshow("laser2", laser2);
	imwrite("F:\\MVS_Data\\laser1.png", laser1);
	imwrite("F:\\MVS_Data\\laser2.png", laser2);
	waitKey(0);
	destroyAllWindows();

	/* 初始化 laser1_OnWorkpiece、laser2_OnWorkpiece 数据成员*/
	laser1_OnWorkpiece = laser1(Range(0, laser1.rows), Range(0, edgeP1.x));	// 行区间，列区间 （左闭右开）
	laser2_OnWorkpiece = laser2(Range(0, laser2.rows), Range(0, edgeP2.x));
	/*imshow("laser1_OnWorkpiece", laser1_OnWorkpiece);
	imshow("laser2_OnWorkpiece", laser2_OnWorkpiece);*/
	imwrite("F:\\MVS_Data\\laser1_OnWorkpiece.png", laser1_OnWorkpiece);
	imwrite("F:\\MVS_Data\\laser2_OnWorkpiece.png", laser2_OnWorkpiece);
	waitKey(0);
	destroyAllWindows();
}

/* 交叉点初步定位 */
/* 拟合2个定位窗口的交叉线，计算并返回 粗略交叉点*/
Point WeldCom::CrossPointlocation_step1(Mat& img)
{
	// img 用来显示原始图像，imgProcess用来做处理，会发生改变
	Mat imgProcess = img.clone();
	Mat fitWdsUp, fitWdsDn;			// 创建 矩阵头

	int height = img.rows, width = img.cols, channels = img.channels();
	int h = 40, w = 20;				// 可以设置成 默认成员属性
	// 上下搜索窗口的上边界的行索引
	int rowIndexUp = 0;
	int rowIndexDn = height - h;
	// 上下定位窗口 的上边界在全图中的行索引
	int wdsUp_row, wdsDn_row;

	// Laser1 上窗口搜索
	int wdsnum = 1;
	while (true) 
	{
		Mat wdsUp = img(Range(rowIndexUp, rowIndexUp + h), Range(0, w));	// 行区间，列区间 （左闭右开）
		// 矩形两个对角顶点为 Point(x, y) 和 Point(x, y)	线粗为 -1, 此矩形将被填充
		rectangle(imgProcess, Point(0, rowIndexUp), Point(w - 1, rowIndexUp + h - 1), Scalar(0, 255, 0), 1);	// 在图像内部 最外圈

		vector<Mat> mv;	split(wdsUp, mv);	Mat red = mv[2];
		Mat threshold, thrRed;
		cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
		medianBlur(threshold, thrRed, 3);

		// 判断当前窗口图像  是否存在光条
		vector<int> lightRowIdx;	// 记录有光条的行索引
		bool exist = findLight(thrRed, lightRowIdx, "row");
		// 当存在光条
		if (exist) 
		{
			int rowUpIdx = lightRowIdx[0];			// 光条最上面的点在roi中的行索引
			int findRow = rowIndexUp + rowUpIdx;	// 光条最上面的点在全图行索引
			cout << "交叉点定位：当前窗口检测到光条，最上行索引：" << findRow << endl;
			// 上定位窗口fitWdsUp 标记		在图像内部最外圈（宽20 高60：19 + 1 + 40）
			rectangle(imgProcess, Point(0, findRow - 19), Point(w - 1, findRow + 40), Scalar(255, 0, 255), 1);
			fitWdsUp = img(Range(findRow - 19, findRow + 41), Range(0, w));
			wdsUp_row = findRow - 19;
			break;
		}
		else 
		{
			// printf("交叉点定位：当前窗口%d未检测到光条\n", wdsnum);
			rowIndexUp += h;
			wdsnum += 1;
		}
	}

	// Laser2 下窗口搜索
	wdsnum = 1;
	while (true) {
		Mat wdsDn = img(Range(rowIndexDn, rowIndexDn + h), Range(0, w));	// 行区间，列区间 （左闭右开）
		// 矩形两个对角顶点为 Point(x, y) 和 Point(x, y)	线粗为 -1, 此矩形将被填充
		rectangle(imgProcess, Point(0, rowIndexDn), Point(w - 1, rowIndexDn + h - 1), Scalar(0, 255, 0), 1);	// 在图像内部 最外圈

		vector<Mat> mv;	split(wdsDn, mv);	Mat red = mv[2];
		Mat threshold, thrRed;
		cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
		medianBlur(threshold, thrRed, 3);

		// 判断当前窗口图像  是否存在光条
		vector<int> lightRowIdx;	// 记录有光条的行索引
		bool exist = findLight(thrRed, lightRowIdx, "row");
		// 当存在光条
		if (exist) 
		{
			int rowDnIdx = lightRowIdx.back();	// 光条最下面的点在roi中的行索引
			int findRow = rowIndexDn + rowDnIdx;	// 光条最下面的点在全图行索引
			cout << "交叉点定位：当前窗口检测到光条，最下行索引：" << findRow << endl;
			// 下定位窗口fitWdsDn 标记		在图像内部最外圈（宽20 高60：19 + 1 + 40）
			rectangle(imgProcess, Point(0, findRow - 40), Point(w - 1, findRow + 19), Scalar(255, 0, 255), 1);
			fitWdsDn = img(Range(findRow - 40, findRow + 20), Range(0, w));
			wdsDn_row = findRow - 40;
			break;
		}
		else 
		{
			// printf("交叉点定位：当前窗口%d未检测到光条\n", wdsnum);
			rowIndexDn -= h;
			wdsnum += 1;
		}
	}
	//imshow("fitWdsUp", fitWdsUp);
	//imshow("fitWdsDn", fitWdsDn);

	// 对窗口处理，拟合出直线斜率
	// 垂直cog，返回中心点坐标
	vector<Point> gravityCoorUp = wds_Roi(fitWdsUp, wdsUp_row);
	vector<Point> gravityCoorDn = wds_Roi(fitWdsDn, wdsDn_row);

	//对中心点1 拟合 
	Vec4f line1_para;	// (vx, vy, x0, y0)，其中(vx, vy)是与直线共线的归一化向量，(x0, y0)是直线上的一个点
	cv::fitLine(gravityCoorUp, line1_para, cv::DIST_L2, 0, 1e-2, 1e-2);	// DIST_L2:最小二乘法
	cout << "line1_para = " << line1_para << endl;
	double k1 = line1_para[1] / line1_para[0];
	double b1 = line1_para[3] - k1 * line1_para[2]; //  A, B, C = k, -1, b

	Point start(gravityCoorUp[0].x, int(round(gravityCoorUp[0].x * k1 + b1)));
	Point end(int(width * 3 / 8), int(round((width * 3 / 8) * k1 + b1)));
	line(imgProcess, start, end, Scalar(255, 255, 0), 1, 8);
	startP1 = gravityCoorUp.back();	// 取最后一个点, 作为之后V型光条点 拟合距离的 中心线的起始点

	//对中心点2 拟合 
	Vec4f line2_para;
	cv::fitLine(gravityCoorDn, line2_para, cv::DIST_L2, 0, 1e-2, 1e-2);	// DIST_L2:最小二乘法
	cout << "line2_para = " << line2_para << endl;
	double k2 = line2_para[1] / line2_para[0];
	double b2 = line2_para[3] - k2 * line2_para[2]; //  A, B, C = k, -1, b

	Point start2(gravityCoorDn[0].x, int(round(gravityCoorDn[0].x * k2 + b2)));
	Point end2(int(width * 3 / 8), int(round((width * 3 / 8) * k2 + b2)));
	line(imgProcess, start2, end2, Scalar(255, 255, 0), 1, 8);
	startP2 = gravityCoorDn.back();	// 取最后一个点, 作为之后V型光条点 拟合距离的 中心线的起始点

	// 3. 计算交点坐标，并设置交叉点ROI
	Point cp;
	cp.x = int(round((b2 - b1) / (k1 - k2)));
	cp.y = int(round(k1 * cp.x + b1));

	circle(imgProcess, cp, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, cp, 5, Scalar(255, 0, 255), 1, 8);
	// 绘制精确定位区域
	int roiH = 80, roiW = 160;
	rectangle(imgProcess, Point(cp.x - roiW / 2, cp.y - roiH / 2), Point(cp.x + roiW / 2, cp.y + roiH / 2), Scalar(0, 255, 0), 1);

	imshow("edgeSearch", imgProcess);
	imwrite("F:\\MVS_Data\\edgeSearch.png", imgProcess);

	return cp;
}

/* 交叉点 精确定位*/
void WeldCom::CrossPointlocation_step2(Mat& img, int lw = 6) {
	// img 用来截取原始图像（没有画图），imgProcess用来做处理，会发生改变
	Mat imgProcess = img.clone();
	Point cp = CrossPointlocation_step1(img);
	int roiH = 80, roiW = 160;
	// 截取的最后的定位窗口
	Mat roi_row = imgProcess(Range(cp.y - roiH / 2, cp.y + roiH / 2), Range(cp.x - roiW / 2, cp.x + roiW / 2));	// 行区间，列区间 （左闭右开）
	// imshow("roi", roi_row);
	rectangle(imgProcess, Point(cp.x - roiW / 2, cp.y - roiH / 2), Point(cp.x + roiW / 2, cp.y + roiH / 2), Scalar(0, 255, 0), 1);

	int height = roi_row.rows, width = roi_row.cols;
	vector<Mat> mv;	split(roi_row, mv);	Mat red = mv[2];
	Mat threshold, thrRed;
	cv::threshold(red, threshold, 140, 255, THRESH_BINARY);
	medianBlur(threshold, thrRed, 3);
	imshow("roi_threshold", thrRed);
	imwrite("F:\\MVS_Data\\xxxxxxxxxxxx.png", thrRed);

	// 创建卷积核，精确定位CrossPoint
	Mat norm = thrRed / 255;	// 归一化单通道图像
	Mat kernel = Multidiagonal(lw);	// 卷积核
	Mat dst;
	dst.create(thrRed.size(), thrRed.type());
	dst = Scalar(0);
	filter2D(norm, dst, norm.depth(), kernel);
	imshow("dst", dst);
	double minValue, maxValue;    // 最大值，最小值
	Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标

	cv::minMaxLoc(dst, &minValue, &maxValue, &minIdx, &maxIdx);

	cout << "CrossPoint在roi中的精确坐标: " << maxIdx << endl;
	circle(roi_row, maxIdx, 1, Scalar(255, 0, 255), -1, 8);
	circle(roi_row, maxIdx, 5, Scalar(255, 0, 255), 1, 8);

	imshow("CrossPoint", roi_row);
	imwrite("F:\\MVS_Data\\CrossPoint.png", roi_row);

	crossPoint.x = cp.x - int(roiW / 2) + maxIdx.x;
	crossPoint.y = cp.y - int(roiH / 2) + maxIdx.y;

	cout << "CrossPoint在全图中的精确坐标: " << crossPoint << endl;
	// regionSegment(img, CrossPoint);
}

void WeldCom::edgePointlocation(Mat& src) 
{
	cout << "------------边缘检测---------------" << endl;
	// src 用来截取原始图像（没有画图），imgProcess用来做处理，会发生改变
	Mat imgProcess = src.clone();
	// 边缘搜索 迭代窗口的大小
	int h = 20, w = 40;		// 可以设置成 默认成员属性，和交叉点搜索的窗口大小不一样

	int hight = imgProcess.rows, width = imgProcess.cols;
	int wdsnum = 1;
	Mat egdeWdsUp, egdeWdsDn;	// 定位窗口
	int lightUpRight_inImg, lightDnRight_inImg;		// 光条的最右点在全图中的列索引
	//1. 搜索窗口迭代
	//第一个窗口左边界的列索引
	int edgeUpLeft = width - w;  //都是窗口的左边界的列索引
	int lightWidth_Up = 0;
	while (true)
	{
		Mat wdsUp = imgProcess(Range(0, h), Range(edgeUpLeft, edgeUpLeft + w));
		rectangle(imgProcess, Point(edgeUpLeft, 0), Point(edgeUpLeft + w - 1, h - 1), Scalar(0, 255, 0), 1);
		vector<Mat> mv;	split(wdsUp, mv);	Mat red = mv[2];
		Mat thrRed;
		cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
		//判断当前窗口图像  是否存在光条
		vector<int> lightColIdx;
		bool exist = findLight(thrRed, lightColIdx, "col");
		if (exist) 
		{
			int lightUpRight_inRoi = lightColIdx.back();	// 最后一行，就是最大值
			lightUpRight_inImg = edgeUpLeft + lightUpRight_inRoi;	// 光条的最右点在全图中的列索引
			cout << "边缘点搜索：上窗口检测到光条，最右列索引：" << lightUpRight_inImg << endl;
			// 标记 定位窗口 —— 在图像内部最外圈（宽60 高20）
			rectangle(imgProcess, Point(lightUpRight_inImg - 40, 0), Point(lightUpRight_inImg + 19, 19), Scalar(255, 0, 255), 1);
			circle(imgProcess, Point(lightUpRight_inImg, 0), 1, Scalar(255, 0, 255), -1, 8);
			// 定位窗口 （和检测光条窗口wdsUp是 不同的）
			egdeWdsUp = src(Range(0, 20), Range(lightUpRight_inImg - 40, lightUpRight_inImg + 20));
			// 计算 定位窗口（包含完全的光条段）的 光条宽度（默认没有噪音）
			vector<Mat> mv;	split(egdeWdsUp, mv);	Mat red = mv[2];
			Mat thrRed;
			cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
			vector<int> lightColIdx;
			findLight(thrRed, lightColIdx, "col");
			lightWidth_Up = lightColIdx.back() - lightColIdx.front();	// 定位窗口内的光条的列索引宽度
			break;
		}
		edgeUpLeft -= w;
		wdsnum += 1;
	}
	wdsnum = 1;
	//第一个窗口左边界的列索引
	int edgeDnLeft = width - w;  //所有窗口的最左边的列索引
	int lightWidth_Dn = 0;
	while (true)
	{
		Mat wdsDn = imgProcess(Range(hight - h, hight), Range(edgeDnLeft, edgeDnLeft + w));
		rectangle(imgProcess, Point(edgeDnLeft, hight - h), Point(edgeDnLeft + w - 1, hight - 1), Scalar(0, 255, 0), 1);
		vector<Mat> mv;	split(wdsDn, mv);	Mat red = mv[2];
		Mat thrRed;
		cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
		//判断当前窗口图像  是否存在光条
		vector<int> lightColIdx;
		bool exist = findLight(thrRed, lightColIdx, "col");
		if (exist) 
		{
			int lightDnRight_inRoi = lightColIdx.back();	// 最后一行，就是最大值
			lightDnRight_inImg = edgeDnLeft + lightDnRight_inRoi; // edgeDnLeft：所有边缘窗口的最左边的列索引
			cout << "边缘点搜索：下窗口检测到光条，光条的最右列索引：" << lightDnRight_inImg << endl;
			// 标记 定位窗口 —— 在图像内部最外圈（宽60 高20）
			rectangle(imgProcess, Point(lightDnRight_inImg - 40, hight - h), Point(lightDnRight_inImg + 19, hight - 1), Scalar(255, 0, 255), 1);
			circle(imgProcess, Point(lightDnRight_inImg, hight - 1), 1, Scalar(255, 0, 255), -1, 8);
			// 定位窗口 （和检测光条窗口wdsUp是 不同的）
			egdeWdsDn = src(Range(hight - 20, hight), Range(lightDnRight_inImg - 40, lightDnRight_inImg + 20));
			// 计算 定位窗口（包含完全的光条段）的 光条宽度（默认没有噪音）
			vector<Mat> mv;	split(egdeWdsDn, mv);	Mat red = mv[2];
			Mat thrRed;
			cv::threshold(red, thrRed, 140, 255, THRESH_BINARY);
			vector<int> lightColIdx;
			findLight(thrRed, lightColIdx, "col");
			lightWidth_Dn = lightColIdx.back() - lightColIdx.front();	// 定位窗口内的光条的列索引宽度
			break;
		}
		edgeDnLeft -= w;
		wdsnum += 1;
	}
	//和下面迭代一致，光条上方的左边点作为起点，设置迭代窗口
	imshow("egdeWdsUp", egdeWdsUp);
	//Up_curWdsLeft_inImg :egdeWdsUp 定位窗口的 左边全图列索引
	int Up_curWdsLeft_inImg = lightUpRight_inImg - 40;	//  lightUpRight_inImg: 光条的最右点在全图中的列索引

	imshow("egdeWdsDn", egdeWdsDn);
	//Dn_curWdsLeft_inImg :egdeWdsDn 定位窗口的 左边全图列索引
	int Dn_curWdsLeft_inImg = lightDnRight_inImg - 40;

	cout << "------------------上图 间断检测 roi 迭代------------------" << endl;
	int num = 1;
	Mat cur_IterRoi = egdeWdsUp;	// 初始的 当前处理窗口 cur_IterRoi是 上定位窗口 egdeWdsUp
	int Previous_wds_up_row = 0;
	int Previous_wds_right_col = 0;
	cout << "lightWidth_Up:" << lightWidth_Up << endl;	// 定位窗口内的光条的列索引宽度
	// 第一次肯定不会发生间断，所以用不到Up_startCol_inImg，不用初始化
	// Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();
	int Up_startCol_inImg = 0;	// Up_startCol_inImg：更新之前窗口的 光条最左边点 的全图列索引
	while (true)
	{
		vector<Mat> mv;	split(cur_IterRoi, mv);	Mat cur_IterRoi_Red = mv[2];
		Mat cur_IterRoi_threshold;
		cv::threshold(cur_IterRoi_Red, cur_IterRoi_threshold, 140, 255, THRESH_BINARY);
		// 求最后的光条列坐标 
		vector<int> lightColIdx, lightRowIdx;
		lightColIdx.clear();
		bool exist = findLight(cur_IterRoi_threshold, lightRowIdx, "row");
		findLight(cur_IterRoi_threshold, lightColIdx, "col");
		// 发生间断的窗口内 肯定是有光条的
		if (true == exist) {
			// 注意：初始定位窗口，cur_IterRoi = egdeWdsUp，肯定不会发生间断
			int lightWidth = lightColIdx.back() - lightColIdx.front();
			// 检测 窗口内 光条的宽度
			if (lightWidth > lightWidth_Up + 5 || lightWidth < lightWidth_Up - 5) {
				cout << "当前窗口宽度较大|较小:" << lightWidth << "   判断为间断" << endl;
				// 标记当前发生间断的窗口
				rectangle(imgProcess, Point(Up_startCol_inImg - 40, (num - 1) * 20), Point(Up_startCol_inImg + 19, num * 20 - 1), Scalar(0, 0, 255), 1);
				break;
			}
			//检测窗口内 光条的 高度
			if (lightRowIdx.back() < 18) {
				cout << "当前窗口光条高度 存在空隙，判断为间断" << endl;
				// 标记当前发生间断的窗口
				rectangle(imgProcess, Point(Up_startCol_inImg - 40, (num - 1) * 20), Point(Up_startCol_inImg + 19, num * 20 - 1), Scalar(0, 0, 255), 1);
				break;
			}
		}
		else {
			cout << "当前窗口不存在光条，判断上一窗口为间断（光条末端 刚好占满上一个窗口）" << endl;
			break;
		}
		// ------------ 当前窗口 没有检测到光条间断，则更新 --------------------
		// 更新前 先记录 当前迭代窗口的 坐标信息（后面最后一个大方框要用到）
		Previous_wds_up_row = (num - 1) * 20;
		Previous_wds_right_col = Up_curWdsLeft_inImg + 59;	// 迭代窗口最右边列索引
		//Up_startCol_inImg：更新之前窗口的 光条最左边点 的全图列索引
		Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();

		// 更新窗口 相关参数
		Up_curWdsLeft_inImg = Up_startCol_inImg - 40;	// 更新窗口 左边界在全图中的 列索引
		// 更新的窗口， 最左边列索引： Up_startCol_inImg - 40， 最右边列索引：Up_startCol_inImg + 19（20是开区间）
		cur_IterRoi = src(Range(num * 20, (num + 1) * 20), Range(Up_curWdsLeft_inImg, Up_startCol_inImg + 20));	// 行区间，列区间 （左闭右开）
		rectangle(imgProcess, Point(Up_curWdsLeft_inImg, num * 20), Point(Up_startCol_inImg + 19, (num + 1) * 20 - 1), Scalar(0, 255, 0), 1);
		num += 1;
	}
	// 退出迭代，标记最后的间断窗口
	Mat finalRoiUp = src(Range(Previous_wds_up_row, Previous_wds_up_row + 40), Range(Up_curWdsLeft_inImg - 60, Up_curWdsLeft_inImg + 60));
	int oriUp_row = Previous_wds_up_row;		// 定位窗口的 上行索引
	int oriUp_col = Up_curWdsLeft_inImg - 60;	// 定位窗口的 左列索引
	// 标记
	rectangle(imgProcess, Point(Previous_wds_right_col - 119, Previous_wds_up_row), Point(Previous_wds_right_col, Previous_wds_up_row + 39),
		Scalar(0, 255, 255), 1);

	cout << "------------------下图 间断检测 roi 迭代------------------" << endl;
	num = 1;
	cur_IterRoi = egdeWdsDn;	// 初始的 当前处理窗口 cur_IterRoi是 上定位窗口 egdeWdsUp
	int Previous_wds_dn_row = 0;
	//int Previous_wds_right_col = 0;
	cout << "lightWidth_Dn:" << lightWidth_Dn << endl;	// 定位窗口内的光条的列索引宽度
	// 第一次肯定不会发生间断，所以用不到Up_startCol_inImg，不用初始化
	// Up_startCol_inImg = Up_curWdsLeft_inImg + lightColIdx.front();
	int Dn_startCol_inImg = 0;	// Up_startCol_inImg：更新之前窗口的 光条最左边点 的全图列索引
	while (true)
	{
		vector<Mat> mv;	split(cur_IterRoi, mv);	Mat cur_IterRoi_Red = mv[2];
		Mat cur_IterRoi_threshold;
		cv::threshold(cur_IterRoi_Red, cur_IterRoi_threshold, 140, 255, THRESH_BINARY);
		// 求最后的光条列坐标
		vector<int> lightColIdx, lightRowIdx;
		bool exist = findLight(cur_IterRoi_threshold, lightRowIdx, "row");
		findLight(cur_IterRoi_threshold, lightColIdx, "col");
		// 发生间断的窗口内 肯定是有光条的
		if (true == exist) {
			// 注意：初始定位窗口，cur_IterRoi = egdeWdsUp，肯定不会发生间断
			int lightWidth = lightColIdx.back() - lightColIdx.front();
			// 检测 窗口内 光条的宽度
			if (lightWidth > lightWidth_Dn + 5 || lightWidth < lightWidth_Dn - 5) {
				cout << "当前窗口宽度较大|较小:" << lightWidth << "   判断为间断" << endl;
				rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - num * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 + 19), Scalar(0, 0, 255), 1);
				break;
			}
			//检测窗口内 光条的 高度
			if (lightRowIdx.back() < 18) {
				cout << "当前窗口光条高度 存在空隙，判断为间断" << endl;
				rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - num * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 + 19), Scalar(0, 0, 255), 1);
				break;
			}
		}
		else {
			cout << "当前窗口不存在光条，判断上一窗口为间断（光条末端 刚好占满上一个窗口）" << endl;
			break;
		}
		// ------------ 当前窗口 没有检测到光条间断，则更新 --------------------
		// 更新前 先记录 当前迭代窗口的 坐标信息（后面最后一个大方框要用到）
		Previous_wds_dn_row = hight - 1 - (num - 1) * 20;
		Previous_wds_right_col = Dn_curWdsLeft_inImg + 59;	// 迭代窗口最右边列索引
		//Up_startCol_inImg：更新之前窗口的 光条最左边点 的全图列索引
		Dn_startCol_inImg = Dn_curWdsLeft_inImg + lightColIdx.front();

		// 更新窗口 相关参数
		Dn_curWdsLeft_inImg = Dn_startCol_inImg - 40;	// 更新窗口 左边界在全图中的 列索引
		// 更新的窗口， 最左边列索引： Up_startCol_inImg - 40， 最右边列索引：Up_startCol_inImg + 19（20是开区间）
		cur_IterRoi = src(Range(hight - (num + 1) * 20, hight - num * 20), Range(Dn_startCol_inImg - 40, Dn_startCol_inImg + 20));	// 行区间，列区间 （左闭右开）
		rectangle(imgProcess, Point(Dn_startCol_inImg - 40, hight - (num + 1) * 20), Point(Dn_startCol_inImg + 19, hight - num * 20 - 1), Scalar(0, 255, 0), 1);
		num += 1;
	}
	// 退出迭代，标记最后的间断窗口
	Mat finalRoiDn = src(Range(Previous_wds_dn_row - 39, Previous_wds_dn_row + 1), Range(Previous_wds_right_col - 119, Previous_wds_right_col + 1));
	int oriDn_row = Previous_wds_dn_row - 39;		// 定位窗口的 上行索引
	int oriDn_col = Previous_wds_right_col - 119;	// 定位窗口的 左列索引
	// 标记
	rectangle(imgProcess, Point(Previous_wds_right_col - 119, Previous_wds_dn_row - 39), Point(Previous_wds_right_col, Previous_wds_dn_row),
		Scalar(0, 255, 255), 1);


	cout << "------------------对 上图最后窗口 finalRoiUp 进行边缘点定位------------------" << endl;
	vector<Mat> mv;	split(finalRoiUp, mv);	Mat finRoiUp_Red = mv[2];
	Mat finRoiUp;
	cv::threshold(finRoiUp_Red, finRoiUp, 140, 255, THRESH_BINARY);

	vector<int> lightColIdx;
	findLight(finRoiUp, lightColIdx, "col");
	// 最大值在的索引
	vector<int>::iterator it;
	int max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// 首元素-最后一个元素
		else
			diff = *(it + 1) - *it;	// 后面元素 - 前面元素
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	int colUp = col_gapLeft;
	// 计算边缘点 行索引
	int n = 0;
	int sum_RowIdx = 0;
	for (int row = 0; row < finRoiUp.rows; row++) {
		if (finRoiUp.at<uchar>(row, colUp) != 0) {
			n += 1;
			sum_RowIdx += row + 1;
		}
	}
	int RowUp = sum_RowIdx / n - 1;
	// 在全图中的坐标
	edgeP2.y = oriUp_row + RowUp;  /* 在原始图像中的行坐标*/
	edgeP2.x = oriUp_col + colUp;
	cout << "下边缘点P2坐标：" << edgeP2 << endl;

	circle(imgProcess, edgeP2, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, edgeP2, 5, Scalar(255, 0, 255), 1, 8);


	cout << "------------------对 下图最后窗口 finalRoiDn 进行边缘点定位------------------" << endl;
	split(finalRoiDn, mv);	Mat finRoiDn_Red = mv[2];
	Mat finRoiDn;
	cv::threshold(finRoiDn_Red, finRoiDn, 140, 255, THRESH_BINARY);
	lightColIdx.clear();
	findLight(finRoiDn, lightColIdx, "col");
	// 最大值在的索引
	max = 0, col_gapLeft = -1;
	for (it = lightColIdx.begin(); it != lightColIdx.end(); it++) {
		int diff;
		if (it == lightColIdx.end() - 1)
			diff = *lightColIdx.begin() - *it;	// 首元素-最后一个元素
		else
			diff = *(it + 1) - *it;	// 后面元素 - 前面元素
		if (diff >= max) {
			max = diff;
			col_gapLeft = *it;
		}
	}
	int colDn = col_gapLeft;
	cout << "colDn" << colDn << endl;
	// 计算边缘点 行索引
	n = 0;
	sum_RowIdx = 0;
	for (int row = 0; row < finRoiDn.rows; row++) {
		if (finRoiDn.at<uchar>(row, colDn) != 0) {
			n += 1;
			sum_RowIdx += row + 1;
		}
	}
	int RowDn = sum_RowIdx / n - 1;
	// 在全图中的坐标
	edgeP1.y = oriDn_row + RowDn;  /* 在原始图像中的行坐标*/
	edgeP1.x = oriDn_col + colDn;

	circle(imgProcess, edgeP1, 1, Scalar(255, 0, 255), -1, 8);
	circle(imgProcess, edgeP1, 5, Scalar(255, 0, 255), 1, 8);

	cout << "上边缘点 P1坐标：" << edgeP1 << endl;
	cout << "下边缘点 P2坐标：" << edgeP2 << endl;
	imshow("edgePointlocation", imgProcess);
	imwrite("F:\\MVS_Data\\edgePointlocation.png", imgProcess);
	waitKey(0);
	destroyAllWindows();
}


// 判断光条是否存在简单，T为像素阈值
bool WeldCom::gapNum(Mat& roi, int T = 2)
{
	vector<Mat> mv;	split(roi, mv);	Mat red = mv[2];
	Mat threshold, thrRed;
	cv::threshold(red, threshold, 160, 255, THRESH_BINARY);
	medianBlur(threshold, thrRed, 3);

	vector<int> lightColIdx;	// 记录有光条的行索引
	bool exist = findLight(thrRed, lightColIdx, "col");

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
		}
	}
	int gapPix = max - 1;
	if (gapPix >= T) 
		return true;	// 存在间断
	else
		return false;	// 不存在间断
}

/* 确定分类数组 */
void WeldCom::typeArray() {
	string type;
	for (int i = 0; i < 4; i++) {
		if (gapNum(regionROI[i])) {
			type.append("1");
		}
		else
			type.append("0");
	}
	cout << "Recognition Type Array：" << type << endl;

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
	cout << "Recognition Weld Type：" << wledType << endl;
}


void WeldCom::weldLine(Mat& src) {
	cout << endl;
	float weld_k = (weldP1.y - weldP2.y) / (weldP1.x - weldP2.x);		// 斜率
	float degree = atan(weld_k) / 3.1415926 * 180;					// 角度
	float missDistance = sqrt(pow((initP1.y - initP2.y), 2) + pow((initP1.x - initP2.x), 2));	// 

	// 上工件 工件边缘和焊缝线 夹角
	float CornerUp = 90.2;
	float radUp = (degree + CornerUp) * 3.1415926 / 180;	// 弧度
	float boundUp_k = tan(radUp);	// 上工件的边缘 的斜率

	// 绘制焊缝特征线的 收尾2个点的坐标
	Point weldSrart, weldEnd;
	weldSrart.x = min(weldP1.x, weldP2.x) - 100;
	weldEnd.x = max(edgeP1.x, edgeP2.x) + 100;

	// 如果焊缝角度不是水平
	if (weld_k != 0) {
		float weld_b = weldP1.y - weld_k * weldP1.x;
		weldSrart.y = weld_k * weldSrart.x + weld_b;
		weldEnd.y = weld_k * weldEnd.x + weld_b;
		// 绘制焊缝线
		line(src, weldSrart, weldEnd, Scalar(255, 255, 0), 1, 8);	/* 青蓝*/
		// 绘制垂线
		float boundK = -1 / weld_k;	// 和焊缝斜率垂直的 斜率

		float bound_b1 = edgeP1.y - boundK * edgeP1.x;
		// 计算焊缝线 和 边缘线  的 交叉点
		initP1.x = (weld_b - bound_b1) / (boundK - weld_k);
		initP1.y = weld_k * initP1.x + weld_b;

		float bound_b2 = edgeP2.y - boundUp_k * edgeP2.x;	// 使用boundUp_k单独计算
		// 计算焊缝线 和 边缘线  的 交叉点
		initP2.x = (weld_b - bound_b2) / (boundUp_k - weld_k);
		initP2.y = weld_k * initP2.x + weld_b;
		line(src, edgeP1, initP1, Scalar(0, 255, 255), 1, 8);	/* */
		line(src, edgeP2, initP2, Scalar(0, 255, 255), 1, 8);	/* */
	}
	else {	// 焊缝水平
		// 绘制焊缝线
		weldSrart.y = weldP1.y;
		weldEnd.y = weldP1.y;
		line(src, weldSrart, weldEnd, Scalar(255, 255, 0), 1, 8);	/* 青蓝*/
		// 绘制垂线
		initP1.x = edgeP1.x;
		initP1.y = weldP1.y;
		initP2.x = edgeP2.x;
		initP2.y = weldP2.y;
		line(src, edgeP1, initP1, Scalar(0, 255, 255), 1, 8);	/* */
		line(src, edgeP2, initP2, Scalar(0, 255, 255), 1, 8);	/* */
	}
	/* 标记所有特征点   初始点、焊缝特征点、边缘点*/
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
	//禁止摆放区域
	circle(src, crossPoint, 40, Scalar(0, 0, 255), 1, 8);
	circle(src, crossPoint, 2, Scalar(255, 0, 255), -1, 8);

	cout << "------------ 焊接参数（汇总）---------------" << endl;
	cout << " 1. 初始点坐标 " << endl;
	cout << "		initP1: " << initP1 << endl;
	cout << "		initP2: " << initP2 << endl;

	cout << " 2. 坡口宽度 " << endl;
	if (wledType == "Lap")
		cout << "		Lap 没有坡口宽度" << endl;
	else if (wledType == "V" || wledType == "Square") {
		/*float laser1Width = weldWidth(p1, p2);
		printf("		laser1 坡口宽度: %0.2f \n", laser1Width);
		float laser2Width = weldWidth(p1, p2);
		printf("		laser2 坡口宽度: %0.2f \n", laser2Width);*/
	}
	else if (wledType == "SingleBevel") {
		/*float laser1Width = weldWidth(p1, weldP1);
		printf("		laser1 坡口宽度: %0.2f \n", laser1Width);
		float laser2Width = weldWidth(p1, weldP2);
		printf("		laser2 坡口宽度: %0.2f \n", laser2Width);*/
	}

	cout << " 3. 焊缝倾斜角度 " << endl;
	printf("		向下倾斜 %0.2f   斜率:= %0.2f \n", degree, weld_k);

	cout << " 4. 初始点错位距离 " << endl;
	if (missDistance > 20)
		printf("		错位距离： %0.2f   工件未对齐 \n", missDistance);
	else
		printf("		错位距离： %0.2f   工件对齐 \n", missDistance);


	if (weldP1.x < weldP2.x) {
		cout << "焊缝在交叉点上方" << endl;
	}
	else
		cout << "焊缝在交叉点下方" << endl;

	imshow("Results", src);
	imwrite("F:\\MVS_Data\\Results.png", src);
	waitKey(0);
	destroyAllWindows();
}

// 计算焊缝宽度
float WeldCom::weldWidth(Point p1, Point p2) {
	float weld_k = (weldP1.y - weldP2.y) / (weldP1.x - weldP2.x);		// 斜率
	//焊缝特征线的直线方程参数
	float A = weld_k, B = -1, C = weldP1.y - weld_k * weldP1.y;
	float d1 = fabs(A * p1.x + B * p1.y + C) / sqrt(pow(A, 2) + pow(B, 2));
	float d2 = fabs(A * p2.x + B * p2.y + C) / sqrt(pow(A, 2) + pow(B, 2));
	return d1 + d2;
}
