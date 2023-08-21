#ifndef T_COMMON_H
#define T_COMMON_H

#include <opencv2\opencv.hpp>
#include <iostream>


using namespace std;
using namespace cv;

/* WeldCom类中识别出 焊缝类型，再根据类型，调用各个模块的处理函数
如果是 其他类继承WeldCom，主要问题是，例如创建Lap对象后，调用识别类型不是Lap类型，就不能调用其他类型的 处理代码
如果是 WeldCom 继承其他4个类型，则可以调用多个父类的处理函数，得到 特征点P1 P2，才能调用weldLine() 画线
*/

// :public Lap, public SingleBevel, public V, public Square
class WeldCom
{
public:
	WeldCom(Mat& src);
	~WeldCom();

	Point CrossPointlocation_step1(Mat&);									/* 交叉点 初略定位*/
	void CrossPointlocation_step2(Mat&, int);								/* 对搜索小窗口内进行 交叉点定位*/
	void edgePointlocation(Mat& src);										/* 边缘点定位*/
	static bool findLight(Mat& binary, vector<int>& lightIdx, string mode);	/* 检测光条存在的 行列索引*/
	
	void weldLine(Mat& src);												/* 显示识别结果*/


private:
	void regionSegment(Mat&);								/* 四区域分割*/
	Mat Multidiagonal(int lw);								/* 创建卷积核矩阵*/
	vector<Point> wds_Roi(Mat& roi, int upperRowIndex);		/* 垂直灰度重心法 ，在CrossPointlocation_step1中 拟合前*/
	bool gapNum(Mat& roi, int T);							/* 检测是否存在间隙（阈值 = 2 pixel）*/
	
	void typeArray();								/* 焊缝类型数组 */
	float weldWidth(Point, Point);					/* 焊缝宽度*/


public:
	Point crossPoint;								/* 交叉点坐标*/
	vector<Mat> regionROI;							/* 存储四个ROI子图*/
	Point startP1;									/* laser1 V型焊缝 点集分割 要用到的起点 (下边)*/
	Point startP2;									/* laser2 V型焊缝 点集分割 要用到的起点 (上边)*/
	Mat laser1;										/* laser1 单线 (下边)*/
	Mat laser2;										/* laser2 单线 (上边)*/

	Point edgeP1;									/* laser1 工件边缘点 (下边)*/
	Point edgeP2;									/* laser2 工件边缘点 (上边)*/
	Mat laser1_OnWorkpiece;							/* 边缘点左边的工件光条图像*/
	Mat laser2_OnWorkpiece;

	string wledType;								/* 识别的焊缝类型*/
	Point initP1;									/* 初始点坐标*/
	Point initP2;
	double weldWidth1;								/* 焊缝坡口宽度*/
	double weldWidth2;
	double degree;									/* 焊缝倾斜角度*/
	Point weldP1;									/* laser1 焊缝特征点 */
	Point weldP2;									/* laser2 焊缝特征点*/
};

#endif	// T_COMMON_H
