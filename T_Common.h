#ifndef T_COMMON_H
#define T_COMMON_H

#include <opencv2\opencv.hpp>
#include <iostream>


using namespace std;
using namespace cv;

/* WeldCom����ʶ��� �������ͣ��ٸ������ͣ����ø���ģ��Ĵ�����
����� ������̳�WeldCom����Ҫ�����ǣ����紴��Lap����󣬵���ʶ�����Ͳ���Lap���ͣ��Ͳ��ܵ����������͵� �������
����� WeldCom �̳�����4�����ͣ�����Ե��ö������Ĵ��������õ� ������P1 P2�����ܵ���weldLine() ����
*/

// :public Lap, public SingleBevel, public V, public Square
class WeldCom
{
public:
	WeldCom(Mat& src);
	~WeldCom();

	Point CrossPointlocation_step1(Mat&);									/* ����� ���Զ�λ*/
	void CrossPointlocation_step2(Mat&, int);								/* ������С�����ڽ��� ����㶨λ*/
	void edgePointlocation(Mat& src);										/* ��Ե�㶨λ*/
	static bool findLight(Mat& binary, vector<int>& lightIdx, string mode);	/* ���������ڵ� ��������*/
	
	void weldLine(Mat& src);												/* ��ʾʶ����*/


private:
	void regionSegment(Mat&);								/* ������ָ�*/
	Mat Multidiagonal(int lw);								/* ��������˾���*/
	vector<Point> wds_Roi(Mat& roi, int upperRowIndex);		/* ��ֱ�Ҷ����ķ� ����CrossPointlocation_step1�� ���ǰ*/
	bool gapNum(Mat& roi, int T);							/* ����Ƿ���ڼ�϶����ֵ = 2 pixel��*/
	
	void typeArray();								/* ������������ */
	float weldWidth(Point, Point);					/* ������*/


public:
	Point crossPoint;								/* ���������*/
	vector<Mat> regionROI;							/* �洢�ĸ�ROI��ͼ*/
	Point startP1;									/* laser1 V�ͺ��� �㼯�ָ� Ҫ�õ������ (�±�)*/
	Point startP2;									/* laser2 V�ͺ��� �㼯�ָ� Ҫ�õ������ (�ϱ�)*/
	Mat laser1;										/* laser1 ���� (�±�)*/
	Mat laser2;										/* laser2 ���� (�ϱ�)*/

	Point edgeP1;									/* laser1 ������Ե�� (�±�)*/
	Point edgeP2;									/* laser2 ������Ե�� (�ϱ�)*/
	Mat laser1_OnWorkpiece;							/* ��Ե����ߵĹ�������ͼ��*/
	Mat laser2_OnWorkpiece;

	string wledType;								/* ʶ��ĺ�������*/
	Point initP1;									/* ��ʼ������*/
	Point initP2;
	double weldWidth1;								/* �����¿ڿ��*/
	double weldWidth2;
	double degree;									/* ������б�Ƕ�*/
	Point weldP1;									/* laser1 ���������� */
	Point weldP2;									/* laser2 ����������*/
};

#endif	// T_COMMON_H
