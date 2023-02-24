// Autor: Jiyuan Zhang
// Date: Frb. 2023

#ifndef _H_MAT
#define _H_MAT


#define MAT_MAX 15 //决定了能处理的最大矩阵

#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

class MAT
{
public:
	MAT();
	MAT(int setm,int setn,int kind);//kind=1单位阵，kind=0零矩阵,其它不初始化内容。
	void init(int setm,int setn,int kind);//kind=1单位阵，kind=0零矩阵,其它不初始化内容。


	//这些关键数本应该作为private的。但是为了方便，我也做成了public
	int m;//行数
	int n;//列数
	double num[MAT_MAX][MAT_MAX];//矩阵数据内容


	//特殊的矩阵
	MAT submat(int a,int b,int lm,int ln);//获取矩阵一部分
	void fillsubmat(int a,int b,MAT s);//填充子矩阵



	//向量专用
	double absvec();//这个是向量的长度。不是个别元素的绝对值。
	double square();//向量长度的平方
	friend MAT operator ^(MAT a,MAT b);//叉乘


	//运算
	friend MAT operator *(double k,MAT a);
	friend MAT operator *(MAT a,double k);
	friend MAT operator /(MAT a,double k);
	friend MAT operator *(MAT a,MAT b);
	friend MAT operator +(MAT a,MAT b);
	friend MAT operator -(MAT a,MAT b);
	friend MAT operator ~(MAT a);//转置	
	friend MAT operator /(MAT a,MAT b);//a*inv(b)
	friend MAT operator %(MAT a,MAT b);//inv(a)*b


	//MAT inv();//逆矩阵

private:
	//为了用高斯消元法，做的一些函数
	void rowexchange(int a, int b);//交换两行
	void rowmulti(int a,double k);//某一行乘以系数
	void rowadd(int a,int b,double k);//对某一行加减另一行的倍数

	
	void columnexchange(int a, int b);//交换两列
	void columnmulti(int a,double k);//某一列乘以系数
	void columnadd(int a,int b,double k);//对某一列加减另一列的倍数
	

};



#endif






