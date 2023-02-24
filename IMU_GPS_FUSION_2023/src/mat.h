// Autor: Jiyuan Zhang
// Date: Frb. 2023

#ifndef _H_MAT
#define _H_MAT


#define MAT_MAX 15 // Determines the maximum matrix that can be processed

#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

class MAT
{
public:
	MAT();
	MAT(int setm,int setn,int kind);//kind=1 Unit matrix ，kind=0 zero matrix
	void init(int setm,int setn,int kind);//kind=1 Unit matrix ，kind=0 zero matrix


	
	int m;// Number of rows
	int n;// Number of colums
	double num[MAT_MAX][MAT_MAX]; //Matrix data content


	//The special matrix
	MAT submat(int a,int b,int lm,int ln);//获取矩阵一部分
	void fillsubmat(int a,int b,MAT s);//填充子矩阵



	//Vector-specific
	double absvec();
	double square();
	friend MAT operator ^(MAT a,MAT b);


	// Operations
	friend MAT operator *(double k,MAT a);
	friend MAT operator *(MAT a,double k);
	friend MAT operator /(MAT a,double k);
	friend MAT operator *(MAT a,MAT b);
	friend MAT operator +(MAT a,MAT b);
	friend MAT operator -(MAT a,MAT b);
	friend MAT operator ~(MAT a);
	friend MAT operator /(MAT a,MAT b);//a*inv(b)
	friend MAT operator %(MAT a,MAT b);//inv(a)*b


	//MAT inv();//逆矩阵

private:
	
	void rowexchange(int a, int b);
	void rowmulti(int a,double k);
	void rowadd(int a,int b,double k);

	
	void columnexchange(int a, int b);
	void columnmulti(int a,double k);
	void columnadd(int a,int b,double k);
	

};



#endif






