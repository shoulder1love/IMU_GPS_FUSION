// Autor: Jiyuan Zhang
// Date: Frb. 2023

#include "mat.h"

double mind(double a,double b)
{
	double c=a;
	if(b<c)
	{
		c=b;
	}
	return c;
	
}
int mini(int a,int b)
{
	int c=a;
	if(b<c)
	{
		c=b;
	}
	return c;
	
}


//不要在成员函数内调用构造函数
MAT::MAT()
{
	init(1,1,0);
}
MAT::MAT(int setm,int setn,int kind)
{
	init(setm,setn,kind);
}
void MAT::init(int setm,int setn,int kind)
{

	m=setm;
	n=setn;
	if((kind==0)||(kind==1))
	{
		memset(num,0,MAT_MAX*MAT_MAX*sizeof(double));
	}

	if(kind==1)
	{
		int x;
		
		int xend=mini(m,n);
		for(x=0;x<xend;x++)
		{
			num[x][x]=1;
		}
		
	}
}


MAT MAT::submat(int a,int b,int lm,int ln)
{

	int aend=a+lm-1;
	int bend=b+ln-1;


	MAT s(lm,ln,-1);
	int x,y;
	for(x=0;x<lm;x++)
	{
		for(y=0;y<ln;y++)
		{
			s.num[x][y]=num[a+x][b+y];
		}
	}
	return s;
}


void MAT::fillsubmat(int a,int b,MAT s)
{
	int x,y;
	for(x=0;x<s.m;x++)
	{
		for(y=0;y<s.n;y++)
		{
			num[a+x][b+y]=s.num[x][y];
		}
	}
}


MAT operator *(double k,MAT a)
{
	MAT b(a.m,a.n,-1);
	int x,y;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<a.n;y++)
		{
			b.num[x][y]=k*a.num[x][y];
		}
	}
	return b;
}
MAT operator *(MAT a,double k)
{
	return k*a;
}
MAT operator /(MAT a,double k)
{
	return (1/k)*a;
}
MAT operator *(MAT a,MAT b)
{
	MAT c(a.m,b.n,-1);
	int x,y,z;
	double s;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<b.n;y++)
		{
			s=0;
			for(z=0;z<a.n;z++)
			{
				s=s+a.num[x][z]*b.num[z][y];
			}
			c.num[x][y]=s;
		}
	}
	return c;

}

MAT operator +(MAT a,MAT b)
{

	MAT c=a;
	int x,y;
	for(x=0;x<c.m;x++)
	{
		for(y=0;y<c.n;y++)
		{
			c.num[x][y]+=b.num[x][y];
		}
	}
	return c;
}
MAT operator -(MAT a,MAT b)
{

	MAT c=a;
	int x,y;
	for(x=0;x<c.m;x++)
	{
		for(y=0;y<c.n;y++)
		{
			c.num[x][y]-=b.num[x][y];
		}
	}
	return c;
}
MAT operator ~(MAT a)
{
	MAT b(a.n,a.m,-1);
	int x,y;
	for(x=0;x<a.m;x++)
	{
		for(y=0;y<a.n;y++)
		{
			b.num[y][x]=a.num[x][y];

		}
	}
	return b;
}



MAT operator /(MAT a,MAT b)
{
	//Gaussian elimination method 高斯消元法

	int x,xb;
	for(x=0;x<b.n;x++)
	{
		//首先找到最佳的列。让起始元素最大
		double s=0;
		int p=x;
		double sxb;
		for(xb=x;xb<b.n;xb++)
		{
			sxb=fabs(b.num[x][xb]);
			if(sxb>s)
			{
				p=xb;
				s=sxb;
			}
		}

		if(x!=p)
		{
			a.columnexchange(x,p);
			b.columnexchange(x,p);
		}

		
		double k=1/b.num[x][x];
		a.columnmulti(x,k);
		b.columnmulti(x,k);


		for(xb=0;xb<b.n;xb++)
		{
			if(xb!=x)
			{
				k=(-b.num[x][xb]);
				a.columnadd(xb,x,k);
				b.columnadd(xb,x,k);
			}
		}
	}
	
	return a;
}


MAT operator %(MAT a,MAT b) 
{

	int x,xb;
	for(x=0;x<a.m;x++)
	{

		double s=0;
		int p=x;
		double sxb;
		for(xb=x;xb<a.m;xb++)
		{
			sxb=fabs(a.num[xb][x]);
			if(sxb>s)
			{
				p=xb;
				s=sxb;
			}
		}

		if(x!=p)
		{
			a.rowexchange(x,p);
			b.rowexchange(x,p);
		}


		double k=1/a.num[x][x];
		a.rowmulti(x,k);
		b.rowmulti(x,k);


		for(xb=0;xb<a.m;xb++)
		{
			if(xb!=x)
			{
				k=(-a.num[xb][x]);
				a.rowadd(xb,x,k);
				b.rowadd(xb,x,k);
			}
		}
	}
	
	return b;
}


void MAT::rowexchange(int a, int b)
{
	double s[MAT_MAX];
	int ncpy=n*sizeof(double);
	memcpy(s,num[a],ncpy);
	memcpy(num[a],num[b],ncpy);
	memcpy(num[b],s,ncpy);
}
void MAT::rowmulti(int a,double k)
{
	int y;
	for(y=0;y<n;y++)
	{
		num[a][y]=num[a][y]*k;
	}
}
void MAT::rowadd(int a,int b,double k)
{
	int y;
	for(y=0;y<n;y++)
	{
		num[a][y]=num[a][y]+num[b][y]*k;
	}
}

void MAT::columnexchange(int a, int b)
{
	double s;
	int x;
	for(x=0;x<m;x++)
	{
		s=num[x][a];
		num[x][a]=num[x][b];
		num[x][b]=s;
	}
}
void MAT::columnmulti(int a,double k)
{
	int x;
	for(x=0;x<m;x++)
	{
		num[x][a]=num[x][a]*k;
	}
}
void MAT::columnadd(int a,int b,double k)
{
	int x;
	for(x=0;x<m;x++)
	{
		num[x][a]=num[x][a]+num[x][b]*k;
	}
}



double MAT::square()
{
	int x;
	double numx;
	double s=0;
	for(x=0;x<m;x++)
	{
		numx=num[x][0];
		s+=(numx*numx);
	}
	return s;
}
double MAT::absvec()
{
	return sqrt(square());
}

MAT operator ^(MAT a, MAT b)
{
	double ax=a.num[0][0];
	double ay=a.num[1][0];
	double az=a.num[2][0];
	double bx=b.num[0][0];
	double by=b.num[1][0];
	double bz=b.num[2][0];

	MAT c(3,1,-1);
	c.num[0][0]=ay*bz-az*by;
	c.num[1][0]=(-(ax*bz-az*bx));
	c.num[2][0]=ax*by-ay*bx;

	return c;
}

