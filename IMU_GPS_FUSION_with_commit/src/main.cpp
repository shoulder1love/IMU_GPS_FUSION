// Autor: Jiyuan Zhang
// Date: Frb. 2023

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include "navi.h"

FILE* fpin;
FILE* fpout;
char buf[300];

MAT ENUoZero(3, 1, 0);
// MAT gps2ecef(double lat, double lon, double alt)
// {
    
//     double pi = 3.1415926;
//     double lambda = lat * pi /180;
//     double phi = lon * pi /180;
//     double s = sin(lambda);
//     double N = 6378137.0/sqrt(1-0.00167781*s*s);

//     double sin_lambda = sin(lambda);
//     double cos_lambda = cos(lambda);
//     double cos_phi = cos(phi);
//     double sin_phi = sin(phi);

//     double x = (alt + N) * cos_lambda * cos_phi; // 其他算法 0.01alt
//     double y = (alt + N) * cos_lambda * sin_phi;
//     double z = (alt + (1 - 0.00167781) * N) * sin_lambda;
    
    

// }

MAT ecef2enu(double lat0, double lon0, double alt0)
{
    MAT pos(3,1,0);
    double pi = 3.1415926;
    double pos_ecef_x = ENUoZero.num[0][0];
    double pos_ecef_y = ENUoZero.num[1][0];
    double pos_ecef_z = ENUoZero.num[2][0];
    
    double lambda = lat0 * pi /180;
    double phi = lon0 * pi /180;
    double s = sin(lambda);
    double N = 6378137.0/sqrt(1-0.00167781*s*s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (alt0 + N) * cos_lambda * cos_phi;
    double y0 = (alt0 + N) * cos_lambda * sin_phi;
    double z0 = (alt0 + (1 - 0.00167781) * N) * sin_lambda;

    double xd = pos_ecef_x - x0;
    double yd = pos_ecef_y - y0;
    double zd = pos_ecef_z - z0;
    
    // xEast
    double x_enu = -sin_phi * xd + cos_phi * yd;
    // yNorth
    double y_enu = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    // zUp
    double z_enu = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    pos.num[0][0] = x_enu;
    pos.num[1][0] = y_enu;
    pos.num[2][0] = z_enu;
    return pos;
}


double inputdouble(char* s, int n)
{
	int p;
	int c = 0;
	double lf;
	for (p = 0; p < 1000; p++)
	{
		if (c >= n)
		{
			break;
		}
		if (s[p] == ',')
		{
			c++;
		}
	}
	sscanf(&s[p], "%lf", &lf);
	return lf;
}

long long inputll(char* s, int n)
{
	int p;
	int c = 0;
	long long lld;
	for (p = 0; p < 1000; p++)
	{
		if (c >= n)
		{
			break;
		}
		if (s[p] == ',')
		{
			c++;
		}
	}
	sscanf(&s[p], "%lld", &lld);
	return lld;
}

MAT accold;
MAT gyroold;
long long timestampeold = 0;
void init(double lat, double lon,double r[4])
{
	para();


	dTins = 0.001;
	
	
	//Set initial values, attitude, velocity, position 设置初值，姿态、速度、位置

	// attitude
	qa.num[0][0] = r[0];
	qa.num[1][0] = r[1];
	qa.num[2][0] = r[2];
	qa.num[3][0] = r[3];

	// velocity
	tspeed.num[0][0] = 0;
	tspeed.num[1][0] = 0;
	tspeed.num[2][0] = 0;

	// position
	tpos.num[0][0] = lat*deg2rad;
	tpos.num[1][0] = lon*deg2rad;
	tpos.num[2][0] = 0;


}

MAT qref(4, 1, 1);// Quaternion  源文件四元数
MAT ouref(3, 1, 0);// Quaternion 参考四元数

int frame = 0; 

double latgnss, longnss;

// Interpolation operations 

// The frequency of IMU in the source data is too low for correct prediction,
// so interpolation is used
void chazhi(long long timestamp,double acc[3],double gyro[3])
{
	MAT accnew(3, 1, 0);
	MAT gyronew(3, 1, 0);
	MAT acct;
	MAT gyrot;
	MAT ou;
    MAT enuold(3, 1, 0);
    MAT enunew(3, 1, 0);
	long long ts;
	accnew.num[0][0] = acc[0];
	accnew.num[1][0] = acc[1];
	accnew.num[2][0] = acc[2];

	gyronew.num[0][0] = gyro[0];
	gyronew.num[1][0] = gyro[1];
	gyronew.num[2][0] = gyro[2];

	if (frame == 1)
	{
	}
	else
	{
		for (ts = timestampeold + 1; ts <= timestamp; ts++)
		{
			acct = (accold * (double)(timestamp - ts) + accnew * (double)(ts - timestampeold)) / (double)(timestamp - timestampeold);
			gyrot = (gyroold * (double)(timestamp - ts) + gyronew * (double)(ts - timestampeold)) / (double)(timestamp - timestampeold);

			
			ins_gyroacc(gyrot, acct);//Inertial navigation (prediction)惯性导航
			stateupdate();// Status Updates

			
			

            double gps_new_0 = ecef2enu(tpos.num[0][0]*rad2deg,tpos.num[1][0] * rad2deg, tpos.num[2][0]).num[0][0];
            double gps_new_1 = ecef2enu(tpos.num[0][0]*rad2deg,tpos.num[1][0] * rad2deg, tpos.num[2][0]).num[1][0];
            double gps_new_2 = ecef2enu(tpos.num[0][0]*rad2deg,tpos.num[1][0] * rad2deg, tpos.num[2][0]).num[2][0]; 	
            
            double gos_old_0 = ecef2enu(latgnss,longnss,0.0).num[0][0];
            double gos_old_1 = ecef2enu(latgnss,longnss,0.0).num[1][0];
            double gos_old_2 = ecef2enu(latgnss,longnss,0.0).num[2][0];
			ou = getoula(qa);


            fprintf(fpout, "%12.6lf %12.6lf %12.6lf ", ou.num[0][0], ou.num[1][0], ou.num[2][0]);
			fprintf(fpout, "%12.6lf %12.6lf %12.6lf ", tspeed.num[0][0], tspeed.num[1][0], tspeed.num[2][0]);
			fprintf(fpout, "%18.9lf %18.9lf %12.6lf ", tpos.num[0][0]*rad2deg, tpos.num[1][0] * rad2deg, tpos.num[2][0]);
			fprintf(fpout,"%18.9lf %18.9lf ",latgnss,longnss);
            fprintf(fpout,"%18.9lf %18.9lf %18.9lf ",gps_new_0,gps_new_1,0.0); // gps_new_2);	
            fprintf(fpout,"%18.9lf %18.9lf %18.9lf\n",gos_old_0,gos_old_1,0.0); // gos_old_2);
		}


		qa = qref; //Aligned stance 对齐姿态


	}

	accold = accnew;
	gyroold = gyronew;
	timestampeold = timestamp;


}

int main(int argc,char* argv[])
{
	
	double acc[3];
	double gyro[3];
	double r[4];
	
	long long timestamp;
	fpin = fopen(argv[1], "r");
	fpout = fopen(argv[2], "w");
 

	while (1)
	{
		// fgets() 函数用来从指定的文件中读取一个字符串，并保存到字符数组中
		if (fgets(buf, 1000, fpin) == NULL)
		{
			break;
		}
		else
		{
			if (frame == 0)// Skip empty rows and table headers 跳过空行和表头
			{
				if (buf[0] == 'l')
				{
					frame++;
				}
			}
			else if (frame == 1)// Initial value 
			{
				latgnss = inputdouble(buf, 0);
				longnss = inputdouble(buf, 1);
				acc[0] = inputdouble(buf, 2);
				acc[1] = inputdouble(buf, 3);
				acc[2] = inputdouble(buf, 4);
				timestamp = inputll(buf, 5);
				gyro[0] = inputdouble(buf, 7);
				gyro[1] = inputdouble(buf, 8);
				gyro[2] = inputdouble(buf, 9);
				r[1] = inputdouble(buf, 18);
				r[2] = inputdouble(buf, 19);
				r[3] = inputdouble(buf, 20);
				r[0] = inputdouble(buf, 21);
                ENUoZero.num[0][0] = latgnss;
                ENUoZero.num[1][0] = longnss;
                ENUoZero.num[2][0] = 0.0;
                
				init(latgnss,longnss,r);

				chazhi(timestamp, acc, gyro);
				frame++;
			}
			else // 方向cos矩阵 3×3
			{			


				acc[0] = inputdouble(buf, 2);
				acc[1] = inputdouble(buf, 3);
				acc[2] = inputdouble(buf, 4);
				timestamp = inputll(buf, 5);
				gyro[0] = inputdouble(buf, 7);
				gyro[1] = inputdouble(buf, 8);
				gyro[2] = inputdouble(buf, 9);
				
				r[1] = inputdouble(buf, 18);
				r[2] = inputdouble(buf, 19);
				r[3] = inputdouble(buf, 20);
				r[0] = inputdouble(buf, 21);
				qref.num[0][0] = r[0];
				qref.num[1][0] = r[1];
				qref.num[2][0] = r[2];
				qref.num[3][0] = r[3];
				ouref = getoula(qref);
				
				chazhi(timestamp, acc, gyro);

				if (buf[0] == ',')
				{
				}
				else
				{
					latgnss = inputdouble(buf, 0);
					longnss = inputdouble(buf, 1);

					sat(latgnss*deg2rad, longnss*deg2rad);
				}
				frame++;
			}
		}
	}


	fclose(fpin);
	fclose(fpout);
	return 0;
}