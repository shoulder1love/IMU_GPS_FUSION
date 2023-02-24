// Autor: Jiyuan Zhang
// Date: Frb. 2023
#ifndef _H_NAVI
#define _H_NAVI

#include "mat.h"

extern const double deg2rad;//角度转换弧度的系数
extern const double rad2deg;//弧度转换角度的系数
extern const double we;//地球自转角速率


extern double dTins;

extern MAT qa;
extern MAT tspeed;
extern MAT tpos;

extern MAT biasacc;
extern MAT biasgyro;

void para();


MAT setoula(double yawdeg, double pitchdeg, double rolldeg);
MAT getoula(MAT qb);

void ins_gyroacc(MAT gyro, MAT acc);


/*
状态量顺序为：纬经高，东北天速度，东北天角度，三个陀螺仪，三个加速度计。
*/

void stateupdate();
void sat(double lati, double longi);//卫星处理。集成了卡尔曼滤波和误差补偿



#endif