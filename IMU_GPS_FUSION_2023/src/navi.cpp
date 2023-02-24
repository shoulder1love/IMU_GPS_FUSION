// Autor: Jiyuan Zhang
// Date: Frb. 2023
#include "navi.h"

const double deg2rad=M_PI/180.0;
const double rad2deg=180.0*M_1_PI;
const double we=7.2921158e-5;

double dTins;

MAT qa(4,1,1);
MAT tspeed(3,1,0);
MAT tpos(3,1,0);

MAT biasacc(3,1,0);
MAT	biasgyro(3,1,0);

double Rprim,Rmeri,ge;

MAT accn;





MAT Cbn(MAT qu)
{
	double q0,q1,q2,q3;
	q0=qu.num[0][0];
	q1=qu.num[1][0];
	q2=qu.num[2][0];
	q3=qu.num[3][0];
	double q00,q11,q22,q33;
	q00=q0*q0;
	q11=q1*q1;
	q22=q2*q2;
	q33=q3*q3;

	MAT c(3,3,0);

	c.num[0][0]=q00+q11-q22-q33;
	c.num[0][1]=2*(q1*q2-q0*q3);
	c.num[0][2]=2*(q1*q3+q0*q2);
	c.num[1][0]=2*(q1*q2+q0*q3);
	c.num[1][1]=q00-q11+q22-q33;
	c.num[1][2]=2*(q2*q3-q0*q1);
	c.num[2][0]=2*(q1*q3-q0*q2);
	c.num[2][1]=2*(q2*q3+q0*q1);
	c.num[2][2]=q00-q11-q22+q33;

	return c;
}

MAT getoula(MAT qb)
{
	MAT cbn=Cbn(qb);
	MAT attideg(3,1,0);
	MAT cnb=(~cbn);
	MAT ou(3,1,0);
	ou.num[0][0]=atan2(-cnb.num[1][0],cnb.num[1][1]);
	ou.num[1][0] = atan2(cnb.num[1][2], sqrt(cnb.num[0][2] * cnb.num[0][2] + cnb.num[2][2] * cnb.num[2][2]));
	ou.num[2][0]=atan2(-cnb.num[0][2],cnb.num[2][2]);
	return ou*rad2deg;
}




MAT quatupdate(MAT q1,MAT th)
{
	double thabs=th.absvec();	


	MAT TH(4,4,0);
	TH.num[0][1]=(-th.num[0][0]);
	TH.num[0][2]=(-th.num[1][0]);
	TH.num[0][3]=(-th.num[2][0]);
	TH.num[1][2]=(th.num[2][0]);
	TH.num[1][3]=(-th.num[1][0]);
	TH.num[2][3]=(th.num[0][0]);
	TH=TH-(~TH);



	MAT A=MAT(4,4,1)*cos(thabs*0.5);

	if(thabs<1e-6)
	{
		A=A+TH*0.5;
	}
	else
	{
		A=A+TH*sin(thabs*0.5)/thabs;
	}
	return A*q1;
}

MAT setoula(double yawdeg,double pitchdeg,double rolldeg)
{
	MAT qu(4,1,1);

	MAT th;
	th.init(3,1,0);
	th.num[2][0]=yawdeg*deg2rad;
	qu=quatupdate(qu,th);

	th.init(3,1,0);
	th.num[0][0]=pitchdeg*deg2rad;
	qu=quatupdate(qu,th);

	th.init(3,1,0);
	th.num[1][0]=rolldeg*deg2rad;
	qu=quatupdate(qu,th);

	return qu;
}




MAT tWe()
{
	double latitude=tpos.num[0][0];
	MAT wiet(3,1,0);
	wiet.num[1][0]=cos(latitude)*we;
	wiet.num[2][0]=sin(latitude)*we;//天
	return wiet;
}

MAT tWv()
{
	//北向速度会引起东向转动；东向速度会引起北向和天向转动
	double lat=tpos.num[0][0];
	double H=tpos.num[2][0];
	double vE=tspeed.num[0][0];
	double vN=tspeed.num[1][0];
	
	MAT wen(3,1,0);
	wen.num[0][0]=(-vN/(Rmeri+H));//东
	wen.num[1][0]=vE/(Rprim+H);//北
	wen.num[2][0]=wen.num[1][0]*tan(lat);//天
	

	return wen;
}

void earthmodelupdate()
{
	const double Re=6378137;
	const double ee=1/298.25722;

	double lat=tpos.num[0][0];//纬度
	double sin_phi=sin(lat);
	double sin2_phi=sin_phi*sin_phi;
	Rprim=Re*(1+ee*sin2_phi);
	Rmeri=Re*(1-2*ee+3*ee*sin2_phi);

	double g0=9.7803267714;
	double sl=sin(lat);
	double s2l=sin(2*lat);
	double sl2=sl*sl;
	double s2l2=s2l*s2l;
	double hr=1+tpos.num[2][0]/Re;


	ge=g0*(1+0.0053024*sl2-0.0000059*s2l2)/(hr*hr);
}




MAT acc1;
MAT gyro1;
MAT wien;
MAT wenn;
MAT wnbb;
MAT gn;
MAT dpos;



void ins_gyroacc(MAT gyro,MAT acc)
{
	


	MAT cbn;
	MAT an;

	double H;


	
	earthmodelupdate();

	
	acc1=acc+biasacc;
	gyro1=gyro+biasgyro;

	
	//一、计算姿态
	wien=tWe();
	wenn=tWv();
	cbn=Cbn(qa);
	wnbb=gyro1-(~cbn)*(wien+wenn);
	qa=quatupdate(qa,wnbb*dTins);//更新姿态


	//二、计算速度
	accn=cbn*acc1;
	gn.init(3,1,0);
	gn.num[2][0]=(-ge);
	an=accn-((wien+wien+wenn)^tspeed)+gn;
	tspeed=tspeed+dTins*an;//更新速度

	tspeed.num[2][0] = 0;//锁定高度

	//三、计算位置
	dpos.init(3,1,0);
	H=tpos.num[2][0];
	dpos.num[0][0]=tspeed.num[1][0]/(Rmeri+H);//北向速度得到纬度
	dpos.num[1][0]=tspeed.num[0][0]/((Rprim+H)*cos(tpos.num[0][0]));//东向速度得到经度
	dpos.num[2][0]=tspeed.num[2][0];
	tpos=tpos+dTins*dpos;//更新位置
}



MAT Phi(15,15,1);
MAT Pk(15,15,0);
MAT Eye(15,15,1);
MAT Q(15,15,0);
MAT Q0(15,15,0);
MAT R(3,3,1);


void para()//调参数
{
	dTins=0.001;

	Pk.num[0][0]=1e-12;
	Pk.num[1][1]=1e-12;
	Pk.num[2][2]=0;
	Pk.num[3][3]=0.1;
	Pk.num[4][4]=0.1;
	Pk.num[5][5]=0.1;
	Pk.num[6][6]=1e-3;
	Pk.num[7][7]=1e-3;
	Pk.num[8][8]=1e-3;

	Pk.num[9][9] = 1e-6;
	Pk.num[10][10] = 1e-6;
	Pk.num[11][11] = 1e-6;

	Pk.num[12][12] = 1e-4;
	Pk.num[13][13] = 1e-4;
	Pk.num[14][14] = 1e-4;


	// 改进参数
	Q0.num[3][3]=1e-2;
	Q0.num[4][4]=1e-2;
	Q0.num[5][5]=1e-2;
	Q0.num[6][6]=1e-4;
	Q0.num[7][7]=1e-4;
	Q0.num[8][8]=1e-4;
	


	Q0.num[9][9] = 1e-8;
	Q0.num[10][10] = 1e-8;
	Q0.num[11][11] = 1e-8;
	Q0.num[12][12] = 1e-6;
	Q0.num[13][13] = 1e-6;
	Q0.num[14][14] = 1e-6;



	R.num[0][0]=1e-12;
	R.num[1][1]=1e-12;
	R.num[2][2] = 0.01;

	qa=setoula(0.5,-0.4,0.3);
	tpos.num[0][0]=40*deg2rad;
	tpos.num[1][0]=120*deg2rad;
	tpos.num[2][0]=500;
}

MAT solve(MAT Z,MAT H)
{
	MAT Pkk=Phi*Pk*(~Phi)+Q;
	MAT K=Pkk*(~H)/(H*Pkk*(~H)+R);
	MAT X=K*Z;//因为每次滤波之后补偿了误差，所以状态预测总是0。
	
	MAT IKH=Eye-K*H;
	Pk=IKH*Pkk*(~IKH)+K*R*(~K);
	

	Phi.init(15,15,1);
	
	
	return X;
}



void sat(double lati,double longi)
{
	MAT Z(3,1,0);
	Z.num[0][0]=tpos.num[0][0]-lati;
	Z.num[1][0]=tpos.num[1][0]-longi;
	Z.num[2][0] = tpos.num[1][0] - 0;

	MAT H(3,15,1);

	MAT X=solve(Z,H);
	Q.init(15,15,0);


	tpos=tpos-X.submat(0,0,3,1);
	tspeed=tspeed-X.submat(3,0,3,1);
	qa=quatupdate(qa,(~Cbn(qa))*X.submat(6,0,3,1));
	biasgyro=biasgyro-X.submat(9,0,3,1);
	biasacc=biasacc-X.submat(12,0,3,1);
}



MAT Fpp(3, 3, 0);
MAT Fvp(3, 3, 0);
MAT Fpv(3, 3, 0);
MAT Fvv(3, 3, 0);
MAT Fav(3, 3, 0);
MAT Fpa(3, 3, 0);
MAT Fva(3, 3, 0);
MAT Faa(3, 3, 0);


MAT getF()
{

	MAT F(15,15,0);
	
	
	double RpH1=1.0/(Rprim+tpos.num[2][0]);
	double RmH1=1.0/(Rmeri+tpos.num[2][0]);

	double cosphi=cos(tpos.num[0][0]);
	double secphi=1.0/cosphi;
	double sinphi=sin(tpos.num[0][0]);
	double tanphi=tan(tpos.num[0][0]);

	double vE=tspeed.num[0][0];
	double vN=tspeed.num[1][0];
	double vU=tspeed.num[2][0];



	Fpp.num[0][2]=RmH1*RmH1*(-vN);
	Fpp.num[1][0]=RpH1*vE*secphi*tanphi;
	Fpp.num[1][2]=RpH1*RpH1*(-vE)*secphi;
	F.fillsubmat(0,0,Fpp);
	
	
	Fvp.num[0][1]=RmH1;
	Fvp.num[1][0]=RpH1*secphi;
	Fvp.num[2][2]=1;
	F.fillsubmat(0,3,Fvp);
	


	Fpv.num[0][0]=2*we*cosphi*vN+2*we*sinphi*vU+vN*vE*RpH1*secphi*secphi;
	Fpv.num[0][2]=RpH1*RpH1*(vE*vU-vN*vE*tanphi);
	Fpv.num[1][0]=(-(2*vE*we*cosphi+vE*vE*RpH1*secphi*secphi));
	Fpv.num[1][2]=RmH1*RmH1*vN*vU+RpH1*RpH1*vE*vE*tanphi;
	Fpv.num[2][0]=(-2.0)*vE*we*sinphi;
	Fpv.num[2][2]=(-RmH1*RmH1*vN*vN-RpH1*RpH1*vE*vE);
	F.fillsubmat(3,0,Fpv);


	Fvv.num[0][0]=(vN*tanphi-vU)*RpH1;
	Fvv.num[0][1]=2.0*we*sinphi+vE*RpH1*tanphi;
	Fvv.num[0][2]=(-2.0)*we*cosphi-vE*RpH1;
	Fvv.num[1][0]=(-2.0)*(we*sinphi+vE*RpH1*tanphi);
	Fvv.num[1][1]=(-RmH1)*vU;
	Fvv.num[1][2]=(-RmH1)*vN;
	Fvv.num[2][0]=2.0*(we*cosphi+vE*RpH1);
	F.fillsubmat(3,3,Fvv);


	double fE=accn.num[0][0];
	double fN=accn.num[1][0];
	double fU=accn.num[2][0];
	
	
	Fav.num[0][1]=(-fU);
	Fav.num[0][2]=fN;
	Fav.num[1][0]=fU;
	Fav.num[1][2]=(-fE);
	Fav.num[2][0]=(-fN);
	Fav.num[2][1]=fE;
	F.fillsubmat(3,6,Fav);


	
	Fpa.num[0][2]=vN*RmH1*RmH1;
	Fpa.num[1][0]=(-we)*sinphi;
	Fpa.num[1][2]=(-vE)*RpH1*RpH1;
	Fpa.num[2][0]=we*cosphi+vE*RpH1*secphi*secphi;
	Fpa.num[2][2]=(-vE)*tanphi*RpH1*RpH1;
	F.fillsubmat(6,0,Fpa);
	

	
	Fva.num[0][1]=(-RmH1);
	Fva.num[1][0]=RpH1;
	Fva.num[2][0]=RpH1*tanphi;
	F.fillsubmat(6,3,Fva);

	Faa.num[0][1]=we*sinphi+vE*RpH1*tanphi;
	Faa.num[2][0]=we*cosphi+vE*RpH1;
	Faa.num[2][1]=vN*RmH1;
	Faa.num[0][2]=(-Faa.num[2][0]);
	Faa.num[1][0]=(-Faa.num[0][1]);
	Faa.num[1][2]=(-Faa.num[2][1]);
	F.fillsubmat(6,6,Faa);
	
	MAT cbn=Cbn(qa);
	F.fillsubmat(6,9,(-1)*cbn);
	F.fillsubmat(3,12,cbn);
	return F;
}


void stateupdate()
{
	Phi=(Eye+getF()*dTins)*Phi;
	Q=Q+Q0*dTins;
}
