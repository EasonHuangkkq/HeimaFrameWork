/*
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============
常用方法封装
// #include"algorithms.h"
=====================================================*/
#pragma once
#include<cmath>
using namespace std;

static const double Pi =3.14159265359;
static const double Pi2=6.28318530718;
static const double P2i=1.57079632679;
static const double Deg2rad=Pi/180;
static const double Rad2deg=180/Pi;

#define For2 for(int i(0);i!=2;++i)
#define For3 for(int i(0);i!=3;++i)
#define For4 for(int i(0);i!=4;++i)
#define For6 for(int i(0);i!=6;++i)
#define For(x) for(int i(0);i!=x;++i)

namespace Alg{
//==符号函数======================
	inline int sign(const int &a){return a<0?-1:1;}
	inline int sign(const float &a){return a<0?-1:1;}
	inline int sign(const double &a){return a<0?-1:1;}
//==双曲正割，钟形=================
	inline double sech(const float &x){return 2/(exp(x)+exp(-x));}
	inline double sech(const double &x){return 2/(exp(x)+exp(-x));}
//==高斯函数，钟形，高阶,更收缩=======
	inline double gause(const float &x){return exp(-x*x);}
	inline double gause(const double &x){return exp(-x*x);}
//==线性插值映射到0~1====================
	double zero2one(const float &low,const float &up,const float &x);
	double zero2one(const double &low,const double &up,const double &x);
//==上下限区间，触发过边界return 1====================
	template<typename T1,typename T2,typename T3>
	bool clip(T1&x, const T2&a,const T3&b);
	template<typename T1,typename T2>
	bool clip(T1&x, T2 lim);//lim不确定正负，不能const
	template<typename T1,typename T2,typename T3>
	T1 cliped(const T1&x, const T2&a,const T3&b){
		T1 tmp=x;
		clip(tmp,a,b);
		return tmp;
	};
	template<typename T1,typename T2>
	T1 cliped(const T1&x, T2 lim){//lim不确定正负，不能const
		T1 tmp=x;
		clip(tmp,lim);
		return tmp;
	}
//==递增函数=============当cmd=tgt时return 1
	template<typename T1, typename T2, typename T3>
	inline bool cmd2out1step(const T1 &cmd,T2 &tgt,T3 step){ //step不确定正负，不能const
		if(step<0){step=-step;}
		if(tgt-cmd<=-step){tgt+=step;return 0;}
		else if(tgt-cmd>=step){tgt-=step;return 0;}
		else{tgt=cmd;return 1;}
	}
//==阀值=============
	template<typename T1,typename T2>
	inline void thresh(T1 &x,T2 threshold){//threshold不确定正负，不能const
		if(threshold<0){threshold=-threshold;}
		if(x>threshold){x-=threshold;}
		else if(x<-threshold){x+=threshold;}
		else{x=0;}
	}
	template<typename T1,typename T2>
	inline T1 threshed(T1 x,T2 threshold){
		thresh(x,threshold);
		return x;
	}
//==双斜率折线============
	template<typename T>
	inline void grind(T &x,const T &threshold,const T &k,const T &k2=1){
		if(x>threshold){x=k2*(x-threshold)+k*threshold;}
		else if(x<-threshold){x=k2*(x+threshold)-k*threshold;}
		else{x*=k;}
	}
	template<typename T>
	inline T grinded(T x,const T &threshold,const T &k,const T &k2=1){
		grind(x,threshold,k,k2);
		return x;
	}
//==一阶滤波 class============
class filterOneClass{
public:
	filterOneClass();
	filterOneClass(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1);
	double init(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1);
	double reset(double cutF,double ki=0,double y0=0);
	double setBase(double y0=0);
	void setCutF(double cutF,double ki=0);
	void setMaxDerivate(double maxDerivate);
	void setMaxAcc(double maxAcc);
	const double &filt(const double &x);
private:
	double dt;
	double y,errSum;
	double k,ks,step,maxStep,maxAcc;
};
//==二阶低通滤波 class============
class filterTwoClass{
public:
	filterTwoClass();
	filterTwoClass(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1,double damp=1);
	double init(double dt,double cutF,double ki=0,double y0=0,double maxDerivate=-1,double damp=1);
	double reset(double cutF,double ki=0,double y0=0,double damp=1);
	double setBase(double y0=0);
	void setCutF(double cutF,double ki=0,double damp=1);
	void setDamp(double damp2);
	void setMaxDerivate(double maxDerivate);
	const double &filt(const double &x);
	double getV();
private:
	double dt;
	double xn1,xn2,y,yn1,yn2;
	double step,errSum,ks,maxStep;
	double wc;
	double b0,b1,b2,a1,a2;
};
}//namespace