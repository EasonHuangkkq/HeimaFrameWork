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
=====================================================*/
#include"algorithms.h"
#include<iostream>
namespace Alg{
//==上下限区间，触发过边界return 1====================
	template<typename T1,typename T2,typename T3>
	bool clip(T1&x,const T2&a, const T3&b){
		if(a<=b){
			if(x<=a){x=(T1)a;return 1;}
			else if(x>b){x=(T1)b;return 1;}
		}else{
			if(x<=b){x=(T1)b;return 1;}
			else if(x>a){x=(T1)a;return 1;}
		}
		return 0;
	}
	template bool clip(int&x,const int&a, const int&b);
	template bool clip(int&x,const int&a, const float&b);
	template bool clip(int&x,const int&a, const double&b);
	template bool clip(int&x,const float&a, const int&b);
	template bool clip(int&x,const float&a, const float&b);
	template bool clip(int&x,const float&a, const double&b);
	template bool clip(int&x,const double&a, const int&b);
	template bool clip(int&x,const double&a, const float&b);
	template bool clip(int&x,const double&a, const double&b);
	template bool clip(float&x,const int&a, const int&b);
	template bool clip(float&x,const int&a, const float&b);
	template bool clip(float&x,const int&a, const double&b);
	template bool clip(float&x,const float&a, const int&b);
	template bool clip(float&x,const float&a, const float&b);
	template bool clip(float&x,const float&a, const double&b);
	template bool clip(float&x,const double&a, const int&b);
	template bool clip(float&x,const double&a, const float&b);
	template bool clip(float&x,const double&a, const double&b);
	template bool clip(double&x,const int&a, const int&b);
	template bool clip(double&x,const int&a, const float&b);
	template bool clip(double&x,const int&a, const double&b);
	template bool clip(double&x,const float&a, const int&b);
	template bool clip(double&x,const float&a, const float&b);
	template bool clip(double&x,const float&a, const double&b);
	template bool clip(double&x,const double&a, const int&b);
	template bool clip(double&x,const double&a, const float&b);
	template bool clip(double&x,const double&a, const double&b);

	template<typename T1,typename T2>
	bool clip(T1&x,T2 lim){//lim不确定正负，不能const
		if(lim<0){lim=-lim;}
		if(x<-lim){x=(T1)(-lim);return 1;}
		else if(x>lim){x=(T1)(lim);return 1;}
		return 0;
	}
	template bool clip(int&x, int lim);
	template bool clip(int&x, float lim);
	template bool clip(int&x, double lim);
	template bool clip(float&x, int lim);
	template bool clip(float&x, float lim);
	template bool clip(float&x, double lim);
	template bool clip(double&x, int lim);
	template bool clip(double&x, float lim);
	template bool clip(double&x, double lim);
//==线性插值映射到0~1====================
	double zero2one(const float &low,const float &up,const float &x){
		if(x<low){return 0;}
		else if(x>up){return 1;}
		return (x-low)/(up-low);
	}
	double zero2one(const double &low,const double &up,const double &x){
		if(x<low){return 0;}
		else if(x>up){return 1;}
		return (x-low)/(up-low);
	}
//==一阶滤波 class============
	filterOneClass::filterOneClass(){
		init(0.001,50);
		maxAcc=-1;
	}
	filterOneClass::filterOneClass(double dt,double cutF,double ki,double y0,double maxDerivate){
		init(dt,cutF,ki,y0,maxDerivate);
		maxAcc=-1;
	}
	double filterOneClass::init(double dt,double cutF,double ki,double y0,double maxDerivate){
		this->dt=dt;
		setCutF(cutF,ki);
		setBase(y0);
		setMaxDerivate(maxDerivate);
		return y;
	}
	double filterOneClass::reset(double cutF,double ki,double y0){
		setCutF(cutF,ki);
		setBase(y0);
		return y;
	}
	double filterOneClass::setBase(double y0){
		y=y0;
		errSum=0;
		return y;
	}
	void filterOneClass::setCutF(double cutF,double ki){
		// k=cutF*dt;
		k=1/(1 +1/(Pi2*cutF*dt));
		clip(k,0,1);
		ks=ki*dt;
		clip(ks,0,k/50);
	}
	void filterOneClass::setMaxDerivate(double maxDerivate){
		this->maxStep=maxDerivate*dt;
	}
	void filterOneClass::setMaxAcc(double maxAcc){
		this->maxAcc=maxAcc*dt;
	}
	const double &filterOneClass::filt(const double &x){
		double tmp=k*(x-y)+errSum;
		if(maxAcc>0){
			cmd2out1step(tmp,step,maxAcc);
		}else{
			step=tmp;
		}
		if(maxStep>0){
			clip(step,maxStep);
		}
		y+=step;
		errSum+=ks*(x-y);
		errSum*=0.98;
		return y;
	}

	// template<typename T>
	// filterOneClass<T>::filterOneClass(){
	// 	init(0.001,50);
	// }
	// template<typename T>
	// filterOneClass<T>::filterOneClass(T dt,T cutF,T ki,T y0,T maxDerivate){
	// 	init(dt,cutF,ki,y0,maxDerivate);
	// }
	// template<typename T>
	// T filterOneClass<T>::init(T dt,T cutF,T ki,T y0,T maxDerivate){
	// 	this->dt=dt;
	// 	setCutF(cutF,ki);
	// 	setBase(y0);
	// 	setMaxDerivate(maxDerivate);
	// 	// this->maxStep=maxDerivate*dt;
	// 	return y;
	// }
	// template<typename T>
	// T filterOneClass<T>::reset(T cutF,T ki,T y0){
	// 	setCutF(cutF,ki);
	// 	setBase(y0);
	// 	return y;
	// }
	// template<typename T>
	// T filterOneClass<T>::setBase(T y0){
	// 	y=y0;
	// 	errSum=0;
	// 	return y;
	// }
	// template<typename T>
	// void filterOneClass<T>::setCutF(T cutF,T ki){
	// 	// k=cutF*dt;
	// 	k=1/(1 +1/(Pi2*cutF*dt));
	// 	clip(k,0,1);
	// 	ks=ki*dt;
	// 	clip(ks,0,k/50);
	// }
	// template<typename T>
	// void filterOneClass<T>::setMaxDerivate(T maxDerivate){
	// 	this->maxStep=maxDerivate*dt;
	// }
	// template<typename T>
	// const T &filterOneClass<T>::filt(const T &x){
	// 	T step=k*(x-y);
	// 	if(maxStep>0){
	// 		clip(step,maxStep);
	// 	}
	// 	y+=step+errSum;
	// 	errSum+=ks*(x-y);
	// 	errSum*=0.98;
	// 	return y;
	// }
	// template class filterOneClass<double>;
	// template class filterOneClass<float>;
//==二阶低通滤波 class============
	filterTwoClass::filterTwoClass(){
		init(0.001,50);
	}
	filterTwoClass::filterTwoClass(double dtt,double cutF,double ki,double y0,double maxDerivate,double damp){
		init(dtt,cutF,ki,y0,maxDerivate,damp);
	}
	double filterTwoClass::init(double dtt,double cutF,double ki,double y0,double maxDerivate,double damp){
		dt=dtt;
		setCutF(cutF,ki,damp);
		setBase(y0);
		setMaxDerivate(maxDerivate);
		return y;
	}
	double filterTwoClass::reset(double cutF,double ki,double y0,double damp){
		setCutF(cutF,ki,damp);
		setBase(y0);
		return y;
	}
	double filterTwoClass::setBase(double y0){
		y=y0;
		xn1=y0,xn2=y0;
		yn1=y0;yn2=y0;
		errSum=0;
		return y;
	}
	void filterTwoClass::setCutF(double cutF,double ki,double damp){
		wc=cutF*Pi2;
		setDamp(damp);
		clip(ki,0,1);
		ks=ki*dt;
	}
	void filterTwoClass::setDamp(double damp){
		double wwtt=wc*wc*dt*dt;
		double a0=wwtt +4*damp*wc*dt +4;
		b0=wwtt/a0;
		b1=2*b0;
		b2=b0;
		a1=(2*wwtt-8)/a0;
		a2=(wwtt -4*damp*wc*dt +4)/a0;
	}
	void filterTwoClass::setMaxDerivate(double maxDerivate){
		maxStep=maxDerivate*dt;
	}
	const double &filterTwoClass::filt(const double &x){
		step=b0*x +b1*xn1 +b2*xn2 -a1*yn1 -a2*yn2    -y;
		step+=errSum;
		if(maxStep>0){
			clip(step,maxStep);
		}
		y+=step;

		errSum+=ks*(x-y);
		errSum*=0.95;

		xn2=xn1;
		xn1=x;
		yn2=yn1;
		yn1=y;
		return y;
	}
	double filterTwoClass::getV(){return step/dt;}
}//namespace