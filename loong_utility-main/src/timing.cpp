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

=====================================================*/
#include"timing.h"
#include<chrono>
#include<thread>
#include<iostream>

using namespace std;
using namespace chrono;


static const int Num=20;


namespace Timing{

class timingClass::impClass{
public:
	double rcd[Num],ave;
	int idx;
	system_clock::time_point tag;
	long bornTag;
};
	timingClass::timingClass():imp(*new impClass()){
		imp.bornTag=high_resolution_clock::now().time_since_epoch().count();
	}
	timingClass::~timingClass(){delete &imp;}
	
	void timingClass::tic(){
		imp.tag=high_resolution_clock::now();
	}
	double timingClass::toc(){
		auto dt=high_resolution_clock::now()-imp.tag;
		return dt.count()/1e6;
	}
	double timingClass::toc20(){
		auto dt=high_resolution_clock::now() -imp.tag;
		imp.rcd[imp.idx]=dt.count()/1e6;
		imp.ave+=imp.rcd[imp.idx]/Num;
		imp.idx++; imp.idx%=Num;
		imp.ave-=imp.rcd[imp.idx]/Num;
		return imp.ave;
	}
	void timingClass::toc20print(const char* preFix){
		auto dt=high_resolution_clock::now() -imp.tag;
		imp.rcd[imp.idx]=dt.count()/1e6;
		imp.ave+=imp.rcd[imp.idx]/Num;
		imp.idx++; imp.idx%=Num;
		imp.ave-=imp.rcd[imp.idx]/Num;
		if(imp.idx==0){cout<<preFix<<"平均耗时(ms)="<<imp.ave<<endl;}
	}
	long timingClass::nowNs(){
		return Timing::nowNs();
	}
	int timingClass::nowMs(){
		return Timing::nowMs();
	}
	int timingClass::nowUs(){
		return Timing::nowUs();
	}
	double timingClass::nowMsUs(){
		return Timing::nowMsUs();
	}
	void timingClass::sleepMs(double ms){
		Timing::sleepMs(ms);
	}
// ===========================
long nowNs(){
	return high_resolution_clock::now().time_since_epoch().count();
}
int nowMs(){
	long tmp=high_resolution_clock::now().time_since_epoch().count();
	tmp=tmp-long(tmp/1e9)*1e9;
	return tmp/1e6;
}
int nowUs(){
	long tmp=high_resolution_clock::now().time_since_epoch().count();
	tmp=tmp-long(tmp/1e6)*1e6;
	return tmp/1e3;
}
double nowMsUs(){
	long tmp=high_resolution_clock::now().time_since_epoch().count();
	double re=tmp-long(tmp/1e9)*1e9;
	tmp=tmp-long(tmp/1e6)*1e6;
	return (re+tmp)/1e6;
}
void sleepMs(double ms){
	this_thread::sleep_for(chrono::microseconds(int(ms*1000)));
}


};