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
时间、计时相关封装
=====================================================*/
#pragma once

namespace Timing{
class timingClass{
public:
	timingClass();
	~timingClass();
	void tic();//开始计时
	double toc();//距离计时，ms
	long nowNs(); //当前时间戳，ns
	int nowMs();//仅当前时间戳的ms3位
	int nowUs();//仅当前时间戳的us3位
	double nowMsUs();//仅当前时间戳的ms3位+小数点us3位
	void sleepMs(double ms);
	double toc20();
	void toc20print(const char* preFix="");//20次平均耗时打印
private:
	class impClass;
	impClass &imp;
};
// ==========================
long nowNs(); //当前时间戳，ns
int nowMs();//仅当前时间戳的ms3位
int nowUs();//仅当前时间戳的us3位
double nowMsUs();//仅当前时间戳的ms3位+小数点us3位
void sleepMs(double ms);

}//namespace
