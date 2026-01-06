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
读取解析配置文件*.ini
字符匹配非常耗力，因此不要在实时循环使用，可以在进实时循环前先找变量赋值

#include"ini.h"
=====================================================*/
#pragma once
#include<string>
#include<vector>
#include<sstream>
#include<fstream>

using namespace std;

namespace Ini{
	//按token分割
	inline void splitString(const string &str, vector<string> &substrings, char token){
		string tmp;
		stringstream ss(str);
		while(getline(ss, tmp, token)){
			substrings.push_back(tmp);
		}
	};
// =================================
class iniClass{
public:
	iniClass();
	iniClass(const string &fileName);//等价于无参构造+open，
	bool open(const string &fileName, bool append=0);//append=1用于打开文件并在原有基础上键值对

	//--不存在section的独立数据获取--
	double getVal(const string &key);       //不要在实时循环使用
	double operator[](const string &key);   //不要在实时循环使用
	const string &getStr(const string &key);//不要在实时循环使用。如需str数组，getStr后可用上面splitString自行分割

	template<typename T>
	int getArray(const string &key, T*value, int n, bool mustMatch=1); //不要在实时循环使用，返回读取到的array size；mustMatch=1时，n和实际size不匹配即报错
	template<typename T,int n>
	int getArray(const string &key, T(&value)[n], bool mustMatch=1){return getArray(key,value,n,mustMatch);};//不要在实时循环使用，返回读取到的array size；mustMatch=1时，n和实际size不匹配即报错
	int getArraySize(const string &key);

	//--有section参数--
	double getVal(const string &section, const string &key);       //不要在实时循环使用
	const string &getStr(const string &section, const string &key);//不要在实时循环使用。如需str数组，getStr后可用上面splitString自行分割
	template<typename T>
	int getArray(const string &section, const string &key, T*value, int n, bool mustMatch=1);//不要在实时循环使用，返回读取到的array size；mustMatch=1时，n和实际size不匹配即报错
	template<typename T,int n>
	int getArray(const string &section, const string &key, T(&value)[n], bool mustMatch=1){return getArray(key,value,n,mustMatch);};//不要在实时循环使用，返回读取到的array size；mustMatch=1时，n和实际size不匹配即报错
	int getArraySize(const string &section, const string &key);
protected:
	class impClass;
	impClass&imp;
};

}//namespace

extern ofstream &fout;