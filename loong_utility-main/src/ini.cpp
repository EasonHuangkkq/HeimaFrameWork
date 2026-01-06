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
<map>查找不要用value=map[key]，这样会插入不存在的key
=====================================================*/
#include"ini.h"
#include<map>
#include<iostream>
#include<unistd.h>

namespace Ini{
	//去除空格、tab
	inline void wipeWhite(string &str){
		size_t id=0;
		while((id=str.find(' ',id)) != string::npos){
			str.erase(id,1);
		}
		id=0;
		while((id=str.find('\t',id)) != string::npos){
			str.erase(id,1);
		}
	};
//==========================
class iniClass::impClass{
public:
	impClass(){};
	impClass(const string &fileName);
	bool open(const string &fileName,bool append);

	map<string,string> defaultSec;//defaultSec是为了加快访问速度，在store中也存在
	map<string,map<string,string>> store;
	string file;

	const map<string,string>&getSection(const string &section);

	double getVal(const string &secName, const map<string,string> &sec, const string &key);//secName仅用于打印错误信息
	const string &getStr(const string &secName, const map<string,string> &sec, const string &key);//secName仅用于打印错误信息
	template<typename T>
	int getArray(const string &secName, const map<string,string> &sec, const string &key,T *value,int n,bool mustMatch);//secName仅用于打印错误信息
	int getArraySize(const string &secName, const map<string,string> &sec, const string &key);//secName仅用于打印错误信息
};
	iniClass::impClass::impClass(const string &fileName){
		open(fileName,0);
	}
	bool iniClass::impClass::open(const string &fileName,bool append){
		ifstream f;
		f.open(fileName);
		if(!f.good()){
			stringstream ss;
			ss<<"=== 未找到"<<fileName<<" !===";
			throw runtime_error(ss.str());
			return 0;
		}
		file=fileName;
		if(!append){
			defaultSec.clear();
			store.clear();
		}
		auto *sec=&defaultSec;
		string secName;
		string line;
		string preKey;
		while(getline(f,line)){
			size_t id=0;
			id=line.find(";");//分号是ini的注释标志
			if(id!=string::npos){
				line=line.substr(0,id);
			}
			wipeWhite(line);
			if(line.size()==0){
				continue;
			}
			if(line.front()=='[' && line.back()==']'){
				secName=line.substr(1,line.length()-2);
				sec=&store[secName];//不存在会自动添加
				preKey.clear();
				// cout<<line.substr(1,line.length()-2)<<endl;
				continue;
			}
			
			id=line.find("=");
			if(id==string::npos){//无“=”号，视为上一行的换行
				if(preKey.size()>0){
					auto &value=(*sec)[preKey];
					if(value[value.size()-1]!=',' && line[0]!=','){
						stringstream ss;
						ss<<"ini：跨行数组拼接，未发现逗号分割符！file="<<fileName<<"，section="<<secName<<"，key="<<preKey;
						cout<<ss.str()<<endl;
						throw runtime_error(ss.str());
					}else{
						value+=line;
					}
				}
			}else{//有“=”号，新键值对
				string key,value;
				key.assign(line,0,id);
				if(sec->find(key) != sec->end()){
					stringstream ss;
					ss<<"ini：重复key！file="<<fileName<<"，section="<<secName<<"，key="<<preKey;
					cout<<ss.str()<<endl;
					throw runtime_error(ss.str());
				}
				value.assign(line,id+1,line.size()-id);
				sec->insert(pair<string,string>(key,value));
				preKey=key;
			}
		}
		f.close();
		return 1;
	}
	const map<string,string>& iniClass::impClass::getSection(const string &section){
		auto it=store.find(section);
		if(it!=store.end()){
			return it->second;
		}else{
			stringstream ss;
			ss<<"ini：未找到section！file="<<file<<"，section="<<section;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
		return store[section];
	}
	double iniClass::impClass::getVal(const string &secName, const map<string,string> &sec,const string &key){
		auto it=sec.find(key);
		if(it!=sec.end()){
			try{
				return stod(it->second);
			}catch(...){
				stringstream ss;
				ss<<"ini:值类型不匹配！file="<<file<<"，section="<<secName<<"，key="<<key;
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
		}else{
			stringstream ss;
			ss<<"ini:找不到key！file="<<file<<"，section="<<secName<<"，key="<<key;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
	};
	const string &iniClass::impClass::getStr(const string &secName, const map<string,string> &sec,const string &key){
		auto it=sec.find(key);
		if(it!=sec.end()){
			return it->second;
		}else{
			stringstream ss;
			ss<<"ini:getStr找不到key！file="<<file<<"，section="<<secName<<"，key="<<key;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
	};
	template<typename T>
	int iniClass::impClass::getArray(const string &secName, const map<string,string> &sec,const string &key,T *value,int n,bool mustMatch){
		auto it=sec.find(key);
		if(it!=sec.end()){
			std::vector<std::string> vs;
			splitString(it->second,vs,',');
			if((mustMatch && vs.size()!=n) || vs.size()<n){
				stringstream ss;
				ss<<"ini:getArray数目不匹配！file="<<file<<"，section="<<secName<<"，key="<<key<<
						"。目标n="<<n<<"，文件内数组size="<<vs.size();
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
			try{
				for(int i=0;i<n;i++){
					value[i]=(T)stod(vs[i]);
				}
			}catch(...){
				stringstream ss;
				ss<<"ini:getArray值类型不匹配！file="<<file<<"，section="<<secName<<"，key="<<key;
				cout<<ss.str()<<endl;
				throw runtime_error(ss.str());
			}
			return vs.size();
		}else{
			stringstream ss;
			ss<<"ini:getArray找不到key！file="<<file<<"，section="<<secName<<"，key="<<key;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
		return 0;
	}
	int iniClass::impClass::getArraySize(const string &secName, const map<string,string> &sec, const string &key){
		auto it=sec.find(key);
		if(it!=sec.end()){
			std::vector<std::string> vs;
			splitString(it->second,vs,',');
			return vs.size();
		}else{
			stringstream ss;
			ss<<"ini:getArraySize找不到key！file="<<file<<"，section="<<secName<<"，key="<<key;
			cout<<ss.str()<<endl;
			throw runtime_error(ss.str());
		}
		return 0;
	}
// =======================================================
	iniClass::iniClass():imp(*new impClass()){}
	iniClass::iniClass(const string &fileName):imp(*new impClass(fileName)){}
	bool iniClass::open(const string &fileName,bool append){return imp.open(fileName,append);}
	//==无section参数(或section为defaut)==
	double iniClass::getVal(const string &key){
		return imp.getVal("默认", imp.defaultSec, key);
	};
	double iniClass::operator[](const string &key){
		return imp.getVal("默认", imp.defaultSec,key);
	};
	const string &iniClass::getStr(const string &key){
		return imp.getStr("默认", imp.defaultSec,key);
	};
	template<typename T>
	int iniClass::getArray(const string &key,T *value,int n, bool mustMatch){
		return imp.getArray("默认", imp.defaultSec, key,value,n, mustMatch);
	}
	template int iniClass::getArray(const string &key,int *value,int n, bool mustMatch);
	template int iniClass::getArray(const string &key,float *value,int n, bool mustMatch);
	template int iniClass::getArray(const string &key,double *value,int n, bool mustMatch);
	int iniClass::getArraySize(const string &key){
		return imp.getArraySize("默认",imp.defaultSec, key);
	}
	//==有section参数========
	double iniClass::getVal(const string &section,const string &key){
		return imp.getVal(section, imp.getSection(section), key);
	};
	const string &iniClass::getStr(const string &section,const string &key){
		return imp.getStr(section, imp.getSection(section), key);
	};
	template<typename T>
	int iniClass::getArray(const string &section,const string &key,T *value,int n, bool mustMatch){
		return imp.getArray(section, imp.getSection(section), key, value, n, mustMatch);
	}
	template int iniClass::getArray(const string &section,const string &key,int *value,int n, bool mustMatch);
	template int iniClass::getArray(const string &section,const string &key,float *value,int n, bool mustMatch);
	template int iniClass::getArray(const string &section,const string &key,double *value,int n, bool mustMatch);
	int iniClass::getArraySize(const string &section, const string &key){
		return imp.getArraySize(section, imp.getSection(section), key);
	}
}//namespace


// 借助单例构造函数提前感知当前目录，给fout
class foutClass{
public:
	static foutClass& instance(){
		static foutClass singtn;
		return singtn;
	}
	ofstream fout;
private:
	foutClass(){
		if(access("../config",0)!=-1){
			fout.open("../zzz.txt");
		}else if(access("../../config",0)!=-1){
			fout.open("../../zzz.txt");
		}else{
			fout.open("zzz.txt");
		}
	}
};

ofstream &fout=foutClass::instance().fout;
