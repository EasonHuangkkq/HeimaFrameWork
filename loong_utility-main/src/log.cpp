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
#include"log.h"
#include"quill/Quill.h"
#include"unistd.h"
#include"iopack.h"

namespace Log{
class logClass::impClass{
public:
	~impClass(){
		quill::flush();
		quill::remove_logger(ql);
	}
	quill::Logger *ql;
	void init(const string& suffix);
	void creatNewFile();
	stringstream prefix;
	bool notInited=1;
	int fileNum=0,lineCnt=0;
	int maxLineNum=180000;//100hz时30min一个文件
};
// ========================
	void logClass::impClass::init(const string& suffix){
		if(!notInited){return;}
		notInited=0;
		bool found=0;
		string dir;
	#ifdef _WIN32
		if(!found){
			if(access("../../config",0)!=-1){
				dir="../../log/";
				found=1;
			}
		}
	#endif
		if(!found){
			if(access("./config",0)!=-1){
				dir="log/";
				found=1;
			}
		}
		if(!found){
			if(access("../config",0)!=-1){
				dir="../log/";
				found=1;
			}
		}
		if(!found){
			// throw runtime_error("=== log 未找到 config 文件夹，无法定位目录 !===");
			cout<<"!! log 未找到 config 文件夹，无法定位 log 目录，将在执行目录建立 log !!";
			dir="log/";
		}

		time_t t;
		time(&t);
		struct tm*tt=localtime(&t);
		char name[128]{};
		strftime(name,128,"log_%Y%m%d_%H%M%S_",tt);
		prefix<<dir<<name<<suffix;
		quill::start();
	}
	void logClass::impClass::creatNewFile(){
		stringstream ss;
		ss<<prefix.str()<<"_"<<fileNum<<".txt";
		auto file_handler = quill::file_handler(ss.str());
		file_handler->set_pattern("%(time)\t%(message)");
		if(ql){
			quill::remove_logger(ql);
		}
		ql=quill::create_logger(ss.str(), move(file_handler));
	}
// =========================
	logClass::logClass():imp(*new impClass()){}
	void logClass::init(const string& suffix){
		imp.init(suffix);
		imp.creatNewFile();
	}
	void logClass::log(const string &s){
		if(imp.notInited){
			throw runtime_error("log未初始化！请确保logClass的init正确调用！");
		}
		LOG_INFO(imp.ql,"{}",s);

		imp.lineCnt++;
		if(imp.lineCnt==imp.maxLineNum){
			imp.fileNum++;
			imp.creatNewFile();
			imp.lineCnt=0;
		}
	}
}//namespace

