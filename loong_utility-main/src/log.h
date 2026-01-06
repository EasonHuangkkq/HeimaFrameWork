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
封装quill，当log到达一定行数，支持自动log新文件
#include"log.h"
=====================================================*/
#pragma once
#include<string>

using namespace std;

namespace Log{
class logClass{
public:
	logClass();
	void init(const string& suffix);
	void log(const string& s);
private:
	class impClass;
	impClass&imp;
};
}//namespace

