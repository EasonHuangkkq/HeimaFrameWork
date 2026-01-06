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
测试环境
集成更易用的print系列
=====================================================*/
#include<sstream>
#include<iomanip>
#include"iopack.h"

namespace Ak12ERc7a{
template<typename Derived>
void eiPrint(ostream &out,const Eigen::MatrixBase<Derived> &m,bool enter){
	auto r=m.rows(),c=m.cols();
	//找最大宽度
	size_t width{0};
	For(r){
		for(int j=0;j!=c;++j){
			stringstream ss;
			if(abs(m(i,j))>1e-4){
				ss<<m(i,j);
				if(ss.str().length()>width){
					width=ss.str().length();
				}
			}
		}
	}
	//列向量行打印
	if(c==1){
		For(r){
			if(abs(m(i,0))<1e-4){out.width(width);out<<0;}
			else{out.width(width);out<<m(i,0);}
			if(i<r-1){out<<" ";}
		}
		out<<";\n";
	}else{
		For(r){
			for(int j=0;j!=c;++j){	
				if(abs(m(i,j))<1e-4){out.width(width);out<<0;}
				else{out.width(width);out<<m(i,j);}
				if(j<c-1){out<<" ";}
			}
			out<<";\n";
		}
	}
	if(enter){out<<endl;}
}
// =====================================
template<typename Derived>
void eiPrintWid(ostream &out,const Eigen::MatrixBase<Derived> &m,bool enter){
	auto r=m.rows(),c=m.cols();
	//找最大宽度
	size_t width{9};
	//列向量行打印
	if(c==1){
		For(r){
			if(abs(m(i,0))<1e-4){out.width(width);out<<0;}
			else{out.width(width);out<<fixed<<m(i,0);}
			if(i<r-1){out<<" ";}
		}
		out<<";\n";
	}else{
		For(r){
			for(int j=0;j!=c;++j){
				if(abs(m(i,j))<1e-4){out.width(width);out<<0;}
				// else{out.width(width);out<<fixed<<setprecision(3)<<m(i,j);}
				else{out.width(width);out<<fixed<<m(i,j);}
				if(j<c-1){out<<" ";}
			}
			out<<";\n";
		}
	}
	if(enter){out<<endl;}
}
}//namespace

// =================================
template<typename Derived>
void printEi(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrint(cout,m,enter);
}
template<typename Derived>
void printEi(int idx,const Eigen::MatrixBase<Derived> &m,bool enter){
	cout<<"【"<<idx<<"】：";
	Ak12ERc7a::eiPrint(cout,m,enter);
}
template<typename Derived>
void printEi(const string &tag,const Eigen::MatrixBase<Derived> &m,bool enter){
	cout<<"【"<<tag<<"】："<<endl;
	Ak12ERc7a::eiPrint(cout,m,enter);
}
// ---------
template<typename Derived>
void fprintEi(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrint(fout,m,enter);
}
template<typename Derived>
void fprintEi(int idx,const Eigen::MatrixBase<Derived> &m,bool enter){
	fout<<"【"<<idx<<"】：";
	Ak12ERc7a::eiPrint(fout,m,enter);
}
template<typename Derived>
void fprintEi(const string &tag,const Eigen::MatrixBase<Derived> &m,bool enter){
	fout<<"【"<<tag<<"】："<<endl;
	Ak12ERc7a::eiPrint(fout,m,enter);
}
// ==============================
template<typename Derived>
void printEiWid(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrintWid(cout,m,enter);
}
template<typename Derived>
void fprintEiWid(const Eigen::MatrixBase<Derived> &m,bool enter){
	Ak12ERc7a::eiPrintWid(fout,m,enter);
}