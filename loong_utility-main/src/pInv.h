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
eigen伪逆封装
=====================================================*/
#pragma once
#include"eigen.h"

namespace Ei{
#define CDR const double &
#define CVR const vecXd &
#define CMR const matXd &

	inline mat3d pInv3(const mat3d &M){
		// return A.completeOrthogonalDecomposition().pseudoInverse();
		mat3d tmp=M*M.transpose();
		For3{tmp(i,i)+=1e-6;}
		return M.transpose()*tmp.inverse();
	}
//==权重松弛===
	template<typename EiMat>
	inline void pInvLeftWeight(CMR MIn, CVR w, const EiMat &MOut, CDR slack=1e-6){
		matXd tmp=MIn.transpose()*w.asDiagonal()*MIn;
		For(tmp.cols()){tmp(i,i)+=slack;}
		const_cast<EiMat &>(MOut)=tmp.inverse()*MIn.transpose()*w.asDiagonal();
	}
	template<typename EiMat>
	inline void pInvRightWeight(CMR MIn, CVR w, const EiMat &MOut, CDR slack=1e-6){
		matXd tmp=MIn *w.asDiagonal().inverse() *MIn.transpose();
		For(tmp.rows()){tmp(i,i)+=slack;}
		const_cast<EiMat &>(MOut)=w.asDiagonal().inverse() *MIn.transpose() *tmp.inverse();
	}
	template<typename EiMat>
	inline void pInvWeight(CMR MIn, CVR w, const EiMat &MOut, CDR slack=1e-6){
		if(int(MIn.rows())<MIn.cols()){
			pInvRightWeight(MIn,w,MOut,slack);
		}else{
			pInvLeftWeight(MIn,w,MOut,slack);
		}
	};
//==权重松弛向量===
	template<typename EiMat>
	inline void pInvLeftWeight(CMR MIn, CVR w, const EiMat &MOut, CVR s){
		matXd tmp=MIn.transpose()*w.asDiagonal()*MIn;
		For(tmp.cols()){tmp(i,i)+=s[i];}
		const_cast<EiMat &>(MOut)=tmp.inverse()*MIn.transpose()*w.asDiagonal();
	}
	template<typename EiMat>
	inline void pInvRightWeight(CMR MIn, CVR w, const EiMat &MOut, CVR s){
		matXd tmp=MIn *w.asDiagonal().inverse() *MIn.transpose();
		For(tmp.rows()){tmp(i,i)+=s[i];}
		const_cast<EiMat &>(MOut)=w.asDiagonal().inverse() *MIn.transpose() *tmp.inverse();
	}
	template<typename EiMat>
	inline void pInvWeight(CMR MIn, CVR w, const EiMat &MOut, CVR s){
		if(int(MIn.rows())<MIn.cols()){
			pInvRightWeight(MIn,w,MOut,s);
		}else{
			pInvLeftWeight(MIn,w,MOut,s);
		}
	};
//==松弛===
	template<typename EiMat>
	inline void pInvLeft(CMR MIn, const EiMat &MOut, CDR slack=1e-6){
		matXd tmp=MIn.transpose()*MIn;
		For(tmp.cols()){tmp(i,i)+=slack;}
		const_cast<EiMat &>(MOut)=tmp.inverse()*MIn.transpose();
	}
	template<typename EiMat>
	inline void pInvRight(CMR MIn, const EiMat &MOut, CDR slack=1e-6){
		matXd tmp=MIn *MIn.transpose();
		For(tmp.rows()){tmp(i,i)+=slack;}
		const_cast<EiMat &>(MOut)=MIn.transpose() *tmp.inverse();
	}
	template<typename EiMat>
	inline void pInv(CMR MIn, const EiMat &MOut, CDR slack=1e-6){
		if(int(MIn.rows())<MIn.cols()){
			pInvRight(MIn,MOut,slack);
		}else{
			pInvLeft(MIn,MOut,slack);
		}
	};
	inline matXd pInv(CMR M, CDR slack=1e-6){
		matXd MOut;
		pInv(M,MOut,slack);
		return MOut;
	};
//==松弛向量===
	template<typename EiMat>
	inline void pInvLeft(CMR MIn, const EiMat &MOut, CVR s){
		matXd tmp=MIn.transpose()*MIn;
		For(tmp.cols()){tmp(i,i)+=s[i];}
		const_cast<EiMat &>(MOut)=tmp.inverse()*MIn.transpose();
	}
	template<typename EiMat>
	inline void pInvRight(CMR MIn, const EiMat &MOut, CVR s){
		matXd tmp=MIn *MIn.transpose();
		For(tmp.rows()){tmp(i,i)+=s[i];}
		const_cast<EiMat &>(MOut)=MIn.transpose() *tmp.inverse();
	}
	template<typename EiMat>
	inline void pInv(CMR MIn, const EiMat &MOut, CVR s){
		if(int(MIn.rows())<MIn.cols()){
			pInvRight(MIn,MOut,s);
		}else{
			pInvLeft(MIn,MOut,s);
		}
	};
	inline matXd pInv(CMR M, CVR s){
		matXd MOut;
		pInv(M,MOut,s);
		return MOut;
	};
//==svd===
	template<typename EiMat>
	void pInvSVD(CMR MIn,const EiMat &MOut){//慢10倍
		Eigen::JacobiSVD svd(MIn,Eigen::ComputeThinU | Eigen::ComputeThinV);
		vecXd s=svd.singularValues();
		for(int k=0;k<s.size();k++){
			if(s[k]>1e-3){s[k]=1/s[k];}
			else{s[k]=0;}
		}
		const_cast<EiMat &>(MOut)=svd.matrixV()*s.asDiagonal()*svd.matrixU().transpose();
	}
	template<typename EiMat>
	void pInvSVDWeight(CMR MIn, CVR w, const EiMat &MOut){//慢10倍
		Eigen::JacobiSVD svd(w.asDiagonal()*MIn,Eigen::ComputeThinU | Eigen::ComputeThinV);
		vecXd s=svd.singularValues();
		for(int k=0;k<s.size();k++){
			if(s[k]>1e-3){s[k]=1/s[k];}
			else{s[k]=0;}
		}
		const_cast<EiMat &>(MOut)=svd.matrixV()*s.asDiagonal()*svd.matrixU().transpose()*w.asDiagonal();
	}
}//namespace
