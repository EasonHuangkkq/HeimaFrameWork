#!/usr/bin/env python3
# coding=utf-8
'''=========== ***doc description @ yyp*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: YYP

onnx推理，角标直白风格，便于后期向cpp迁移
======================================================'''
import numpy as np
import onnxruntime as ort
from sdk.loong_jnt_sdk.loong_jnt_sdk_datas import jntSdkSensDataClass
from scipy.spatial.transform import Rotation
from collections import deque
import copy

class obsClass:
	def __init__(self):
		self.reset()
	def reset(self):
		self.cmd=np.zeros(3, np.float32)
		self.z=np.array([0.42], np.float32)
		self.standFlag=np.array([0.5], np.float32) #stand 0.5  step -0.5 walk 0
		self.w=np.zeros(3, np.float32)
		self.g=np.array([0,0,-1], np.float32)
		self.action=np.zeros(12, np.float32)
		self.q=np.zeros(12, np.float32)
		self.qd=np.zeros(12, np.float32)

class onnxRunnerClass:
	def __init__(self,onnxFile):
		self.lim=np.array([1, 0.2, 0.8], np.float32) #vx vy wz限幅
		self.ort=ort.InferenceSession(onnxFile)
		self.inName=self.ort.get_inputs()[0].name
		self.outName=self.ort.get_outputs()[0].name
		self.cmdAdd=np.array([0,0,0], np.float32)
		self.obs=obsClass()
		self.obsHist=deque([obsClass()]*3, maxlen=3)
		self.input=np.zeros(5+42+126+4200, np.float32)
		self.action=np.zeros(12, np.float32)
		self.legQStd=np.array([ 0, 0, 0.305913, -0.670418, 0.371265, 0,
								0, 0, 0.305913, -0.670418, 0.371265, 0], np.float32)
	def reset(self, sens:jntSdkSensDataClass):
		self.obs.reset()
		self.obs.w=sens.gyr
		self.obs.g=Rotation.from_euler('xyz',sens.rpy,degrees=False).as_matrix().transpose().dot(np.array([0,0,-1]))
		self.obs.q=sens.actJ[-12:]
		self.obs.qd.fill(0)

		self.obsHist.append(self.obs)
		self.obsHist.append(self.obs)
		self.obsHist.append(self.obs)
		self.obsHist.append(self.obs)#挤出之前的

		self.obs2input(reset=1)
		for i in range(99):
			self.input[173+42*i : 173+42*(i+1)]=self.input[-42:]
		self.output=np.zeros(12, np.float32)

	def obs2input(self, reset=0):
		self.input[0:47]=np.concatenate([self.obs.cmd,
										self.obs.z,
										self.obs.standFlag,
										self.obs.w,
										self.obs.g,
										self.obs.action,
										self.obs.q,
										self.obs.qd])
		self.input[47:56]=np.concatenate([h.w for h in self.obsHist])
		self.input[56:65]=np.concatenate([h.g for h in self.obsHist])
		self.input[65:101]=np.concatenate([h.action for h in self.obsHist])
		self.input[101:137]=np.concatenate([h.q for h in self.obsHist])
		self.input[137:173]=np.concatenate([h.qd for h in self.obsHist])
		self.input[173:173+4158]=self.input[-4158:]*1
		self.input[-42:]=np.concatenate([self.input[11:47], self.input[5:11]])

	def step(self, sens:jntSdkSensDataClass):
		if(sens.key[0]==6):
			self.obs.standFlag.fill(0)
			# print('开始踏步')
		elif(sens.key[0]==7):
			self.obs.standFlag.fill(0.5)
			# print('停止踏步')
		cmd=sens.joy[:3]*1

		if(abs(cmd[0])<0.09):
			cmd[0]=0
		if(abs(cmd[1])<0.04):
			cmd[1]=0
		if(abs(cmd[2])<0.19):
			cmd[2]=0
			
		cmd+=self.cmdAdd
		self.obs.cmd=0.95*self.obs.cmd +0.05*cmd
		self.obs.z=0.95*self.obs.z +0.05*(0.42 +sens.joy[3]*10) #不是标准单位
		self.obs.cmd=np.clip(self.obs.cmd, -self.lim, self.lim)
		self.obs.z=np.clip(self.obs.z, -1, 1)
		self.obs.w=sens.gyr
		self.obs.g=Rotation.from_euler('xyz',sens.rpy,degrees=False).as_matrix().transpose().dot(np.array([0,0,-1]))
		# self.obs.g=rpy2R(sens.rpy).transpose().dot(np.array([0,0,-1]))
		# self.obs.g=rpy2R(sens.rpy).T @ np.array([0,0,-1])
		self.obs.action=self.action*1
		self.obs.q=sens.actJ[-12:]*1
		self.obs.qd=sens.actW[-12:]*1
		
		self.obs2input()
		self.input.resize((1,4373))
		self.action=self.ort.run([self.outName],{self.inName:self.input})[0]
		self.action.resize((12))
		self.input.resize((4373))

		a=copy.deepcopy(self.obs)
		self.obsHist.append(a)
		
		return self.action +self.legQStd
		
