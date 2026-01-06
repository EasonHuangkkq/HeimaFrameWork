#!/usr/bin/env python3
# coding=utf-8
'''=========== ***doc description @ lq*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: LQ

onnx推理，py风格，便于py编程与debug
======================================================'''
import numpy as np
import onnxruntime as ort
from sdk.loong_jnt_sdk.loong_jnt_sdk_datas import jntSdkSensDataClass
from scipy.spatial.transform import Rotation
from collections import deque

class onnxRunnerClass:
	def __init__(self,onnxFile):
		self.ort = ort.InferenceSession(onnxFile)
		self.inName = self.ort.get_inputs()[0].name
		self.outName = self.ort.get_outputs()[0].name
		self.cmdAdd = [0, 0, 0]
		self.obsLen = 5 + 42 + 126 + 4200
		self.actionLen = 12
		self.cmdLen = 3
		self.shortHistLen = 3
		self.loogHistLen = 100
		self.legQStd = [0, 0, 0.305913, -0.670418, 0.371265, 0,
						0, 0, 0.305913, -0.670418, 0.371265, 0]
		self.lim = [1, 0.2, 0.8] #vx vy wz限幅
		self.filterRate = 0.05
	
	def reset(self, sens:jntSdkSensDataClass):
		self.cmdAdd = np.array(self.cmdAdd)
		self.legQStd = np.array(self.legQStd)
		self.lim = np.array(self.lim)
		self.standFlag = np.array([0.5])
		self.preAction = np.zeros(self.actionLen)
		
		newObs, loogHistData = self.sensParser(sens)
		self.shortHist = deque([newObs for _ in range(self.shortHistLen)], maxlen=self.shortHistLen)
		self.loogHist = deque([loogHistData for _ in range(self.loogHistLen)], maxlen=self.loogHistLen)
		self.preCmd = self.cmd
		self.preZ = self.z

	def step(self, sens:jntSdkSensDataClass):
		netInput = self.inputConstructer(sens)
		netOutput = self.infer(netInput)
		action = self.actionTrans(netOutput)
		return action
	
	def inputConstructer(self, sens:jntSdkSensDataClass):
		newObs, loogHistData = self.sensParser(sens)
		cmd = (1-self.filterRate)*self.preCmd +self.filterRate*self.cmd
		cmd = np.clip(cmd,-self.lim, self.lim)
		z = (1-self.filterRate)*self.preZ +self.filterRate*self.z
		z = np.clip(z, -1, 1)
		self.preCmd, self.preZ = cmd, z
		standFlag = self.standFlag
		obs = np.concatenate(newObs)
		shortHist = np.concatenate([np.concatenate(tmp) for tmp in zip(*self.shortHist)])
		self.shortHist.append(newObs)
		self.loogHist.append(loogHistData)
		loogHist = np.concatenate(self.loogHist)
		res = np.concatenate([
			cmd,
			z,
			standFlag,
			obs,
			shortHist,
			loogHist
		], dtype=np.float32)
		return res

	def sensParser(self, sens:jntSdkSensDataClass):
		if(sens.key[0]==6):
			self.standFlag=np.array([0])
		elif(sens.key[0]==7):
			self.standFlag=np.array([0.5])
		cmd=sens.joy[:3]
		cmd[0]=0 if(abs(cmd[0])<0.09) else cmd[0]
		cmd[1]=0 if(abs(cmd[1])<0.04) else cmd[1]
		cmd[2]=0 if(abs(cmd[2])<0.19) else cmd[2]
		self.cmd = cmd + self.cmdAdd
		self.z = np.array([0.42 +sens.joy[3]*10], np.float32)
		
		w = sens.gyr
		g = Rotation.from_euler('xyz',sens.rpy,degrees=False).as_matrix().transpose().dot(np.array([0,0,-1]))
		q = sens.actJ[-self.actionLen:]
		qd = sens.actW[-self.actionLen:]
		newObs = [w, g, self.preAction, q, qd]
		loogHistData = np.concatenate([self.preAction, q, qd, w, g])
		return newObs, loogHistData

	def infer(self, netInput:np.ndarray):
		netInput.resize((1, self.obsLen))
		return self.ort.run([self.outName],{self.inName:netInput})[0]
	
	def actionTrans(self, netOutput:np.ndarray):
		self.preAction = netOutput[0]
		return netOutput + self.legQStd
