#!/usr/bin/env python3
# coding=utf-8
'''=========== ***doc description @ yyp*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: YYP

调用 jnt sdk，接口定义见loong_jnt_sdk_datas.py

======================================================'''
import time
import numpy as np
import sys
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append("..")
from sdk.loong_jnt_sdk.loong_jnt_sdk_datas import jntSdkSensDataClass, jntSdkCtrlDataClass

jntNum=31
fingerDofLeft=6
fingerDofRight=6

useUdp=1  # 0=共享内存  1=udp

if(useUdp==0):   # 共享内存版，仅可本机运行，亚毫秒级延迟，最高1khz，且需匹配目录层级（父级含config目录），否则报错！！
	from sdk.loong_jnt_sdk.loong_jnt_sdk_shm import jntSdkClass
	sdk=jntSdkClass(jntNum, fingerDofLeft, fingerDofRight)
elif(useUdp==1): # udp版，毫秒级延时，最高200hz
	from sdk.loong_jnt_sdk.loong_jnt_sdk_udp import jntSdkClass
	sdk=jntSdkClass('0.0.0.0',8006,jntNum, fingerDofLeft, fingerDofRight)
	sdk.waitSens()


ctrl=jntSdkCtrlDataClass(jntNum, fingerDofLeft, fingerDofRight)
ctrl.reset()
ctrl.filtRate=1
ctrl.kp=np.array([
	10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,
	10,10,10,10,10,
	500,400,500,500,200,200,
	500,400,500,500,200,200,], np.float32)
ctrl.kd=np.array([
	0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
	0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
	0.1, 0.1, 0.1, 0.1, 0.1,
	1, 1, 2, 2, 1, 1,
	1, 1, 2, 2, 1, 1,], np.float32)


dT=0.02
tim=time.time()
stdJnt=ctrl.getStdJnt()

for i in range(1000):
	ctrl.state=5
	ctrl.j[0]=stdJnt[0] +0.5*np.sin(i/50)
	ctrl.j[10]=stdJnt[10] +0.5*np.sin(i/100)
	ctrl.j[21]=stdJnt[21] +0.2*np.sin(i/20)
	ctrl.j[28]=stdJnt[28] +0.5*np.sin(i/50)

	# ctrl.j[19]=0.3
	# ctrl.j[25]=0.3


	sdk.send(ctrl)
	sens=sdk.recv()
	print('---')
	# sens.print()
	delayT=time.time()-sens.timestamp
	print(delayT)
	# print(sens.joy)
	# print(ctrl.j)
	# print(sens.tgtJ)
	# print(sens.actJ)

	tim+=dT
	dt=tim-time.time()
	if(dt>0):
		time.sleep(dt)



