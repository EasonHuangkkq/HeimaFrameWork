#!/usr/bin/env python3
# coding=utf-8
'''=========== ***doc description @ yyp*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: YYP
调用 mani sdk udp，接口定义见之
======================================================'''
import time
import sys
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append("..")
from sdk.loong_mani_sdk.loong_mani_sdk_udp import maniSdkCtrlDataClass, maniSdkClass, maniSdkSensDataClass
import numpy as np

jntNum=19
armDof=7
fingerDofLeft=6
fingerDofRight=6
neckDof=2
lumbarDof=3

ctrl=maniSdkCtrlDataClass(armDof, fingerDofLeft, fingerDofRight, neckDof, lumbarDof)
# sdk=maniSdkClass("192.168.1.201", 8003, jntNum, fingerDofLeft, fingerDofRight)
sdk=maniSdkClass("0.0.0.0", 8003, jntNum, fingerDofLeft, fingerDofRight)

ctrl.inCharge  =1
ctrl.filtLevel =1
ctrl.armMode   =4
ctrl.fingerMode=3
ctrl.neckMode  =5
ctrl.lumbarMode=0
ctrl.armCmd   =np.array([[0.4, 0.4, 0.1,   0,0,0,   0.5],
						[0.2,-0.4, 0.1,   0,0,0,   0.5]],np.float32)
ctrl.armFM    =np.zeros((2,6),np.float32)
ctrl.fingerLeft=np.zeros(fingerDofLeft, np.float32)
ctrl.fingerRight=np.zeros(fingerDofRight, np.float32)
ctrl.neckCmd  =np.zeros(2,np.float32)
ctrl.lumbarCmd=np.zeros(3,np.float32)


dT=0.02
tim=time.time()
for i in range(1000):
	ctrl.armCmd[0][0]=0.4 +0.1*np.sin(i*dT*2)
	ctrl.armCmd[0][2]=0.1 +0.1*np.sin(i*dT*2)
	ctrl.armCmd[1][0]=0.2 +0.1*np.sin(i*dT*2)
	ctrl.fingerLeft[0]=40 +30*np.sin(i*dT)
	ctrl.fingerRight[3]=40 +30*np.sin(i*dT)
	sdk.send(ctrl)
	sens=sdk.recv()
	sens.print()

	tim+=dT
	dt=tim-time.time()
	if(dt>0):
		time.sleep(dt)
