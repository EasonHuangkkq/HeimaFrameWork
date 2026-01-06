#!/usr/bin/env python3
# coding=utf-8
'''=========== ***doc description @ yyp*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: YYP

调用 sdk udp，接口定义见之
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

useUdp=0  # 0=共享内存  1=udp

if(useUdp==0):   # 共享内存版，仅可本机运行，亚毫秒级延迟，最高1khz，且需匹配目录层级（父级含config目录），否则报错！！
	from sdk.loong_jnt_sdk.loong_jnt_sdk_shm import jntSdkClass
	sdk=jntSdkClass(jntNum, fingerDofLeft, fingerDofRight)
elif(useUdp==1): # udp版，毫秒级延时，最高200hz
	from sdk.loong_jnt_sdk.loong_jnt_sdk_udp import jntSdkClass
	sdk=jntSdkClass('0.0.0.0',8006, jntNum, fingerDofLeft, fingerDofRight)
	sdk.waitSens()


# ===测试onnx，以下测试二选一，功能一致
from onnx_runner_yyp import onnxRunnerClass #偏数组角标访问风格，便于后期cpp迁移
# from onnx_runner_lq import onnxRunnerClass  #偏对象打包风格，py环境运行效率高
# ===
onnx=onnxRunnerClass("../model/onnx/policy_3052.onnx")


ctrl=jntSdkCtrlDataClass(jntNum, fingerDofLeft, fingerDofRight)
ctrl.reset()
ctrl.state=5
ctrl.torLimitRate=1
ctrl.filtRate=0.05
ctrl.kp=np.array([
	10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,
	10,10,10,10,10,
	400, 200, 400, 400, 120, 120,
	400, 200, 400, 400, 120, 120,], np.float32)
ctrl.kd=np.array([
	0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
	0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
	0.1, 0.1, 0.1, 0.1, 0.1,
	2, 2, 2, 4, 0.5, 0.5,
	2, 2, 2, 4, 0.5, 0.5], np.float32)


dT=0.02
tim=time.time()
sens=sdk.recv()
onnx.reset(sens)
file=open("zzz.txt",'w')
ttt=0
ccc=1
while(1):
	t1=time.time()
	sens=sdk.recv()
	print('通信延时=',time.time()-sens.timestamp)

	# sens.print()

	ctrl.j[-12:]=onnx.step(sens)#逻辑处理
	sdk.send(ctrl)
	# print(ctrl.j[-12:], file=file)

	t2=time.time()
	print('周期耗时=',t2-t1)
	ttt+=t2-t1
	ccc+=1
	print('平均周期耗时=',ttt/ccc)


	tim+=dT
	dt=tim-time.time()
	if(dt>0):
		time.sleep(dt)
	# sdk.loopTimeAdapt(50)


