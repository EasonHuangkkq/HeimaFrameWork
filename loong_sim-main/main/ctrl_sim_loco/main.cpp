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
直接对控制算法仿真，全身
=====================================================*/
#include<sstream>
#include<chrono>
#include<thread>
#include"nabo_locomotion/nabo.h"
#include"sim.h"
#include"algorithms.h"
#include"iopack.h"

using namespace std;

Ini::iniClass iniSim;
double dt;
Nabo::inputStruct in;
Nabo::outputStruct out;
bool floatBaseFlag;//浮基or固定，根据自由度自动判断
bool runFlag=1;
bool assistFlag;//模拟外部悬挂，若固定需注释xml内freejoint
double assistH=1.2;
int slowRanderMs;//渲染减缓，每周期等待毫秒
stringstream msg;

static const int NMotMain=31;

double jntKp[NMotMain],jntKd[NMotMain],jntMaxTor[NMotMain];
Alg::filterOneClass fil[NMotMain];
// 动力学模型：臂7*2 +头2 +腰3 +腿6*2
//           0     14   16   19   31
bool firstFlag=1;

void runCtrl(){
	mjData &d=Sim::getMjData();
	if(firstFlag){
		in.cmd.key=17;
		firstFlag=0;
	}
	if(floatBaseFlag){
		mjtNum*q=d.qpos+3;
		in.sens.rpy[0]=atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
		in.sens.rpy[1]=asin(2*(q[0]*q[2]-q[3]*q[1]));
		in.sens.rpy[2]=atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
		for(int i=0;i<3;i++){
			in.sens.gyr[i]=d.qvel[i+3];
			in.sens.acc[i]=d.qacc[i];
			in.sens.supP[i]=d.qpos[i];
			in.sens.supV[i]=d.qvel[i];
		}
		in.sens.supP[2]-=0.07;//ankle到脚底高度0.04，W系
		For(NMotMain){
			in.sens.j[i]=d.qpos[i+7];//当存在浮基，qpos前7个数为【浮动基3d位置+四元数】
			in.sens.w[i]=d.qvel[i+6];//当存在浮基，qvel前6个数为【浮动基3d速度+角速度】
			in.sens.t[i]=d.actuator_force[i];
		}
		if(assistFlag){
			double kp=1500,kd=400,maxF=600;
			For2{
				d.xfrc_applied[6+i]=Alg::grinded(-kp*d.qpos[i], maxF, 1., 0.2) -kd*d.qvel[i];
			}
			d.xfrc_applied[6+2]=Alg::grinded(kp*(assistH-d.qpos[2]), maxF, 1., 0.2) -kd*d.qvel[2];
			For3{
				d.xfrc_applied[6+3+i]=Alg::grinded(-kp*in.sens.rpy[i], maxF, 1., 0.2) -kd*d.qvel[3+i];
			}
			For6{
				Alg::clip(d.xfrc_applied[6+i], maxF);
			}
			d.xfrc_applied[6+2]+=800;
		}else{
			For(6){d.xfrc_applied[6 +i]=0;}
		}
	}else{
		For(NMotMain){
			in.sens.j[i]=d.qpos[i];
			in.sens.w[i]=d.qvel[i];
			in.sens.t[i]=d.actuator_force[i];
		}
	}
	Nabo::step(in,out);
	double tmp;
	For(NMotMain){//驱动无需映射
		//driver层pd
		tmp=out.param.kp[i]*(out.ctrl.j[i]-in.sens.j[i]) +out.param.kd[i]*(out.ctrl.w[i]-in.sens.w[i]);
		d.ctrl[i]=fil[i].filt(tmp) +out.ctrl.t[i];
		//模拟驱动器内部pd
		d.ctrl[i]+=jntKp[i]*(out.ctrl.j[i]-in.sens.j[i]) +jntKd[i]*(out.ctrl.w[i]-in.sens.w[i]);
		Alg::clip(d.ctrl[i],jntMaxTor[i]);
	}
}
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
	// backspace: reset simulation
	if(act==GLFW_PRESS){
		switch(key){
		case GLFW_KEY_BACKSPACE:
			Sim::reset();
			in.sens.cnt=0;
			Nabo::init(dt);
			in.cmd.key=0;
			// break;//没有break
		case GLFW_KEY_SPACE:
			in.cmd.vx=0;in.cmd.vy=0;in.cmd.wz=0;break;
		case GLFW_KEY_W:
			in.cmd.vx+=0.1;break;
		case GLFW_KEY_S:
			in.cmd.vx-=0.1;break;
		case GLFW_KEY_A:
			in.cmd.vy+=0.1;break;
		case GLFW_KEY_D:
			in.cmd.vy-=0.1;break;
		case GLFW_KEY_J:
			in.cmd.wz+=0.1;break;
		case GLFW_KEY_L:
			in.cmd.wz-=0.1;break;
		case GLFW_KEY_Q:
			in.cmd.key=6;break;//6启动踏步，7stop
		case GLFW_KEY_E:
			in.cmd.key=7;break;//6启动踏步，7stop
		case GLFW_KEY_F:
			assistFlag=!assistFlag;break;//悬挂与否
		case GLFW_KEY_G:
			assistH-=0.01;break;//悬挂h-
		case GLFW_KEY_H:
			assistH+=0.01;break;//悬挂h+
		case GLFW_KEY_P:
			runFlag=!runFlag;break;
		case GLFW_KEY_O:
			if(!runFlag){
				For(10){
					runCtrl();
					Sim::runStep();
				}
			};break;
		case GLFW_KEY_1://dis
			in.cmd.key=13;break;
		case GLFW_KEY_2://rc
			in.cmd.key=2;break;
		case GLFW_KEY_3://idle
			in.cmd.key=3;break;
		case GLFW_KEY_4://wk
			in.cmd.key=4;break;
		case GLFW_KEY_5://op
			in.cmd.key=16;break;
		case GLFW_KEY_6://damp
			in.cmd.key=20;break;
		case GLFW_KEY_7://rl
			in.cmd.key=21;break;
		}
		if(abs(in.cmd.vx)<1e-4){in.cmd.vx=0;}
		if(abs(in.cmd.vy)<1e-4){in.cmd.vy=0;}
		if(abs(in.cmd.wz)<1e-4){in.cmd.wz=0;}
		Alg::clip(in.cmd.vx,0.5);
		Alg::clip(in.cmd.vy,0.2);
		Alg::clip(in.cmd.wz,0.5);
		msg.clear();
		msg.str("");
		msg<<"cmd: vx="<<in.cmd.vx<<", vy="<<in.cmd.vy<<", wz="<<in.cmd.wz<<", key="<<in.cmd.key;
	}
}

int main(int argc, char* argv[]){
	if(argc>1){
		slowRanderMs=stoi(string(argv[1]));//渲染减缓，每周期等待毫秒
	}
	#ifdef _WIN32
	LoadKeyboardLayout("0x409", KLF_ACTIVATE | KLF_SETFORPROCESS);
	system("CHCP 65001");//改变命令行为utf8编码，会被360报毒
	Sim::init("../../scene.xml");
	iniSim.open("../../config/driver_mujoco.ini");
	#else
	Sim::init("../model/scene.xml",keyboard);
	iniSim.open("../config/driver_mujoco.ini");
	#endif

	assistFlag=iniSim["assistFlag"];
	assistH=iniSim["assistH"];
	double cutF[NMotMain];
	iniSim.getArray("jntKp",jntKp,NMotMain);
	iniSim.getArray("jntKd",jntKd,NMotMain);
	iniSim.getArray("jntMaxTor",jntMaxTor,NMotMain);
	iniSim.getArray("cutF",cutF,NMotMain);

	in.sens.init(NMotMain,6,6);
	in.jntSdk.init(NMotMain, 6, 6);
	in.maniSdk.init(7,6,6,2,3);
	out.ctrl.init(NMotMain,6,6);
	out.param.init(NMotMain);

	dt=Sim::getMjModel().opt.timestep;
	For(NMotMain){
		fil[i].init(dt,cutF[i]);
	}
	Nabo::init(dt);
	if(Sim::getMjModel().nq>NMotMain){floatBaseFlag=1;}
	else{floatBaseFlag=0;in.sens.supP[2]=1;}
	msg<<"keyboard:\n {w s a d j l} for movments\n {f g- h+} for assist";

	mjData &d=Sim::getMjData();
	while(!glfwWindowShouldClose(&Sim::getWindow())){
		mjtNum simstart=d.time;
		//小循环内mj_step按硬件能力进行场景计算，小循环外按30帧更新显示
		while(d.time-simstart < 1.0/30){
			if(runFlag){
				runCtrl();
				Sim::runStep();
				//延时为了人眼看清楚。另：不同旋转视角渲染速度也不同
				if(slowRanderMs){this_thread::sleep_for(chrono::milliseconds(slowRanderMs));}
			}else{
				this_thread::sleep_for(chrono::milliseconds(10));
				break;
			}
		}
		Sim::rander(msg.str().data());
	}
	Sim::close();
	return 0;
}