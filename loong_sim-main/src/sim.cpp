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
#include"sim.h"
// #include"timing.h"
#ifdef _WIN32
#include<windows.h>
#endif

namespace Sim{

mjModel *m;
mjData *d;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window;

bool button_left;
bool button_middle;
bool button_right;
double lastx;
double lasty;

void mouse_button(GLFWwindow* window, int button, int act, int mods){
	// update button state
	button_left=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos){
	if(!button_left && !button_middle && !button_right){return;}
	// compute mouse displacement, save
	double dx=xpos-lastx, dy=ypos-lasty;
	lastx=xpos; lasty=ypos;
	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	// get shift key state
	bool mod_shift=(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
	// determine action based on mouse button
	mjtMouse action;
	if(button_right){
		action=mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	}else if(button_left){
		action=mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	}else{
		action=mjMOUSE_ZOOM;
	}
	mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
//==scroll callback=============
void scroll(GLFWwindow* window, double xoffset, double yoffset){
  // emulate vertical mouse motion=5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}
//===================================================================
void init(const string& sceneFile,void(*keyboardFunc)(GLFWwindow* window, int key, int scancode, int act, int mods)){
	char errorMsg[64]{"fail to load xml!\n"};
	m=mj_loadXML(sceneFile.data(),0,errorMsg,64);
	d=mj_makeData(m);

	if(!glfwInit()){mju_error("Could not initialize GLFW");}
	// create window, make OpenGL context current, request v-sync
	window=glfwCreateWindow(1800, 1200, "nabo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	cam.lookat[2]=0.9;
	cam.azimuth=90;//角度
	cam.elevation=-5;
	cam.distance=3;
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);
	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_100);
	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboardFunc);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);
}
void reset(){
	mj_resetData(m, d);
	mj_forward(m, d);
}
void runStep(){
	mj_step(m, d);
}
void rander(const char*msg){
	mjrRect viewport ={0, 0, 0, 0};
	glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
	mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
	mjr_render(viewport, &scn, &con);

	mjrRect rect{0,0,200,50};
	mjr_overlay(mjFONT_BIG, mjGRID_BOTTOMLEFT, rect, msg, 0, &con);
	glfwSwapBuffers(window);// swap OpenGL buffers(blocking call due to v-sync)
	glfwPollEvents();// process pending GUI events, call GLFW callbacks
}
void close(){
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
	mj_deleteData(d);
	mj_deleteModel(m);
	// terminate GLFW(crashes with Linux NVidia drivers)
	#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
	#endif
}

mjModel &getMjModel(){return *m;}
mjData &getMjData(){return *d;}
GLFWwindow &getWindow(){return *window;}

}//namespace