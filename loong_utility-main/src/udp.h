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
udp封装
=====================================================*/
#pragma once
namespace Udp{

class udpServerClass{
public:
	udpServerClass();
	//断线后内部会自动记录调用次数的负数。调用负次数小于allowNewCnt后，自动允许新连接。allowNewCnt>=0，不自动允许新连接
	udpServerClass(unsigned short port,bool singleClient=1, int allowNewCnt=-500);
	~udpServerClass();
	//断线后内部会自动记录调用次数的负数。调用负次数小于allowNewCnt后，自动允许新连接。allowNewCnt>=0，不自动允许新连接
	bool openPort(unsigned short port,bool singleClient=1, int allowNewCnt=-500);
	void allowNew();//手动允许新连接
	int recv(void*buf);//等待时间10us，正常接收到返回长度，多联返回0，失联返回调用次数的负数
	void send(void*buf,int len);
	bool isConnect();
private:
	udpServerClass(const udpServerClass&)=delete;
	udpServerClass& operator=(const udpServerClass&)=delete;
	class impClass;
	impClass&imp;
};
// =================================
class udpClientClass{
public:
	udpClientClass();
	udpClientClass(const char*ip, unsigned short port);
	~udpClientClass();
	bool connect(const char*ip, unsigned short port);
	int recv(void*buf);//等待时间10us，接收到返回长度，否则返回调用次数的负数
	void send(void*buf,int len);
private:
	udpClientClass(const udpClientClass&)=delete;
	udpClientClass& operator=(const udpClientClass&)=delete;
	class impClass;
	impClass&imp;
};

}//namespace

