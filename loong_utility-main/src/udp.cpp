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
#include"udp.h"
#include"iopack.h"
#include<sys/socket.h>
#include<netinet/in.h>
#include<sys/ioctl.h>
#include<arpa/inet.h>
#include<fcntl.h>
#include<unistd.h>
#include<sstream>

const int MaxBufLen=1024*1024;

namespace Udp{

class udpServerClass::impClass{
public:
	impClass();
	~impClass();
	bool openPort(unsigned short port);
	int recv(void*buf);
	void send(void*buf,int len);

	socklen_t addrLen;
	int sockfd=-1;
	unsigned short port;
	bool hasConnect=0,singleClient=1;
	int lostCnt,allowNewCnt;//断线allowNewCnt后，自动允许新连接。
	char rcvBuf[MaxBufLen];//防止接收覆盖

	sockaddr_in serverAddr, clientAddr, tmpAddr;
};
	udpServerClass::impClass::impClass(){}
	udpServerClass::impClass::~impClass(){
		if(sockfd>0){
			close(sockfd);
		}
	}
	bool udpServerClass::impClass::openPort(unsigned short port){
		if(sockfd>0){
			return 0;
		}
		hasConnect=0;
		this->port=port;
		addrLen=sizeof(serverAddr);
		sockfd=socket(AF_INET, SOCK_DGRAM, 0);
		if(sockfd<0){
			perror("## socket 创建失败");
			return 0;
		}
		int op=1;
		setsockopt(sockfd,SOL_SOCKET,SO_RCVBUF,(const void*) &op,sizeof(int));
		timeval timeout;
		timeout.tv_sec=0;
		timeout.tv_usec=10000;
		setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
		int flags=fcntl(sockfd, F_GETFL, 0);
		fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
		
		serverAddr.sin_family=AF_INET;
		serverAddr.sin_port=htons(port);
		serverAddr.sin_addr.s_addr=INADDR_ANY;

		if(bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) <0){
			stringstream ss;
			ss<<"$$ socket bind 失败，port="<<port;
			perror(ss.str().data());
			cout<<"可通过 sudo lsof -i:"<<port<<" 查看端口占用"<<endl;
			throw runtime_error("$$ socket bind 失败");
			return 0;
		}
		unsigned long on=1;
		ioctl(sockfd, FIONBIO, &on);
		lostCnt=0;
		return 1;
	}
	int udpServerClass::impClass::recv(void*buf){
		int len=recvfrom(sockfd, rcvBuf, MaxBufLen, 0, (sockaddr*)&tmpAddr, &addrLen);
		// print(len);
		// 防止多连接
		if(singleClient && len>0){
			if(!hasConnect){
				clientAddr=tmpAddr;
				hasConnect=1;
			}else if(clientAddr.sin_addr.s_addr != tmpAddr.sin_addr.s_addr){
				len=0;
			}
		}
		if(len>0){
			memcpy(buf, rcvBuf, len);
			((char*)buf)[len]='\0';
			lostCnt=0;
			return len;
		}else{
			lostCnt--;
		}
		if(lostCnt<allowNewCnt && allowNewCnt<0){
			hasConnect=0;
		}
		return lostCnt;
	}
	void udpServerClass::impClass::send(void*buf,int len){
		if(hasConnect){
			sendto(sockfd,buf,len,0,(sockaddr*)&clientAddr, addrLen);
		}
	}
// =============================================
	udpServerClass::udpServerClass():imp(*new impClass()){}
	udpServerClass::udpServerClass(unsigned short port,bool singleClient, int allowNewCnt):imp(*new impClass()){
		imp.openPort(port);
		imp.singleClient=singleClient;
		imp.allowNewCnt=allowNewCnt;
	}
	udpServerClass::~udpServerClass(){
		delete &imp;
	}
	bool udpServerClass::openPort(unsigned short port,bool singleClient, int allowNewCnt){
		imp.singleClient=singleClient;
		imp.allowNewCnt=allowNewCnt;
		return imp.openPort(port);
	}
	void udpServerClass::allowNew(){
		imp.hasConnect=0;
	}
	int udpServerClass::recv(void*buf){
		return imp.recv(buf);
	}
	void udpServerClass::send(void*buf,int len){
		imp.send(buf,len);
	}
	bool udpServerClass::isConnect(){
		return imp.hasConnect;
	}
// ###############################################

class udpClientClass::impClass{
public:
	impClass(udpClientClass*omp);
	~impClass();
	bool connect(const char*ip, unsigned short port);
	void send(void*buf,int len);
	int recv(void*buf);

	udpClientClass&omp;
	socklen_t addrLen;
	int sockfd=-1;
	unsigned short port;

	sockaddr_in serverAddr;
	int lostCnt;
	bool appointed=0;
};
	udpClientClass::impClass::impClass(udpClientClass*omp):omp(*omp){}
	bool udpClientClass::impClass::connect(const char*ip, unsigned short port){
		if(sockfd>0){
			return 0;
		}
		this->port=port;
		addrLen=sizeof(serverAddr);
		sockfd=socket(AF_INET, SOCK_DGRAM, 0);
		if(sockfd<0){
			perror("$$ socket 创建失败");
			throw runtime_error("$$ socket 创建失败");
			return 0;
		}
		int op=1;
		setsockopt(sockfd,SOL_SOCKET,SO_RCVBUF,(const void*) &op,sizeof(int));
		timeval timeout;
		timeout.tv_sec=0;
		timeout.tv_usec=10000;
		setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
		int flags=fcntl(sockfd, F_GETFL, 0);
		fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
		
		serverAddr.sin_family=AF_INET;
		serverAddr.sin_port=htons(port);
		serverAddr.sin_addr.s_addr=inet_addr(ip);

		unsigned long on=1;
		ioctl(sockfd, FIONBIO, &on);

		lostCnt=0;
		appointed=1;
		return 1;
	}
	udpClientClass::impClass::~impClass(){
		if(sockfd>0){
			close(sockfd);
		}
	}
	int udpClientClass::impClass::recv(void*buf){
		int len=recvfrom(sockfd, buf, MaxBufLen, 0, 0, 0);
		if(len>0){
			((char*)buf)[len]='\0';
			lostCnt=0;
			return len;
		}else if(len==0){
			return 0;
		}else{
			lostCnt--;
		}
		return lostCnt;
	}
	void udpClientClass::impClass::send(void*buf,int len){
		if(appointed){
			sendto(sockfd,buf,len,0,(sockaddr*)&serverAddr,addrLen);
		}
	}
// =============================================
	udpClientClass::udpClientClass():imp(*new impClass(this)){}
	udpClientClass::udpClientClass(const char*ip, unsigned short port):imp(*new impClass(this)){
		imp.connect(ip,port);
	}
	udpClientClass::~udpClientClass(){
		delete &imp;
	}
	bool udpClientClass::connect(const char*ip,unsigned short port){
		return imp.connect(ip,port);
	}
	void udpClientClass::send(void*buf,int len){
		imp.send(buf,len);
	}
	int udpClientClass::recv(void*buf){
		return imp.recv(buf);
	}
}//namespace

