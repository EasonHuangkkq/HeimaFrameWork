#!/usr/bin/env python3
'''=========== ***doc description @ yyp*** ===========
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Author: YYP

======================================================'''
import socket
import tkinter as tk
import struct
from threading import Thread
import time
import sys
from functools import partial

# ip='192.168.1.201'
# ip='192.168.114.75'
ip='0.0.0.0'
port=8000

sk=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
cmd=bytearray([0x81,0,0,0,0x60,0,
				0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,
				0x29,0x5c,0xf,0x3f,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0,  0x9a,0x99,0x19,0x3e,
				0,13,0,
				0,0,0,0,0,0,0,0,0,0])

def skLoop():
    while(1):
        sk.sendto(cmd,(ip,port))
        time.sleep(0.5)

vx=0;vy=0;wz=0
def clearV():
    global vx,vy,wz
    vx=0;vy=0;wz=0
    updateVcmd()
def updateVcmd():
#     float leftJoyLeft;//4,[7,]
#     float rightJoyRight;//4,[11,]
#     float leftJoyDown;//4,[15,]
    cmd[7:11]=struct.pack('<f',vy*100)
    cmd[11:15]=struct.pack('<f',-wz*100)
    cmd[15:19]=struct.pack('<f',-vx*100)
    global joyLabel
    joyLabel["text"]=f'joy={vx:.2f},  {vy:.2f},  {wz:.2f},  0(底层有非线性映射)'
#==按钮函数==
def disable():
    clearV()
    cmd[84]=13
    sk.sendto(cmd,(ip,port))
def clearErr():
    clearV()
    tmp=cmd[84]
    cmd[84]=230
    sk.sendto(cmd,(ip,port))
    cmd[84]=tmp

def setCmd(key):
    cmd[84]=key
def setCmdV0(key):
    clearV()
    cmd[84]=key

def butCmd():
    root.focus_set()
    tmp=entCmd.get()
    if(len(tmp)>0):
        cmd[84]=int(tmp)
        print(tmp)


#==键盘按键==
def keyPress(ev):
    if(entCmd==entCmd.focus_get()):
        joyLabel["text"]="输入框占有焦点，键盘快捷键失效"
        return
    global vx,vy,wz
    key=ev.keysym
    print(key)
    if(key=='q'):
        clearV()
        cmd[84]=6
    elif(key=='e'):
        clearV()
        cmd[84]=7
    if(key=='w'):
        vx+=0.1
        updateVcmd()
    elif(key=='s'):
        vx-=0.1
        updateVcmd()
    elif(key=='a'):
        vy+=0.1
        updateVcmd()
    elif(key=='d'):
        vy-=0.1
        updateVcmd()
    elif(key=='j'):
        wz+=0.1
        updateVcmd()
    elif(key=='l'):
        wz-=0.1
        updateVcmd()
    elif(key=='space'):
        clearV()

#==界面==
root=tk.Tk()
# root.configure(background='#ccc')
if sys.platform.startswith('win'):
    root.geometry('800x400')
    length=100
else:
    if(root.winfo_screenwidth()>5000):
        root.geometry('1200x600+4600+1000')
        length=120
    else:
        root.geometry('1000x500+2600+200')
        length=100

# ==系统 按键====
tk.Label(root,text="系统:").place(anchor='nw',relx=0.02,rely=0.05)
tk.Button(root,text='[230]\n清错',command=clearErr
          ).place(anchor='nw',relx=0.1,rely=0,width=length,height=length)
tips=['cctv介出', 'cctv介入',    '导航介出', '导航介入',   'mani介出', 'mani介入', 'mani自动',]
keys=[200, 201,   204, 205,   220, 221, 222]
for i in range(len(tips)):
    tk.Button(root,text=f'[{keys[i]}]\n{tips[i]}',command=partial(setCmd, keys[i])
              ).place(anchor='nw',relx=0.1*i+0.2, rely=0,width=length,height=length)

# ==loco==
tk.Label(root,text="loco:").place(anchor='nw',relx=0.02,rely=0.25)
tips=['en','dis','idle','damp','rc','rl','jntSdk']
keys=[1,13,3,12,2, 4,23]
for i in range(len(tips)):
    tk.Button(root,text=f'[{keys[i]}]\n{tips[i]}',command=partial(setCmd, keys[i])
              ).place(anchor='nw',relx=0.1*i+0.1, rely=0.2,width=length,height=length)

# ==提示栏==
tk.Label(root,text="[6]start(键盘q)，[7]stop(键盘e)，键盘wasdjl模拟摇杆增量，空格清零",foreground='#888'
         ).place(anchor='nw',relx=0.05,rely=0.4)
global joyLabel
joyLabel=tk.Label(root,text="提示栏")
joyLabel.place(anchor='nw',relx=0.05,rely=0.48)

# ==输入框==
entCmd=tk.Entry(root)
entCmd.place(anchor='nw',relx=0.8,rely=0.45,width=length,height=length*0.6)
tk.Button(root,text='send\n/转移焦点',command=butCmd
          ).place(anchor='nw',relx=0.9,rely=0.4,width=length,height=length)

# ==mani==
tk.Label(root,text="mani:").place(anchor='nw',relx=0.02,rely=0.65)
tips=['en','dis','idle','damp','rc','act','mani']
keys=[1,13,112,113,114,115,116]
for i in range(len(tips)):
    tk.Button(root,text=f'[{keys[i]}]\n{tips[i]}',command=partial(setCmdV0, keys[i])
              ).place(anchor='nw',relx=0.1*i+0.1, rely=0.6,width=length,height=length)

tips=['','暂停','启动','回正/遥操','抱拳/遥操','左抬/遥操','右抬/遥操','左挥/遥操','右挥/遥操','握拳/遥操']
for i in range(1,10):
    tk.Button(root,text=f'[15{i}]\n{tips[i]}',command=partial(setCmd, 150+i)
              ).place(anchor='nw',relx=0.1*i-0.1, rely=0.8,width=length,height=length)

# ==自定义按键==

# def test55():
#     cmd[84]=55
# def test56():
#     cmd[84]=56
# tk.Button(root,text='[55]\n自定义',command=test55).place(anchor='nw',relx=0.8,rely=0.2,width=length,height=length)
# tk.Button(root,text='[56]\n自定义',command=test56).place(anchor='nw',relx=0.9,rely=0.2,width=length,height=length)




root.bind('<Key>',keyPress)
# root.bind('<FocusIn>',focus)
th=Thread(target=skLoop)
th.daemon=1
th.start()
tk.mainloop()
