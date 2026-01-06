#!/bin/bash
# =========== ***doc description @ yyp*** ===========
# Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# Author: YYP
# ===================================================


if [ ! -e "../log" ]; then
	mkdir ../log
else
	echo 将删除最近20个以外的log文件
	cd ../log
	files=$(ls -lt | tail -n +21 | awk '{print $9}')
	for file in $files; do
		sudo rm -f "$file"
	done
fi
echo ==========
if [ "$(arch)" = "x86_64" ]; then
	tgtArch=x64
else
	tgtArch="a64"
fi

cd ../bin

sudo ./loong_interface_$tgtArch |sudo tee ../log/terminal_interface.txt

