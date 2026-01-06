#!/bin/bash


if [ ! -e "../log" ]; then
	mkdir ../log
else
	echo 将删除最近20个以外的log文件
	cd ../log
	files=$(ls -lt | tail -n +21 | awk '{print $9}')
	for file in $files; do
		rm -rf "$file"
	done
fi
echo ==========

cd ../build
if [ "$1" == "make.sh" ]; then
	./main_mani
else 
	./main_mani $1
fi