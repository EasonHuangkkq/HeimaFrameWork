#!/bin/bash
# 切换到脚本所在目录
cd "$(dirname "$0")"

# 如果 venv 文件夹不存在，则自动创建并安装依赖
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
    
    echo "Installing requirements..."
    ./venv/bin/pip install -r requirements.txt
fi

echo "Starting Web Server..."
# 使用 venv 里面的 Python 执行服务器
./venv/bin/python web_server.py
