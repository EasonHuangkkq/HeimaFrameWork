#!/bin/bash
# 安全清理日志（保留最近 N 天，或按文件名过滤）

cd "$(dirname "$0")/../log"

echo "当前 log 目录内容："
ls -lh

echo ""
echo "选择清理方式："
echo "1) 清理所有日志 (危险！)"
echo "2) 清理 driver 日志 (terminal_driver.txt)"
echo "3) 清理 locomotion 日志 (terminal_locomotion.txt)"
echo "4) 清理 interface 日志 (terminal_interface.txt)"
echo "5) 清理 7 天前的日志"
echo "6) 查看日志大小"
echo "q) 退出"
read -p "请输入选项 [1-6,q]: " choice

case $choice in
    1)
        echo "清理所有日志..."
        sudo rm -rf ./*
        echo "✅ 已清理所有日志"
        ;;
    2)
        echo "清理 driver 日志..."
        sudo rm -f terminal_driver.txt
        echo "✅ 已清理 driver 日志"
        ;;
    3)
        echo "清理 locomotion 日志..."
        sudo rm -f terminal_locomotion.txt
        echo "✅ 已清理 locomotion 日志"
        ;;
    4)
        echo "清理 interface 日志..."
        sudo rm -f terminal_interface.txt
        echo "✅ 已清理 interface 日志"
        ;;
    5)
        echo "清理 7 天前的日志..."
        sudo find . -name "terminal_*.txt" -mtime +7 -delete
        echo "✅ 已清理 7 天前的日志"
        ;;
    6)
        echo "日志文件大小："
        du -sh terminal_*.txt 2>/dev/null || echo "无日志文件"
        ;;
    q)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo ""
echo "当前 log 目录内容："
ls -lh
