#!/bin/bash
# 停止所有仿真进程
# 使用: ./stop_all.sh

echo "停止所有仿真进程..."

killall -9 px4 gzserver gzclient rosmaster roscore rviz 2>/dev/null

echo "已停止所有进程"
