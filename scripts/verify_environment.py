#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
环境验证脚本
===========

验证 ROS + MAVROS 环境是否正确配置

运行方式:
    python3 verify_environment.py
"""

import subprocess
import sys
import os


class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    END = '\033[0m'


def print_ok(msg):
    print(f"{Colors.GREEN}[✓]{Colors.END} {msg}")


def print_fail(msg):
    print(f"{Colors.RED}[✗]{Colors.END} {msg}")


def print_warn(msg):
    print(f"{Colors.YELLOW}[!]{Colors.END} {msg}")


def check_command(cmd, name):
    """检查命令是否可用"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            print_ok(f"{name}: 可用")
            return True
        else:
            print_fail(f"{name}: 不可用")
            return False
    except Exception as e:
        print_fail(f"{name}: 检查失败 - {e}")
        return False


def check_ros():
    """检查 ROS 安装"""
    print("\n=== 检查 ROS ===")

    # 检查 ROS 版本
    ros_ok = check_command("rosversion -d", "ROS 发行版")

    # 检查 roscore
    check_command("which roscore", "roscore")

    # 检查环境变量
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print_ok(f"ROS_DISTRO: {ros_distro}")
    else:
        print_fail("ROS_DISTRO 未设置，请执行 source /opt/ros/noetic/setup.bash")
        ros_ok = False

    return ros_ok


def check_mavros():
    """检查 MAVROS 安装"""
    print("\n=== 检查 MAVROS ===")

    mavros_ok = check_command("rospack find mavros", "mavros 包")
    check_command("rospack find mavros_extras", "mavros_extras 包")

    return mavros_ok


def check_px4():
    """检查 PX4 安装"""
    print("\n=== 检查 PX4 ===")

    px4_dir = os.path.expanduser("~/PX4-Autopilot")
    if os.path.isdir(px4_dir):
        print_ok(f"PX4 目录: {px4_dir}")

        # 检查编译产物
        build_dir = os.path.join(px4_dir, "build/px4_sitl_default")
        if os.path.isdir(build_dir):
            print_ok("PX4 SITL 已编译")
            return True
        else:
            print_warn("PX4 SITL 未编译，请运行: cd ~/PX4-Autopilot && make px4_sitl gazebo")
            return False
    else:
        print_fail(f"PX4 目录不存在: {px4_dir}")
        return False


def check_gazebo():
    """检查 Gazebo 安装"""
    print("\n=== 检查 Gazebo ===")

    gazebo_ok = check_command("gazebo --version", "Gazebo")

    # 检查 ROS 集成
    check_command("rospack find gazebo_ros", "gazebo_ros 包")

    return gazebo_ok


def check_python_deps():
    """检查 Python 依赖"""
    print("\n=== 检查 Python 依赖 ===")

    deps = ['numpy', 'matplotlib', 'scipy']
    all_ok = True

    for dep in deps:
        try:
            __import__(dep)
            print_ok(f"{dep}")
        except ImportError:
            print_fail(f"{dep} - 请安装: pip3 install {dep}")
            all_ok = False

    return all_ok


def main():
    print("=" * 50)
    print("  无人机仿真环境验证")
    print("=" * 50)

    results = {
        'ROS': check_ros(),
        'MAVROS': check_mavros(),
        'PX4': check_px4(),
        'Gazebo': check_gazebo(),
        'Python': check_python_deps()
    }

    print("\n" + "=" * 50)
    print("  验证结果汇总")
    print("=" * 50)

    all_passed = True
    for name, passed in results.items():
        if passed:
            print_ok(name)
        else:
            print_fail(name)
            all_passed = False

    print("")
    if all_passed:
        print_ok("所有检查通过! 可以开始仿真了。")
        print("\n下一步:")
        print("  1. 终端1: cd ~/PX4-Autopilot && make px4_sitl gazebo")
        print("  2. 终端2: roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\"")
        print("  3. 终端3: python3 offboard_control.py")
    else:
        print_fail("部分检查未通过，请根据上述提示修复问题。")

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
