# Copyright (c) 2025 RobotPilots-SZU
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

zephyr_syscall_include_directories(include)  # 让Zephyr系统调用器处理syscall时加入include目录

zephyr_include_directories(include)  # 将 include 目录加入 Zephyr 的全局头文件搜索路径

add_subdirectory(include)
add_subdirectory(lib)
