cmake_minimum_required(VERSION 3.0.2)
project(sim_utils)

## 纯 python + launch 的包, 不需要编译任何 C++.
find_package(catkin REQUIRED)

catkin_package()

## 注意: 这里**故意不用** catkin_install_python.
## catkin_install_python 会把脚本第一行 shebang 改写成 cmake 配置时的
## PYTHON_EXECUTABLE; 本机 PATH 里有 conda, catkin_make 因此选中了
## /home/safina/miniconda3/bin/python3, 生成的
##   devel/lib/sim_utils/vio_offset.py  ->  #!/home/safina/miniconda3/bin/python3
## 而 conda 的 python3.13 里没有 netifaces, rospy 建订阅连接线程时直接
## ModuleNotFoundError 挂掉, 表现为节点跑起来了、但一个话题都收不到.
## 改成 install(PROGRAMS): 保留源码里写死的 #!/usr/bin/python3,
## devel 空间则直接使用源码目录 scripts/vio_offset.py.
install(PROGRAMS scripts/vio_offset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
