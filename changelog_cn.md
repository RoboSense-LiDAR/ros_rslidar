# RSLidar ROS驱动修改记录

### v1.0.0(2018-12-03)
---
初版提交

### v1.1.0(2019-03-18)
---
修改点：
* 新增**max_distance**和**min_distance**参数Launch文件设置功能
* 增加R值校正
* 兼容适配0.5cm精度雷达
* 增加支持反射率模式3
* 新增支持解析雷达**双回波模式**数据
* 将多雷达同步功能的获取时间戳代码从input.cc移到rsdriver.cpp
* 增加**resolution**和**intensity**通过launch文件设置

