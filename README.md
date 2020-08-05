# Smoothly RRT
基于B样条曲线的连续曲率RRT算法。

## 使用
在configs.py中设定好初始位置和终止位置等参数后，运行main.py。

单击鼠标左键添加障碍物（注意不可离起始位置和终止位置过近以免影响随机树生成），再次单击则停止添加；单击鼠标中键清空已有障碍物；单击鼠标右键开始生成轨迹。最终结果如下图所示。

## References：

Wei K, Ren B. A method on dynamic path planning for robotic manipulator autonomous obstacle avoidance based on an improved RRT algorithm[J]. Sensors, 2018, 18(2): 571.
