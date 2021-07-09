#文件夹内容说明：

-html  
存放网页布局用html文件  
-images  
存放标定图片/结果展示/等图片  
-include  
存放各种类编写的内容以及工具库
-js
网页控制脚本

#文件内容说明
  
识别相关文件：  
include/tag.py:  用于存放单个物块类的编写  
include/Detector.py:   用于存放识别类的编写   
使用时，调用一个单独的线程进行摄像头的读入以及物块的各参数，可通过tag_list调用识别得到的物块的信息    
  
  
键盘鼠标模拟控制相关文件：  
include/mouseControl.py：  用于存放键盘模拟信号输入的控制类   
/mouseTest.py: 用于测试物块控制键盘鼠标任务的测试文件，其中的fakemouse 和 fakekeyboard函数分别用于测试鼠标和键盘   
  
区域划分相关文件：   
include/areaDivide.py:  用于存放区域划分的类，后续添加意义属性的赋予

工具库：  
include/angle_change.py:  旋转向量与欧拉角的转换函数    
include/area2D.py:  二维区域类，用于区域划分  
include/area3D.py:  三维区域类，用于区域划分  
include/kalmanFilter.py:  二维点滤波  
include/KalmanFilter.py:  边框滤波  