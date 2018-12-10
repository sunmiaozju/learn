##### 1. [ros tf模块介绍](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter8/8.1.html)
  ROS中的tf是一个可以让用户随时记录多个坐标系的软件包，坐标变换包括了位置和姿态两个方面的变换。tf保持缓存的树形结构中存储了坐标系之间的关系，并且允许用户在任何期望的时间点在任何两个坐标系之间转换点，矢量等．

  tf模块存储不同坐标系的坐标关系是用树形结构来表示的，称为tf tree。ROS中机器人模型包含大量的部件，这些部件称之为link,每一个link上面对应着一个frame, 即一个坐标系．link和frame概念是绑定在一起的。tf tree的每一个圆圈代表一个frame,对应着机器人上的一个link，任意的两个frame之间都必须是联通的，如果出现某一环节的断裂，就会**引发tf error系统报错** 。

  完整的tf tree不能有任何断层的地方，这样我们才能查清楚任意两个frame之间的关系．每两个frame之间都有一个broadcaster,这就是为了使得两个frame之间能够正确连通，中间都会有一个Node来发布消息来broadcaster.如果缺少Node来发布消息维护连通，那么这两个frame之间的连接就会断掉．broadcaster就是一个publisher,如果两个frame之间发生了相对运动，broadcaster就会发布相关消息．　

  下面就是tf/tfMessage.msg或tf2_msgs/TFMessage消息的标准格式规范

  ```
    geometry_msgs/TransformStamped[] transforms
        std_msgs/Header header
                uint32 seq
                time stamp
                string frame_id
        string child_frame_id
        geometry_msgs/Transform transform
                geometry_msgs/Vector3 translation
                        float64 x
                        float64 y
                        float64 z
                geometry_msgs/Quaternion rotation
                        float64 x
                        float64 y
                        flaot64 z
                        float64 w
  ```
  每一个broadcaster都会发出这样的消息到/tf 话题下面，众多的这个话题消息构成了TransformStamped数组，一个TransformStamped数组就是一个TF tree。

##### 2. [tf 数学基础](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter8old/7.1.1.html)

简要介绍：
  - 旋转矩阵
  - 欧拉角 “航偏-俯仰-翻滚”（yaw-pitch-roll）。可以简单记忆rpy-xyz
  - 四元数 旋转矩阵用9个量来描述3自由度的旋转，具有冗余性；欧拉角虽然用3个量来描述3自由度的旋转，但是具有万向锁的问题，因此一般选择用四元数（ROS当中描述转向的都是采用的四元数）
  - 四元树、旋转矩阵、欧拉角都是可以两两相互转换的

##### 3. clion 快捷键
- ctrl + / 快捷注释
- Ctrl + Alt + L 格式化代码
- Ctrl + Alt + -/+ 展开折叠当前函数、类
- Ctrl + Shift + -/+ 展开折叠所有函数、类

##### 4. imu介绍
IMU（Inertial Measurement Unit）学名惯性测量单元

理论力学告诉我们，所有的运动都可以分解为一个直线运动和一个旋转运动，故这个惯性测量单元就是测量这两种运动，直线运动通过加速度计可以测量，旋转运动则通过陀螺。一般的，一个IMU包含了三个单轴的加速度计和三个单轴的陀螺，加速度计检测物体在载体坐标系统独立三轴的加速度信号，而陀螺检测载体相对于导航坐标系的角速度信号，测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。在导航中用着很重要的应用价值。为了提高可靠性，还可以为每个轴配备更多的传感器。一般而言IMU要安装在被测物体的重心上。
