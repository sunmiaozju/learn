[TOC]

### 1. ros tf模块介绍
参考链接：[ros tf模块](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter8/8.1.html)

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

### 2. tf 数学基础
参考链接：[tf 数学基础](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter8old/7.1.1.html)

简要介绍：
  - 旋转矩阵
  - 欧拉角 “航偏-俯仰-翻滚”（yaw-pitch-roll）。可以简单记忆rpy-xyz
  - 四元数 旋转矩阵用9个量来描述3自由度的旋转，具有冗余性；欧拉角虽然用3个量来描述3自由度的旋转，但是具有万向锁的问题，因此一般选择用四元数（ROS当中描述转向的都是采用的四元数）
  - 四元树、旋转矩阵、欧拉角都是可以两两相互转换的

### 3. clion 快捷键
- ctrl + / 快捷注释
- Ctrl + Alt + L 格式化代码
- Ctrl + Alt + -/+ 展开折叠当前函数、类
- Ctrl + Shift + -/+ 展开折叠所有函数、类

### 4. imu介绍
IMU（Inertial Measurement Unit）学名惯性测量单元

理论力学告诉我们，所有的运动都可以分解为一个直线运动和一个旋转运动，故这个惯性测量单元就是测量这两种运动，直线运动通过加速度计可以测量，旋转运动则通过陀螺。一般的，一个IMU包含了三个单轴的加速度计和三个单轴的陀螺，加速度计检测物体在载体坐标系统独立三轴的加速度信号，而陀螺检测载体相对于导航坐标系的角速度信号，测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。在导航中用着很重要的应用价值。为了提高可靠性，还可以为每个轴配备更多的传感器。一般而言IMU要安装在被测物体的重心上。

通过加速度计一次积分可以得到速度，加速度计可以二次积分得出位移，陀螺仪一次积分可以得到欧拉角度

### 5. ros机器人坐标系
最常用的就是**map，odom，base_link，base_laser**坐标系

- map:地图坐标系，顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。

- base_link:机器人本体坐标系，与机器人中心重合，当然有些机器人(PR 2)是base_footprint,其实是一个意思。

- odom：里程计坐标系，这里要区分开odom topic，这是两个概念，一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。但是两者也有关系，odom topic转化得位姿矩阵是odom-->base_link的tf关系。这时可有会有疑问，odom和map坐标系是不是重合的？可以很肯定的告诉你，机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.

- base_laser:激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的。

### 6. tesnorboard使用
参考链接：[TensorBoard可视化网络结构和参数](https://blog.csdn.net/helei001/article/details/51842531)

### 7. tensorflow模型恢复和重建
参考链接：[模型恢复和重建](https://blog.csdn.net/tan_handsome/article/details/79303269)

### 8. c++本地时间及log文件流
```c++
// Set log file name.
std::ofstream ofs;
char buffer[80];
//get local time
std::time_t now = std::time(NULL);
std::tm* pnow = std::localtime(&now);
std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
filename = "ndt_matching_" + std::string(buffer) + ".csv";
ofs.open(filename.c_str(), std::ios::app);
```
### 9. ros节点句柄及其命名空间
有关于ros句柄命名空间的例子，可以确定某一个句柄能访问到参数服务器的哪些变量。

launch文件中可能定义了ns=="node_namespace"，也可能没定义，这里我们假设launch中没有定义

如果要是定义了，那么就像下面这样：
```xml
<node pkg="image_processor" type="image_rectifier" name="rectifier" ns="$(arg camera_id)" output="screen">
```

这个是定义这个节点的名字node_name，一般在私有句柄使用
```
ros::init(argc, argv, "node_name");
```
默认生成的节点句柄实例，命名空间为全局变量或者是launch文件中定义的ns=="node_namespace"（这里我们假设launch里面没有定义ns）
```
ros::NodeHandle n;
```
n1命名空间为/sub1
```
ros::NodeHandle n1("sub1");
```
n2命名空间为/sub1/sub2
```
ros::NodeHandle n2(n1,"sub2");
```
这个用～定义的节点句柄代表私有句柄，pn1的命名空间为/node_name，节点名称在前面ros::init指定的

```
ros::NodeHandle pn1("~");
```
pn2命名空间为/node_name/sub
```
ros::NodeHandle pn2("~sub");
ros::NodeHandle pn2("~/sub"); //这么写和上面一样
```
gn的命名空间为/global

不推荐这么做，因为这里指定了符号“/”，使得这个节点不能嵌套在其他节点的命名空间中，最好还是用上面说的相对指定的方法
```
ros::NodeHandle gn("/global");
```
再补充一点，私有句柄只能访问参数服务器中的本节点的参数

比如，下面是autoware中lidar_localizer软件包中的ndt_matching节点的launch文件，那么这个节点的私有句柄就只能访问这个节点下面的param参数。
```
<launch>

  <arg name="method_type" default="0" /> <!-- pcl_generic=0, pcl_anh=1, pcl_anh_gpu=2, pcl_openmp=3 -->
  <arg name="use_gnss" default="1" />
  <arg name="use_odom" default="false" />
  <arg name="use_imu" default="false" />
  <arg name="imu_upside_down" default="false" />
  <arg name="imu_topic" default="/imu_raw" />
  <arg name="queue_size" default="1" />
  <arg name="offset" default="linear" />
  <arg name="get_height" default="false" />
  <arg name="use_local_transform" default="false" />
  <arg name="sync" default="false" />

  <node pkg="lidar_localizer" type="ndt_matching" name="ndt_matching" output="log">
    <param name="method_type" value="$(arg method_type)" />
    <param name="use_gnss" value="$(arg use_gnss)" />
    <param name="use_odom" value="$(arg use_odom)" />
    <param name="use_imu" value="$(arg use_imu)" />
    <param name="imu_upside_down" value="$(arg imu_upside_down)" />
    <param name="imu_topic" value="$(arg imu_topic)" />
    <param name="queue_size" value="$(arg queue_size)" />
    <param name="offset" value="$(arg offset)" />
    <param name="get_height" value="$(arg get_height)" />
    <param name="use_local_transform" value="$(arg use_local_transform)" />
    <remap from="/points_raw" to="/sync_drivers/points_raw" if="$(arg sync)" />
  </node>

</launch>
```
### 10. python 多进程处理
参考链接：[python 多进程](https://www.liaoxuefeng.com/wiki/0014316089557264a6b348958f449949df42a6d3a2e542c000/001431927781401bb47ccf187b24c3b955157bb12c5882d000)

### 11. np.random.choice()用法
```
a1 = np.random.choice(a=5, size=3, replace=False, p=None)
```
函数的作用是从[0,5)中以概率P，随机选择3个, p没有指定的时候相当于是一致的分布，

replace代表是不是放回抽样，如果是False的话，那么是不放回抽样，出来的数都不一样，如果是
True的话， 是放回抽样，出来的数有可能会重复。

输出的结果如下所示：
```
>>> a1 = np.random.choice(a=5, size=3, replace=False, p=None)
>>> print a1
[4 1 2]
>>> a1 = np.random.choice(a=5, size=3, replace=False, p=None)
>>> print a1
[0 4 2]
>>> a1 = np.random.choice(a=5, size=3, replace=False, p=None)
>>> print a1
[4 2 1]
>>> a1 = np.random.choice(a=5, size=3, replace=False, p=None)
>>> print a1
[4 1 2]
>>> a1 = np.random.choice(a=5, size=3, replace=False, p=None)
>>> print a1
[2 0 4]
```
如果是要非一致的分布，可以指定每一个的概率：
```
a2 = np.random.choice(a=5, size=3, replace=False, p=[0.2, 0.1, 0.3, 0.4, 0.0])
```

### 12. numpy.transpose 详解

参考链接：[numpy.transpose 详解](https://blog.csdn.net/u012762410/article/details/78912667)

### 13. 使用alias设置方便的快捷指令
alias是shell的内建命令，可以用于创建命令的别名，可以把我们需要经常输入的长命令替换为自定义的简短命令，更加方便。语法非常简单：
```
alias your_alias='old_long_command'
```
比如我们可以把远程连接ssh的，常用的git命令起个别名，例如，首先打开
```
gedit ~/.bashrc
```
在后面输入下面的内容
```
# set alias
alias gpush='git push origin master'
alias gadd='git add .'
alias gcommit='git commit -m'
alias gstatus='git status'
alias yunle='ssh yunle@10.0.0.3'
alias smartcar='ssh smartcar@10.214.143.121'
```
然后
```
source ~/.bashrc
```
之后，就可以使用新定义的别名来代替原来的长命令了，而且还支持tab补全

### 13. linux 上一个命令
linux命令行，两个感叹号代表上一次输入的命令

### 14. 查看文件夹大小
查看当前目录下所有目录以及子目录的大小：
```
du -h
```
查看当前文件夹下面深度为一层的文件夹大小
```
du -h --max-depth=1
```

### 15. C++ string 转化为 C语言的 char *
```
your_string_name.c_str()
```
### 16、 ROS设置更新频率
LOOP_RATE_ = 30 代表你的处理会被归一化到30hz
```
ros::Rate loop_rate(LOOP_RATE_);
while(ros::ok){
    ros::spinOnce();
    //your_process_code is here
    loop_rate.sleep();
}
```
### 17、设置ROS通讯连接
假设A是ROS的主机,先确定主机的ip地址和ROS_MASTER_URI参数
```
echo $ROS_MASTER_URI
ifconfig
```
输出：
```
http://localhost:11311
```
然后，需要连接到一个ROS通讯系统的设备要和主机连接到同一个局域网上
将设备的ROS_MASTER_URI设置为
```
export ROS_MASTER_URI="http://10.0.0.23:11311"
```
写到～/.basherc

同时还要修改host，具体方法参考如下链接：
https://blog.csdn.net/heyijia0327/article/details/42065293

### 18、ROS launch文件导入其他launch文件和yaml文件
```
<launch>
  <include file="$(find kinect2_bridge)/launch/kinect2_bridge_ir.launch"/>
  <node pkg="detection" type="ground_based_people_detector" name="ground_based_people_detector" output="screen">
    <rosparam command="load" file="$(find detection)/conf/ground_based_people_detector_kinect2.yaml" />
  </node>
</launch>
```

### 19、 vscode 常用快捷键
快捷键|说明
---|---
Ctrl + K Ctrl + 0  | 折叠所有代码块
Ctrl + K Ctrl + j  | 展开所有代码块
Ctrl + Shift + i |  格式化代码
Ctrl + Shift + v |  查看markdown文件


### 20、std::getline

getline从输入流中读取字符, 并把它们转换成字符串.可以使用std::getline()拆分字符串

getline(input, str, ',')

参数
```
input - 流中获取数据
str - 把数据转换成字符串
delim - 分隔符
```
示例：
拆分字符串代码：
```c++
std::string ss("whis is a test")
std::istringstream ss(line);
std::string column;
while (std::getline(ss, column, ' '))
  {
    //todo
  }
```
需要注意的是，流文件读取数据的时候，类似于输入流，输入的东西都是按顺序进行的，输入进来第一行，再次调用getline()输入进来的就是后面的一行了

### 21、string.c_str()
将C++里面的string类型转化为C里面的char *类型

### 22、C++字符串处理：文件路径提取文件名称
```c++
std::string filepath("/home/sunmiao/test.csv");
std::string suffix("_replanned");
std::string tmp = file_path;
const std::string::size_type idx_slash = tmp.find_last_of("/");
if (idx_slash != std::string::npos)
{
    tmp.erase(0, idx_slash);
}
const std::string::size_type idx_dot = tmp.find_last_of(".");
const std::string::size_type idx_dot_allpath = file_path.find_last_of(".");
if (idx_dot != std::string::npos && idx_dot != tmp.size() - 1)
{
    file_path.erase(idx_dot_allpath, file_path.size() - 1);
}
new_file_path += suffix + ".csv";

```
find_last_of()代表从后向前查找

所有的string.find查找函数都返回一个size_type类型，这个返回值一般都是所找到字符串的位置，如果没有找到，则返回string::npos.

### 23、vector中insert()的用法详解
```
iterator insert( iterator loc, const TYPE &val );
void insert( iterator loc, size_type num, const TYPE &val );
void insert( iterator loc, input_iterator start, input_iterator end );
```
insert() 函数有以下三种用法:

> 在指定位置loc前插入值为val的元素,返回指向这个元素的迭代器
>
> 在指定位置loc前插入num个值为val的元素
>
> 在指定位置loc前插入区间[start, end)的所有元素 .

### 24、OpenCV-截取图片中的一部分

一般常见的方法是：
```
Mat image= imread(“图片路径”)；
Rect rect(10, 20, 100, 50);
Mat image_roi = image(rect);
```
rect的4个参数为起点x坐标，y坐标，x距离，y距离

### 25、OpenCV compare()函数介绍
OpenCV中定义在core.hpp中的compare()函数原型如下：
```
void compare(InputArray src1, InputArray src2, OutputArray dst, int cmpop);
```
函数作用：

按照指定的操作cmpop，比较输入的src1和src2中的元素，输出结果到dst中

参数解释：
> src1：原始图像1（必须是单通道）或者一个数值，比如是一个Mat或者一个单纯的数字n；
>
> src2：原始图像2（必须是单通道）或者一个数值，比如是一个Mat或者一个单纯的数字n；
>
> dst：结果图像，类型是CV_8UC1，即单通道8位图，大小和src1和src2中最大的那个一样，比较结果为真的地方值为         255，否则为0；
>
> cmpop：操作类型，有以下几种类型：
>
>       enum { CMP_EQ=0,    //相等
>
> 	           CMP_GT=1,   //大于
>
> 	           CMP_GE=2,   //大于等于
>
> 	           CMP_LT=3,   //小于
>
> 	           CMP_LE=4,   //小于等于
>
> 	           CMP_NE=5 }; //不相等


函数原理：

从参数的要求可以看出，compare函数只对以下三种情况进行比较：

1. array和array

    此时输入的src1和src2必须是相同大小的单通道图，否则没办法进行比较了。计算过程就是：

    dst(i) = src1(i) cmpop src2(i)

    也就是对src1和src2逐像素进行比较。

2. array和scalar

    此时array仍然要求是单通道图，大小无所谓，因为scalar只是一个单纯的数字而已。比较过程是把array中的每个元素逐个和scalar进行比较，所以此时的dst大小和array是一样 的。计算过程是：

    dst(i) = src1(i) cmpop scalar

3. scalar和array

    这个就是2的反过程了，只是比较运算符cmpop左右的参数顺序不一样了而已。计算过程如下：

    dst(i) = scalar cmpop src2(i)

用途举例：

这个函数有一个很有用的地方就是：当你需要从一幅图像中找出那些特定像素值的像素时，可以用这个函数。类似与threshold()函数，但是threshold()函数是对某个区间内的像素值进行操作，compare()函数则可以只是对某一个单独的像素值进行操作。比如我们要从图像中找出像素值为50的像素点，可以下面这样做：
```
cv::Mat result;
cv::compare(image,50, result, cv::CMP_EQ);
```

### 26、通过 iterable 对象来迭代

for i in range(1000): pass
会导致生成一个 1000 个元素的 List，而代码：

for i in xrange(1000): pass
则不会生成一个 1000 个元素的 List，而是在每次迭代中返回下一个数值，内存空间占用很小。因为 xrange 不返回 List，而是返回一个 iterable 对象。

### 27、使用yield的函数

```python
#!/usr/bin/python
# -*- coding: UTF-8 -*-

def fab(max):
    n, a, b = 0, 0, 1
    while n < max:
        yield b      # 使用 yield
        # print b
        a, b = b, a + b
        n = n + 1

for n in fab(5):
    print n
```
yield的操作，使得函数获得了iterable的效果。

执行函数的输出结果
```
1
1
2
3
5
```

简单地讲，yield 的作用就是把一个函数变成一个 generator，带有 yield 的函数不再是一个普通函数，Python 解释器会将其视为一个 generator，调用 fab(5) 不会执行 fab 函数，而是返回一个 iterable 对象！

在 for 循环执行时，每次循环都会执行 fab 函数内部的代码，执行到 yield b 时，fab 函数就返回一个迭代值，下次迭代时，代码从 yield b 的下一条语句继续执行，而函数的本地变量看起来和上次中断执行前是完全一样的，于是函数继续执行，直到再次遇到 yield。

也可以手动调用 fab(5) 的 next() 方法，因为 fab(5) 是一个 generator 对象，该对象具有 next() 方法，这样我们就可以更清楚地看到 fab 的执行流程：

示例：
```
>>>f = fab(5)
>>> f.next()
1
>>> f.next()
1
>>> f.next()
2
>>> f.next()
3
>>> f.next()
5
>>> f.next()
Traceback (most recent call last):
 File "<stdin>", line 1, in <module>
StopIteration
```
当函数执行结束时，generator 自动抛出 StopIteration 异常，表示迭代完成。在 for 循环里，无需处理 StopIteration 异常，循环会正常结束。

我们可以得出以下结论：

一个带有 yield 的函数就是一个 generator，它和普通函数不同，生成一个 generator 看起来像函数调用，但不会执行任何函数代码，直到对其调用 next()（在 for 循环中会自动调用 next()）才开始执行。虽然执行流程仍按函数的流程执行，但每执行到一个 yield 语句就会中断，并返回一个迭代值，下次执行时从 yield 的下一个语句继续执行。看起来就好像一个函数在正常执行的过程中被 yield 中断了数次，每次中断都会通过 yield 返回当前的迭代值。

yield 的好处是显而易见的，把一个函数改写为一个 generator 就获得了迭代能力，比起用类的实例保存状态来计算下一个 next() 的值，不仅代码简洁，而且执行流程异常清晰。

### 28、opencv C++图像通道拆分与拼接
```
cv::Mat SrcMat = cv::imread("1.jpg",1);                 // 1.jpg为三通道图像
std::vector<cv::Mat>SrcMatpart(SrcMat.channels());      //生成与通道数数目相等的图像容器
cv::split(SrcMat,SrcMatpart);                           //分解与通道数数目相等的图像容器
cv::Mat MergeMat;
cv::merge(SrcMatpart,MergeMat);                         //合成与通道数数目相等的图像容器
```

SrcMat被分为SrcMatpart[0]、 SrcMatpart[1]、SrcMatpart[2]三部分。

SrcMatpart[0]存储B颜色分量,SrcMatpart[1]存储G颜色分量,SrcMatpart[2] 存储R颜色分量。

MergeMat为合成后BGR分量后的图片。

### 29、vector中insert()的用法详解
```c++
iterator insert( iterator loc, const TYPE &val );
void insert( iterator loc, size_type num, const TYPE &val );
void insert( iterator loc, input_iterator start, input_iterator end );
```
insert() 函数有以下三种用法:
- 在指定位置loc前插入值为val的元素,返回指向这个元素的迭代器,
- 在指定位置loc前插入num个值为val的元素
- 在指定位置loc前插入区间[start, end)的所有元素 .

举例:
```c++
//创建一个vector,置入字母表的前十个字符
vector <char> alphaVector;
for( int i=0; i < 10; i++ )
  alphaVector.push_back( i + 65 );

//插入四个C到vector中
vector <char>::iterator theIterator = alphaVector.begin();
alphaVector.insert( theIterator, 4, 'C' );

//显示vector的内容
for( theIterator = alphaVector.begin(); theIterator != alphaVector.end(); theIterator++ )
  cout < < *theIterator;
```
输出结果为：
CCCCABCDEFGHIJ

### 30、c++ 初始化列表
c++ 初始化列表要按照先后声明的顺序来进行初始化，否则会报下面这个warning
```
warning: 'CAsyncSQL::m_iCopiedQuery' will be initialized after [-Wreorder]
```

### 31、ROS remap使用
remap的作用就是将原来的一个话题的消息内容，“复制”到另一个话题里面，可以修改订阅的话题，也可以修改发布的话题，例如
```xml
<node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
</node>
```
这种remap的写法是写在launch里面，是将当前节点订阅的话题input的映射到话题turtlesim1/turtle1

from的就是原来话题的内容，to的就是新的话题内容，里面的消息是和from对应的话题是一样的。

也可以写在命令行里面，但是比较麻烦，如下所示：
```
rosbag play ros.bag  /image_raw:=/camera/image_raw
```
这个意思就是将原来的话题消息/image_raw,映射到了/camera/imgae_raw，两个话题里面的消息都是一致的

### 32、ROS设置判断条件是否启动节点
```xml
 <arg name="compressed_stream" default="false" />

<node if="$(arg compressed_stream)" pkg="image_transport" name="decompress" type="republish"  output="screen" args="compressed in:=/$(arg camera_id)/$(arg image_src) raw out:=/$(arg camera_id)/$(arg image_src)" />
```
分析上面的代码，就是说如果compressed_stream变量为true，那么就启动这个节点，如果变量为false，那么就不启动这个节点。

后面的args=参数的含义是进行了一个话题的remap操作。

### 33. vscode markdown 预览
快捷命令 ctrl + shift + v

### 34、旋转矩阵推导
https://blog.csdn.net/TOM_00001/article/details/62054572

### 35、roslaunch 启动 rviz
```
<node pkg="rviz" type="rviz" name="Rviz_for_ndt_localization" args="-d $(find visualization)/rviz/visualization.rviz" />
```


### 36、ubuntu主题美化
https://blog.csdn.net/White_Idiot/article/details/78973575

### 37、设置某一数据类型的最大值
例如：设置float数据类型的最大值
```
std::numeric_limits<float>::max()
```

### 38、ubuntu查看历史命令
Ctrl+R 快捷键。此快捷键让你对命令历史进行搜索，对于想要重复执行某个命令的时候非常有用。当找到命令后，通常再按回车键就可以执行该命令。如果想对找到的命令进行调整后再执行，则可以按一下左或右方向键

### 39、ubuntu 终端常用快捷方式
快捷键|说明
---|---
Ctrl+Alt+T| 启动终端
F11	| 全屏切换
Ctrl+Shift+C|	 复制
Ctrl+Shift+V|	 粘贴
Ctrl+Shift+T|	 新建标签页
Ctrl+Shift+W|	 关闭标签页
Ctrl+R|	 反向搜索历史命令
Ctrl+L|清除当前屏幕内容（同 clear命令功能)

### 40、配置TX2同时连接激光雷达有线网和wifi无线网

链接激光雷达要配置有限网静态ip地址

打开/etc/network/interfaces，修改部分如下所示
```
auto eth0
iface eth0 inet static
    address 192.168.1.77
    netmask 255.255.255.0
    gateway 192.168.1.1
```
然后，重启网络
```
/etc/init.d/networking restart
```
这时候，打开浏览器输入192.168.1.201可以看到激光雷达的配置文件，说明此时可以连接到了激光雷达有线网。

但是，会发现此时无法使用wifi连接公网，这时候我们还需要配置一下默认网关。

查看网关信息：
```
ip route show
```
显示如下：
```
default via 192.168.1.1 dev eth0 onlink
default via 10.0.0.1 dev wlan0  proto static  metric 600
10.0.0.0/24 dev wlan0  proto kernel  scope link  src 10.0.0.14  metric 600
169.254.0.0/16 dev eth0  scope link  metric 1000
172.17.0.0/16 dev docker0  proto kernel  scope link  src 172.17.0.1 linkdown
192.168.1.0/24 dev eth0  proto kernel  scope link  src 192.168.1.77
192.168.55.0/24 dev l4tbr0  proto kernel  scope link  src 192.168.55.1 linkdown
```
这里面最上面的默认网关192.168.1.1是有线网的网关，下面的默认网关10.0.0.1是无线网的默认网关，我们删除有线网的默认网关，只保留无线网的网关，这样系统如果找不到可用的网关，就把数据包发给默认指定的无线网关，由这个网关来处理数据包。
```
sudo route del default gw 192.168.1.1
```
再查看网关信息显示如下：
```
default via 10.0.0.1 dev wlan0  proto static  metric 600
10.0.0.0/24 dev wlan0  proto kernel  scope link  src 10.0.0.14  metric 600
169.254.0.0/16 dev eth0  scope link  metric 1000
172.17.0.0/16 dev docker0  proto kernel  scope link  src 172.17.0.1 linkdown
192.168.1.0/24 dev eth0  proto kernel  scope link  src 192.168.1.77
192.168.55.0/24 dev l4tbr0  proto kernel  scope link  src 192.168.55.1 linkdown
```
这样我们在打开浏览器，就可以正常访问公网和激光雷达网络了。

但是要注意的是，这个方法在重启之后就会失效了，需要每次开机重新来一次，而且如果执行`sudo /etc/init.d/networking restart`也会失效。

### 41、递进新生成新文件夹
```
mkdir -p catkin_velodyne/src
```
### 42、C++纯虚函数
纯虚函数是在基类中声明的虚函数，它在基类中没有定义，但要求任何派生类都要定义自己的实现方法。

在基类中实现纯虚函数的方法是在函数原型后加“=0”
```
virtual void funtion1()=0
```

纯虚函数是在基类中声明的虚函数，它要求任何派生类都要定义自己的实现方法，以实现多态性。实现了纯虚函数的子类，该纯虚函数在子类中就变成了虚函数。

定义纯虚函数是为了实现一个接口，用来规范派生类的行为，也即规范继承这个类的程序员必须实现这个函数。派生类仅仅只是继承函数的接口。纯虚函数的意义在于，让所有的类对象（主要是派生类对象）都可以执行纯虚函数的动作，但基类无法为纯虚函数提供一个合理的缺省实现。所以类纯虚函数的声明就是在告诉子类的设计者，“你必须提供一个纯虚函数的实现，但我不知道你会怎样实现它”。

**把相应的虚函数, 末尾添加"=0",该虚函数就变为纯虚函数, 可以不用添加定义;**

**如果是其他虚函数, 即使不使用, 也必须定义(define);**

### 43、 c++构造函数
我们知道有时候当我们仅创建了有参构造函数后，如果你想调用无参构造函数编译是会报错的。

因为一旦你自己定义了构造函数，系统的默认构造函数是被屏蔽的，也就是说此时是没有无参构造函数的，所以我们需要自己定义一个无参构造函数。

但是现在在C++11中，如果我们仅定义了有参构造函数，可以通过default关键字让默认构造函数恢复

```c++
class CString
{
    char* _str = nullptr;
public:
    CString() = default;  //恢复默认构造函数
    CString(const char* pstr) : _str(nullptr)  //自定义的有参构造
    {
        UpdateString(pstr);
    }
}
```

### 44、c++11 : default关键字的作用

为了解决两个问题：1. 减轻程序员的编程工作量；2. 获得编译器自动生成的默认特殊成员函数的高的代码执行效率，C++11 标准引入了一个新特性：defaulted 函数。

程序员只需在函数声明后加上“=default;”，就可将该函数声明为 defaulted 函数，编译器将为显式声明的 defaulted 函数自动生成函数体。例如：

```c++
class X{
public:
  X()= default;
  X(int i){
    a = i;
  }
private:
  int a;
};

X x;
```
如上所示，编译器会自动生成默认构造函数 X::X(){}，该函数可以比用户自己定义的默认构造函数获得更高的代码效率。

https://www.ibm.com/developerworks/cn/aix/library/1212_lufang_c11new/index.html

### 45、ros package.xml几个标签的区别
package.xml
Your package dependencies are declared in package.xml. If they are missing or incorrect, you may be able to build from source and run tests in your own workspace, but your package will not work correctly when released to the ROS community. Others rely on this information to install the software they need for using your package.

------------------

**`<depend>`**

It is generally sufficient to mention each ROS package dependency once, like this:

**`<depend>roscpp</depend>`**

Sometimes, you may need or want more granularity for certain dependencies. The following sections explain how to do that. If in doubt, use the <depend> tag, it’s simpler.

------------------

**`<build_depend>`**

If you only use some particular dependency for building your package, and not at execution time, you can use the <build_depend> tag. For example, the ROS angles package only provides C++ headers and CMake configuration files:

**`<build_depend>angles</build_depend>`**

With this type of dependency, an installed binary of your package does not require the angles package to be installed.

But, that could create a problem if your package exports a header that includes the <angles/angles.h> header. In that case you also need a <build_export_depend>.

----------------------

**`<build_export_depend>`**

If you export a header that includes <angles/angles.h>, it will be needed by other packages that <build_depend> on yours:

**`<build_export_depend>angles</build_export_depend>`**

This mainly applies to headers and CMake configuration files. Library packages referenced by libraries you export should normally specify <depend>, because they are also needed at execution time.

--------------------------

**`<exec_depend>`**

This tag declares dependencies for shared libraries, executables, Python modules, launch scripts and other files required when running your package. For example, the ROS openni_launch package provides launch scripts, which are only needed at execution time:

**`<exec_depend>openni_launch</exec_depend>`**

### 46、c++ volatile 用法解析

一个定义为volatile的变量是说这变量可能会被意想不到地改变，这样，编译器就不会去假设这个变量的值了。精确地说就是，优化器在用到这个变量时必须每次都小心地重新读取这个变量的值，而不是使用保存在寄存器里的备份。

volatile 影响编译器编译的结果，volatile 变量是随时可能发生变化的，与volatile变量有关的运算，不要进行编译优化，以免出错（VC++ 在产生release版可执行码时会进行编译优化，加volatile关键字的变量有关的运算，将不进行编译优化）
例如：
```
volatile int i=10;
int j = i;
...
int k = i;
```
volatile 告诉编译器i是随时可能发生变化的，每次使用它的时候必须从i的地址中读取，因而编译器生成的可执行码会重新从i的地址读取数据放在k中。

而优化做法是，由于编译器发现两次从i读数据的代码之间的代码没有对i进行过操作，它会自动把上次读的数据放在k中。而不是重新从i里面读。这样以来，如果i是一个寄存器变量或者表示一个端口数据就容易出错，所以说volatile可以保证对特殊地址的稳定访问，不会出错。


下面是需要使用volatile变量的几个例子：

- 1) 并行设备的硬件寄存器（如：状态寄存器）
- 2) 一个中断服务子程序中会访问到的非自动变量(Non-automatic variables)
- 3) 多线程应用中被几个任务共享的变量

回答不出这个问题的人是不会被雇佣的。我认为这是区分C程序员和嵌入式系统程序员的最基本的问题。搞嵌入式的家伙们经常同硬件、中断、RTOS等等打交道， 所有这些都要求用到volatile变量。不懂得volatile的内容将会带来灾难。假设被面试者正确地回答了这是问题（嗯，怀疑是否会是这样），我将稍微深究一下，看一下这家伙是不是直正懂得volatile完全的重要性。

1)一个参数既可以是const还可以是volatile吗？解释为什么。
2); 一个指针可以是volatile 吗？解释为什么。
3); 下面的函数有什么错误：
```
int square(volatile int *ptr)
{
return *ptr * *ptr;
}
```
下面是答案：
1)是的。一个例子是只读的状态寄存器。它是volatile因为它可能被意想不到地改变。它是const因为程序不应该试图去修改它。
2); 是的。尽管这并不很常见。一个例子是当一个中服务子程序修该一个指向一个buffer的指针时。
3) 这段代码有点变态。这段代码的目的是用来返指针*ptr指向值的平方，但是，由于*ptr指向一个volatile型参数，编译器将产生类似下面的代码：
```
int square(volatile int *ptr)
{
int a,b;
a = *ptr;
b = *ptr;
return a * b;
}
由于*ptr的值可能被意想不到地该变，因此a和b可能是不同的。结果，这段代码可能返不是你所期望的平方值！正确的代码如下：
long square(volatile int *ptr)
{
int a;
a = *ptr;
return a * a;
}
```
### 47、有关于函数各参数的求值顺序
```c++
#include <stdio.h>

int main(void)
{
    int a = 5;

    int *p = &a;

    int b = (*p)++; //等价于b = a++; 即b = a; a = a + 1;

    int c = ++(*p); //等价于c = ++a; 即a = a + 1; c = a;

    printf("b = %d, c = %d\n", b, c);

    printf("(*p)++ = %d, ++(*p) = %d\n", (*p)++, ++(*p));

    return 0;
}
```
例子输出结果： 
```
b = 5, c = 7
(*p)++ = 8, ++(*p) = 8
```
在这个例子中，通过\*p来间接地操作a，第9行的\*p一定要用小括号括起来，否则含义就不一样了。而第11行的++（*p）也可以写成++*p（用GCC验证过），那是因为对操作数p来说它只有一个运算符*在计算它，所以无关乎运算符优先级和结合性的问题。

**值得注意的是，由于C语言没有指定函数各参数的求值顺序，所以第15行的代码是不可移植的，用不同的编译器可能会产生不同的结果（对于这个例子，GCC是先计算++（\*p），后计算（\*p）++，所以两者都等于8）**

### 48、GCC编译器 CFLAGS编译参数说明
例如：
```
make CFLAGS="-g -O0" -j 4

```

> * 注意,如果是纯C项目那么使用CFLAGS，如果是C++项目，那么使用CXXFLAGS
> * 这里使用了 -j 选项，其值是 4，表示开启 4 个进程同时编译，加快编译速度
>
编译参数|说明
---|---
-S|只是编译不汇编，生成汇编代码
-E|只进行预编译，不做其他处理
-g|在可执行程序中包含标准调试信息
-o file|把输出文件输出到file里
-v|打印出编译器内部编译各过程的命令行信息和编译器的版本
-I dir|在头文件的搜索路径列表中添加dir目录
-L dir|在库文件的搜索路径列表中添加dir目录
-static|链接静态库
-llibrary|连接名为library的库文件

报警信息|说明
---|---
-ansi	|支持符合ANSI标准的C程序
-pedantic|	允许发出ANSI C标准所列的全部警告信息
-pedantic-error	|允许发出ANSI C标准所列的全部错误信息
-w	|关闭所有告警
-Wall	|允许发出Gcc提供的所有有用的报警信息
-werror	|把所有的告警信息转化为错误信息，并在告警发生时终止编译过程

优化选项|说明
---|---
-O0  |不优化
-O1～-O3  |等级更高的优化
虽然优化选项可以加速代码的运行速度，但对于调试而言将是一个很大的挑战。因为代码在经过优化之后，原先在源程序中声明和使用的变量很可能不再使用，控制流也可能会突然跳转到意外的地方，循环语句也有可能因为循环展开而变得到处都有，所有这些对调试来讲都将是一场噩梦。所以笔者建议在调试的时候最好不使用任何优化选项，只有当程序在最终发行的时候才考虑对其进行优化

### 49、内存里面的堆栈增长方向
在内存管理中，与栈对应是堆。对于堆来讲，生长方向是向上的，也就是向着内存地址增加的方向；对于栈来讲，它的生长方式是向下的，是向着内存地址减小的方向增长。在内存中，“堆”和“栈”共用全部的自由空间，只不过各自的起始地址和增长方向不同，它们之间并没有一个固定的界限，如果在运行时，“堆”和 “栈”增长到发生了相互覆盖时，称为“栈堆冲突”，系统肯定垮台。

在常见的x86中内存中栈的增长方向就是从高地址向低地址增长。

### 50、程序调试的单步调试
next 单步步过（step over），即遇到函数调用直接跳过，不进入函数体内部。
step 单步步入（step into），即遇到函数调用，进入函数内部。
