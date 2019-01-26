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

### 13. !!
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
Ctrl + Shift + i |  格式化代码

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
