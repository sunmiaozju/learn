1、np.unique()
 获取非相同元素

2、一些os操作命令：持续更新：
os.listdir()    用于返回指定的文件夹包含的文件或文件夹的名字的列表。这个列表以字母顺序。 它不包括 '.' 和'..' 即使它在文件夹中。
os.path.abspath(__file__) 列出当前执行文件的绝对路径。例如：/home/sunmiao/projects/caffe/pointnet/train.py
__file__ 是自己文件的文件名。例如：train.py
os.path.dirname(os.path.abspath(__file__) ) 返回当前执行文件所在的文件夹，即文件绝对路径去掉自己本身的那个路径。例如：/home/sunmiao/projects/caffe/pointnet
os.path.join() 用于路径连接，但是路径间不需要田间‘/’，它会自动添加
os,path.exists() 判断路径是否存在
os.mkdir() 创建一个新的路径
os.system(''cmd'')  类似于在shell执行系统命令cmd

3、函数： tf.Graph.as_default()
返回值：返回一个上下文管理器，这个上下文管理器使用这个图作为默认的图
说明：如果你想在一个进程里创建多个图，可能会用到这个方法。
   为了方便，如果你没有显式创建一个图的话，系统提供了一个全局默认的图，默认把所有的操作都添加到全局默认图中。使用这个方法的话，（配合with关键词使用），可以只把with块里的操作添加到默认图中。默认图是当前线程的一个属性，如果你创建了一个新的线程，想使用全局默认图，必须显式调用这个方法。

4、
ohup command > out.file 2>&1 &  让command命令在后台执行，同时输出标准输出以及错误输出（由2>&1代表）到
out.file文件中。 nohup表示no hang up,即程序一直执行不挂起，&表示在后台执行

5、
str.split(str=‘’，num=) 通过指定分隔符对字符串进行切片，返回分片后的列表。如果参数 num 有指定值，则仅分隔 num 个子字符串

6、
sys.argv  是获取运行python文件的时候命令行参数,整个命令行python后面的字符都被返回为一个列表，可以通过sys.argv[i]来获取相应的具体命令参数

7、
f = open('colormap2.txt', 'r')
for line in f.readlines():
items = line.strip().split(' ')
获取文件内容，按每行读取，并且去除两边的空格，并以空格分割字段，返回列表

8、
np.where()用法： https://www.zhihu.com/question/62844162/answer/300561552


9、
import random
random.uniform(x, y) 将随机生成下一个实数，它在 [x, y) 范围内。
x -- 随机数的最小值，包含该值。
y -- 随机数的最大值，不包含该值。

10、
图像的透射
透视需要的是一个3*3的矩阵，同理opencv在构造这个矩阵的时候还是采用一种点对应的关系来通过函数自己寻找的，因为我们自己很难计算出来。这个函数是M = cv2.getPerspectiveTransform(pts1,pts2)，其中pts需要变换前后的4个点对应位置。得到M后在通过函数cv2.warpPerspective(img,M,(200,200))进行。
cv2.warpPerspective(mask, trans2, (output_w2, output_h2), cv2.INTER_NEAREST)
主要作用：对图像进行透视变换
参数详解：
InputArray src：输入的图像
OutputArray dst：输出的图像
InputArray M：透视变换的矩阵
Size dsize：输出图像的大小
int flags=INTER_LINEAR：输出图像的插值方法

11、
python 直接赋值与浅copy、深copy的区别

直接赋值：其实就是对象的引用（别名）。
浅拷贝(copy)：拷贝父对象，不会拷贝对象的内部的子对象。即如果被拷贝的前者里面存在对象的话，浅拷贝后里面的对象是一个东西，就是引用过来的。
深拷贝(deepcopy)： copy 模块的 deepcopy 方法，完全拷贝了父对象及其子对象，即前后二者完全隔离开，但是需要引入import copy模块，a = copy.deepcopy(b)

12、
python set集合添加元素操作
set.add() 将被添加元素作为一个整体添加到集合中
set.update()将被添加元素的内容拆分，分别作为元素添加到集合中

13、
numpy.argsort
函数返回的是数组值从小到大的索引值
一维数组
    >>> x = np.array([3, 1, 2])
    >>> np.argsort(x)
    array([1, 2, 0])
二维数组
    >>> x = np.array([[0, 3], [2, 2]])
    >>> np.argsort(x, axis=0)
    array([[0, 1],
           [1, 0]])
    >>> np.argsort(x, axis=1)
    array([[0, 1],
           [0, 1]])
从小到大排序：
numpy.argsort(x)
c从大到小排序：
numpy.argsort(-x)

14、
str.lower()方法转换字符串中所有大写字符为小写。

15、
编译c++源文件：
./config
mkdir build && cd build && cmake .. && make

16、
图片叠加：
cv2.addWeighted(src1, alpha, src2, beta, gamma[, dst[, dtype]])
其中，alpha 为 src1 透明度，beta 为 src2 透明度. 权重越大，透明度越低
overlapping = cv2.addWeighted(bottom, 0.8, top, 0.2, 0)

17、
函数中的命令行参数设置：
import argparse
def init_args():
parser = argparse.ArgumentParser()
parser.add_argument('--dataset_dir', type=str, default='.', help='The training dataset dir path')
parser.add_argument('--net', type=str, default='.', help='Which base net work to use', default='vgg')
parser.add_argument('--weights_path', type=str, default='.', help='The pretrained weights path')
return parser.parse_args()
调用：
args = init_args()
train_net(args.dataset_dir, args.weights_path, net_flag=args.net)

打印或者记录log信息：
记录下来的argoaser信息可以直接统一打印出来：
print str(args)
输出：Namespace(batch_size=32, decay_rate=0.7, decay_step=200000, gpu=0, learning_rate=0.001, log_dir='log', max_epoch=250, model='pointnet_cls', momentum=0.9, num_point=1024, optimizer='adam')

argarse.ArgumentParser.parse_known_args()
https://blog.csdn.net/m0_37041325/article/details/77934623
这个api可以帮助当传入较多的参数时，有些参数没有定义，那么可以接受进来给其他程序使用。
FLAGS, unparsed = parser.parse_known_args()

18、
断言
assert 表达式 , 描述
表达式为我们的预期结果，当表达式的结果为False时，抛出 AssertionError 异常，如无异常捕获则结束程序运行。
如果表达式结果为True， 程序继续向下运行。

19、
python面向对象基本概念
http://www.runoob.com/python/python-object.html

20、
numpy.squeeze()函数
语法：numpy.squeeze(a,axis = None)
1）a表示输入的数组；
2）axis用于指定需要删除的维度，但是指定的维度必须为单维度，否则将会报错；
3）axis的取值可为None 或 int 或 tuple of ints, 可选。若axis为空，则删除所有单维度的条目；
4）返回值：数组
5) 不会修改原数组；
作用：从数组的形状中删除单维度条目，即把shape中为1的维度去掉

21、
tf.one_hot()独热编码
https://blog.csdn.net/LoseInVain/article/details/78819390

22、
tf.reduce_mean()
reduce_mean(input_tensor,axis=None,keep_dims=False,name=None, reduction_indices=None)
函数作用： 沿着tensor的某一维度，计算元素的平均值。由于输出tensor的维度比原tensor的低，这类操作也叫降维。
参数：
input_tensor：需要降维的tensor。
axis：axis=none, 求全部元素的平均值；axis=0, 按列降维，求每列平均值；axis=1，按行降维，求每行平均值。
keep_dims：若值为True，可多行输出平均值。
name：自定义操作的名称。
reduction_indices：axis的旧名，已停用。

23、
tf.TensorArray()
https://blog.csdn.net/z2539329562/article/details/80639199

24、
tf.while_loop()
可以这样理解
loop = []
while cond(loop):
loop = body(loop)
即loop参数先传入cond 判断条件是否成立，成立之后，把 loop参数传入body 执行操作， 然后返回 操作后的 loop 参数，即loop参数已被更新，再把更新后的参数传入cond, 依次循环，直到不满足条件。
https://blog.csdn.net/u011509971/article/details/78805727

25、
tf.unique_with_counts
https://www.w3cschool.cn/tensorflow_python/tensorflow_python-cdi62o34.html

26、
tf.clip_by_value(A, min, max)
输入一个张量A，把A中的每一个元素的值都压缩在min和max之间。小于min的让它等于min，大于max的元素的值等于max。

27、
tf.train.exponential_decay
https://blog.csdn.net/wuguangbin1230/article/details/77658229

28、
np.all() np.any() ==
数组元素的比对，我们可以直接使用“==”进行比较，
但是当数组元素较多时，查看输出结果便变得很麻烦，这时我们可以使用all（）方法，直接比对a矩阵和b矩阵的所有对
应的元素是否相等。
而any（）方法是查看两矩阵是否有一个对应元素相等。事实上，all（）操作就是对两个矩阵的比对结果再做一次与运算，
而any则是做一次或运算
https://blog.csdn.net/qq_28618765/article/details/78086478

29、
使用下面这种格式去压缩一个目录：
tar -zcvf archive_name.tar.gz directory_to_compress
解压缩tar.gz文件：
tar -zxvf archive_name.tar.gz
解压tar.bz2文件

tar - jxvf  ×××.tar.bz2


30、
绘制车道线并且提取不同的车道线轮廓
img_mask是二值化的车道线图，但是为3通道

imgray = cv2.cvtColor(img_mask, cv2.COLOR_BGR2GRAY)
这样就变成了一通道

ret, thresh = cv2.threshold(imgray, 250, 255, 0)
确保像素值250以上都归为255

im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
计算轮廓线，返回的是一系列轮廓点坐标的列表

img_contours = np.zeros(img.shape, dtype=np.uint8)
生成图片底板

cv2.drawContours(img_contours, contours, 0, White_color, -1, 1)
绘制轮廓线，第三个参数代表绘制轮廓线的第几条，如果为负数代表全部都绘制，第四个参数代表绘制的颜色,第五个-1代
表填充绘制
https://blog.csdn.net/sunny2038/article/details/12889059

31、python添加中文编码
```
# -*- coding: utf-8 -*-
```
32、
https://blog.csdn.net/tengfei461807914/article/details/76626631
opencv轮廓检测合集

33、
jpg格式的文件是有损的，png格式的图片是无损的，也就是说jpg格式图片的像素值是不确定的，不能用它来当作图像处理
的mask。

34、
Python ceil() 函数
描述：ceil() 函数返回数字的上入整数。
语法
import math
math.ceil( x )
注意：ceil()是不能直接访问的，需要导入 math 模块，通过静态对象调用该方法。

35、
Python enumerate() 函数
描述：enumerate() 函数用于将一个可遍历的数据对象(如列表、元组或字符串)组合为一个索引序列，同时列出数据和
数据下标，一般用在 for 循环当中

36、
bgr图变成灰度图
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

37、
https://www.jianshu.com/p/dcecaf62da71
python opencv形态学处理：
获取结构参数、腐蚀远算、膨胀运算、开运算、闭运算，黑帽运算、顶帽运算。

38、
求取图像的连通域函数
nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray_image,
                                                                    connectivity=8,
                                                                    ltype=cv2.CV_32S)
输入的参数是二值化的灰度图，connectivity可以选择4 or 8,
输出四个参数：
第一个是连通区域的总个数
第二个是输出图，和输入图维度一样，但是每一个像素均被标注为了各个连通域的label,(0,1,2....其中，背景是0）
第三个是连通域的统计信息，维度[nlabels,5]，每个连通域五个统计信息
具体信息如下：
Statistics output for each label, including the background label, see below for available statistics. Statistics are accessed via stats[label, COLUMN] where available columns are defined below.
cv2.CC_STAT_LEFT The leftmost (x) coordinate which is the inclusive start of the bounding box in the horizontal direction.
cv2.CC_STAT_TOP The topmost (y) coordinate which is the inclusive start of the bounding box in the vertical direction.
cv2.CC_STAT_WIDTH The horizontal size of the bounding box
cv2.CC_STAT_HEIGHT The vertical size of the bounding box
cv2.CC_STAT_AREA The total area (in pixels) of the connected component
第四个是每个连通域的质心中点的坐标，维度为[nlabels,2]

39、
numpy.flip()
根据axis可以交换元素
>>> A = np.arange(8).reshape((2,2,2))
>>> A
array([[[0, 1],
[2, 3]],
      [[4, 5],
       [6, 7]]])
>>> flip(A, 0)
array([[[4, 5],
[6, 7]],
      [[0, 1],
       [2, 3]]])
>>> flip(A, 1)
array([[[2, 3],
[0, 1]],
      [[6, 7],
       [4, 5]]])

40、
python list的增删改查：
增：list.append(x)   list.insert(index, x)
删：list.pop()去除最后一个元素
      list.pop(index) 删除同时返回相应的元素
      del list[index]
      list.remove(x) 去除第一个出现的x
改：list[index] = x
      list.sort() 从小到大排序
      list.reverse()  元素顺序反转
查：list.count(x)统计x的出现个数
      list.index(x) x第一次出现的坐标

41、
pkg_config的用法
https://blog.csdn.net/suochao90/article/details/7291587

42、
读写json文件
https://zhuanlan.zhihu.com/p/27917664

43、
生成列表的快速写法：
>>> sqlist=[x*x for x in range(1,11) if x%2 != 0]
>>> sqlist
[1, 9, 25, 49, 81]

可迭代对象都可以这么使用

44、
python异常处理：
自己生成异常，但是会终止程序的运行
if anumber < 0:
raise RuntimeError("You can't use a negative number")
else:
print(math.sqrt(anumber))

捕获异常，程序从except继续运行
>>> try:
print(math.sqrt(anumber))
except:
print("Bad Value for square root")
print("Using absolute value instead")
print(math.sqrt(abs(anumber)))

45、
python子类继承父类，一般要在构造函数里面显式声明父类构造函数：
有两种方法：
LogicGate.__init__(self)
super(UnaryGate,self).__init__()
其中，
LogicGate是父类，
UnaryGate是子类

46、
linux cat命令
cat 命令用于连接文件并打印到标准输出设备上。
把 textfile1 的文档内容加上行号后输入 textfile2 这个文档里：
     cat -n textfile1 > textfile2
把 textfile1 和 textfile2 的文档内容加上行号（空白行不加）之后将内容附加到 textfile3 文档里：
     cat -b textfile1 textfile2 >> textfile3
把前几个文件连接起来，汇总成为train.txt
     cat 2007_train.txt 2007_val.txt 2012_*.txt > train.txt

47、
程序工作目录更改
切换到某一个目录
os.chdir("/var/www/html" )
获取进程当前目录
os.getcwd()

48、
python的文本读写问题
读：
Python 将文本文件的内容读入可以操作的字符串变量非常容易。文件对象提供了三个“读”方法： .read()、.readline() 和 .readlines()。
.read() 每次读取整个文件，它通常用于将文件内容放到一个字符串变量中。然而 .read() 生成文件内容最直接的字符串表示，
但对于连续的面向行的处理，它却是不必要的，并且如果文件大于可用内存，则不可能实现这种处理。
.readline() 和 .readlines() 非常相似。
.readline() 和 .readlines()之间的差异是后者一次读取整个文件，象 .read()一样。.readlines()自动将文件内容分析成一个行的列表，
该列表可以由 Python 的 for... in ... 结构进行处理。
另一方面，.readline()每次只读取一行，通常比 .readlines()慢得多。仅当没有足够内存可以一次读取整个文件时，才应该使用.readline()。

写：
writeline()是输出后换行，下次写会在下一行写。write()是输出后光标在行末不会换行，下次写会接着这行写

49、
c++ vector容器的使用
容器的概念就相当于一个为固定大小的数组，可以在不清楚数组大小的情况下最大限度的节省空间
需要使用命名空间using namespace std, 或者使用std::vector<std::类型>

具体使用就是：
vector <int> a; 等于声明了一个int数组a[],大小没有指定,可以动态的向里面添加删除（或者写为std::vector<std::int> a;）
如果是整型二维数组，就相当于一个一维数组序列，每一个元素都是数组，也就是数组的首地址，也就是一个指向整型的指针，
那么就可以定义为:vector <int *> a
也可以在定义容器的时候就初始化开始的容器元素个数，如
vector <int> a(4) 开始先确定容器元素个数为4.
常用的操作如下：
a.push_back()   在数组的最后添加一个数据
a.pop_back()    去掉数组的最后一个数据
a.at()                得到编号位置的数据
a.size           当前容器中元素的个数
后续用到继续补充........

50、
string.c_str()的作用：
c_str()是Borland封装的String类中的一个函数，它返回当前字符串的首字符地址。换种说法，c_str()函数返回一个指向正规C字符串的指针常量，内容与本string串相同。这是为了与C语言兼容，在C语言中没有string类型，故必须通过string类对象的成员函数c_str()把string对象转换成C中的字符串样式。
因此就有了下面这样的使用:
weightsPath = "/home/sunmiao/yolov3"
weights = new char[weightsPath.length() + 1];
strcpy(weights, weightsPath.c_str());

51、
递归创建文件夹
mkdir -p catkin_ws/src/***/***/***

52、
python在原位置覆盖打印：
import sys
sys.stdout.writelines('prediction time for single image : {:.5f}s'.format(t_cost) + '\r')

53、
python import 导入动态模块
当我们需要根据不同情况导入不同模块的时候，就不能直接import ***这样写死了，需要设置导入动态模块
https://python3-cookbook.readthedocs.io/zh_CN/latest/c10/p10_import_modules_using_name_given_in_string.html

用法：
import importlib

MODEL = importlib.import_module(FLAGS.model) 括号里面是字符串变量，这样就可以通过参数传递进来一个字符
串变量，来动态引入不同模块了，尤其适合于引入不同的network文件，将模型训练和模型网络构建解耦。
在举一个例子：
import importlib
math = importlib.import_module('math')
print math.sin(2)
0.9092974268256817

54、
tf.get_variable()和tf.Variable()的区别
前者可以获取同样scope空间下面的变量，但是前提是scope设置重用（reuse = True）,如果空间下没有同样名字的变量，则重新创建一个新变量，初始化方法可以有很多种（可以不用指定2初始化某个固定值）

后者仅仅用于重新创建一个变量，但是初始化方法必须为固定的指定值。

具体使用例子可以参考：
https://blog.csdn.net/u012223913/article/details/78533910

55、
np.eye(k, dtype=int)  生层对角矩阵K*K
***.flatten()  将一个矩阵展平为一维矩阵，仅仅适用于numpy对象。

56、
tf.transpose(input, [dimension_1, dimenaion_2,..,dimension_n])
这个函数主要适用于交换输入张量的不同维度用的，如果输入张量是二维，就相当是转置。dimension_n是整数，如果张量是三维，就是用0,1,2来表示。这个列表里的每个数对应相应的维度。如果是[2,1,0]，就把输入张量的第三维度和第一维度交换。
举个列子：
如果我们想交换矩阵的维度，从NHWC变换到NCHW：
那么我们就可以：
new_points = tf.transpose(new_points, [0,3,1,2])
可以方便的进行矩阵的维度变换。

57、
tensorflow 求取矩阵的最大值和平均值
求最大值tf.reduce_max(input_tensor, reduction_indices=None, keep_dims=False, name=None)
求平均值tf.reduce_mean(input_tensor, reduction_indices=None, keep_dims=False, name=None)
参数1--input_tensor:待求值的tensor。
参数2--reduction_indices:在哪一维上求解。

58、
tensorflow求取范数
tf.norm(tensor, ord='euclidean', axis=None, keep_dims=False, name=None)
功能：求取范数。
输入：ord：范数类型，默认为‘euclidean’，支持的有‘fro’，‘euclidean’，‘0’，‘1’，‘2’，‘np.inf’;
axis：默认为‘None’，tensor为向量。
keep_dims:默认为‘None’，结果为向量，若为True，保持维度。
其中，如果范数类型ord=1，则代表相加；如果ord=2,则代表求平方和再开根号；如果ord='euclidean‘就是求欧几里德距离，和2范数一样。
axis代表在哪一个维度求取范数，如果为none则对全体求取范数
keep_dim如果为 True，则 axis 中指定的轴将保持为大小 1。否则，坐标轴中的尺寸将从 "输出" 形状中移除。

举个例子如下所示：
a = tf.constant([1, 2, 3, 4, 5, 6], shape=[2, 3],dtype=tf.float32)
z = tf.norm(a)
z2=tf.norm(a,ord=1)
z3=tf.norm(a,ord=2)
z4=tf.norm(a,ord=1,axis=0)
z5=tf.norm(a,ord=1,axis=1)
z6=tf.norm(a,ord=1,axis=1，keep_dims=True)

a==>[[1, 2, 3],
          [4, 5, 6]]
z==>9.53939
z2==>21.0
z3==>9.53939
z4==>[5. 7. 9.]
z5==>[6. 15.]
z6==>[[6.]
            [15.]]
59、
’ 空格’.join(字符串) 这样可以将字符串以空格来进行插入，空格可以替换为其他人以一种想插入的字符。

60、
python -m SimpleHTTPServer 8880
远程查看文件

61、
ubuntu 翻墙代理设置
https://blog.csdn.net/totorocyx/article/details/80032556

62、
rospy.spin和rospy.spinOnce的区别
ros::spin()
这句话的意思是循环且监听反馈函数（callback）。循环就是指程序运行到这里，就会一直在这里循环了。监听反馈函数的意思是，如果这个节点有callback函数，那写一句ros::spin()在这里，就可以在有对应消息到来的时候，运行callback函数里面的内容。
就目前而言，以我愚见，我觉得写这句话适用于写在程序的末尾（因为写在这句话后面的代码不会被执行），适用于订阅节点，且订阅速度没有限制的情况。
ros::spinOnce()
这句话的意思是监听反馈函数（callback）。只能监听反馈，不能循环。所以当你需要监听一下的时候，就调用一下这个函数。
这个函数比较灵活，尤其是我想控制接收速度的时候。配合ros::ok()效果极佳。
例如

ros::Rate loop_rate(10);
while(ros::ok())
{
    ros::spinOnce();
    loop_rate.sleep();
}
可以控制10Hz速度，运行callback函数，非常方便。
如果只有
while(ros::ok())
{
    ros::spinOnce();
}
这就等于ros::spin()。

63、
python 线程设计
https://www.jianshu.com/p/8301f1083d5e

64、
pyhon读取文件的几种方式：
with open('文件名 ', '读写方式 ') as f:

r：以只读的方式打开文本文件，文件必须存在；

w：以只写的方式打开文本文件，文件若存在则清空文件内容从文件头部开始写，若不存在则根据文件名创建新文件并只写打开；

a：以只写的方式打开文本文件，文件若存在则从文件尾部以追加的方式开始写，文件原来存在的内容不会清除（除了文件尾标志EOF），若不存在则根据文件名创建新文件并只写打开；

r+：以可读写的方式打开文本文件，文件必须存在；

w+：以可读写的方式打开文本文件，其他与w一样；

a+：以可读写的方式打开文本文件，其他与a一样；

若打开二进制文件，可在后面加个b注明，其他一样，如rb，r+b（或rb+）
为什么需要使用b呢，因为一般读取文件的时候，读到EOF就认为读到了文件末尾停止读取，而二进制文件的‘0x1A’和EOF是一样的
因此，使用'r'的时候，如果碰到'0x1A'，就视为文件结束，就是EOF。使用'rb'就不会认为‘Ox1A’是停止符，不存在这个问题。

65
Python File seek() 方法
f.seek(-2,1)
用于修改文件读取时候的光标位置
参数
offset -- 开始的偏移量，也就是代表需要移动偏移的字节数
whence：可选，默认值为 0。给offset参数一个定义，表示要从哪个位置开始偏移；0代表从文件开头开始算起，1代表从当前位置开始算起，2代表从文件末尾算起。

66、
tmux使用
tmux有三个概念，分别是会话session、窗口window、窗格pane，这三者是分级关系，最终我们可以在不同的窗格里面输入命令


tmux速查表
https://gist.github.com/ryerh/14b7c24dfd623ef8edc7

启动新会话：
     tmux [new -s 会话名 -n 窗口名]
列出当前会话
     tmux ls

67、
np.diag(第一个变量，第二个变量)
第一个变量：如果是一个二位矩阵，则返回结果是这个二维矩阵的对角线元素，是一个一维结果。
                   如果是一个一维向量，则返回以这个一维向量为对角线的二维矩阵
第二个变量k：如果第一个变量是二维矩阵，那么k的作用事修改对角线的位置，k>0表示向主对角线上方偏移，k<0表示向主对角线下方偏移

68、
math.radians(）
将角度转换为弧度

69、
获取当前python的文件名可以使用 __file__
例如
print(__file__ + " start!!")

70、
numpy.diff(a, n=1,axis=-1) 沿着指定轴计算第N维的离散差值
参数：
     a：输入矩阵
     n：可选，代表要执行几次差值
     axis：默认是最后一个
示例：
A = np.arange(2 , 14).reshape((3 , 4))
A[1 , 1] = 8
print('A:' , A)

A: [[ 2 3 4 5]
[ 6 8 8 9]
[10 11 12 13]]

print(np.diff(A))

[[1 1 1]
[2 0 1]
[1 1 1]]
从输出结果可以看出，其实diff函数就是执行的是后一个元素减去前一个元素

71、
对于成对的xy坐标点,比如
ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
可以通过zip将他们打包，然后在一起提取出来。
for (idx, idy) in zip(dx, dy)]：
     .....

72、
np.cumsum(矩阵，axis=)
这是一个累加函数，根据不同的轴来指定累加的方向

a = np.cumsum([[1,2],[2,3]])
print(a)
[1 3 5 8]  不指定轴信息，由前面的值依次累加

b = np.cumsum([[1,2],[2,3]],axis=0)
print(b)
[[1 2]
 [3 5]]
每列累加

c = np.cumsum([[1,2],[2,3]],axis=1)
print(c)
[[1 3]
 [2 5]]
每行累加

73、
list1.extend(list2)
用于在list1后面完整添加list2的内容，也就是扩展列表

74、
python解方程
numpy.linalg.solve 可以直接求解线性方程组.

一般地，我们设解线性方程组形如 Ax=b，其中 A 是系数矩阵，b 是一维列向量，x 是未知变量。再拿上面地最简单的二元一次方程组为例，我们用 numpy.linalg.solve 可以这样写：

import numpy as np
A = np.mat('1,2; 4,5')       # 构造系数矩阵 A
b = np.mat('3,6').T           # 构造转置矩阵 b （这里必须为列向量）
r = np.linalg.solve(A,b) # 调用 solve 函数求解
print r
...:
Out[1]: [[-1.]
            [ 2.]]

75、
python的bisect模块：用于方便的排序任务

import bisect

常用的几个方法：
     使用这个模块要先确保操作的列表的已经排序的
     data = [4,2,9,7]]
     data.sort()
     print data
     >>> [2,4,7,9]

a、bisect.insort(data, 3)
     print data
     >>> [2,3,4,7,9]
     这个插入的结果不会影响原有的排序
b、bisect.bisect(data, 1)
     >>> 0
     print data
     >>> [2,3,4,7,9]
     这个方法在于查找该数值将要插入的位置并且返回，而不会真的插入
c、接着看 bisect_left 和 bisect_right 函数，该函数用入处理将会插入重复数值的情况，返回将会插入的位置，但不会真的插入：
     bisect.bisect_left(data,4)
     >>> 2
     bisect.bisect_right(data,4)
     >>> 3
     print data
     >>> [2,3,4,7,9]
d、如果要是插入的话，对应的插入函数是 insort_left  和 insort_right ：
     bisect.insort_left(data,4)
     print data
     >>> [2,3,4,4,7,9]
     data = [2,3,4,7,9]
     bisect.insort_right(data,4)
     print data
     >>> [2,3,4,4,7,9]

76、
https://www.jianshu.com/p/cee2de32ca28
terminator终端配置

77、
cin输入字符串以空格，换行符，制表符结尾

78、
c++容器
map：
map.find
map.count()  查询map有没有相应的元素，有返回1，没有返回0，因为map的键都是不能重复的，所以只能是1或0
map.remove
map.insert

stack:
stack.push() 入栈
stack.pop() 出栈
stack.top() 返回栈顶元素，但是不修改栈

queue：
queue.push()
queue.pop()
queue.front()
queue.empty()

优先队列：priority_queue<int> s
s.push()
s.pop()
s.top() 取队首元素，但是不删除

78、c++生成随机数
头文件<cstdlib>
rand()
这个生成的随机数是均匀随机整数，生成随机整数的范围为[0,RAND_MAX],RAND_MAX至少为32767（2十五次方 - 1）
不可以修改，当我们想生成自己想要范围[0,n]内的均匀随机数时，
可以将rand()/RAND_MAX，得到[0,1]范围内的均匀随机小数，然后再乘以n

80、main(argc argv)
argc 代表传入参数的个数
argv  代表传入的参数表(各个参数的值，并且是以字符串的形式来表示)

81、
c++
对于局部变量的内存地址是放在栈里面，
对于malloc或者new显示申请内存的变量，是放在堆里面，叫heap
对于全局变量的内存地址，如果是初始化好的全局变量，放在一个连续的内存空间。如果是没有初始化好的全局变量，也全部放在一起，一个连续的内存空间。

82、
W_update=np.zeros_like(W);

函数主要是想实现构造一个矩阵W_update，其维度与矩阵W一致，并为其初始化为全0；这个函数方便的构造了新矩阵，无需参数指定shape大小；


83、
gsettings set com.canonical.Unity.Launcher launcher-position Bottom/Left
修改ubuntu的菜单启动栏的位置

84、
python没有显示声明私有变量和公有变量的关键字。
一般来将，self._name    前面一个下划线的是被保护变量，可以被本类对象和子类对象调用
self.__name  前面两个下划线的是私有变量，只能被本类对象使用

85、tensorflow中tf.app.run()的使用方法
https://blog.csdn.net/fxjzzyo/article/details/80466321
这是一个函数入口：
如果你的代码中的入口函数不叫main()，而是一个其他名字的函数，如test()，则你应该这样写入口tf.app.run(test())
如果你的代码中的入口函数叫main()，则你就可以把入口写成tf.app.run()
86、python读取文件
with open('odom.txt', 'r') as f:
        data = f.readlines() #txt中所有字符串读入data

for line in data:
        odom = line.split() #将单个数据分隔开存好
        numbers_float = map(float, odom) #转化为浮点数
        print numbers_float

87、opencv c++ 中图像数据类型Mat的数据格式的种类
一般的图像文件格式使用的是 Unsigned 8bits吧，Mat矩阵对应的参数类型就是CV_8UC1，CV_8UC2，CV_8UC3
（最后的1、2、3表示通道数，譬如RGB3通道就用CV_8UC3）
而float 是32位的，对应CvMat数据结构参数就是：CV_32FC1，CV_32FC2，CV_32FC3...
double是64bits，对应CvMat数据结构参数：CV_64FC1，CV_64FC2，CV_64FC3等。

88、c++ main参数
(1).int main(void)
(2).int main(int argc,char *argv[]) = int main(int argc,char **argv).
其参数argc和argv用于运行时,把命令行参数传入主程序.其中ARG是指arguments,即参数.具体含义如下:
(参照Arguments to main和C++ Primer7.2.6节)
(1).int argc:英文名为arguments count(参数计数)
count of cmd line args,运行程序传送给main函数的命令行参数总个数,包括可执行程序名,其中当argc=1时表示只有一个程序名称,此时存储在argv[0]中.
(2).char **argv:英文名为arguments value/vector(参数值)
pointer to table of cmd line args,字符串数组,用来存放指向字符串参数的指针数组,每个元素指向一个参数,空格分隔参数,其长度为argc.数组下标从0开始,argv[argc]=NULL.
argv[0] 指向程序运行时的全路径名
argv[1] 指向程序在DOS命令中执行程序名后的第一个字符串
argv[2] 指向执行程序名后的第二个字符串
argv[argc] 为NULL.

89、c++中static的使用含义：
static 全局变量：
如果在全局变量前面加上static，那么这个全局变量的作用域是整个文件，在其他文件中是不可见的，变量地址放在静态存储区
好处：
定义全局静态变量的好处：
<1>不会被其他文件所访问，修改
<2>其他文件中可以使用相同名字的变量，不会发生冲突。

static 局部变量：
变量地址在静态存储区，只有当程序结束的时候内存才会释放掉，但是作用域不变，仍然只能在局部作用域使用，但是出了局部作用域，静态局部变量并没有被销毁，而是仍然保留着内存，在下一次进入函数局部作用域的时候，还可以继续使用静态局部变量上次的数值。

static 函数
如果在函数返回值前面加上static，代表他是静态函数，静态函数的只能在当前文件使用，因此可以避免其他文件出现同名函数的问题。而且静态函数不能被其他文件所调用。
而且静态函数会更快：
静态函数会被自动分配在一个一直使用的存储区，直到退出应用程序实例，避免了调用函数时压栈出栈，速度快很多。
https://www.jianshu.com/p/f413ba3b2728

90、
EIgen:Matricx和vector类的定义和使用
https://blog.csdn.net/xuehuafeiwu123/article/details/75408755

91、
C++ vector模板类的问题
std::vector<PointT, Eigen::aligned_allocator<PointT> > points；
模板类实例化的格式一般是 vector<数据类型> 名称，但是std::vector<PointT, Eigen::aligned_allocator<PointT>> points 这个语句明显有两个数据类型了
实际上模板和函数一样，是可以有默认参数的，std::vector的声明是
template<
    class T,
    class Allocator = std::allocator<T>
> class vector;
有两个模板参数，T 是元素类型，而 Allocator 负责提供 vector 需要用到的动态内存。其中 Allocator 参数有默认值，一般的使用不需要指定这个参数。但有时对内存有特殊需求，就需要提供自己定义的内存管理类。
把容器操作和内存管理分开，这是STL的一个亮点，你在设计容器时也可以学习

92、
内存对齐问题：
https://blog.csdn.net/huntinux/article/details/39957951

93、
string的比较：
bool operator==(const string &s1,const string &s2)const;//比较两个字符串是否相等
运算符">","<",">=","<=","!="均被重载用于字符串的比较；
int compare(const string &s) const;//比较当前字符串和s的大小
int compare(int pos, int n,const string &s)const;//比较当前字符串从pos开始的n个字符组成的字符串与s的大小
int compare(int pos, int n,const string &s,int pos2,int n2)const;//比较当前字符串从pos开始的n个字符组成的字符串与s中pos2开始的n2个字符组成的字符串的大小

94、
const * char c_str()
将string转换为 const* char。

95、
make_shared() shared_prt()的详解区别
https://blog.csdn.net/fanwenbo/article/details/16872153

96、
eigen坐标变换

97、
协方差矩阵：
协方差矩阵对角元素表示的是对应的元素的方差，非对角元素则表示对应的两个元素（行与列）的相关性。
对于ndt算法来说：
用正态分布来表示原本离散的点云有诸多好处，这种分块的（通过一个个cell）光滑的表示是连续可导的，每一个概率密度函数可以被认为是一个局部表面（local surface）的近似,它不但描述了这个表面在空间中的位置，同时还包含了这个表面的方向和光滑性等信息。
局部表面的方向和光滑性则可以通过协方差矩阵的特征值和特征向量反映出来。我们以三维的概率密度函数为例，如果三个特征值很接近，那么这个正态分布描述的表面是一个球面，如果一个特征值远大于另外两个特征值，则这个正态分布描述的是一条线，如果一个特征值远小于另外两个特征值，则这个正态分布描述的是一个平面。

98、
有关于ros句柄命名空间的例子，可以确定某一个句柄能访问到参数服务器的哪些变量。
// launch 文件中 ns=="node_namespace"

ros::init(argc, argv, "node_name"); // node name

ros::NodeHandle n; //n 命名空间为/node_namespace

ros::NodeHandle n1("sub"); // n1命名空间为/node_namespace/sub

ros::NodeHandle n2(n1,"sub2");// n2命名空间为/node_namespace/sub/sub2

ros::NodeHandle pn1("~"); //pn1 命名空间为/node_namespace/node_name

ros::NodeHandle pn2("~sub"); //pn2 命名空间为/node_namespace/node_name/sub

ros::NodeHandle pn3("~/sub"); //pn3 命名空间为/node_namespace/node_name/sub

ros::NodeHandle gn("/global"); // gn 命名空间为/global

99、
c++ 枚举类型（强制类型的枚举）
https://blog.csdn.net/u012333003/article/details/20612267

100、
c++中的类型转换：隐式转换和强制类型转换（与c不同）
https://www.cnblogs.com/chio/archive/2007/07/18/822389.html
