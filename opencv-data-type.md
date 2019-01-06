在以下两个场景中使用OpenCV时，我们必须事先知道矩阵元素的数据类型：

- 使用 at 方法访问数据元素的时候要指明数据类型
- 做数值运算的时候，比如究竟是整数除法还是浮点数除法。

cv::Mat 类的对象有一个成员函数type()用来返回矩阵元素的数据类型，
返回值是 int 类型，不同的返回值代表不同的类型,具体对应关系如下所示

类型|C1|C2|C3|C4|
--|--|--|--|--|
CV_8U  |  0 |  8 | 16  |  24 |
CV_8U  |  1 |  9 | 17  |  25 |
CV_8U  |  2 |  10 |18   | 26  |
CV_8U  |  3 |  11 |19   | 27  |
CV_8U  |  4 |  12 | 20  | 28  |
CV_8U  |  5 |  13 | 21  | 29  |
CV_8U  |  6 |  14 | 22  | 30  |

表头的 C1, C2, C3, C4 指的是通道(Channel)数,例如：

- 灰度图像只有 1 个通道，是 C1；

- JPEG格式 的 RGB 彩色图像就是 3 个通道，是 C3

- PNG 格式的彩色图像除了 RGB 3个通道外，还有一个透明度通道，所以是 C4。

如果仅仅是为了在数值计算前明确数据类型，那么看到这里就可以了

如果是要使用 at 方法访问数据元素，那么还需要下面一步

因为以单通道为例，at 方法接受的是 uchar 这样的数据类型，而非 CV_8U。

在已知通道数和每个通道数据类型的情况下，指定给 at 方法的数据类型如下表所示：

类型| C1|C2|C3|C4|C6
--|--|--|--|--|--|
|uchar   | uchar  | cv::Vec2b  | cv::Vec3b  | cv::Vec4b  |   |
|short   | short  | cv::Vec2s  | cv::Vec3s  | cv::Vec4b  |   |
|int     | int    | cv::Vec2i  | cv::Vec3i  | cv::Vec4i  |   |
|float   | float  | cv::Vec2f  | cv::Vec3f  | cv::Vec4f  | cv::Vec6f  |
|double  | double | cv::Vec2d  | cv::Vec3d  | cv::Vec4d  | cv::Vec6f  |

现在，就可以使用at来访问图像的像素了：
```c++
    cv::Vec3b vec3b = img.at<cv::Vec3b>(0,0);
    uchar vec3b0 = img.at<cv::Vec3b>(0,0)[0];
    uchar vec3b1 = img.at<cv::Vec3b>(0,0)[1];
    uchar vec3b2 = img.at<cv::Vec3b>(0,0)[2];
    std::cout<<"vec3b = "<<vec3b<<std::endl;
    std::cout<<"vec3b0 = "<<(int)vec3b0<<std::endl;
    std::cout<<"vec3b1 = "<<(int)vec3b1<<std::endl;
    std::cout<<"vec3b2 = "<<(int)vec3b2<<std::endl;
```

上述数据类型以及取值范围

数值|具体类型 |取值范围
--|--|--
CV_8U|	8 位无符号整数	|（0…..255）
CV_8S|8 位符号整数|（-128…..127）
CV_16U|16 位无符号整数|（0……65535）
CV_16S|16 位符号整数|（-32768…..32767）
CV_32S|32 位符号整数|（-2147483648……2147483647）
CV_32F|32 位浮点数|（-FLT_MAX ………FLT_MAX，INF，NAN)
CV_64F|64 位浮点数|（-DBL_MAX ……….DBL_MAX，INF，NAN)

Vec类的定义：
```c++
template<typename _Tp, int n> class Vec : public Matx<_Tp, n, 1> {...};

typedef Vec<uchar, 2> Vec2b;
typedef Vec<uchar, 3> Vec3b;
typedef Vec<uchar, 4> Vec4b;

typedef Vec<short, 2> Vec2s;
typedef Vec<short, 3> Vec3s;
typedef Vec<short, 4> Vec4s;

typedef Vec<int, 2> Vec2i;
typedef Vec<int, 3> Vec3i;
typedef Vec<int, 4> Vec4i;

typedef Vec<float, 2> Vec2f;
typedef Vec<float, 3> Vec3f;
typedef Vec<float, 4> Vec4f;
typedef Vec<float, 6> Vec6f;

typedef Vec<double, 2> Vec2d;
typedef Vec<double, 3> Vec3d;

typedef Vec<double, 4> Vec4d;
typedef Vec<double, 6> Vec6d;
```
参考链接：https://www.jianshu.com/p/204f292937bb
