在复制图像时，有两种情况，一种是浅拷贝，一种是深拷贝。

所谓浅拷贝仅仅是引用，即创建了一个新的矩阵头，仍然指向原来的数据空间。

而所谓的深拷贝，是指完全创建一整套新的Mat对象（包括矩阵头和数据空间）。

操作（）和 = 操作都属于浅拷贝，例如：
```c++
Mat image = imread("1.png" , 0) ;
Mat image1(image) ;//仅是创建了Mat的头部分，image1与image共享数据区
Mat image2 = image ;//仅是创建了Mat的头部分，image1与image共享数据区
```

clone()和copyto()属于深拷贝，因为它们都会创建一个独立的空间，不会相互影响,例如
```c
Mat image3 = image.clone() ;//完全拷贝，把image中的所有信息拷贝到image3中
Mat image4;
image.copyTo(image4) ;//拷贝image的数据区到image4中，
```

但是.copyTo()多了一个很有用的功能：使用蒙版拷贝。如下所示，可以通过mask有选择性的复制。
```
src.copyTo(dst,mask)
```
这个是将src中经过mask过滤之后（src中mask矩阵对应的非零部分）的结果复制到dst中。

mask作为掩模板，如果mask对应位置像素点为0，则dst对应的像素点值保持不变。如果不为0，则将src中对应像素点的值赋值给dst。

注意的是mask的数据类型，必须是CV_8U，且通道数或者是1，或者与被蒙版处理的src的通道数一致。
