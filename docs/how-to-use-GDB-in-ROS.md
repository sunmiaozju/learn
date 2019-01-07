## 引言
在写ROS工程代码，有时候找一个bug非常麻烦，尤其是运行时出错的bug，这时候借助一些调试器可以极大的提高查找bug的效率。

下面介绍如何使用GDB调试器来进行ROS C++项目的调试

## 在debug模式编译
编译器有些优化会让debug无法进行。为了避免这种情况，程序编译时要加上debug选项，让cmake以debug模式编译，不然可能会在gdb调试的时候不能跳转到源代码，只能进入断点。

如果用命令行catkin_make，在输入catkin_make时加上一个参数：
```
catkin_make -DCMAKE_BUILD_TYPE=Debug
```
或者直接修改CMakelist.txt，添加以下代码,
```
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```
## 添加GDB调试指令
修改ROS launch文件，在node标签中添加一句话
```xml
launch-prefix="xterm -e gdb -ex run --args "
```

修改后的launch文件如下所示
```xml
<node pkg="waypoint_follower" type="pure_persuit" name="pure_pursuit" output="screen" launch-prefix="xterm -e gdb --args">
        <param name="is_linear_interpolation" value="$(arg is_linear_interpolation)"/>
</node>
```
这样在执行原来例子的时候，就会先打开一个新的Shell界面，被调试的程序就在这个Shell中被执行了。

如果使用的是Python来写ROS，则需要修改调试器为pdb,如下：
```
launch-prefix="xterm -e python -m pdb "
```
> 附：roslaunch node前缀
>
> The launch-prefix attribute of the <node> tag that, among other things, makes it easy to debug a ROS node process. Here are some example launch-prefixes you might find useful:
>
> **launch-prefix="xterm -e gdb --args"** : run your node in a gdb in a separate xterm window, manually type run to start it
>
> **launch-prefix="gdb -ex run --args"** : run your node in gdb in the same xterm as your launch without having to type run to start it
>
> **launch-prefix="stterm -g 200x60 -e gdb -ex run --args"** : run your node in gdb in a new stterm window without having to type run to start it
>
> **launch-prefix="valgrind"** : run your node in valgrind
>
> 这个valgrind工具可以用于检测内存泄露，并执行性能分析
>
> **launch-prefix="xterm -e"** : run your node in a separate xterm window
>
> **launch-prefix="nice"** : nice your process to lower its CPU usage
>
> **launch-prefix="screen -d -m gdb --args"** : useful if the node is being run on another machine; you can then ssh to that machine and do screen -D -R to see the gdb session
>
> **launch-prefix="xterm -e python -m pdb"** : run your python node a separate xterm window in pdb for debugging; manually type run to start it



## GDB常用命令

### 基本
命令 | 描述
----|---  |
gdb |打开调试器
file FILE |装载指定可执行文件
r|代表run,从头开始运行程序直到断点。在一次debug中你可以通过用 r 来多次重新运行程序，而不必重新rosrun 或 roslaunch.
q|退出debug。
bt|列出调用堆栈。
### 显示被调试文件信息
命令 | 描述
----|---
info files| 显示被调试文件的详细信息
info func |显示所有函数名称
info local |显示当前函数中的局部变量信息
info prog |显示被调试程序的执行状态
info var |显示所有的全局和静态变量名称
### 查看/修改内存
命令    | 描述
----------|---
p x|相当于“print x”。显示当前变量 x 的值。
display x|和print的区别是，x不是只显示一次就消失，而是一直显示，每次继续运行程序都会刷新。相当于VS的“watch”功能。
undisplay x|停止对变量x的display
x address|查看指针所指位置的值。
set x = 12 set x = y|修改变量x的值。
call function()|调用某函数。这个函数可以是你程序里定义的函数，甚至是标准库函数，我的理解是只要在当前位置可访问到的函数都可以调用。这是一个极其有用的功能，生生把c++用成Matlab 。

### 断点
命令    | 描述
----------|---
b|b即break。在当前行设置断点。
b 45 | 在某行设置断点。
b functionName  |  在某函数开始处设置断点。常用：b main 在程序开始设置断点。
watch x == 3|设置条件断点。这个比VS的条件断点更方便，因为它不需要设置在哪一行！时刻监控！
info break|查看当前存在的所有断点。每个断点都有自己的编号。
delete N|删除编号为N的那个断点。

### 调试运行
命令    | 描述
----------|---
n|“next”。运行一行代码。 相当于VS的step over。
s|“step”。运行一个指令。相当于VS的step in。n和s都可以一次运行多行，比如n 5
c|“continue”。继续运行直到下一个断点。
f|“finish”，运行完当前程序。相当于VS的 step out。
