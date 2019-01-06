## 引言
在vscode下面配置用于ROS项目开发的环境

包括头文件目录的配置，catkin_make命令的配置，GDB debug的配置，以及ROS插件。


## vscode头文件目录配置

##### 安装“c/c++”插件
到vscode左边栏的EXTENSIONS中，搜索“C/C++”并安装

##### 生成c_cpp_properties.json
vscode自身配置文件全部在./.vscode/目录下

但是，在最开始对自己新建的目录和文件进行编辑后，文件夹里面是没有.vscode目录的

同时，我们的cpp文件代码中的 #include <> 这句话是有下划线警示的，提示找不到文件

这时使用鼠标悬浮功能，点击“红色灯泡”，点击edit c_cpp_properties.json选项，vscode会自动在配置文件夹中新建.vscode/文件夹，同时在里面初始化了c_cpp_properties.json文件

##### 输出编译命令文件
这时，可能还有一些头文件找不到，比如ros/ros.h，我们还需要配置一些东西。

用命令行编译我们写的c++代码，同时输出编译信息文件，这里以ROS为例
```
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
这个命令会输出一个compile_commands.json文件在ROS工作空间的build文件夹下面

然后在c_cpp_properties.json文件添加下面一段话
```
"compileCommands": "${workspaceFolder}/build/compile_commands.json"
```
修改后的c_cpp_properties.json文件如下所示：
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "clang-x64",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],

    "version": 4
}
```
添加之后，记得reload或者重启一下vscode

这样，就基本可以找到全部头文件了，然后就可以使用代码提示来码代码了。

## catkin_make设置
vscode没有内置make功能，需要借助Task功能进行配置

Ctrl+shift+P进入命令模式，键入tasks: Configure Task

此时会在.vscode文件夹下面自动生成task.json文件，如下所示：
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，前者代表输出总是build信息，后者代表仅在有错误的时候输出build信息，如果没有错误，则不输出信息
            },
            "problemMatcher": "$msCompile"
        },
    ]
}
```
其中，这行设置
```json
"group": {"kind":"build","isDefault":true},
```
代表将我们定义的这个task添加到build组里面，这样就可以中Ctrl+Shift+B快捷键来找到编译命令，命令名称就是在label里面定义的，如果"isDefault":true那么就代表直接执行command，如果为false还需要在build下拉里面选一下，我们这里就是label名字：catkin_make

还需要提一下，我们打开vscode一定要在我们的ROS工作空间目录打开：
```
code .
```
因为这样你的vscode的Base path就是你打开vscode的位置，在我们执行catkin_make的时候，需要用的这个Base path，必须是我们的ROS工作空间来可以正常catkin_make

这样配置好了之后，我们之后再进行编译ROS工作空间的时候，就可以方便的使用快捷方式
```
Ctrl+Shift+B
```

## GDB debug的配置
GDB调试器是调试C++代码的神器，ROS项目本质上也是一个ROS项目，因此也可以用GDB进行调试

在vscode里面已经继承了GDB调试器，我们需要做的就是配置launch.json文件

点击左侧工具栏”Debug“，点击”齿轮“按钮，此时.vscode文件夹下面就会自动生成launch.json文件，如下所示
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch", // 配置名称，将会在调试配置下拉列表中显示
            "type": "cppdbg",  // 调试器类型 该值自动生成
            "request": "launch",  // 调试方式,还可以选择attach
            "program": "${workspaceRoot}/devel/lib/waypoint_follower/pure_persuit", //要调试的程序（完整路径，支持相对路径）
            "args": [],  // 传递给上面程序的参数，没有参数留空即可
            "stopAtEntry": false,  // 是否停在程序入口点（停在main函数开始）
            "cwd": "${workspaceRoot}",  // 调试程序时的工作目录
            "environment": [], //针对调试的程序，要添加到环境中的环境变量. 例如: [ { "name": "squid", "value": "clam" } ]
            "externalConsole": false,   //如果设置为true，则为应用程序启动外部控制台。 如果为false，则不会启动控制台，并使用VS Code的内置调试控制台。
            "MIMode": "gdb",  // VSCode要使用的调试工具
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```
需要注意的是，这里面的“program”参数是需要自己给定的，比如我要调试一个ROS节点，那么就需要找到这个节点生成的可执行目标，就是可执行的二进制文件，然后添加到“program”参数后面，如果要调试其他节点，那么还需要手动修改这里

除此之外，因为我们不是roslaunch启动的节点，还需要一个终端运行roscore,否则会找不到ROS MASTER

"request"参数里面，gdb在vscode里面提供了launch和attach两个配置任务。两者的区别是launch实际上是启动一个node执行指定代码，同时可以在vscode里面打断点调试。 attach是执行监听的任务。

使用vscode进行调试的手段主要包括单步执行，观察跟踪变量值等等

更多的GDB调试命令可以在vscode下方的DEBUG_CONSOLE窗口直接输入GDB命令 但是要注意需要在原来的GDB命令前面家加上一个前缀“-exec”，如下所示：
```
-exec b main
```
有关于常用的GDB命令总结：[GDB的使用](how-to-use-GDB-in-ROS.md)

还需要注意的是在Watch窗口添加需要Watch的变量时候，变量名称要写全局名称，包括前面的命令空间都要写上

或者也可以直接选中这个变量，右键点击，选择“Debug:add to watch”,这样更方便。

基于上面的描述，我们就可以开心的debug了

## 添加ROS插件
#### 安装

打开vscode的快捷输入窗口(Ctrl+P)

输入以下命令，即可安装ROS插件
```
ext install ajshort.ros
```
#### 用法
可以在右键点击一个文件夹，然后选择creat catkin package，创建一个ROS package

还可以按下(Ctrl+Shift+P),输入
```
ros::showMasterStatus
```
这个命令可以显示出当前ROS通信系统的详细信息，包括当前的所有话题，已经话题的所有发布者和订阅者

基本上这个vscode的ROS插件就这两个有用
