## CMakeList.txt 分析
### 一、简介
ROS使用cmake来编译代码，cmake首先会一步一步处理CMakeLists.txt的内容，然后生成一个MakeFile文件，系统再通过这个文件的设置进行程序的编译。

换句话说，Makefile文件一般是由Cmake根据CMakeLists.txt生成的,我们不要去修改，我们只需要修改CMakeLists.txt文件就可以了，下面我们一句一句来解读CMakeList.txt的含义。
### 二、整体结构
ROS中的CMakeLists.txt主要包括下面几个部分：

- 所需CMake版本(**cmake_minimum_required**)

- 软件包名称(**project()**)

- 查找构建所需的其他CMake / Catkin软件包 (**find_package()**)

- 消息/服务/动作生成器(**add_message_files(), add_service_files(), add_action_files()**)

- 生成消息/服务/动作等自定义消息(**generate_messages()**)

- 指定包的构建信息输出 (**catkin_package()**)

- 要建立的库/可执行文件(**add_library() / add_executable() / target_link_libraries()**)

- 测试(**catkin_add_gtest()**)

- 安装规则( **install()** )

### 三、版本和名称
下面的代码来自于 autoware 的 [lidar_localizer模块](https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/localization/packages/lidar_localizer/CMakeLists.txt)
```
cmake_minimum_required(VERSION 2.8.3)
```
CMakeLists.txt都要以此开始，catkin编译需要2.8.3版本以上的cmake。
```
project(lidar_localizer)
```
通过project()这个函数指定包的名字，在CMake中指定后，**你可在其他地方通过使用变量${PROJECT_NAME}来引用它**

### 四、查找相关包
```
find_package(PCL REQUIRED)
IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(CUDA)
if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_conversions
        velodyne_pointcloud
        ndt_tku
        ndt_cpu
        ${PCL_OPENMP_PACKAGES}
        )
```
上面的代码指明了构建这个package需要依赖的所有package，其中，catkin是必备依赖，因为ros需要用catkin_make来编译代码

一旦一个包被find_package找到，则会自动生成有关包所在路径的CMake环境变量，这些环境变量后面将在CMake的脚本中用到，这些变量描述了包中头文件的位置，源文件的位置，包所依赖的库以及这些库的路径。这些变量的名字依照的惯例是：
> \<NAME>_FOUND：这个变量说明这个库是否被找到，如果找到就被设置为true，否则设为false；
>
> \<NAME>_INCLUDE_DIRS or<NAME>_INCLUDES：这个包输出的头文件目录；
>
> \<NAME>_LIBRARIES or <NAME>_LIBS：这个包输出的库文件。
```
find_package(roscpp REQUIRED）
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
```
还有一点，如果按照上面的方法添加每一个需要的package包，那么每个依赖的package都会产生几个变量，这样很不方便。所以还有另外一种方式：
```
find_package(catkin REQUIRED COMPONENTS
               roscpp
               rospy
               std_msgs
               message_generation
               )
```
这样，它会把所有pacakge里面的头文件和库文件等等目录加到一组变量上，比如叫：catkin_INCLUDE_DIRS，这样，我们就可以用这个变量查找需要的文件，最终就只产生一组变量了，会更方便。

### 五、catkin_package()
catkin_package()是一个catkin提供的CMake宏，用于将catkin特定的信息输出到构建系统上，用于生成pkg配置文件以及CMake文件。

这个命令必须在add_library()或者add_executable()之前调用，该函数有5个可选参数：

- INCLUDE_DIRS - 导出包的include路径
- LIBRARIES - 导出项目中的库
- CATKIN_DEPENDS - 该项目依赖的其他catkin项目
- DEPENDS - 该项目所依赖的非catkin CMake项目。
- CFG_EXTRAS - 其他配置选项

例如：autoware中的相应内容
```
catkin_package(
        CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs autoware_config_msgs ndt_tku ndt_cpu ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)
```
### 六、添加头文件路径和库路径
在生成目标之前，需要先写好包含路径和库路径，不然编译会失败。
```
include_directories（<dir1>，<dir2>，...，<dirN>）
link_directories（<dir1>，<dir2>，...，<dirN>）
```
#### 6.1 头文件路径
```
include_directories()
```
include_directories()中的参数应该是调用find_package调用时生成的\<NAME>_INCLUDE_DIRS变量。例如autoware的示例：
```
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
```
第一个参数“include”表示包中的include目录也是路径的一部分。

还有一种是指定给某一个构建目标添加头文件路径
```
target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
```
但是这一句一般要在add_executable()调用之后写

#### 6.2 库路径
```
link_directories（）
```
link_directories()函数可用于添加额外的动态链接库路径

但通常不推荐这样做，因为所有catkin和CMake软件包在find_packaged时都会自动添加链接信息。 只需写下面这样就可以了
```
target_link_libraries（）
```
例如，autoware是这么写的，代表给生成的可执行文件ndt_matching添加链接库路径
```
add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})
```
但真要写，就按照下面那样来写，括号内容写额外添加的链接库所在的目录
```
link_directories(~/my_libs)
```
回过头来，一般推荐的写法是这样
```
target_link_libraries(<executableTargetName>, <lib1>, <lib2>, ... <libN>)
```
target_link_libraries()来指定可执行目标链接的库，但是这一句一般要在add_executable()调用之后写

### 七、 指定目标及其源文件
构建目标可以有多种形式，但通常主要有以下两种：

- 执行文件目标 - 可以运行的程序

- 库目标 - 可在构建和/或运行时给可执行目标使用的库

#### 7.1 构建可执行文件作为目标
要指定需要构建的可执行目标由哪些源文件生成，需要使用add_executable()函数。
例如：
```
add_executable(myProgram
                src/main.cpp
                src/some_file.cpp)
```
构建名字叫myProgram的目标可执行文件，需要2个源文件：src/main.cpp 和 src/some_file.cpp

autoware里面也是一样：
```
add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
```
#### 7.2 构建库作为目标
add_library() 用于指定要构建的库。
```
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```
例如autoware中的示例：
```
add_library(ndt_matching_monitor_lib SHARED
        nodes/ndt_matching_monitor/ndt_matching_monitor.h
        nodes/ndt_matching_monitor/ndt_matching_monitor.cpp
        )
target_include_directories(ndt_matching_monitor_lib PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
add_dependencies(ndt_matching_monitor_lib
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ndt_matching_monitor_lib ${catkin_LIBRARIES})
```
上面代码指定了构建库目标所需要的源文件、头文件路径、依赖关系、动态链接库路径等等

SHARE代表构建共享库，默认不指明也会构建共享库

### 八、 添加依赖关系
如果我们构建的可执行文件需要其他的可执行文件才能完成，，那么就需要添加依赖关系
```
add_dependencies(your_target_name ${some_TARGETS})
```
例如
```
add_executable(ndt_matching_tku nodes/ndt_matching_tku/ndt_matching_tku.cpp)
target_link_libraries(ndt_matching_tku ${catkin_LIBRARIES})
add_dependencies(ndt_matching_tku ${catkin_EXPORTED_TARGETS})
```
ndt_matching_tku可执行文件就需要我们前面定义的catkin及其组件的一堆可执行文件作为依赖

### 九、设置install
构建后，构建目标通常被放置在catkin工作区中。但有时候我们希望将目标安装到系统其他地方，以便其他人或本地文件夹可以使用它们来测试

换句话说，如果想要做一个“make install”的代码，你需要指定目标应该生成到哪里。

使用CMake install()函数可以完成，其参数如下所示：
```
TARGETS - 需要install的各种目标
ARCHIVE DESTINATION - 静态库和DLL（Windows）.lib存根
LIBRARY DESTINATION - 非DLL共享库和模块
RUNTIME DESTINATION - 可执行目标和DLL（Windows）样式共享库
```
例如，autoware的示例如下：
```
install(TARGETS ndt_matching_tku ndt_mapping_tku ndt_mapping_tku ndt_matching_monitor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
```
其他资源，如launch启动文件，也可以安装
```
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
```
例如autoware的示例如下：
```
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
```
