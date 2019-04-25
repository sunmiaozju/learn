[TOC]

### 1、管道与查找
```
ps -ef | grep redis-cli
```
ps ，即process status，显示进程状态信息，参数-e代表显示全部进程，-f代表显示进程之间的关系

|, 管道命令

grep ，即global search regular expression(RE) and print out the line，全面搜索正则表达式并把行打印出来,是一种强大的文本搜索工具，它能使用正则表达式搜索文本，并把匹配的行打印出来。

### 2、linux系统调用错误码 errno

为防止和正常的返回值混淆，系统调用并不直接返回错误码，而是将错误码放入一个名为errno的全局变量中。如果一个系统调用失败，你可以读出errno的值来确定问题所在。

errno不同数值所代表的错误消息定义在errno.h中，你也可以通过命令"man 3 errno"来查看它们。

需要注意的是，errno的值只在函数发生错误时设置，如果函数不发生错误，errno的值就无定义，并不会被置为0。另外，在处理errno前最好先把它的值存入另一个变量，因为在错误处理过程中，即使像printf()这样的函数出错时也会改变errno的值。

简单的说，errno在标准C中是一个整型变量，在errno.h中声明，C标准库中实现。多线程技术中，为了使errno线程安全，使用宏定义替代了简单的extern int errno声明。

查看错误代码errno是调试程序的一个重要方法。当C api函数发生异常时，一般会将errno变量(需include errno.h)赋一个整数值，不同的值表示不同的含义，可以通过查看该值推测出错的原因。

在/usr/include/asm/errno.h中有对应错误码的定义

---

在系统编程中错误通常通过函数返回值来表示，并通过特殊变量errno来描述。

errno这个全局变量在<errno.h>头文件中声明如下：extern int errno;

errno是一个由POSIX和ISO C标准定义的符号，看（用）起来就好像是一个整形变量。当系统调用或库函数发生错误的时候，比如以只读方式打开一个不存在的文件时，它的值将会被改变，根据errno值的不同，我们就可以知道自己的程序发生了什么错误，然后进行相应的处理。

为什么，要强调errno看起来好像是一个整形变量呢？因为有的标准(如ISO C)只规定了errno的作用，而没有规定它的实现方式，它可能被定义成一个变量，也有可能被定义成一个宏，这个具体要看编译器自己的实现。早些时候，POSIX.1曾把errno定义成extern int errno这种形式，但现在这种方式比较少见了。因为以这种形式来实现errno，在多线程环境下errno变量是被多个线程共享的，这样可能线程A发生某些错误改变了errno的值，线程B虽然没有发生任何错误，但是当它检测errno的值的时候，线程B会以为自己发生了错误。所以现在errno在Linux中被实现成extern int * __errno_location(void): #define errno (*__errno_location())，这样每个线程都有自己的errno，不会再发生混乱了。

关于errno有三点需要特别注意：

> * 1、如果系统调用或库函数正确执行的话，errno的值是不会被清零（置0，注意这里是不会被清零，不是不会被改变）的，假若执行函数A的时候发生了错误errno被改变，接下来直接执行函数B，如果函数B正确执行的话，errno还保留函数A发生错误时被设置的值。所以，在利用errno之前，最好先对函数的返回值进行判断，看是否发生了错误，返回值错误再利用errno判断时哪里发生了错误。所以如果一个函数无法从返回值上判断正误，而只能通过errno来判断出错，那你在调用它之前必须手动将errno清零！
> * 2、系统调用或库函数正确执行，并不保证errno的值不会被改变！
> * 3、任何错误号（即发生错误时errno的取值）都是非0的。

综上所述，当需要用errno来判断函数是否正确执行的时候，最好先将errno清零，函数执行结束时，通过其返回值判断函数是否正确执行，若没有正确执行，再根据errno判断时哪里发生了错误。

### 3、系统调用的标准使用方法
系统调用，一般是指 glibc 中的包装函数。这些函数会在执行系统调用前设置寄存器的状态，并仔细检查输入参数的有效性。系统调用执行完成后，会从 EAX 寄存器中获取内核代码执行结果。

内核执行系统调用时，一旦发生错误，便将 EAX 设置为一个负整数，包装函数随之将这个负数去掉符号后，放置到一个全局的 errno 中，并返回 −1。若没有发生错误，EAX 将被设置为 0，包装函数获取该值后，并返回 0，表示执行成功，此时无需再设置 errno。

综上，系统调用的标准使用方法可总结为：根据包装函数返回值的正负，确定系统调用是否成功。如果不成功，进一步通过 errno 确定出错原因，根据不同的出错原因，执行不同的操作；如果成功，则继续执行后续的逻辑。代码示例如下：

```c++
int ret = syscallx(...);
if(ret < 0)
{
    //有错误了，通过 errno 确定出错的原因，执行不同的操作
}
else
{
    //调用成功，继续干活
}
```
大多数系统调用都遵循这一过程，errno 是一个整数，可以用 perror 或 strerror 获得对应的文字描述信息。

### 4、系统调用 错误处理函数

#### 4.1 perror

perror函数是用来打印错误提示信息的，原型是：
```c++
#include <stdio.h>
void perror(const char *s);
```
它先打印s指向的字符串，然后输出**当前errno值**所对应的错误提示信息，例如当前errno若为12，调用perror("ABC")，会输出"ABC: Cannot allocate memory"。

测试程序：
```c
#include <stdio.h>
#include <unistd.h>

int main(void)
{
    int fd = 10;
    int ret;
    ret = close(fd);
    if(ret == -1)
        perror("close error");
    return 0;
}
```
上述代码会输出：close error : Bad file descriptor
#### 4.2 strerror

strerror返回errnum的值所对应的错误提示信息，例如errnum等于12的话，它就会返回"Cannot allocate memory"。

函数原型：

```c
#include <string.h>

char *strerror(int errnum);
```

测试程序：
```c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

int main(void)
{
    int fd = 10;
    int ret;
    ret = close(fd);
    if(ret == -1)
        fprintf(stderr, "close error with msg is: %s\n",strerror(errno));
    return 0;
}
```
输出信息：close error with msg is : Bad file descriptor

### 5、errno 的多线程问题
思考一个问题，每个系统调用失败后都会设置 errno，如果在多线程程序中，不同线程中的系统调用设置的 errno 会不会互相干扰呢？

如果 errno 是一个全局变量，答案是肯定的。如果真是这样的话，那系统调用的局限性也就太大了，总不能在每个系统调用之前都加锁保护吧。优秀的 Linux 肯定不会这么弱，那么，这个 errno 的问题又是怎么解决的呢？

根据 man 手册，要使用 errno，首先需要包含 errno.h 这个头文件。我们先看看 errno.h 里面有什么东西。
```
vim /usr/include/errno.h
```
执行以上代码，会发现该文件中有这样几行关键内容：
```c
#include <bits/errno.h>
.......
#ifndef errno
extern int errno;
#endif
```
根据官方提供的代码注释，bits/errno.h 中应该有一个 errno 的宏定义。如果没有，则会在外部变量中寻找一个名为 errno 的整数，它自然也就成了全局整数。否则，这个 errno 只是一个 per-thread 变量，每个线程都会拷贝一份。

关于 per-thread 变量更详细的信息，我们会在后面的课程中介绍。现在，你只需知道，这个 errno，每个线程都会独立拷贝一份，所以在多线程程序中使用它是不会相互影响的。

#### 5.1 实现原理
具体是怎么做到的呢？我们可以再打开 bits/errno.h 看一眼。
```c
<bits/errno.h>
# ifndef __ASSEMBLER__
extern int *__errno_location (void) __THROW __attribute__ ((__const__));

#  if !defined _LIBC || defined _LIBC_REENTRANT
#   define errno (*__errno_location ())
#  endif
#endif
```
原来，当 libc 被定义为可重入时，errno 就会被定义成一个宏，该宏调用外部 __errno_location 函数返回的内存地址中所存储的值。在 GCC 源码中，我们还发现一个测试用例中定义了 __errno_location 函数的 Stub，是这样写的：
```
extern __thread int __libc_errno __attribute__ ((tls_model ("initial-exec")));
int * __errno_location (void)
{
  return &__libc_errno;
}
```
这一简单的测试用例充分展现了 errno 的实现原理。errno 被定义为 per-thread（用 __thread 标识的线程局部存储类型）变量 __libc_errno，之后 __errno_location 函数返回了这个线程局部变量的地址。所以，在每个线程中获取和设置 errno 的时候，操作的是本线程内的一个变量，不会与其他线程相互干扰。

至于 __thread 这个关键字，需要在很“严苛”的条件下才能生效——需要 Linux 2.6 以上内核、pthreads 库、GCC 3.3 或更高版本的支持。不过，放到今天，这些条件已成为标配，也就不算什么了。

### 6、强行修改const只读变量

修改const只读变量，只能修改局部常量，不能修改全局静态常量。

例如：
```c
// 某函数内部
const int const_value = 100;
int * ptr = (int *)&const_value;
*ptr = 200;
```

因为在函数内部声明的 const_value，其本质上还是一个函数内的局部变量，存储区在该函数的栈帧内，而程序对该内存区拥有修改的权限。

相应地，用同样方法试图修改全局或静态常量数据的值，如下所示：
```c
char * pc = (char *)const_data;
*pc = 'X';
```
编译器并不会报告任何错误，编译可以通过。但当程序运行到第二行代码时，就会因为 Segment Violation 而崩溃，原因在于程序对该位置的内存区没有修改权限。

### 7、fork() 系统调用

fork() 系统调用将创建一个与父进程几乎一样的新进程，之后继续执行下面的指令。程序可以根据 fork() 的返回值，确定当前处于父进程中，还是子进程中——在父进程中，返回值为新创建子进程的进程 ID，在子进程中，返回值是 0。一些使用多进程模型的服务器程序（比如 sshd），就是通过 fork() 系统调用来实现的，每当新用户接入时，系统就会专门创建一个新进程，来服务该用户。

fork() 系统调用所创建的新进程，与其父进程的内存布局和数据几乎一模一样。在内核中，它们的代码段所在的只读存储区会共享相同的物理内存页，可读可写的数据段、堆及栈等内存，内核会使用写时拷贝技术，为每个进程独立创建一份。

在 fork() 系统调用刚刚执行完的那一刻，子进程即可拥有一份与父进程完全一样的数据拷贝。对于已打开的文件，内核会增加每个文件描述符的引用计数，每个进程都可以用相同的文件句柄访问同一个文件。

深入理解了这些底层行为细节，就可以顺理成章地理解 fork() 的一些行为表现和正确使用规范，无需死记硬背，也可获得一些别人踩过坑后才能获得的经验。

比如，使用多进程模型的网络服务程序中，为什么要在子进程中关闭监听套接字，同时要在父进程中关闭新连接的套接字呢？

原因在于 fork() 执行之后，所有已经打开的套接字都被增加了引用计数，在其中任一个进程中都无法彻底关闭套接字，只能减少该文件的引用计数。因此，在 fork() 之后，每个进程立即关闭不再需要的文件是个好的策略，否则很容易导致大量没有正确关闭的文件一直占用系统资源的现象。

再比如，下面这段代码是否存在问题？为什么在输出文件中会出现两行重复的文本？
```c
int main()
{
    FILE * fp = fopen("output.txt", "w");
    fputs("Message in parent\n", fp);
    switch(fork())
    {
    case -1:
        perror("fork failed");
        return -1;
    case 0:
        fputs("Message in Child\n", fp);
        break;
    default:
        break;
    }
    fclose(fp);
    return 0;
}
```
输入文本：
```shell
[root@TealCode process]# cat output.txt
Message in parent
Message in parent
Message in Child
```
原因是 fputs 库函数带有缓冲，fork() 创建的子进程完全拷贝父进程用户空间内存时，fputs 库函数的缓冲区也被包含进来了。所以，fork() 执行之后，子进程同样获得了一份 fputs 缓冲区中的数据，导致“Message in parent”这条消息在子进程中又被输出了一次。要解决这个问题，只需在 fork() 之前，利用 fflush 打开文件即可

### 8、execve() 系统调用
execve() 系统调用的作用是运行另外一个指定的程序。它会把新程序加载到当前进程的内存空间内，当前的进程会被丢弃，它的堆、栈和所有的段数据都会被新进程相应的部分代替，然后会从新程序的初始化代码和 main 函数开始运行。同时，进程的 ID 将保持不变。

execve() 系统调用通常与 fork() 系统调用配合使用。从一个进程中启动另一个程序时，通常是先 fork() 一个子进程，然后在子进程中使用 execve() 变身为运行指定程序的进程。 例如，当用户在 Shell 下输入一条命令启动指定程序时，Shell 就是先 fork() 了自身进程，然后在子进程中使用 execve() 来运行指定的程序。

execve() 系统调用的函数原型为：
```c
int execve(const char *filename, char *const argv[], char *const envp[]);
```
filename 用于指定要运行的程序的文件名，argv 和 envp 分别指定程序的运行参数和环境变量。除此之外，该系列函数还有很多变体，它们执行大体相同的功能，区别在于需要的参数不同，包括 execl、execlp、execle、execv、execvp、execvpe 等。它们的参数意义和使用方法请读者自行查看帮助手册。

需要注意的是，exec 系列函数的返回值只在遇到错误的时候才有意义。如果新程序成功地被执行，那么当前进程的所有数据就都被新进程替换掉了，所以永远也不会有任何返回值。

对于已打开文件的处理，在 exec() 系列函数执行之前，应该确保全部关闭。因为 exec() 调用之后，当前进程就完全变身成另外一个进程了，老进程的所有数据都不存在了。如果 exec() 调用失败，当前打开的文件状态应该被保留下来。让应用层处理这种情况会非常棘手，而且有些文件可能是在某个库函数内部打开的，应用对此并不知情，更谈不上正确地维护它们的状态了。

所以，对于执行 exec() 函数的应用，应该总是使用内核为文件提供的执行时关闭标志（FD_CLOEXEC）。设置了该标志之后，如果 exec() 执行成功，文件就会被自动关闭；如果 exec() 执行失败，那么文件会继续保持打开状态。使用系统调用 fcntl() 可以设置该标志。

### 9、fexecve() 系统调用
glibc 从 2.3.2 版本开始提供 fexecv() 函数，它与 execve() 的区别在于，第一个参数使用的是打开的文件描述符，而非文件路径名。

增加这个函数是为了满足这样的应用需求：有些应用在执行某个程序文件之前，需要先打开文件验证文件内容的校验和，确保文件内容没有被恶意修改过。

在这种情景下，使用 fexecve 是更加安全的方案。组合使用 open() 和 execve() 虽然可以实现同样的功能，但是在打开文件和执行文件之间，存在被执行的程序文件被掉包的可能性。

### 10、监控子进程状态
在 Linux 应用中，父进程需要监控其创建的所有子进程的退出状态，可以通过如下几个系统调用来实现。

> * pid_t wait(int * statua)
  一直阻塞地等待任意一个子进程退出，返回值为退出的子进程的 ID，status 中包含子进程设置的退出标志。
> * pid_t waitpid(pid_t pid, int * status, int options)
可以用 pid 参数指定要等待的进程或进程组的 ID，options 可以控制是否阻塞，以及是否监控因信号而停止的子进程等。
> * int waittid(idtype_t idtype, id_t id, siginfo_t *infop, int options)
提供比 waitpid 更加精细的控制选项来监控指定子进程的运行状态。
> * wait3() 和 wait4() 系统调用
可以在子进程退出时，获取到子进程的资源使用数据。

更详细的信息请参考帮助手册。

要重点说明的是：即使父进程在业务逻辑上不关心子进程的终止状态，也需要使用 wait 类系统调用的底层原因。

这其中的要点在于：在 Linux 的内核实现中，允许父进程在子进程创建之后的任意时刻用 wait() 系列系统调用来确定子进程的状态。

也就是说，如果子进程在父进程调用 wait() 之前就终止了，内核需要保留该子进程的终止状态和资源使用等数据，直到父进程执行 wait() 把这些数据取走。

在子进程终止到父进程获取退出状态之间的这段时间，这个进程会变成所谓的僵尸状态，在该状态下，任何信号都无法结束它。如果系统中存在大量此类僵尸进程，势必会占用大量内核资源，甚至会导致新进程创建失败。

如果父进程也终止，那么 init 进程会接管这些僵尸进程并自动调用 wait ，从而把它们从系统中移除。但是对于长期运行的服务器程序，这一定不是开发者希望看到的结果。所以，父进程一定要仔细维护好它创建的所有子进程的状态，防止僵尸进程的产生。

### 11、进程的终止
正常终止一个进程可以用 _exit 系统调用来实现，原型为：
```c
void _exit(int status);
```
其中的 status 会返回 wait() 类的系统调用。进程退出时会清理掉该进程占用的所有系统资源，包括关闭打开的文件描述符、释放持有的文件锁和内存锁、取消内存映射等，还会给一些子进程发送信号（后面课程再详细展开）。该系统调用一定会成功，永远不会返回。

在退出之前，还希望做一些个性化的清理操作，可以使用库函数 exit() 。函数原型为：
```c
void exit(int status);
```
这个库函数先调用退出处理程序，然后再利用 status 参数调用 _exit() 系统调用。这里的退出处理程序可以通过 atexit() 或 on_exit() 函数注册。其中 atexit() 只能注册返回值和参数都为空的回调函数，而 on_exit() 可以注册带参数的回调函数。退出处理函数的执行顺序与注册顺序相反。它们的函数原型如下所示：
```c
int atexit(void (*func)(void));
int on_exit(void (*func)(int, void *), void *arg);
```
通常情况下，个性化的退出处理函数只会在主进程中执行一次，所以 exit() 函数一般在主进程中使用，而在子进程中只使用 _exit() 系统调用结束当前进程。

### 12、线程同步与互斥锁
相比多进程模型，多线程模型最大的优势在于数据共享非常方便，同一进程内的多个线程可以使用相同的地址值访问同一块内存数据。但是，当多个线程对同一块内存数据执行“读−处理−更新”操作时，会由于线程的交叉执行而造成数据的错误。

例如以下代码段，当 thread_func() 同时在多个线程中执行时，更新到 glob_value 中的值就会互相干扰，产生错误结果。
```c
#define LOOP_COUNT   1000000
int glob_value = 0;

void * thread_func(void * args)
{
    int counter = 0;
    while(counter++ < LOOP_COUNT)
    {
        int local = glob_value;
        local++;
        glob_value = local;
    }
}
```
解决这类问题的关键在于，当一个线程正在执行“读−处理−更新”操作时，保证其他线程不会中途闯入与其交叉执行。不可被打断的执行序列称为临界区，保证多个线程不会交叉执行同一临界区的技术称为线程同步。

#### 12.1 互斥锁的使用
最常用的线程同步技术是互斥锁，Linux 线程库中的相关函数有：
```c
int pthread_mutex_lock(pthread_mutex_t *mutex);
int pthread_mutex_unlock(pthread_mutex_t *mutex);
```
这里pthread的p代表POSIX线程

所有线程都有一个线程号，也就是Thread ID。其类型为pthread_t。通过调用pthread_self()函数可以获得自身的线程号。

pthread_mutex_lock() 负责在进入临界区之前对临界区加锁；
pthread_mutex_unlock() 负责在执行完临界区处理时给临界区解锁。

当某个线程试图给一个已经处在加锁状态的临界区再次加锁时，该线程就会被临时挂起，一直等到该临界区被解锁后，才会被唤醒并继续执行。

如果同时有多个线程等待某个临界区解锁，那下次被唤醒的进程取决于内核的调度策略，并没有固定的顺序。

静态分配的 mutex 变量在使用之前应该被初始化为 PTHREAD_MUTEX_INITIALIZER，而动态分配的 mutex 需要调用 pthread_mutex_init() 进行初始化，且只被某个线程初始化一次，可以利用 pthread_once() 函数方便完成。
```c
int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr);
int pthread_once(pthread_once_t *once_control, void (*init_routine)(void));
```
多个线程在临界区上的执行是串行的，开发者应该尽量减少程序在临界区内的停留时间，以提高程序的并行性。因此，临界区不应该包含任何非必须的逻辑，以及任何可能带来高延迟的 IO 等操作

#### 12.2 互斥锁的保护范围和使用顺序
对互斥锁加锁的不恰当使用会造成线程的死锁，比如下面这两种情况。

> 1. 典型的情况是，两个线程执行时都需要锁定互斥锁 A 和 B，在一个线程中，锁定顺序是先锁定 A，后锁定 B，而另一个线程的锁定顺序是先锁定 B，再锁定 A。这种情况下，当一个线程已经锁定了 A 而另一个线程恰好锁定了 B 时，双方因互相争用对方已锁定的互斥锁，谁也不让步，而陷入死锁状态。

> 2. 另一种情况是，一个线程已经锁定了互斥锁 A，但在其后的处理逻辑中试图再次锁定 A，这时该线程会让自己陷入睡眠状态，再也等不到被唤醒的时候。

因此，开发者需要仔细规划互斥锁保护范围和使用顺序

#### 12.2 避免死锁的两个加锁函数
为了避免出现死锁问题，可以使用另外两种变体的锁定函数，如下所示：
```c
int pthread_mutex_trylock(pthread_mutex_t *mutex);
int pthread_mutex_timedlock(pthread_mutex_t *restrict mutex, const struct timespec *restrict abs_timeout);
```
前者可以在锁定失败后立即返回，后者可以在一段超时时间后返回，

应用这两个函数可以处理这种错误情况，而避免陷入无限的死锁中。

在 Linux 中，实现互斥锁采用的是 Futex（Fast Userspace Mutex）方案。在该实现中，只有发生了锁的争用才需要陷入到内核空间中处理，否则所有的操作都可以在用户空间内快速完成。在大多数情况下，互斥锁本身的效率很高，其平均开销大约相当于几十次内存读写和算数运算所花费的时间。

### 13、线程的连接和分离

新创建的线程和进程一样，也需要被连接以监听其退出状态，否则也会变成僵尸线程。背后原因与进程一样，其退出之后，内核会为它保留退出状态数据，直到有人取走为止。连接线程的库函数如下所示：
```c
int pthread_join(pthread_t thread, void **retval);
```
进程连接与线程连接在以下几个方面存在一些区别：

> 任何线程都可以监听一个指定线程的退出，而不需要是创建该线程的线程；

> 线程连接函数只能连接一个指定ID的线程，而不能像进程一样监听任意线程的退出；

> **线程创建之后可以使用分离函数设置其不需要等待被连接，这种情况下，线程结束之后会被自动清理。**

设置线程分离的函数为：
```c
int pthread_detach(pthread_t thread);
```
处于分离状态的线程，无法被任何线程执行连接获取其状态，也无法再返回到可连接状态。

----

pthread_join使一个线程等待另一个线程结束。

代码中如果没有pthread_join主线程会很快结束从而使整个进程结束，从而使创建的线程没有机会开始执行就结束了。加入pthread_join后，主线程会一直等待直到等待的线程结束自己才结束，使创建的线程有机会执行。

### 14、pthread线程库函数
所有线程都有一个线程号，也就是Thread ID。其类型为pthread_t。通过调用pthread_self()函数可以获得自身的线程号。
#### 14.1 创建线程
创建线程的函数如下：
```
int pthread_create(pthread_t *restrict thread, const pthread_attr_t *restrict attr, void *(*start_routine)(void*), void *restrict arg);
```
>thread：所创建的线程号。
>attr：所创建的线程属性，这个将在后面详细说明。
>start_routine：即将运行的线程函数。
>art：传递给线程函数的参数。

在编译的时候需要注意，由于线程创建函数在libpthread.so库中，所以在编译命令中需要将该库导入。命令如下：
```
gcc –o createthread –lpthread createthread.c
```

如果想传递参数给线程函数，可以通过其参数arg，其类型是void *。如果你需要传递多个参数的话，可以考虑将这些参数组成一个结构体来传递。另外，由于类型是void *，所以你的参数不可以被提前释放掉。

```c
pthread_t thread1_id;
struct char_print_parms thread1_args;
/* Create a new thread to print 30,000 x’s. */
thread1_args.character = ’x’;
thread1_args.count = 30000;
pthread_create (&thread1_id, NULL, &char_print, &thread1_args);
```
char_print是一个函数名字

#### 14.2 连接线程并等待线程退出
```
int pthread_join(pthread_t thread, void **value_ptr);
```
>thread：等待退出线程的线程号。
>value_ptr：退出线程的返回值。

可以通过pthread_join()函数来使主线程阻塞等待其他线程退出，这样主线程可以清理其他线程的环境。
#### 14.2 detacted 线程

有一些线程，更喜欢自己来清理退出的状态，他们也不愿意主线程调用pthread_join来等待他们。我们将这一类线程的属性称为detached。如果我们在调用pthread_create()函数的时候将属性设置为NULL，则表明我们希望所创建的线程采用默认的属性，也就是jionable。

在线程设置为joinable后，可以调用pthread_detach()使之成为detached。但是相反的操作则不可以。还有，如果线程已经调用pthread_join()后，则再调用pthread_detach()则不会有任何效果。

也可以在创建线程的时候直接设置线程属性为detached
```c
pthread_t thread_id;
pthread_attr_t attr;
pthread_attr_init(&attr);
pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
pthread_create(&thread_id,&attr,start_run,NULL);
pthread_attr_destroy(&attr);
```
#### 14.3 线程结束
线程可以通过自身执行结束来结束，也可以通过调用pthread_exit()来结束线程的执行。另外，线程甲可以被线程乙被动结束。这个通过调用pthread_cancel()来达到目的。
```c
int pthread_cancel(pthread_t thread);
```
函数调用成功返回0。
当然，线程也不是被动的被别人结束。它可以通过设置自身的属性来决定如何结束。

线程的被动结束分为两种，一种是异步终结，另外一种是同步终结。异步终结就是当其他线程调用pthread_cancel的时候，线程就立刻被结束。而同步终结则不会立刻终结，它会继续运行，直到到达下一个结束点（cancellation point）。当一个线程被按照默认的创建方式创建，那么它的属性是同步终结。

通过调用pthread_setcanceltype()来设置终结状态。
```
int pthread_setcanceltype(int type, int *oldtype);
```
type：要设置的状态，可以为PTHREAD_CANCEL_DEFERRED或者为PTHREAD_CANCEL_ASYNCHRONOUS。

那么前面提到的结束点又是如何设置了？最常用的创建终结点就是调用pthread_testcancel()的地方。该函数除了检查同步终结时的状态，其他什么也不做。
上面一个函数是用来设置终结状态的。还可以通过下面的函数来设置终结类型，即该线程可不可以被终结：
```
int pthread_setcancelstate(int state, int *oldstate);
```
state：终结状态，可以为PTHREAD_CANCEL_DISABLE或者PTHREAD_CANCEL_ENABLE。具体什么含义通过单词意思即可明白。

#### 14.4 linux线程本质

线程的本质。其实在Linux中，新建的线程并不是在原先的进程中，而是系统通过一个系统调用clone()。该系统copy了一个和原先进程完全一样的进程，并在这个进程中执行线程函数。不过这个copy过程和fork不一样。copy后的进程和原先的进程共享了所有的变量，运行环境。这样，原先进程中的变量变动在copy后的进程中便能体现出来。

### 15、进程内存布局

进程访问的地址是自己进程空间内的线性地址，内核负责把线性地址映射为实际的物理地址。

操作系统以内存页为单位管理物理内存。在 Linux 中，默认的内存分页大小是 4KB，也就是说，操作系统把物理内存分割成一个个大小为 4KB 的格子，进而管理它们，内存的换入换出也以这样的格子为基本单位。

在每个进程的内核数据结构中，都会维护一个内存页表，记录线性地址到物理内存页的映射关系。

### 16、静态库和共享库（动态链接库）的区别

#### 16.1 静态库
在共享库出现之前，公用功能是以静态库的形式存在的，它把通用功能模块的多个目标文件打包在一起，用到它的程序只需要在链接时指定这个库文件，链接器就会从这个库中抽取出用到的功能代码拷贝到目标程序中，而不需要每次都对这些通用功能代码重新编译。

静态库体现出了很好的模块化思想，但是随着计算机产业规模的发展，静态库逐渐暴露出了自身两个比较严重的问题。

> 一是磁盘和内存空间占用大。静态库虽然加快了编译速度，提高了不同部门间的协作效率，但是在每个与静态库链接的程序中，都会保存一份引用到的通用功能代码的拷贝，而且在运行时，每一份拷贝都要占用相应的物理内存。
> 二是库的版本升级非常麻烦。一旦公用库有修改，每个引用到它的程序都需要与新版本的库重新链接。在库与应用是由不同的公司或组织维护的场景下，升级工作将变得异常复杂。通用库中如果有 Bug 修复，使用该库的所有应用都需要分别升级。

#### 16.2 共享库
为了解决这两个问题，共享库技术应运而生。

首先，使用共享库的应用在编译链接时，并不把库中的功能代码拷贝到目标文件中，而只在目标文件中记录一条引用信息，标记引用到的库函数，直到程序运行时才由动态链接器去定位功能代码的位置，因此生成的可执行程序的体积得以明显地减小。

其次，每个共享库在物理内存中只有一份副本，多个应用会在各自的虚拟地址空间内映射这同一份可执行文件，因此可以节省可观的内存空间。

共享库的这种工作方式大大方便了库的升级，当共享库发布新版本时，用户只需要升级这个共享库，所有使用这个库的应用就可以自动获得新库中的特性或 Bug 修复，而不需要单独升级每个应用。

### 17、Linux 共享库命名规则
linux 里面共享库总是存在很多链接文件

```shell
lrwxrwxrwx. 1 root root        24 Aug 19  2017 libavahi-client.so.3 -> libavahi-client.so.3.2.9
-rwxr-xr-x. 1 root root     69968 Nov  5  2016 libavahi-client.so.3.2.9
lrwxrwxrwx. 1 root root        24 Aug 19  2017 libavahi-common.so.3 -> libavahi-common.so.3.5.3
-rwxr-xr-x. 1 root root     53848 Nov  5  2016 libavahi-common.so.3.5.3
lrwxrwxrwx. 1 root root        22 Aug 19  2017 libavahi-core.so.7 -> libavahi-core.so.7.0.2
-rwxr-xr-x. 1 root root    220776 Nov  5  2016 libavahi-core.so.7.0.2
lrwxrwxrwx. 1 root root        22 Aug 19  2017 libavahi-glib.so.1 -> libavahi-glib.so.1.0.2
-rwxr-xr-x. 1 root root     15672 Nov  5  2016 libavahi-glib.so.1.0.2
lrwxrwxrwx. 1 root root        25 Aug 19  2017 libavahi-gobject.so.0 -> libavahi-gobject.so.0.0.4
-rwxr-xr-x. 1 root root     49672 Nov  5  2016 libavahi-gobject.so.0.0.4
lrwxrwxrwx. 1 root root        25 Aug 19  2017 libavahi-ui-gtk3.so.0 -> libavahi-ui-gtk3.so.0.1.4
-rwxr-xr-x. 1 root root     54000 Nov  5  2016 libavahi-ui-gtk3.so.0.1.4
```

在 Linux 中，共享库文件的命名规则为 libname.so.x.y.z。

其中，lib 是共享库文件的固定前缀，而后面的 x、y、z 分别是主版本号、次版本号和发布版本号。

>主版本号不同的共享库是不能相互兼容的。
>如果主版本号相同，而次版本号有升级，表示这个共享库有新接口的添加，但是同时所有老的接口和行为表现都保持不变，是向后兼容的。
>如果主次版本号都相同，只有发布版本号不同，则表示只有 Bug 修复和性能优化，对外的接口和表现都完全相同。

创建共享库时可以通过 -soname 参数指定共享库的别名，通常使用带主版本号的库名称作为共享库的别名，如我们的例子中，可以使用下面的命令生成发布的共享库：

```
gcc -fPIC -shared -soname,libshared.so.1 -o libshared.so.1.0.0 shared.c
```

这样生成的共享库的 ELF 文件中会带有 DT_SONAME 的标签。当指定一个带 SONAME 的共享库与应用进行链接时，链接器会把该 SONAME 放入生成的可执行文件的 .dynamic 段中的 NEEDED 项中，以告知动态链接器该程序需要的库的兼容版本。例如上面我们看到的 libshared.so 依赖的 C 库，就是带主版本号的形式 libc.so.6。

### 18. 用数组名作函数参数

关于用数组名作函数参数有两点要说明:

(1) 如果函数实参是数组名,形参也应为数组名(或指针变量),形参不能声明为普通变量(如int array;)。实参数组与形参数组类型应一致(现都为int型),如不一致,结果将出错。

(2) 需要特别说明的是: 数组名代表数组首元素的地址,并不代表数组中的全部元素。因此用数组名作函数实参时,不是把实参数组的值传递给形参,而只是将实参数组首元素的地址传递给形参。形参可以是数组名,也可以是指针变量,它们用来接收实参传来的地址。如果形参是数组名,它代表的是形参数组首元素的地址。在调用函数时,将实参数组首元素的地址传递给形参数组名。这样,实参数组和形参数组就共占同一段内存单元

==**声明形参数组并不意味着真正建立一个包含若干元素的数组,在调用函数时也不对它分配存储单元,只是用array[]这样的形式表示array是一维数组名,以接收实参传来的地址。因此array[]中方括号内的数值并无实际作用,编译系统对一维数组方括号内的内容不予处理。形参一维数组的声明中可以写元素个数,也可以不写。**==

C++实际上只把形参数组名作为一个指针变量来处理,用来接收从 实参传过来的地址

如果用二维数组名作为实参和形参,在对形参数组声明时,必须指定第二维(即列)的大小,且应与实 参的第二维的大小相同。第一维的大小可以指定,
也可以不指定。如
int array[3][10]; //形参数组的两个维都指定或
int array[][10]; //第一维大小省略

### 19、浮点数比较大小

float 类型不能比较相等或不等，但可以比较>,<,>=,<=

用\==从语法上说没错，但是本来应该相等的两个浮点数由于计算机内部表示的原因可能略有微小的误差，这时用==就会认为它们不等。应该使用两个浮点数之间的差异的绝对值小于某个可以接受的值来判断判断它们是否相等,比如用
```
    if (fabs(price - p) < 0.000001)
```
来代替
```
    if (price == p)
```

很多工程的应用都是采用计算精确度的方式

定义一个精度，用差的绝对值比较，在精度范围内就认为是相等的；大小可以直接比较。

【规则4-3-3】不可将浮点变量用“\==”或“！=”与任何数字比较。

千万要留意，无论是float 还是double 类型的变量，都有精度限制。所以一定要
避免将浮点变量用“==”或“！=”与数字比较，应该设法转化成“>=”或“<=”形式。
假设浮点变量的名字为x，应当将
```
if (x == 0.0) // 隐含错误的比较
```
转化为
```
if ((x>=-EPSINON) && (x<=EPSINON))
```
其中EPSINON 是允许的误差（即精度）。

### 20、c与c++混合编程
#### 20.1 引言
在用C++的项目源码中，经常会不可避免的会看到下面的代码：
```
#ifdef __cplusplus
extern "C" {
#endif

/*...*/

#ifdef __cplusplus
}
#endif
```
它到底有什么用呢，你知道吗？而且这样的问题经常会出现在面试or笔试中。下面我就从以下几个方面来介绍它：

>1. #ifdef _cplusplus/#endif _cplusplus及发散
>2. extern "C"
>2.1. extern关键字
>2.2. "C"
>2.3. 小结extern "C"
>3. C和C++互相调用
>3.1. C++的编译和连接
>3.2. C的编译和连接
>3.3. C++中调用C的代码
>3.4. C中调用C++的代码
>4. C和C++混合调用特别之处函数指针

#### 20.2 #ifdef _cplusplus/#endif _cplusplus及发散

在介绍extern "C"之前，我们来看下#ifdef _cplusplus/#endif _cplusplus的作用。

很明显#ifdef/#endif、#ifndef/#endif用于条件编译

 #ifdef _cplusplus/#endif _cplusplus——表示如果定义了宏_cplusplus，就执行#ifdef/#endif之间的语句，否则就不执行。

在这里为什么需要#ifdef _cplusplus/#endif _cplusplus呢？因为C语言中不支持extern "C"声明，如果你明白extern "C"的作用就知道在C中也没有必要这样做，这就是条件编译的作用！在.c文件中包含了extern "C"时会出现编译时错误。

既然说到了条件编译，我就介绍它的一个重要应用——避免重复包含头文件。还记得腾讯笔试就考过这个题目，给出类似下面的代码（下面是我最近在研究的一个开源web服务器——Mongoose的头文件mongoose.h中的一段代码）：

```
#ifndef MONGOOSE_HEADER_INCLUDED
#define    MONGOOSE_HEADER_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*.................................
 * do something here
 *.................................
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MONGOOSE_HEADER_INCLUDED */
```
然后叫你说明上面宏#ifndef/#endif的作用？为了解释一个问题，我们先来看两个事实：

这个头文件mongoose.h可能在项目中被多个源文件包含（#include "mongoose.h"），而对于一个大型项目来说，这些冗余可能导致错误，因为一个头文件包含类定义或inline函数，在一个源文件中mongoose.h可能会被#include两次（如，a.h头文件包含了mongoose.h，而在b.c文件中#include a.h和mongoose.h）——这就会出错（在同一个源文件中一个结构体、类等被定义了两次）。

从逻辑观点和减少编译时间上，都要求去除这些冗余。然而让程序员去分析和去掉这些冗余，不仅枯燥且不太实际，最重要的是有时候又需要这种冗余来保证各个模块的独立。
为了解决这个问题，上面代码中的
```
#ifndef MONGOOSE_HEADER_INCLUDED
#define    MONGOOSE_HEADER_INCLUDED
/*……………………………*/
#endif /* MONGOOSE_HEADER_INCLUDED */
```
就起作用了。如果定义了MONGOOSE_HEADER_INCLUDED，#ifndef/#endif之间的内容就被忽略掉。因此，编译时第一次看到mongoose.h头文件，它的内容会被读取且给定MONGOOSE_HEADER_INCLUDED一个值。之后再次看到mongoose.h头文件时，MONGOOSE_HEADER_INCLUDED就已经定义了，mongoose.h的内容就不会再次被读取了。

#### 20.3、extern "C"
首先从字面上分析extern "C"，它由两部分组成——extern关键字、"C"。下面我就从这两个方面来解读extern "C"的含义。

extern关键字：

在一个项目中必须保证函数、变量、枚举等在所有的源文件中保持一致，除非你指定定义为局部的。首先来一个例子：

```
//file1.c:
    int x=1;
    int f(){do something here}
//file2.c:
    extern int x;
    int f();
    void g(){x=f();}
```

在file2.c中g()使用的x和f()是定义在file1.c中的。extern关键字表明file2.c中x，仅仅是一个变量的声明，其并不是在定义变量x，并未为x分配内存空间。变量x在所有模块中作为一种全局变量只能被定义一次，否则会出现连接错误。但是可以声明多次，且声明必须保证类型一致，如：

```
//file1.c:
    int x=1;
    int b=1;
    extern c;
//file2.c:
    int x;// x equals to default of int type 0
    int f();
    extern double b;
    extern int c;
```
在这段代码中存在着这样的三个错误：

>x被定义了两次
>b两次被声明为不同的类型
>c被声明了两次，但却没有定义

回到extern关键字，extern是C/C++语言中表明函数和全局变量作用范围（可见性）的关键字，该关键字告诉编译器，其声明的函数和变量可以在本模块或其它模块中使用。通常，在模块的头文件中对本模块提供给其它模块引用的函数和全局变量以关键字extern声明。例如，如果模块B欲引用该模块A中定义的全局变量和函数时只需包含模块A的头文件即可。这样，模块B中调用模块A中的函数时，在编译阶段，模块B虽然找不到该函数，但是并不会报错；它会在连接阶段中从模块A编译生成的目标代码中找到此函数。

与extern对应的关键字是 static，被它修饰的全局变量和函数只能在本模块中使用。因此，一个函数或变量只可能被本模块使用时，其不可能被extern “C”修饰。

#### 20.4、"C"

典型的，一个C++程序包含其它语言编写的部分代码。类似的，C++编写的代码片段可能被使用在其它语言编写的代码中。不同语言编写的代码互相调用是困难的，甚至是同一种编写的代码但不同的编译器编译的代码。例如，不同语言和同种语言的不同实现可能会在注册变量保持参数和参数在栈上的布局，这个方面不一样。

为了使它们遵守统一规则，可以使用extern指定一个编译和连接规约。例如，声明C和C++标准库函数strcpy()，并指定它应该根据C的编译和连接规约来链接：
```
extern "C" char* strcpy(char*,const char*);
```
注意它与下面的声明的不同之处：
```
extern char* strcpy(char*,const char*);
```
下面的这个声明仅表示在连接的时候调用strcpy()。

extern "C"指令非常有用，因为C和C++的近亲关系。注意：extern "C"指令中的C，表示的一种编译和连接规约，而不是一种语言。C表示符合C语言的编译和连接规约的任何语言，如Fortran、assembler等。

还有要说明的是，extern "C"指令仅指定编译和连接规约，但不影响语义。例如在函数声明中，指定了extern "C"，仍然要遵守C++的类型检测、参数转换规则。

再看下面的一个例子，为了声明一个变量而不是定义一个变量，你必须在声明时指定extern关键字，但是当你又加上了"C"，它不会改变语义，但是会改变它的编译和连接方式。

如果你有很多语言要加上extern "C"，你可以将它们放到extern "C"{ }中。

#### 20.5、小结extern "C"
通过上面两节的分析，我们知道extern "C"的真实目的是实现类C和C++的混合编程。在C++源文件中的语句前面加上extern "C"，表明它按照类C的编译和连接规约来编译和连接，而不是C++的编译的连接规约。这样在类C的代码中就可以调用C++的函数or变量等。（注：我在这里所说的类C，代表的是跟C语言的编译和连接方式一致的所有语言）

#### 20.6、C和C++互相调用
我们既然知道extern "C"是实现的类C和C++的混合编程。下面我们就分别介绍如何在C++中调用C的代码、C中调用C++的代码。首先要明白C和C++互相调用，你得知道它们之间的编译和连接差异，及如何利用extern "C"来实现相互调用。

##### 20.6.1、C++的编译和连接
C++是一个面向对象语言（虽不是纯粹的面向对象语言），它支持函数的重载，重载这个特性给我们带来了很大的便利。为了支持函数重载的这个特性，C++编译器实际上将下面这些重载函数：
```
void print(int i);
void print(char c);
void print(float f);
void print(char* s);
```
编译为：
```
_print_int
_print_char
_print_float
_pirnt_string
```
这样的函数名，来唯一标识每个函数。注：不同的编译器实现可能不一样，但是都是利用这种机制。所以当连接是调用print(3)时，它会去查找_print_int(3)这样的函数。下面说个题外话，正是因为这点，重载被认为不是多态，多态是运行时动态绑定（“一种接口多种实现”），如果硬要认为重载是多态，它顶多是编译时“多态”。

C++中的变量，编译也类似，如全局变量可能编译g_xx，类变量编译为c_xx等。连接是也是按照这种机制去查找相应的变量。

##### 20.6.1、C的编译和连接
C语言中并没有重载和类这些特性，故并不像C++那样print(int i)，会被编译为_print_int，而是直接编译为_print等。因此如果直接在C++中调用C的函数会失败，因为连接是调用C中的print(3)时，它会去找_print_int(3)。因此extern "C"的作用就体现出来了。

##### 20.6.2、C++中调用C的代码
假设一个C的头文件cHeader.h中包含一个函数print(int i)，为了在C++中能够调用它，必须要加上extern关键字（原因在extern关键字那节已经介绍）。它的代码如下：
```
#ifndef C_HEADER
#define C_HEADER

extern void print(int i);

#endif C_HEADER
```

相对应的实现文件为cHeader.c的代码为：

```
#include <stdio.h>
#include "cHeader.h"
void print(int i)
{
    printf("cHeader %d\n",i);
}
```
现在C++的代码文件C++.cpp中引用C中的print(int i)函数：
```
extern "C"{
#include "cHeader.h"
}

int main(int argc,char** argv)
{
    print(3);
    return 0;
}
```

##### 20.6.3、C中调用C++的代码
现在换成在C中调用C++的代码，这与在C++中调用C的代码有所不同。如下在cppHeader.h头文件中定义了下面的代码：
```
#ifndef CPP_HEADER
#define CPP_HEADER

extern "C" void print(int i);

#endif CPP_HEADER
```
相应的实现文件cppHeader.cpp文件中代码如下：
```
#include "cppHeader.h"

#include <iostream>
using namespace std;
void print(int i)
{
    cout<<"cppHeader "<<i<<endl;
}
```
在C的代码文件c.c中调用print函数：

```
extern void print(int i);
int main(int argc,char** argv)
{
    print(3);
    return 0;
}
```
注意在C的代码文件中直接#include "cppHeader.h"头文件，编译出错。而且如果不加extern int print(int i)编译也会出错。

### 21、字节对齐问题详述
文章最后本人做了一幅图，一看就明白了，这个问题网上讲的不少，但是都没有把问题说透。

一、概念
　　
　　 对齐跟数据在内存中的位置有关。如果一个变量的内存地址正好位于它长度的整数倍，他就被称做自然对齐。比如在32位cpu下，假设一个整型变量的地址为0x00000004，那它就是自然对齐的。
　　
二、为什么要字节对齐
　　
　　 需要字节对齐的根本原因在于CPU访问数据的效率问题。假设上面整型变量的地址不是自然对齐，比如为0x00000002，则CPU如果取它的值的话需要访问两次内存，第一次取从0x00000002-0x00000003的一个short，第二次取从0x00000004-0x00000005的一个short然后组合得到所要的数据，如果变量在0x00000003地址上的话则要访问三次内存，第一次为char，第二次为short，第三次为char，然后组合得到整型数据。而如果变量在自然对齐位置上，则只要一次就可以取出数据。一些系统对对齐要求非常严格，比如sparc系统，如果取未对齐的数据会发生错误，举个例：
　　
　　char ch[8];
　　char *p = &ch[1];
　　int i = *(int *)p;
　　
　　
　　运行时会报segment error，而在x86上就不会出现错误，只是效率下降。
　　
三、正确处理字节对齐
　　
　　 对于标准数据类型，它的地址只要是它的长度的整数倍就行了，而非标准数据类型按下面的原则对齐：
　　
　　数组 ：按照基本数据类型对齐，第一个对齐了后面的自然也就对齐了。
　　联合 ：按其包含的长度最大的数据类型对齐。
　　结构体： 结构体中每个数据类型都要对齐。
　　比如有如下一个结构体：
　　
　　struct stu{
　　 char sex;
　　 int length;
　　 char name[10];
　　};
　　struct stu my_stu;
　　
　　
　　由于在x86下，GCC默认按4字节对齐，它会在sex后面跟name后面分别填充三个和两个字节使length和整个结构体对齐。于是我们sizeof(my_stu)会得到长度为20，而不是15.

　　四、__attribute__选项
　　
　　我们可以按照自己设定的对齐大小来编译程序，GNU使用__attribute__选项来设置，比如我们想让刚才的结构按一字节对齐，我们可以这样定义结构体

```
　　struct stu{
　　 char sex;
　　 int length;
　　 char name[10];
　　}__attribute__ ((aligned (1)));
　　
　　struct stu my_stu;
```
　　
　　则sizeof(my_stu)可以得到大小为15。
　　
　　上面的定义等同于
```　　
　　struct stu{
　　 char sex;
　　 int length;
　　 char name[10];
　　}__attribute__ ((packed));
　　struct stu my_stu;
```
　　\__attribute__((packed))得变量或者结构体成员使用最小的对齐方式，即对变量是一字节对齐，对域（field）是位对齐.
　　
五、什么时候需要设置对齐
　　
　　 在设计不同CPU下的通信协议时，或者编写硬件驱动程序时寄存器的结构这两个地方都需要按一字节对齐。即使看起来本来就自然对齐的也要使其对齐，以免不同的编译器生成的代码不一样.

----

一、快速理解

1. 什么是字节对齐？

在C语言中，结构是一种复合数据类型，其构成元素既可以是基本数据类型（如int、long、float等）的变量，也可以是一些复合数据类型（如数组、结构、联合等）的数据单元。在结构中，编译器为结构的每个成员按其自然边界（alignment）分配空间。各个成员按照它们被声明的顺序在内存中顺序存储，第一个成员的地址和整个结构的地址相同。

为了使CPU能够对变量进行快速的访问,变量的起始地址应该具有某些特性,即所谓的”对齐”. 比如4字节的int型,其起始地址应该位于4字节的边界上,即起始地址能够被4整除.

2. 字节对齐有什么作用？

字节对齐的作用不仅是便于cpu快速访问，同时合理的利用字节对齐可以有效地节省存储空间。

对于32位机来说，4字节对齐能够使cpu访问速度提高，比如说一个long类型的变量，如果跨越了4字节边界存储，那么cpu要读取两次，这样效率就低了。但是在32位机中使用1字节或者2字节对齐，反而会使变量访问速度降低。所以这要考虑处理器类型，另外还得考虑编译器的类型。在vc中默认是4字节对齐的，GNU gcc 也是默认4字节对齐。

3. 更改C编译器的缺省字节对齐方式

在缺省情况下，C编译器为每一个变量或是数据单元按其自然对界条件分配空间。一般地，可以通过下面的方法来改变缺省的对界条件：
· 使用伪指令#pragma pack (n)，C编译器将按照n个字节对齐。
· 使用伪指令#pragma pack ()，取消自定义字节对齐方式。

另外，还有如下的一种方式：
· __attribute((aligned (n)))，让所作用的结构成员对齐在n字节自然边界上。如果结构中有成员的长度大于n，则按照最大成员的长度来对齐。
· __attribute__ ((packed))，取消结构在编译过程中的优化对齐，按照实际占用字节数进行对齐。

4. 举例说明

例1
```
struct test
{
char x1;
short x2;
float x3;
char x4;
};
```
由于编译器默认情况下会对这个struct作自然边界（有人说“自然对界”我觉得边界更顺口）对齐，结构的第一个成员x1，其偏移地址为0，占据了第1个字节。第二个成员x2为short类型，其起始地址必须2字节对界，因此，编译器在x2和x1之间填充了一个空字节。结构的第三个成员x3和第四个成员x4恰好落在其自然边界地址上，在它们前面不需要额外的填充字节。在test结构中，成员x3要求4字节对界，是该结构所有成员中要求的最大边界单元，因而test结构的自然对界条件为4字节，编译器在成员x4后面填充了3个空字节。整个结构所占据空间为12字节。

例2
```
#pragma pack(1) //让编译器对这个结构作1字节对齐
struct test
{
char x1;
short x2;
float x3;
char x4;
};
#pragma pack() //取消1字节对齐，恢复为默认4字节对齐
```
这时候sizeof(struct test)的值为8。

例3
```
#define GNUC_PACKED __attribute__((packed))
struct PACKED test
{
char x1;
short x2;
float x3;
char x4;
}GNUC_PACKED;
```
这时候sizeof(struct test)的值仍为8。

----

二、深入理解

一、什么是字节对齐,为什么要对齐?
TragicJun 发表于 2006-9-18 9:41:00 现代计算机中内存空间都是按照byte划分的，从理论上讲似乎对任何类型的变量的访问可以从任何地址开始，但实际情况是在访问特定类型变量的时候经常在特定的内存地址访问，这就需要各种类型数据按照一定的规则在空间上排列，而不是顺序的一个接一个的排放，这就是对齐。
      对齐的作用和原因：各个硬件平台对存储空间的处理上有很大的不同。一些平台对某些特定类型的数据只能从某些特定地址开始存取。比如有些架构的CPU在访问一个没有进行对齐的变量的时候会发生错误,那么在这种架构下编程必须保证字节对齐.其他平台可能没有这种情况，但是最常见的是如果不按照适合其平台要求对数据存放进行对齐，会在存取效率上带来损失。比如有些平台每次读都是从偶地址开始，如果一个int型（假设为32位系统）如果存放在偶地址开始的地方，那么一个读周期就可以读出这32bit，而如果存放在奇地址开始的地方，就需要2个读周期，并对两次读出的结果的高低字节进行拼凑才能得到该32bit数据。显然在读取效率上下降很多。

二.字节对齐对程序的影响:

先让我们看几个例子吧(32bit,x86环境,gcc编译器):
设结构体如下定义：
```
struct A
{
        int a;
        char b;
        short c;
};
struct B
{
        char b;
        int a;
        short c;
};
```
现在已知32位机器上各种数据类型的长度如下:
char:1(有符号无符号同)   
short:2(有符号无符号同)   
int:4(有符号无符号同)   
long:4(有符号无符号同)   
float:4        double:8
那么上面两个结构大小如何呢?
结果是:
sizeof(strcut A)值为8
sizeof(struct B)的值却是12

结构体A中包含了4字节长度的int一个，1字节长度的char一个和2字节长度的short型数据一个,B也一样;按理说A,B大小应该都是7字节。
之所以出现上面的结果是因为编译器要对数据成员在空间上进行对齐。上面是按照编译器的默认设置进行对齐的结果,那么我们是不是可以改变编译器的这种默认对齐设置呢,当然可以.例如:
```
#pragma pack (2) /*指定按2字节对齐*/
struct C
{
        char b;
        int a;
        short c;
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
```
sizeof(struct C)值是8。
修改对齐值为1：
```
#pragma pack (1) /*指定按1字节对齐*/
struct D
{
        char b;
        int a;
        short c;
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
```
sizeof(struct D)值为7。
后面我们再讲解#pragma pack()的作用.

三.编译器是按照什么样的原则进行对齐的?

先让我们看四个重要的基本概念：

1.数据类型自身的对齐值：
      对于char型数据，其自身对齐值为1，对于short型为2，对于int,float,double类型，其自身对齐值为4，单位字节。
2.结构体或者类的自身对齐值：其成员中自身对齐值最大的那个值。
3.指定对齐值：#pragma pack (value)时的指定对齐值value。
4.数据成员、结构体和类的有效对齐值：自身对齐值和指定对齐值中小的那个值。
有了这些值，我们就可以很方便的来讨论具体数据结构的成员和其自身的对齐方式。有效对齐值N是最终用来决定数据存放地址方式的值，最重要。有效对齐N，就是表示“对齐在N上”，也就是说该数据的"存放起始地址%N=0".而数据结构中的数据变量都是按定义的先后顺序来排放的。第一个数据变量的起始地址就是数据结构的起始地址。结构体的成员变量要对齐排放，结构体本身也要根据自身的有效对齐值圆整(就是结构体成员变量占用总长度需要是对结构体有效对齐值的整数倍，结合下面例子理解)。这样就不能理解上面的几个例子的值了。
例子分析：
分析例子B；
```
struct B
{
        char b;
        int a;
        short c;
};
```
假设B从地址空间0x0000开始排放。该例子中没有定义指定对齐值，在笔者环境下，该值默认为4。第一个成员变量b的自身对齐值是1，比指定或者默认指定对齐值4小，所以其有效对齐值为1，所以其存放地址0x0000符合0x0000%1=0.第二个成员变量a，其自身对齐值为4，所以有效对齐值也为4，所以只能存放在起始地址为0x0004到0x0007这四个连续的字节空间中，复核0x0004%4=0,且紧靠第一个变量。第三个变量c,自身对齐值为2，所以有效对齐值也是2，可以存放在0x0008到0x0009这两个字节空间中，符合0x0008%2=0。所以从0x0000到0x0009存放的都是B内容。再看数据结构B的自身对齐值为其变量中最大对齐值(这里是b）所以就是4，所以结构体的有效对齐值也是4。根据结构体圆整的要求，0x0009到0x0000=10字节，（10＋2）％4＝0。所以0x0000A到0x000B也为结构体B所占用。故B从0x0000到0x000B共有12个字节,sizeof(struct B)=12;其实如果就这一个就来说它已将满足字节对齐了,因为它的起始地址是0,因此肯定是对齐的,之所以在后面补充2个字节,是因为编译器为了实现结构数组的存取效率,试想如果我们定义了一个结构B的数组,那么第一个结构起始地址是0没有问题,但是第二个结构呢?按照数组的定义,数组中所有元素都是紧挨着的,如果我们不把结构的大小补充为4的整数倍,那么下一个结构的起始地址将是0x0000A,这显然不能满足结构的地址对齐了,因此我们要把结构补充成有效对齐大小的整数倍.其实诸如:对于char型数据，其自身对齐值为1，对于short型为2，对于int,float,double类型，其自身对齐值为4，这些已有类型的自身对齐值也是基于数组考虑的,只是因为这些类型的长度已知了,所以他们的自身对齐值也就已知了.
同理,分析上面例子C：
```
#pragma pack (2) /*指定按2字节对齐*/
struct C
{
        char b;
        int a;
        short c;
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
```
第一个变量b的自身对齐值为1，指定对齐值为2，所以，其有效对齐值为1，假设C从0x0000开始，那么b存放在0x0000，符合0x0000%1=0;第二个变量，自身对齐值为4，指定对齐值为2，所以有效对齐值为2，所以顺序存放在0x0002、0x0003、0x0004、0x0005四个连续字节中，符合0x0002%2=0。第三个变量c的自身对齐值为2，所以有效对齐值为2，顺序存放
在0x0006、0x0007中，符合0x0006%2=0。所以从0x0000到0x00007共八字节存放的是C的变量。又C的自身对齐值为4，所以C的有效对齐值为2。又8%2=0,C只占用0x0000到0x0007的八个字节。所以sizeof(struct C)=8.

四.如何修改编译器的默认对齐值?

1.在VC IDE中，可以这样修改：[Project]|[Settings],c/c++选项卡Category的Code Generation选项的Struct Member Alignment中修改，默认是8字节。
2.在编码时，可以这样动态修改：#pragma pack .注意:是pragma而不是progma.

五.针对字节对齐,我们在编程中如何考虑?
        如果在编程的时候要考虑节约空间的话,那么我们只需要假定结构的首地址是0,然后各个变量按照上面的原则进行排列即可,基本的原则就是把结构中的变量按照类型大小从小到大声明,尽量减少中间的填补空间.还有一种就是为了以空间换取时间的效率,我们显示的进行填补空间进行对齐,比如:有一种使用空间换时间做法是显式的插入reserved成员：
```
struct A{
    char a;
    char reserved[3];//使用空间换时间
    int b;
}
```

reserved成员对我们的程序没有什么意义,它只是起到填补空间以达到字节对齐的目的,当然即使不加这个成员通常编译器也会给我们自动填补对齐,我们自己加上它只是起到显式的提醒作用.

六.字节对齐可能带来的隐患:

代码中关于对齐的隐患，很多是隐式的。比如在强制类型转换的时候。例如：
```
unsigned int i = 0x12345678;
unsigned char *p=NULL;
unsigned short *p1=NULL;

p=&i;
*p=0x00;
p1=(unsigned short *)(p+1);
*p1=0x0000;
```
最后两句代码，从奇数边界去访问unsignedshort型变量，显然不符合对齐的规定。
在x86上，类似的操作只会影响效率，但是在MIPS或者sparc上，可能就是一个error,因为它们要求必须字节对齐.

七.如何查找与字节对齐方面的问题:

如果出现对齐或者赋值问题首先查看
1. 编译器的big little端设置
2. 看这种体系本身是否支持非对齐访问
3. 如果支持看设置了对齐与否,如果没有则看访问时需要加某些特殊的修饰来标志其特殊访问操作

举例：
```
#include <stdio.h>
main()
{
struct A {
    int a;
    char b;
    short c;
};

struct B {
    char b;
    int a;
    short c;
};

#pragma pack (2) /*指定按2字节对齐*/
struct C {
    char b;
    int a;
    short c;
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/



#pragma pack (1) /*指定按1字节对齐*/
struct D {
    char b;
    int a;
    short c;
};
#pragma pack ()/*取消指定对齐，恢复缺省对齐*/

int s1=sizeof(struct A);
int s2=sizeof(struct B);
int s3=sizeof(struct C);
int s4=sizeof(struct D);
printf("%d\n",s1);
printf("%d\n",s2);
printf("%d\n",s3);
printf("%d\n",s4);
}
```
输出：

8

12

8

7

 

修改代码：
```
struct A {
   // int a;
    char b;
    short c;
};

struct B {
    char b;
   // int a;
    short c;
};
```
输出：

4

4

输出都是4，说明之前的int影响对齐！
