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

### 3、 系统调用的标准使用方法
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

#### 4.1  perror

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
#### 4.2、strerror

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

### 7、 fork() 系统调用

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

### 8、 execve() 系统调用
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
