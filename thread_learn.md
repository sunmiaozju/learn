## c++多线程学习总结

#### 一、创建线程
```
#include <pthread.h>
pthread_t thread;
pthread_create (thread, attr, start_routine, arg)
```
创建线程成功时，函数返回 0，若返回值不为 0 则说明创建线程失败。
参数|描述
---|---
thread|指向线程标识符指针。
attr	|一个不透明的属性对象，可以被用来设置线程属性。一般使用默认值 NULL。
start_routine|	线程运行函数起始地址，一旦线程被创建就会执行。
arg	|运行函数的参数。它必须通过把引用作为指针**强制转换为 void 类型**进行传递。如果没有传递参数，则使用 NULL。
例如：
```
rc = pthread_create(&threads, NULL, PrintHello, (void *)&indexes);
```

#### 二、线程终止
```
pthread_exit(status)
```
pthread_exit 用于显式地退出一个线程。通常情况下，pthread_exit() 函数是在线程完成工作后无需继续存在时被调用。

如果在main()的最后写了pthread_exit()，那么如果main函数在它所创建的线程之前结束，其他线程将继续执行，不受main结束的影响。否则，它们将在main()结束时自动被终止。

#### 三、阻塞等待一个线程结束
函数pthread_join用来等待一个线程的结束。函数原型为：
```
extern int pthread_join __P ((pthread_t __th, void**__thread_return));
```
第一个参数为被等待的线程标识符，第二个参数为一个用户定义的指针，它可以用来存储被等待线程的返回值。这个函数是一个线程阻塞的函数，调用它的函数将一直等待到被等待的线程结束为止，当函数返回时，被等待线程的资源被收回。

一个线程不能被多个线程等待，也就是说对一个线程只能调用一次pthread_join，否则只有一个能正确返回，其他的将返回ESRCH错误。

#### 四、多线程互斥锁
互斥锁常用的有四个函数，如下所示
```
#include <pthread.h>
int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *mutexattr); //初始化互斥锁
int pthread_mutex_lock(pthread_mutex_t *mutex); //加锁
int pthread_mutex_unlock(pthread_mutex_t *mutex); //解锁
int pthread_mutex_destroy(pthread_mutex_t *mutex); //销毁互斥锁
```
互斥锁的类型
```
ptread_mutex_t mutex;
```
linux下为了多线程同步，通常用到锁的概念。
posix下抽象了一个锁类型的结构：ptread_mutex_t。通过对该结构的操作，来判断资源是否可以访问。顾名思义，加锁(lock)后，别人就无法打开，只有当锁没有关闭(unlock)的时候才能访问资源。

在加锁和解锁的之间的代码称为临界段代码，只会由一个线程来执行。互斥锁通过确保一次只有一个线程执行代码的临界段来同步多个线程。互斥锁还可以保护单线程代码。

##### 4.1 互斥锁初始化：
```
int pthread_mutex_init(pthread_mutex_t *restrict mutex,
                       const pthread_mutexattr_t *restrict attr);
```
该函数用于C函数的多线程编程中，互斥锁的初始化。函数成功完成之后会返回零，其他任何返回值都表示出现了错误。

pthread_mutex_init()函数是以动态方式创建互斥锁的，参数attr指定了新建互斥锁的属性。如果参数attr为NULL，则使用默认的互斥锁属性，默认属性为快速互斥锁 。互斥锁的属性在创建锁的时候指定，不同的锁类型在试图对一个已经被锁定的互斥锁加锁时表现不同。

##### 4.2 互斥锁属性

互斥锁的属性在创建锁的时候指定，在LinuxThreads实现中仅有一个锁类型属性，不同的锁类型在试图对一个已经被锁定的互斥锁加锁时表现不同。当前（glibc2.2.3,linuxthreads0.9）有四个值可供选择：

- PTHREAD_MUTEX_TIMED_NP，这是缺省值，也就是普通锁。当一个线程加锁以后，其余请求锁的线程将形成一个等待队列，并在解锁后按优先级获得锁。这种锁策略保证了资源分配的公平性。

- PTHREAD_MUTEX_RECURSIVE_NP，嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。

- PTHREAD_MUTEX_ERRORCHECK_NP，检错锁，如果同一个线程请求同一个锁，则返回EDEADLK，否则与PTHREAD_MUTEX_TIMED_NP类型动作相同。这样就保证当不允许多次加锁时不会出现最简单情况下的死锁。

- PTHREAD_MUTEX_ADAPTIVE_NP，适应锁，动作最简单的锁类型，仅等待解锁后重新竞争。

##### 4.3 其他锁操作

锁操作主要包括加锁pthread_mutex_lock()、解锁pthread_mutex_unlock()和测试加锁 pthread_mutex_trylock()三个，不论哪种类型的锁，都不可能被两个不同的线程同时得到，而必须等待解锁。

对于普通锁和适应锁类型，解锁者可以是同进程内任何线程；

检错锁则必须由加锁者解锁才有效，否则返回EPERM

对于嵌套锁，文档和实现要求必须由加锁者解锁，但实验结果表明并没有这种限制，这个不同目前还没有得到解释。在同一进程中的线程，如果加锁后没有解锁，则任何其他线程都无法再获得锁。
```
int pthread_mutex_lock(pthread_mutex_t *mutex)

int pthread_mutex_unlock(pthread_mutex_t *mutex)

int pthread_mutex_trylock(pthread_mutex_t *mutex)
```
pthread_mutex_trylock()语义与pthread_mutex_lock()类似，不同的是在锁已经被占据时返回EBUSY而不是挂起等待。

##### 4.4 死锁:

死锁主要发生在有多个依赖锁存在时, 会在一个线程试图以与另一个线程相反顺序锁住互斥量时发生. 如何避免死锁是使用互斥量应该格外注意的东西。

总体来讲, 有几个不成文的基本原则:

- 对共享资源操作前一定要获得锁。

- 完成操作以后一定要释放锁。

- 尽量短时间地占用锁。

- 如果有多锁, 如获得顺序是ABC连环扣, 释放顺序也应该是ABC。

- 线程错误返回时应该释放它所获得的锁。
