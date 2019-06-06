[TOC]

### 1、C++11 类成员的内部初始化

在C++98标准里，只有static const声明的整型成员能在类内部初始化，并且初始化值必须是常量表达式。这些限制确保了初始化操作可以在编译时期进行。例如：
```
    int var = 7;
    class X {
        static const int m1 = 7;   // 正确
        const int m2 = 7;    // 错误：无static
        static int m3 = 7;              // 错误：无const
        static const int m4 = var;  // 错误：初始化值不是常量表达式
        static const string m5 = “odd”; //错误：非整型
        // …
    };
```
C++11的基本思想是，允许非静态（non-static）数据成员在其声明处（在其所属类内部）进行初始化。这样，在运行过程中，需要初始值时构造函数可以使用这个初始值。考虑下面的代码：
```
    class A {
    public:
        int a = 7;
    };
这等同于：

    class A {
    public:
        int a;
        A() : a(7) {}
    };
```
从代码来看，这样可以省去一些文字的输入，但是真正受益的是拥有多个构造函数的类。在大多数情况下，所有的构造函数都会使用成员变量的常见初始值：
```
class A {
    public:
         A(): a(7), b(5), hash_algorithm(“MD5″), s(“Constructor run”) {}
        A(int a_val) :
          a(a_val), b(5), hash_algorithm(“MD5″),
          s(“Constructor run”)
          {}

        A(D d) : a(7), b(g(d)),
            hash_algorithm(“MD5″), s(“Constructor run”)
            {}
        int a, b;
    private:
        // 哈希加密函数可应用于类A的所有实例
        HashingFunction hash_algorithm;
        std::string s;  // 指示的字符串变量将持续整个对象的生命周期
    };
```
hash_algorithm和s每个都有一个单独的默认值的事实会由于杂乱的代码而不明显，这会在程序维护时造成麻烦。作为替代，可以将数据成员的初始值提取出来：
```
class A {
    public:
        A(): a(7), b(5) {}
        A(int a_val) : a(a_val), b(5) {}
        A(D d) : a(7), b(g(d)) {}
        int a, b;
    private:
        //哈希加密函数可应用于类A的所有实例
        HashingFunction hash_algorithm{“MD5″};
        //指示的字符串变量将持续整个对象的生命周期
        std::string s{“Constructor run”};
    };
```
如果一个成员同时在类内部初始化时和构造函数内被初始化，则只有构造函数的初始化有效（这个初始化值“优先于”默认值）（译注：可以认为，类内部初始化先于构造函数初始化进行，如果是对同一个变量进行初始化，构造函数初始化会覆盖类内部初始化）。因此，我们可以进一步简化：
```
class A {
    public:
        A() {}
        A(int a_val) : a(a_val) {}
        A(D d) : b(g(d)) {}
        int a = 7;
        int b = 5;
    private:
        //哈希加密函数可应用于类A的所有实例
        HashingFunction hash_algorithm{“MD5″};
        //指示的字符串变量将持续整个对象的生命周期
        std::string s{“Constructor run”};
};
