# git 用法总结

版本控制的形象比喻：
>好比玩RPG游戏时，每通过一关就会自动把游戏状态存盘，如果某一关没过去，你还可以选择读取前一关的状态。有些时候，在打Boss之前，你会手动存盘，以便万一打Boss失败了，可以从最近的地方重新开始。Git也是一样，每当你觉得文件修改到一定程度的时候，就可以“保存一个快照”，这个快照在Git中被称为commit。一旦你把文件改乱了，或者误删了文件，还可以从最近的一个commit恢复，然后继续工作，而不是把几个月的工作成果全部丢失。

### 1 配置用户
新下载git之后，需要配置自己机器的git用户：
```
git config --global user.name "Your Name"
git config --global user.email "email@example.com"
```
因为Git是分布式版本控制系统，所以，每个机器都必须自报家门：你的名字和Email地址

注意git config命令的--global参数，用了这个参数，表示你这台机器上所有的Git仓库都会使用这个配置
当然也可以对某个仓库指定不同的用户名和Email地址。

 2 第一次使用git
git可以将一个文件夹变成版本库，版本库又名仓库，英文名repository，里面的所有文件都可以被Git管理起来，每个文件的修改、删除，Git都能跟踪，以便任何时刻都可以追踪历史，或者在将来某个时刻可以“还原”。

新建一个git仓库
```
git init
```

添加文件到git暂存区
```
git add 文件名 或者  git add .
```

把暂存区的内容提交到当前分支
```
git commit -m "此次修改的注释"
```

注意：这样分两步做的原因是：git add 可反复多次使用，添加多个文件到暂存区，而 git commit 可以一次把这些文件都提交上去

上面的两步就可以在自己的电脑上是git来进行版本控制了，但是和远程库的关联还没有，也没有进行多人协作控制。

也可以跳过暂存区域直接从分支中取出修改，或者直接提交修改到分支中。

- git commit -a 直接把所有文件的修改添加到暂存区然后执行提交
- git checkout HEAD -- files 从分支中取出最后一次修改，可以用来进行回滚操作

### 3 git支持管理的文件
各种版本控制系统，包括git，能跟踪改动的文件都是文本文件，代码程序，html等文件，它可以告诉你在第几行修改了什么内容。

但是能跟踪的文件不包括视频，图片等文件，因为他们是二进制的文件，只能跟踪修改后文件的大小变化，不能告诉你修改的细节。

例如windows下的word文件保存的就是二进制文件，git是不能跟踪变化细节的。

### 4 版本回滚：
```
git reset --hard HEAD^
```
这代表回滚到前1个版本，如果要回滚到前n版本，可以后面改成HEAD～n 或者HEAD后面写n个^符号
```
git reset --hard commit-id
```
这个使用id号来指定恢复到哪一个版本，具体commit-id号可以通过 git log查看
```
git log
```
查看历史修改日志，里面可以得到每次修改的描述注释和commit-id,但是如果回滚到之前的版本，那么那个版本之后的修改日志就没了，就相当于回到了那个历史版本的时间点，因此丢失了后面的版本的信息。
```
git reflog
```
可以记录所有历史操作，包括reset操作，里面保存了所有版本的commit-id,因此可以补救版本回滚后又想恢复的操作，即想回到未来的版本

### 5 工作区、版本库、暂存区
git中有几个个区域的概念：工作区，版本库，暂存区，当前分支
- 工作区：我们修改代码所在的文件夹就是工作区，是我们明文件显示的区域。
- 版本库：初始化git之后，会生成一个隐藏文件夹，叫.git/ 这个隐藏文件夹就是版本库，里面包含两个重要部分，分别是暂存区和当前分支。
- 当前分支：git会自动生成一个且唯一一个master分支，这个分支就记录了当前的状态
- 暂存区：凡是git add 操作的之后的文件，都会先放在暂存区,然后git commit操作之后将暂存区的文件一起提交到当前分支上，同时暂存区就会被清空。

### 6 查看当前状态
```
git status
```
可以查看当前工作区的状态，可以显示出：
- 当前跟踪的文件的修改细节
- 当前是否有新添加的文件没有被跟踪
- 当前是否有被跟踪的文件被删除
- 当前的工作区的内容是否和远程库一致
- 当前的暂存区是否有文件没有提交到master分支上面

当前工作区的内容都和本地版本库中的当前分支master一致,则会显示当前工作是干净的。
如果发现某一个文件被修改了，git status不能显示出被修改的细节，但是可以使用以下命令来查看修改的细节：
```
git diff 文件名
```

### 7 查看修改内容
```
git diff HEAD -- 要对比的文件
```
例如：git diff HEAD -- readme.txt
     这个命令可以查看当前工作区的指定文件和当前master分支的相应文件的区别

### 8 撤销修改
```
git checkout -- 要修改的文件名
```

撤销操作：
可以撤销当前在工作区的修改：
- 如果该次修改还没有提交到暂存区，那么撤销后和当前分支的版本一致
- 如果该次修改提交到了暂存区，但是还没有commit，那么撤销后的版本和暂存区的版本一致
- 如果该次修改已经commit，那么就不是撤销修改了，需要进行版本回滚

这里的修改也包括删除文件，比如把一个文件删除了，可以使用 git checkout -- file 来恢复被删除的文件，
撤销操作本质上就是用当前分支或暂存区的文件，来替换工作区的文件。

总结一下
```
git reset -- files 使用当前分支上的修改覆盖暂存区，用来撤销最后一次 git add files
git checkout -- files 使用暂存区的修改覆盖工作目录，用来撤销本地修改
git checkout HEAD -- files 从分支直接取出最后一次修改放在工作区，跳过了暂存区，可以用来进行回滚操作
```

### 9 暂存区撤回
```
git reset HEAD file
```
这个命令是git add的逆操作，可以把暂存区的内容返回来，重新放回工作区

### 10 删除文件
一种是使用
```
git rm file
```
这样的话就是把工作区的文件和当前分支的相应文件都删除了,然后再使用commit就可以了

另一种是直接在工作区删除，使用rm file，那么工作区和版本库就不一致，需要进行 add 和 commit来同步版本库和工作区

### 11 git添加远程库

```
git remote add 远程库起的名字 远程库的http或ssh地址
```

例如：
git remote add origin git@server-name:path/repo-name.git
其中orign就是git默认的远程库名字，可以不进行修改，这样一般情况下看到orign就知道是远程库

删除本地关联的远程库：
```
git remote rm origin
```

如果要在本地的版本库添加多个远程库，比如github和gitee，那么添加远程库的命令就不能只用一个默认的origin了
可以如下所示：
```
git remote add gitee git@gitee.com:liaoxuefeng/learngit.git
git remote add github git@github.com:michaelliao/learngit.git
```
这样再推送本地版本库到本地的时候，就可以相应的修改远程库的名称
```
git push github master
git push gitee master
```
### 12 把当前分支提交到远程库
```
git push -u origin master
```
这个用于第一次提交本地master的所有内容，其中-u参数不但可以推送本地内容到远程分支，还可以把本地的master分支和远程的master分支关联起来，这样下次就不用关联了。
```
git push origin master
```
之后再进行提交代码到远程库，就可以去掉-u参数了

### 13 克隆远程库
```
git clone git@github.com:michaelliao/gitskills.git
```
后面跟的地址可以是ssh或者是https，上面的例子是ssh的，还可以用https://github.com/michaelliao/gitskills.git这样的地址。

使用https除了速度慢以外，还有个最大的麻烦是每次推送都必须输入口令，但是在某些只开放http端口的公司内部就无法使用ssh协议而只能用https

使用ssh需要设置ssh密钥，好处是每次push的时候就不用输入git用户名和密码了

### 14 使用分支
当需要对代码进行修改一个功能的时候，如果直接在主分支上进行修改存在风险，可以先建一个dev分支，然后在dev分支上面进修改，确定可以无误之后，将dev分支和master分支进行合并，然后删除dev分支。

git分支操作

查看当前所有分支：该命令会显示出所有分支，并且在当前分支前面加×符号
```
git branch
```
创建新分支：
```
git branch name
```
切换到某一个分支：
```
git checkout name
```
同时创建分支并且切换到该分支
```
git checkout -b name
```
删除一个分支:
```
git branch -d name
```
合并某一个分支到到当前分支，当前分支是被合并的分支，name代表的分支是合并进来的分支
```
git merge name
```
查看本地所有分支
```
git branch -a
```
查看远程分支
```
git branch -r
```
注意：建立远程分支和本地分支关联的时候，分支的名字一定要取得一样。

一般规范：master 分支应该是非常稳定的，只用来发布新版本；日常开发在开发分支 dev 上进行。

### 15 分支冲突
当我们假设有两个分支，分别是master和dev，分别对文件1进行了不同的修改（两个文件的内容不是包含关系），那么当前的版本库的分支图就会出现分叉的现象，因为两个分支的文件1是不同的，那么当要进行分支合并的时候就会出现冲突。

git在发现冲突的时候，会自动修改冲突的文件，把两个分支的内容都在文件1里面标注清楚

然后我们需要手工排解冲突，修改后的文件1就作为两个分支的新的文件1内容，接着提交add和commit之后，这两个分支的文件1就就会统一，分支就被合并了。

Git 会使用 <<<<<<< ，======= ，>>>>>>> 标记出不同分支的内容，只需要把不同分支中冲突部分修改成一样就能解决冲突。
```
<<<<<<< branch-name1
Creating a new branch is quick & simple.
=======
Creating a new branch is quick AND simple.
>>>>>>> branch-name2
```
可以使用
```
git log --graph
```
来查看分支合并简图，可以发现修复冲突之后，两个分支的分叉部分又合并为了同一条线

### 16 分支合并的两种方式
1. 当我们创建了一个master以外的分支dev并且在上面进行修改时，dev分支就是master的领先分支。如果切换回master分支，并且使用git merge dev进行合并的时候，会默认使用Fast forward模式进行合并，也就是直接修改master指针到dev指针处，但这种模式下，删除分支dev后，会丢掉分支dev存在过的信息，即log graph中就会显示不出dev分支存在过。
2. 如果使用
    ```
      git merge --no--ff -m "注释"  dev
    ```
    就会禁止使用Fast forward模式（里面包含了一次隐藏的commit），而是当作类似冲突修复那样。这样可以保留dev分支存在过的信息，即使删除dev分支，log graph也会保留之前dev的分支线

### 17 远程分支操作
查看远程库的信息，可以显示本地版本库关联的远程库的名字和push pull路径：
```
git remote
git remote -v
```
获取远程repo所有分支,如果本地没有获取远程分支，那么建立本地分支和远程分支的时候会找不到远程分支
```
git fetch (远程代码仓库名，例如origin)
```

提交本地分支:
```
git push 远程库名字（一般是origin）远程分支的名字（例如master或者dev）
git push origin master
git push origin dev
```
如果出现推送失败，一般是因为远程分支比本地当前分支要新，需要使用以下命令拉下来分支
```
git pull
```
在使用第一次使用git pull拉下来一个远程分支的时候，需要指定拉下来的分支和本地哪一个分支合并，即需要建立远程分支和本地分支的关联

建立本地分支和远程分支的关联：
```
git branch --set-upstream-to=origin/branch-name dev（本地分支名）
```
建立连接之后，就可以git pull了，而且pull会自动进行远程对应分支和本地分支的合并，如果存在冲突，需要手动解决冲突，然后add和commit即可

### 18 下拉远程其他分支
当我们使用git clone拉下来一个项目的时候，默认是拉下来master分支
如果想在远程dev上面进行操作，那么需要拉下来远程对应的dev分支到本地，使用命令：
```
git checkout -b branch-name origin/branch-name
```
这个命令新建了一个对应远程分支的本地分支，

同时建立了和远程分支的push连接，即本地修改dev分支之后，可以推送push到远程的dev分支，二者是对应的，
但是没有建立pull连接，如果想要git pull远程的dev分支，需要使用前面的建立本地分支和远程分支的关联，然后使用git pull就可以了

### 19 变基操作：
```
git rebase
```

### 20 fork仓库的含义：
如果直接clone别人的代码库，那么自己是不能push上去别人远程库的，因此就需要使用fork

自己fork的代码库，然后通过自己的clone连接拉下来代码到本地进行修改，然后修改完毕之后是可以push到自己fork的代码库的。

如果想对原作者的代码提出修改建议，可以使用pull request，等待对方是否接受。

### 21 .gitignore 文件
忽略以下文件：

操作系统自动生成的文件，比如缩略图；
编译生成的中间文件，比如 Java 编译产生的 .class 文件；
自己的敏感信息，比如存放口令的配置文件。
不需要全部自己编写，可以到 https://github.com/github/gitignore 中进行查询。

### 22 SSH 传输设置
Git 仓库和 Github 中心仓库之间的传输是通过 SSH 加密。

如果工作区下没有 .ssh 目录，或者该目录下没有 id_rsa 和 id_rsa.pub 这两个文件，可以通过以下命令来创建 SSH Key：
```
$ ssh-keygen -t rsa -C "youremail@example.com"
```
然后把公钥 id_rsa.pub 的内容复制到 Github “Account settings” 的 SSH Keys 中。
