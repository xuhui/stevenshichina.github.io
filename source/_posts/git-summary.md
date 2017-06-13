---
title: Git 实用命令总结
date: 2017-06-09 10:16:08
categories: Linux
tags: Git Command
comments: true
---
使用 Git 那么久也没来得及总结一下，今天抽空将 Git 经常用到的一些命令记录下来，以备不时之需。
# 创建代码仓
如果远程已有代码仓可以直接 clone 至本地：
   ```
 $ git clone https://github.com/username/repository.git
   ```
使用时请将 username 和 repository 替换成自己需要的远程代码仓地址，比如 clone 我的 learning_tf 代码仓至本地：
   ```
$ git clone https://github.com/StevenShiChina/learning_tf.git
   ```
值得一提得是 git 不仅支持 https 协议，还支持 ssh 协议等，具体可参考 [git-protocols](https://git-scm.com/book/id/v2/Git-on-the-Server-The-Protocols)。
也可以在本地创建一个 git 项目：
   ```
 $ git init
   ```
<!--more-->
# 本地修改与更新
在本地新建文件或修改文件后，要将文件修改提交到暂存区：
   ```
 $ git add --all // 当前目录下的所有文件都提交到暂存
   ```
也可以使用如下命令：
   ```
 $ git add . // ‘.’指当前目录下所有文件
   ```
只添加某个文件：
   ```
 $ git add -p <file>
   ```
如果忘记了当前目录下修改了哪个文件可以用以下命令查看：
   ```
 $ git status
   ```
修改完之后可以提交本地所有修改到暂存区：
   ```
 $ git commit -a
   ```
为了区分每次提交的不同可以附加消息提交：
   ```
 $ git commit -m "your message to add here"
   ```
 如果在提交到暂存前提示错误，可以删除 git 的缓存，重新提交到暂存：
   ```
 $ git rm -r --cached .
 $ git add --all
 $ git commit -m "your message to add here"
   ```

# 配置
在将本地代码仓推送至远程之前，我们还需要对本地 git 进行一些必要的配置，包括全局用户名以及邮件，这个可以配置为全局的也可以配置在自己的代码仓下：
   ```
 $ ssh-keygen -t rsa -C "your_email@youremail.com"  // 生成SSH Key
 $ git config --global user.name "your username" //配置远程仓的用户名
 $ git config --global user.email "your_email@youremail.com" //配置邮件地址
   ```
需要将生成的 SSH Key 添加到 github 具体可参考站内文章 [hexo+github建立个人博客](http://stevenshi.me/2017/05/07/hexo-blog/)。
可以通过以下命令查看是否能连接到远程：
   ```
 $ ssh -T git@github.com
   ```
# 分支
如果是本地新建的代码仓，需要通过以下命令将远程主机的代码仓与本地对应起来：
   ```
 $ git remote add origin https://github.com/username/repository.git
   ```
查看远程分支：
   ```
 $ git remote -v
 $ git remote show <分支名> //查看某个分支信息
   ```
如果远程有多个分支，可以通过以下命令切换：
   ```
 $ git checkout <分支名>
 $ git checkout -b <分支名> //该命令可以创建分支并切换到分支
   ```
如果搞错了，可以删除本地分支：
   ```
 $ git branch -d <分支名>
   ```
可以通过 tag 给当前版本打个标签：
   ```
 $ git tag <tag-name>
   ```
# 发布
本地修改完成后，需要推送至远程：
   ```
 $ git push origin <分支名>
   ```
# 本地同步
在本地可以通过 pull 命令来同步远程文件：
   ```
 $ git pull  // 同步所有
 $ git pull origin <分支名> //同步某个分支
   ```
# 合并

   ```
 $ git merge <分支名>
   ```
# 回撤
如果修改错了，想回退到上一个版本：
   ```
 $ git reset --hard HEAD
   ```
未完待续...