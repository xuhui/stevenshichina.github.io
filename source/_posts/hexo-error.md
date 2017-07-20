---
title: hexo问题汇总
date: 2017-05-13 17:42:11
tags: hexo
categories: hexo
---
# Unable to call the return value of (posts["first"])["updated"]["toISOString"], which is undefined or falsey
 运行*hexo g*生成静态文件时提示错误：
　　```
 $ hexo g
 INFO  Start processing
 FATAL Something's wrong. Maybe you can find the solution here: http://hexo.io/docs/troubleshooting.html
 Template render error: (unknown path) [Line 7, Column 23]
 Error: Unable to call `the return value of (posts["first"])["updated"]["toISOString"]`, which is undefined or falsey
　　```
 <!--more-->
 当移除掉插件hexo-generator-feed和hexo-generator-sitemap后错误消失，怀疑该插件与hexo兼容性不好
　　```
 $npm uninstall hexo-generator-feed
 $npm uninstall hexo-generator-sitemap
　　```
# WARN No layout: index.html?
 查看主题目录是否为空，如果为空下载主题
# fatal: AggregateException encountered
 当推送至远程分支时提示错误:
　　```
 $git push origin hexo
 fatal: AggregateException encountered
   •¢ÉúÒ»¸ö»ò¶à¸ö´íÎó¡£
 Username for 'https://github.com':
　　```
 多数是网络问题，重来一次即可。
## Cannot GET /
 当启动服务器后，浏览器输入*http://localhost:4000*，却提示错误：*Cannot GET*
 一般是配置文件错误，当在其他电脑从远程clone下来代码仓后，安装必要的工具使用 *hexo clean* 以及 *hexo g* 后产生的 *public* 文件与本地电脑原有源码仓生成的 *public* 不一样，查看里面并没有生成以日期命名的文章文件，怀疑是配置文件出了问题，对比两个配置文件都一样，最后发现 *node_modules* 文件夹问题，就是hexo的配置和安装问题，其他文件都推送到远程仓库，唯独没有推送这个文件夹，它保存有hexo的一些可执行文件。按照hexo安装教程在clone下来的代码仓重新安装一遍hexo，最后还是不行。使用 [WinMerge](http://winmerge.org/) 比较原电脑中和现在电脑中的两个node_modules文件的不同，发现hexo的很多二进制文件都不同。无语-
 当使用 *hexo init* 后，再把远程仓库的配置文件覆盖到本地电脑后，可以正常生成静态网页。肯定是 *hexo* 安装不完整缺少某些依赖项。
 所以，当在一台新电脑部署 *hexo* 的正确做法是安装好 *hexo* 之后需要 *hexo init* 一下，这样会安装依赖项，得到完整的 *node_modules*文件夹：
　　```
 $npm install hexo --save
 $npm install hexo-server  --save
 $npm install hexo-generator-search --save
 $npm install hexo-deployer-git --save
 $hexo init //
　　```
 初始化之后，保留node_modules文件夹，其它删除，之后获取远程仓库最新更新：
　　```
 $git remote add origin https://github.com/yourusername/yourusername.github.io.git
 $git pull origin hexo
　　```
# bash: /dev/tty: No such device or address.error: failed to execute prompt script (exit code 1)
 当部署自己的博客文件时：
　　```
 $hexo d 
　　```
 出现以下错误：
　　```
 bash: /dev/tty: No such device or address
 error: failed to execute prompt script (exit code 1)
 fatal: could not read Username for 'https://github.com': Invalid argument
 FATAL Something's wrong. Maybe you can find the solution here: http://hexo.io/docs/troubleshooting.html
 Error: bash: /dev/tty: No such device or address
 error: failed to execute prompt script (exit code 1)
 fatal: could not read Username for 'https://github.com': Invalid argument.
　　```
 出现该现象是 *windows* 环境下的*git bash shell*配置问题，当改用 *github for windows* 下的*git bash shell* 再次部署时问题解决。由于国内的网络问题，官网下载很难安装成功，这里附上一个CSDN上的大神的离线安装包地址 [GithubforWindows](http://download.csdn.net/user/devsplash)。
# Permission denied (publickey).
 当部署博客文件时提示错误：
　　```
 $hexo d
 The authenticity of host 'github.com (192.30.255.112)' can't be established.
 Permission denied (publickey).
　　```
 本机没有配置SSH Key：
 　```
 $ssh-keygen -t rsa -C "your_email@youremail.com"
　 ```
 将生成的SSH Key拷贝下来，粘贴到自己的github网站的new ssh key中即可。或者使用原来电脑的SSH Key。完成后查看配置是否成功：
 　```
 $ssh -T git@github.com
 　```
# Changes not staged for commit: Untracked files:
当每次git commit提交更改时，总是提示以下错误：
　　```
 $git commit -m ""
 On branch hexo
 Changes not staged for commit:
       
 Untracked files:
　　```
 通过输入git status查看提示：
　　```
 $ git status
 On branch hexo
 Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git checkout -- <file>..." to discard changes in working directory)

        
 Untracked files:
  (use "git add <file>..." to include in what will be committed)

        
 no changes added to commit (use "git add" and/or "git commit -a")
 
　　```
 *Changes not staged for commit* 说明 *git* 已经跟踪到这些文件的修改，但还没有放到暂存区，需要使用 *git add* 命令提交更新到暂存区。之后再通过 *git status* 查看状态：
　　```
 $git status
 On branch hexo
 Changes to be committed:
  (use "git reset HEAD <file>..." to unstage)

       
　　```
 状态已经变成了Changes to be committed。提示未被跟踪的文件Untracked files说明是新建立的文件，在git之前的提交中没有这些文件，git不会自动将它们纳入跟踪范围，必须手动添加这些文件。使用 *git add* 命令跟踪新文件：
　　```
 $git add "file" //添加未被跟踪的文件
　　```
 如果还有错误，直接清除缓存:
　　```
 $git rm -r --cached .
 $git add --all //跟踪所有文件
 $git commit -m "your commit" //提交更新
 $git push origin "yourbranch" //推送到远程分支yourbranch
　　```