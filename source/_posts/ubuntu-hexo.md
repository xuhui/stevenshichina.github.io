---
title: ubuntu 下 hexo 安装与配置
date: 2017-05-23 20:39:56
categories: hexo
tags: ubuntu hexo
comments: true
---
# 前言
有时候调试 *linux*，写代码都是在 *ubuntu* 环境下，随时需要记录一些调试的过程，这时候就需要在 *ubuntu* 下配置 *hexo* 环境，并撰写博客。
<!--more-->
# 安装git

 ```
 $sudo apt-get install git
  ```
# 生成SSH Key
  ```
 $git config --global user.name "your username"  
 $git config --global user.email "your_email@youremail.com"
 $ssh-keygen -t rsa -C "youremail@email.com"
  ```
如果自己使用多台电脑更新博客，可以使用多个 *SSH Key*,但这样做根本没必要，完全可以使用一个。拷贝之前电脑~/.ssh/中生成的id_rsa和id_rsa.pub到本电脑目录 *~/.ssh/* 中，并确保文件的权限，私钥 *id_rsa* 的权限是600，公钥 *id_rsa.pub* 权限是644。
# github验证机制
说到这里就多说一下github的验证机制。公钥和私钥是成对使用的，一般是使用 *RSA*算法生成 *id_rsa.pub* 和 *id_rsa*。公钥是可以暴露在网络上传输的，但私钥不可以，私钥只能放在本地。所以这两个文件的权限也不同。当使用时，客户端发出公钥登录的请求，服务器端返回一段随机字符，客户端收到该字符使用私钥对这段字符进行加密，并发送给服务器端，服务器端使用事先存储的公钥去解密这段字符，如果成功即表示客户端身份验证通过。
# 安装Node.js
不建议使用 *apt-get* 命令安装，安装可能不成功
使用如下命令安装参考 [nvm-github](https://github.com/creationix/nvm):
 ```
export NVM_DIR="$HOME/.nvm" && (
  git clone https://github.com/creationix/nvm.git "$NVM_DIR"
  cd "$NVM_DIR"
  git checkout `git describe --abbrev=0 --tags --match "v[0-9]*" origin`
) && . "$NVM_DIR/nvm.sh"
 ```
使nvm开机自启动，将如下内容添加到 *~/.bashrc* 中：
  ```
  export NVM_DIR="$HOME/.nvm"
 [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
   ```
不要忘记重启 *terminal* 或者重新 *source* 一下 *~/.bashrc*
执行以下命令：
   ```
   $nvm ls-remote //列出所有安装包的版本信息
   $nvm install stable //安装最稳定版本
   $nvm use node //使用当前版本
   $nvm run node --version //也可以使用该命令 使用当前版本
   $nvm alias default node //设置默认版本
    ```
如果需要更新则手动更新 *nvm*:
     ```
 (
  cd "$NVM_DIR"
  git fetch origin
  git checkout `git describe --abbrev=0 --tags --match "v[0-9]*" origin`
 ) && . "$NVM_DIR/nvm.sh"
 ```
# 安装hexo
```
$npm install  hexo-cli -g
$npm install hexo-server -g
$npm install hexo-deployer-git -g
$npm install hexo-util -g
```
接下来建立博客目录，进入给目录下：
```
$hexo init //初始化 下载必要的建站文件
$npm install //安装依赖项
```
# 安装remarkable
linux 下有很多好用的 *markdown* 博客撰写工具，诸如[Atom](https://atom.io/)、[Haroopad](http://pad.haroopress.com/)、[Mark My Words](https://github.com/voldyman/MarkMyWords)、[remarkable](http://remarkableapp.github.io/linux/download.html) 等等。其中 [remarkable](http://remarkableapp.github.io/linux/download.html) 最为流行。 [remarkable](http://remarkableapp.github.io/linux/download.html) 是linux下一款免费的 *markdown* 编辑器。关于它的介绍这里不多说，可自行查阅。
安装之前先安装一些依赖：
```
$sudo apt-get install   python3-markdown   python3-bs4  wkhtmltopdf
```
下载[remarkable-deb](http://remarkableapp.github.io/linux/download.html) 目前的版本为 1.87：
```
$wget http://remarkableapp.github.io/files/remarkable_1.87_all.deb
```
安装：
```
$sudo dpkg -i remarkable_1.87_all.deb
$sudo apt-get install -f 补足有可能的缺失依赖项
```
接下来就可以开始博客的撰写了，关于博客的撰写与发布请参考我站内关于 *hexo* 的文章。
