---
title: hexo+github建立个人博客
date: 2017-05-07 11:07:34
categories: hexo
tags: hexo
comments: true
---
# 前言
*hexo*是一个高效简洁的静态博客框架，基于*Node.js*使用Markdown解析文章，几秒内即可利用靓丽的主题生成静态网页。

#环境： 
*windows10 64bits*
#所需软件： 
　　1. [Node.js](https://nodejs.org/en/ "Java Script运行环境")　基于Chrome V8引擎的JavaScript运行环境；
　　2. [Git-for-windows](https://github.com/git-for-windows/git/releases "git for windows")　windows平台下git 运行环境；也可以使用 [github-for-windows](https://github.com/gavincook/githubOfflineInstaller) 里的 *git shell*, 由于在线安装 [github-for-windows](https://github.com/gavincook/githubOfflineInstaller) 在国内不翻墙是不可能成功的，所以上面的链接是离线版的安装版本。
　　3. [MarkdownPad](http://markdownpad.com/ "markdown Editor for Windows")　博客撰写工具

*开始之前先去 [github](www.github.com) 注册一个账户，并新建一个名为username.github.io的仓库，最后生成的仓库地址：https://github.com/username/username.github.io  注：username替换为自己的用户名*
# hexo安装与配置
首先在本地建立自己的博客文件夹，比如 *F:\github\hexo*，进入该文件夹内，右键点击*Git Bash Here*,打开*git for windows*环境，开始安装 [hexo](https://hexo.io/zh-cn/docs/ "hexo说明")：
<!--more-->
　　```
　$npm install hexo-cli -g //注: -g表示全局安装
　$npm install hexo-server -g //注： hexo3.0之后server模块是独立的,需要单独安装
　　```
完成之后可以通过命令查看*hexo*是否安装成功：
　　```
　$hexo -v
　　```
*hexo*根目录下的*_config.yml*文件是全局配置文件，修改该文件实现自己的配置。具体可参考 [hexo](https://hexo.io/zh-cn/docs/ "hexo说明") 官网。
## hexo常用命令
hexo常用命令可以参考hexo的帮助说明，下面列举几个常用命令：
　　```
　$hexo g //等同于hexo generate,用于生成静态文件
　$hexo s //等同于hexo server,用于启动本地服务器，预览网页
　$hexo d //等同于hexo deploy,用于部署文件到github
　　```

## 建站
在自己的博客文件夹下*F:\github\hexo*,执行命令：
　　```
　$hexo init //初始化，自动下载搭建网站所需的所有文件
　$npm install //安装依赖包
　$hexo g //产生静态文件
　$hexo s //启动本地服务器，开启预览功能
　　```
这个时候在浏览器输入localhost:4000即可查看到生成的静态网页：![Alt text](hexo-blog/hello-world.jpg)
## 部署到github
在hexo根目录下找到配置文件_config.yml，使用 [MarkdownPad](http://markdownpad.com/ "markdown Editor for Windows") 打开，编辑最后的deploy字段，将自己的github仓库地址输入进去：
　　```
　deploy:
　　　type: git //注意冒号':'后面的空格一定要有
　　　repository: https://github.com/username/username.github.io.git
　　　branch: master
　　```
### 配置SSH Key
在桌面空白处鼠标右键选择*Git Bash Here*,输入以下命令配置SSH Key:
　　```
 $ssh-keygen -t rsa -C "your_email@youremail.com"
　　```
引号里面的内容输入邮箱地址，然后回车，会提示文件保存的路径，这时候按回车键确认，然后会提示你输入密码，然后会确认输入一次，在*c:/user/username/.ssh/*文件夹内看到生成了两个文件，一个是id_rsa，另一个是id_rsa.pub，打开id_rsa.pub然后选中里面的全部内容，复制下来。登录 [github](https://github.com/login),点击*setting*,左边有一个*SSH and GPG keys*选项，进入*New SSH*选项，*title*随意，将之前复制的内容粘贴到key里面去。回到*Git bash*命令行，输入以下命令验证:
　　```
 $ssh -T git@github.com
　　```
### 配置用户名和密码
　　```
 $git config --global user.name "your username"  
 $git config --global user.email "your_email@youremail.com"
　　```
重新生成一下静态文件：
　　```
　$hexo clean //清除缓存文件db.json和已生成的静态文件public
　$hexo g //产生静态文件
　$hexo d //部署到github
　　```
在运行*hexo d*之前需要安装git部署工具
　　```
　$npm install hexo-deployer-git -g
　　```

第一次执行*hexo d*会弹出对话框提示输入github的用户名和密码。此时在浏览器输入：
*https://github.com/username/username.github.io*即可看到自己的博客已经搭建成功。
注: username替换为自己的用户名.
## 文章置顶

修改 hexo-generator-index 插件，位于  node_modules/hexo-generator-index/lib目录下，文件名为 generator.js,将该文件内容修改为：
　　```
'use strict';
var pagination = require('hexo-pagination');
module.exports = function(locals){
  var config = this.config;
  var posts = locals.posts;
    posts.data = posts.data.sort(function(a, b) {
        if(a.top && b.top) { // 两篇文章top都有定义
            if(a.top == b.top) return b.date - a.date; // 若top值一样则按照文章日期降序排
            else return b.top - a.top; // 否则按照top值降序排
        }
        else if(a.top && !b.top) { // 以下是只有一篇文章top有定义，那么将有top的排在前面（这里用异或操作居然不行233）
            return -1;
        }
        else if(!a.top && b.top) {
            return 1;
        }
        else return b.date - a.date; // 都没定义按照文章日期降序排
    });
  var paginationDir = config.pagination_dir || 'page';
  return pagination('', posts, {
    perPage: config.index_generator.per_page,
    layout: ['index', 'archive'],
    format: paginationDir + '/%d/',
    data: {
      __index: true
    }
  });
};
　　```
参考 [generator.js](https://github.com/ehlxr/java-utils/blob/master/resources/generator.js),在要置顶的文章中添加 top 值，值越大文章越靠前：
　　```
 ---
 title: ROS 学习
 categories: ROS
 tags: [picture,naruto]
 date: 2017-05-02 11:30:04
 top: 1000
 ---
　　```
# nexT主题
hexo安装完成后，默认的主题是landscape，保存在hexo/themes/文件夹下。[nexT](http://theme-next.iissnan.com/) 主题需要自行下载。在博客文件目录下F:\github\hexo右键单击Git Bash Here，输入命令:
　　```
　$git clone https://github.com/iissnan/hexo-theme-next.git themes/nexT
　　```
下载后的nexT主题会保存在themes目录下。修改hexo根目录下的配置文件_config.yml使主题生效:
　　```
　# Extensions
　## Plugins: https://hexo.io/plugins/
　## Themes: https://hexo.io/themes/
　theme: nexT
　　```
接下来选择nexT的Scheme，打开nexT的配置文件themes/nexT/_config.yml中，修改:
　　```
　# Schemes
　#scheme: Muse //默认Scheme，黑白主调，大量留白
　#scheme: Mist //紧凑版本，整洁有序
　scheme: Pisces //双栏显示，清新
　　```
更多的设置请参考: [nexT](http://theme-next.iissnan.com/getting-started.html)
## 设置语言
打开hexo的配置文件F:\github\hexo\_config.yml，修改:
　　```
　language: zh-Hans //中文汉字
　　```
# 访问量统计
### 文章阅读量统计
使用 [leancloud](https://leancloud.cn) 实现文章阅读次数统计。注册完成后，点击访问控制台，选择创建新应用:![Alt-text](hexo-blog/leancloudc.jpg)

创建名为Counter的class。之后找到该应用的AppID、AppKey，将这两个值写入到nexT的主题配置文件中_config.yml:
　　```
　# Show number of visitors to each article.
　# You can visit https://leancloud.cn get AppID and AppKey.
　leancloud_visitors:
  　　enable: true
  　　app_id: #<app_id> //复制刚才创建的应用的AppID
  　　app_key: #<app_key> //复制刚才创建的应用的AppKey
　　```
为了安全，将自己的域名加入安全中心->web安全域名中，防止恶意修改。完成后部署网站，看效果是不是已经有了文章阅读量显示了。
### 站点访问统计
使用 [不蒜子](http://ibruce.info/2015/04/04/busuanzi/) 脚本实现。
在themes\nexT\layout\_partials目录下找到文件footer.swig，在文件的头部添加:
　　```
　<script async src="//dn-lbstatics.qbox.me/busuanzi/2.3/busuanzi.pure.mini.js">
　</script>
　　```




　　

　　
　　

　　