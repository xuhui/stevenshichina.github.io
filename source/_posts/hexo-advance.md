---
title: hexo进阶
date: 2017-05-09 14:32:38
categories: hexo
tags: hexo
---
# 添加页面
　　页面指的是分类页*categories*、标签页*tags*、关于页*about*等。使用以下命令生成以上页面:
　　```
 $hexo new page categories
 $hexo new page tags
 $hexo new page about　
　　```
　　在 *hexo* 配置文件 *_config.yml* 文件中设置 *post_asset_folder* 为true,当新建页面后会自动生成一个同名文件夹，用于存储所需的资源图片等。生成以上页面后在主题配置文件中 *themes/nexT/_config.yml* 打开相关配置项:
　　```
menu:
  home: /
  categories: /categories
  about: /about
  archives: /archives
  tags: /tags
  #sitemap: /sitemap.xml
  #commonweal: /404.html
　　```
<!--more-->
# 发表文章
 新文章的发表使用以下命令：
　　```
 $hexo new post "文章名" 或者 hexo n "文章名"
　　```
 文章存储在*hexo\source\_posts*目录下，后缀为*.md*。可以在适当位置插入以下符号用于部分显示:
　　```
   <!--more-->
　　```
 插入该符号之前的文字可以直接显示出来，之后的需要鼠标点击*阅读全文*来显示全部。
　　```
 $hexo g //生成静态文件
 $hexo d //部署
　　```
# 添加RSS
 安装RSS插件：
　　```
 $npm install hexo-generator-feed --save
　　```
 修改*hexo\_config.yml*根目录下的全局配置文件，添加以下内容:
　　```
 feed:
  type: atom
  path: atom.xml
  limit: 20
　　```
 修改主题目录下的配置文件*themes、nexT/_config.yml*
　　```
 rss: /atom.xml
　　```
# 添加sitemap
 安装*sitemap*插件:
　```
 $npm install hexo-generator-sitemap --save
　　```
 修改*hexo/_config.yml*根目录下的全局配置文件:
　　```
 sitemap:
　　　path: sitemap.xml
　　```
# 添加侧栏社交链接
 修改主题目录下的配置文件*themes/nexT/_config.yml*
　　```
 social:
   Github: https://github.com/stevenshichina
　　```
# 添加本地搜索
 安装*hexo-generator-serarch*
　　```
 $npm install hexo-generator-search --save
　　```
 修改*hexo/_config.yml*根目录下的全局配置文件添加:
　　```
 search:
  path: search.xml
  field: post
　　```
# 设置头像
 在hexo/source/目录下建立images文件夹，将头像放置在该文件夹下，修改主题目录下的配置文件*themes/nexT/_config.yml*
　　```
 avatar: /images/avatar.jpg
　　```
# 生成网站二维码
 二维码生成工具很多，这个[Custom QR Code Generator](https://www.unitag.io/qrcode)比较不错。生成后将二维码添加到关于页面。
