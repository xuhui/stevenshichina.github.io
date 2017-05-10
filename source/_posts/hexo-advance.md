---
title: hexo进阶
date: 2017-05-09 14:32:38
categories: hexo
tags: hexo
---
# 添加页面
　　页面指的是分类页*categories*、标签页*tags*、关于页*about*等。使用以下命令生成以上页面参考[nexT](http://theme-next.iissnan.com/theme-settings.html):
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
 设置自己的标签页面:
  ```
 title: 标签
date: 
tags:
 - Qt
 - Linux
 - Hexo
 - ROS
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
 #RSS 订阅支持
 plugin:
 - hexo-generator-feed
 #feed Atom
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
 # Social Links
 # Key is the link label showing to end users.
 # Value is the target link (E.g. GitHub: https://github.com/iissnan)
 social:
   Github: https://github.com/username
   知乎: http://www.zhihu.com/people/username
　　```
 设置链接的图标，对应的字段是*social_icons*在主题配置文件中:
　```
 # Social Icons
 social_icons:
  enable: true
  # Icon Mappings
  GitHub: github
  Twitter: twitter
  微博: weibo
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
# 设置代码高亮主题
 NexT 使用[TomorrowTheme](https://github.com/chriskempson/tomorrow-theme)作为代码高亮，共有5款主题供选择。 打开主题配置文件*themes/nexT/_config.yml*修改:
　　```
 # Code Highlight theme
 # Available value:
 #    normal | night | night eighties | night blue | night bright
 # https://github.com/chriskempson/tomorrow-theme
 highlight_theme: night bright

　　```
# 开启打赏功能
 只需要主题配置文件中填入微信和支付宝收款二维码图片地址，即可开启该功能。修改主题配置文件*themes/nexT/_config.yml*
　　```
 reward_comment: 您的支持是我原创的动力
  wechatpay: /path/to/wechat-reward-image
  alipay: /path/to/alipay-reward-image

　　```
 鼠标放置在二维码上面时，下面的字来回晃眼，修改如下文件:
*themes/next/source/css/_common/components/post/post-reward.styl*
 注释掉以下代码：![Alt text](hexo-advance/pay.jpg)
# 友情链接
 修改主题配置文件*themes/next/_config.yml*
　　```
 # Blog rolls
 links_title: 友情链接
 #links_layout: block
 #links_layout: inline
 links:
  hexo: https://hexo.io/zh-cn/docs
  nexT: http://theme-next.iissnan.com/
　　```
 