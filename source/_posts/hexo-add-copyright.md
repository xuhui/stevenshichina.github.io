---
title: hexo文章末尾添加版权信息
date: 2017-05-26 15:56:25
categories: hexo
tags: Copyright
comments: true
---
# 前言
在文章的末尾添加版权信息，并标出文章链接、作者等信息，这些信息应该便于复制，在文章的末尾自动生成。
<!--more-->
# 实现
我使用的主题是 [nexT](http://theme-next.iissnan.com/),为实现版权功能需要手动修改主题目录下的 layout/_macro/post.swig 文件，找到 post-footer 所在的标签，添加以下内容：
　　```
    <footer class="post-footer"> 原有内容
    <div>    
     {# 此处判断是否在索引列表中 #}
     {% if not is_index %}
    <ul class="post-copyright">
      <li class="post-copyright-author">
          <strong>本文作者：</strong>{{ theme.author }}
      </li>
      <li class="post-copyright-link">
        <strong>本文链接：</strong>
        <a href="{{ url_for(page.path) }}" title="{{ page.title }}">{{ page.path }}</a>
      </li>
      <li class="post-copyright-license">
        <strong>版权： </strong>
        本站文章均采用 <a href="http://creativecommons.org/licenses/by-nc-sa/3.0/cn/" rel="external nofollow" target="_blank">CC BY-NC-SA 3.0 CN</a> 许可协议，请勿用于商业，转载注明出处！
      </li>
    </ul>
    {% endif %}
    </div>
　　```
# 添加显示格式
修改主题目录下的 source/css/_custom/custom.styl 文件：
　　```
.post-copyright {
    margin: 1em 0 0;
    padding: 0.5em 1em;
    border-left: 3px solid #ff1700;
    background-color: #f9f9f9;
    list-style: none;
}
　　```
重新部署以下看效果是否显示出来。