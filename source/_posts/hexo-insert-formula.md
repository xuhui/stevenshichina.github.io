---
title: hexo中插入数学公式
date: 2017-06-26 11:28:55
categories: hexo
tags: formula
comments: true
---
原生hexo并不支持数学公式，需要安装插件 [mathJax](https://www.mathjax.org/)。[mathJax](https://www.mathjax.org/) 是一款运行于浏览器中的开源数学符号渲染引擎，使用 [mathJax](https://www.mathjax.org/) 可以方便的在浏览器中嵌入数学公式。[mathJax](https://www.mathjax.org/) 使用网络字体产生高质量的排版，因此可适应各种分辨率，它的显示是基于文本的而非图片，因此显示效果更好。这些公式可以被搜索引擎使用，因此公式里的符合一样可以被搜索引擎检索到。先看一下矩阵的显示效果:

$\left[
\begin{matrix}
V_A \\\\
V_B \\\\
V_C \\\\
\end{matrix}
\right] =
\left[
\begin{matrix}
1 & 0 & L \\\\
-cosψ & sinψ & L \\\\
-cosψ & -sinψ & L
\end{matrix}
\right]
\left[
\begin{matrix}
V_x \\\\
V_y \\\\
W \\\\
\end{matrix}
\right] $

<!--more-->
# 安装与配置
　　```
 $ npm install hexo-math --save
　　```
在站点配置文件 *_config.yml* 中添加：
　　```
math:
  engine: 'mathjax' # or 'katex'
  mathjax:
    # src: custom_mathjax_source
    config:
      # MathJax config
　　```
在 next 主题配置文件中 *themes/next-theme/_config.yml* 中将 mathJax 设为 true:
　　```
# MathJax Support
mathjax:
  enable: true
  per_page: false
  cdn: //cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML
　　```
也可以在文章的开始集成插件支持，但不建议这么做：
　　```
<script type="text/javascript"
   src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
</script>
　　```
# 使用
公式插入格式：
　　```
$数学公式$ 行内 不独占一行
$$数学公式$$ 行间 独占一行
　　```
例如：
　　```
$f(x)=ax+b$
　　```
显示效果为：$f(x)=ax+b$
如果是行间则使用：
　　```
$$f(x)=ax+b$$
　　```
显示效果为：$$f(x)=ax+b$$
# 语法格式
## 上标与下标
使用 ^ 表示上标，使用 _ 表示下标，如果上下标的内容多于一个字符，可以使用大括号括起来：
　　```
$$f(x) = a_1x^n + a_2x^{n-1} + a_3x^{n-2}$$
　　```
显示效果为：$$f(x) = a_1x^n + a_2x^{n-1} + a_3x^{n-2}$$
如果左右两边都有上下标可以使用 \sideset 语法：
　　```
$$\sideset{^n_k}{^x_y}a$$
　　```
显示效果为：$$\sideset{^n_k}{^x_y}a$$


## 括号
在 markdown 语法中，\, $, {, }, _都是有特殊含义的，所以需要加\转义。小括号与方括号可以使用原始的() [] 大括号需要转义\也可以使用\lbrace和 \rbrace
　　```
\{x*y\}
\lbrace x*y \rbrace
　　```
显示效果为：$ \lbrace x*y \rbrace $
原始符号不会随着公式大小自动缩放，需要使用 \left 和 \right 来实现自动缩放：
　　```
$$\left \lbrace \sum_{i=0}^n i^3 = \frac{(n^2+n)(n+6)}{9} \right \rbrace$$
　　```
效果：
$$\left \lbrace \sum_{i=0}^n i^3 = \frac{(n^2+n)(n+6)}{9} \right \rbrace$$
不使用\left 和 \right的效果：
　　```
$$ \lbrace \sum_{i=0}^n i^3 = \frac{(n^2+n)(n+6)}{9}  \rbrace$$
　　```
$$ \lbrace \sum_{i=0}^n i^3 = \frac{(n^2+n)(n+6)}{9}  \rbrace$$
## 分数与开方
可以使用\frac 或者 \over 实现分数的显示：
　　```
$\frac xy$
$ x+3 \over y+5 $
　　```
分别显示为：$\frac xy$ 和 $ x+3 \over y+5 $。
开方使用\sqrt:
　　```
$ \sqrt{x^5} $
$ \sqrt[3]{\frac xy} $
　　```
分别显示为：$ \sqrt{x^5} $ 和 $ \sqrt[3]{\frac xy} $
## 求和与积分
求和使用\sum,可加上下标，积分使用\int可加上下限，双重积分用\iint:
　　```
$ \sum_{i=0}^n $
$ \int_1^\infty $
$ \iint_1^\infty $
　　```
分别显示:$ \sum_{i=0}^n $ 和 $ \int_1^\infty $ 以及 $ \iint_1^\infty $
## 极限
极限使用\lim:
　　```
$ \lim_{x \to 0} $
　　```
显示为：$ \lim_{x \to 0} $

## 表格与矩阵
表格样式lcr表示居中，|加入一条竖线，\hline表示行间横线，列之间用&分隔，行之间用\\分隔
　　```
$$\begin{array}{c|lcr}
n & \text{Left} & \text{Center} & \text{Right} \\\\
\hline
1 & 1.97 & 5 & 12 \\\\
2 & -11 & 19 & -80 \\\\
3 & 70 & 209 & 1+i \\\\
\end{array}$$
　　```
显示效果为：$$\begin{array}{c|lcr}
n & \text{Left} & \text{Center} & \text{Right} \\\\
\hline
1 & 1.97 & 5 & 12 \\\\
2 & -11 & 19 & -80 \\\\
3 & 70 & 209 & 1+i \\\\
\end{array}$$
矩阵显示和表格很相似
　　```
$$\left[
\begin{matrix}
V_A \\\\
V_B \\\\
V_C \\\\
\end{matrix}
\right] =
\left[
\begin{matrix}
1 & 0 & L \\\\
-cosψ & sinψ & L \\\\
-cosψ & -sinψ & L
\end{matrix}
\right]
\left[
\begin{matrix}
V_x \\\\
V_y \\\\
W \\\\
\end{matrix}
\right] $$
　　```
显示效果：　
$$\left[
\begin{matrix}
V_A \\\\
V_B \\\\
V_C \\\\
\end{matrix}
\right] =
\left[
\begin{matrix}
1 & 0 & L \\\\
-cosψ & sinψ & L \\\\
-cosψ & -sinψ & L
\end{matrix}
\right]
\left[
\begin{matrix}
V_x \\\\
V_y \\\\
W \\\\
\end{matrix}
\right] $$
还有其他矩阵如内联矩阵增广矩阵方程组等，下次再补充，未完待续...
参考：
[mathJax](https://www.mathjax.org)
[LaTeX](http://mohu.org/info/symbols/symbols.htm)
[mathjax-basic-tutorial-and-quick-reference](https://math.meta.stackexchange.com/questions/5020/mathjax-basic-tutorial-and-quick-reference/5044)
[MathJax公式简介](http://mlworks.cn/posts/introduction-to-mathjax-and-latex-expression)




