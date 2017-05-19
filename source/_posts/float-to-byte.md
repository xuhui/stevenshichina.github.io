---
title: 浮点数与16进制的相互转换
date: 2017-05-11 13:35:03
categories: Linux
tags: Linux
comments: true
---
# 前言
 在嵌入式中经常会遇到浮点数的传输问题，一般将浮点数转换成4字节的16进制传输，这里的浮点数仅限于单精度浮点。大多数人的做法都是使用指针实现强制类型转换，其实浮点数在CPU里就是以4字节存储，既然在同一块内存，那我们能不能使用联合体呢？答案是肯定的。除了强制类型转换，这里介绍使用联合体实现浮点和16进制的转换，简单易用，你绝对会爱上这种方法。
<!--more-->
# 强制转换法
 浮点转换成4字节16进制:
　　```
 void float2bytes(float p,unsigned char *bytes)
 {
  unsigned char *pchar = (unsigned char*)&p;
  for(int i=0;i < sizeof(float);i++)
  {
    *bytes = *pchar;
    pchar++;
    bytes++;
  }
 }

　　```
 16进制4字节转换成浮点：
　　```
 float bytes2float(unsigned char *bytes)
 {
  return *((float*)bytes);//强制转换
 }
　　```
# 联合体法
 话不多说，直接上代码：
　```
 typedef union{

	unsigned char cvalue[4];
	float fvalue;

 }float_union;
　```
 定义联合体变量:
　```
 float_union trans_data;
　```
 此时如果给变量trans_data赋值一个浮点数:
 　```
 trans_data.fvalue = 10.05;
　 ```
 那么直接调用cvalue即可获取浮点数10.05的4字节16进制数据：
　 ```
 for(int i=0;i<4;i++)
 printf(" 0x%02x",trans_data.cvalue[i]);
 
 　```
 是不是非常简单实用。
