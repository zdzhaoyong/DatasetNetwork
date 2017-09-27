# DatasetNetwork

## Introduction

本工程主要用来定义RTMapper网络输入接口，同时附Server和Client测试演示代码。
其中网络协议在src/dbnet/RTMapperNetInterface.h中定义，用户给RTMapper的输入串流格式为：

```
header|payload|header|payload|header|header|payload
```
这样的模式，其中header定义了UAV的基本状态信息和payload信息，payload为图像内容信息。

其中图像传输协议有无压缩码，JPEG压缩码，H264/H265压缩码等。

## 编译

目前编译在Ubuntu 16.04测试通过，Client端只依赖Qt 4.8, Server端依赖opencv。
