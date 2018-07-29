### python C++ 混合编程教程

 ##### 安装环境

我们主要安装编译环境，因为opencv3需要编译，安装之后，则编译环境就好了

```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
cd ~/
git clone https://github.com/Itseez/opencv.git 
cd ~/opencv
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j8 # -j8 runs 8 jobs in parallel.
sudo make install
```

##### 最小化程序

我们首先写一段C++程序，存储为main.cpp

```
int main()
{
    return 0;
}
extern "C"
{
    void helloworld()
    {
        printf("helloworld\n");
    }
}
```

我们选择CMake作为工具编译C++程序

```
cmake_minimum_required(VERSION 2.8.7)
project(sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

find_package(OpenCV 3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
)

file(GLOB SOURCES
    "*.cpp"
    )

set( PROJECT_LINK_LIBS
    ${OpenCV_LIBRARIES}
)

add_executable(sample ${SOURCES})
target_link_libraries(sample ${PROJECT_LINK_LIBS})
add_library(test SHARED ${SOURCES})
target_link_libraries(test ${PROJECT_LINK_LIBS})

```

然后开始编译

```
mkdir build
cd build
cmake ..
make
cp libtest.so ../
./sample
```

在Python中我们使用ctypes来导入C++代码库

```
import ctypes
from ctypes import *
dir_path = os.path.dirname(os.path.realpath(__file__))
lib = cdll.LoadLibrary(dir_path + '/libindustrysimulation.so')
```

然后在python中就像调用C++函数一样调用

```
lib.helloworld()
```

这个时候会打印

```
helloworld
```

#### 一个更贴近产品的例子

首先创建main.cpp

我们自己生成一副图片，并把接口暴露给python。

```
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

int main()
{
    return 0;
}

unsigned char *m_imgRGB;
extern "C" 
{
	Init()
	{
        m_imgRGB = new unsigned char[640*480*3];
	}
    void GetRGBOnce(unsigned char* imgRGB)
    {
        for (int j = 0; j < 480; j++)
        {
            for (int i = 0; i < 640; i++)
            {
            	for (int k = 0; k < 3; k++)
            	{
                	if (i < 320)
                	{
                        m_imgRGB[3*(j*640 + i) + k] = 255;
                	}
                	else
                	{
                        m_imgRGB[3*(j*640 + i) + k] = 0;
                	}
                }
            }
        }
        return;
    }
}
```

按照上述CMake编译之后获得libtest.so

写一个python的借口程序来调用C++ 接口

ImageInterface.py

```
import numpy as np
import sys
import random

import threading
import os
import ctypes
from ctypes import *
dir_path = os.path.dirname(os.path.realpath(__file__))
lib = cdll.LoadLibrary(dir_path + '/libtest.so')
from numpy.ctypeslib import ndpointer

def Init():
	lib.Init()

def getRGBImg():
 	lib.GetRGBOnce.argtypes = [POINTER(c_ubyte)]
 	imagedata = np.zeros((480, 640, 3), dtype=np.ubyte)
 	data = imagedata.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
 	lib.GetRGBOnce(data)
 	return imagedata
```

然后在写一个测试程序测试这个接口

首先为了展示测试结果，我们再安装一遍python opencv

```
pip install opencv-python
```

test.py

```
import ImageInterface as imageinterface
import cv2

if __name__ == "__main__":
	
	imageinterface.Init()

	imgRGB = imageinterface.getRGBImg()
	cv2.imshow('test', imgRGB)
	cv2.waitKey(30)
```

