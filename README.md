# RoboMaster-CV
 
 
![image](https://github.com/For-LD/RoboMaster-ComputerVision/blob/main/source/FrameWork.jpg)

## 1.通过基于YOLO V3进行目标检测
## 2.将检测目标区域通过位姿解算得到目标运动及位置信息
## 3.利用卡尔曼滤波进行目标运动跟踪及预测

# 视频展示自瞄效果
### [<<视频展示>> 自瞄效果](https://live.csdn.net/v/177587)

# 步兵外观图
![image](https://github.com/For-LD/RoboMaster-ComputerVision/blob/main/source/Infantry.jpg)

# 另外秀一下自己组装的无人机
![image](https://github.com/For-LD/RoboMaster-ComputerVision/blob/main/source/Drone.jpg)
### [<<视频展示>> 无人机炸机之旅](https://live.csdn.net/v/177529)


## 源码目录
```
│  .gitattributes
│  .gitignore
│  README.md
│  
├─armor_detect
│  │  CMakeLists.txt
│  │  package.xml
│  │  
│  ├─3rdparty
│  │  ├─include
│  │  │      darknet.h
│  │  │      DxImageProc.h
│  │  │      GxIAPI.h
│  │  │      image_opencv.hpp
│  │  │      
│  │  └─lib
│  │          83d9e27abc6804c6cd0356cfdc6a22d538aab450.cache
│  │          GxU3VTL.cti
│  │          libgxiapi.so
│  │          
│  ├─data
│  │  │  121.jpg
│  │  │  9k.labels
│  │  │  9k.names
│  │  │  9k.tree
│  │  │  coco.names
│  │  │  coco9k.map
│  │  │  dog.jpg
│  │  │  eagle.jpg
│  │  │  giraffe.jpg
│  │  │  goal.txt
│  │  │  horses.jpg
│  │  │  imagenet.labels.list
│  │  │  imagenet.shortnames.list
│  │  │  inet9k.map
│  │  │  kite.jpg
│  │  │  openimages.names
│  │  │  person.jpg
│  │  │  resize.jpg
│  │  │  resize4.jpg
│  │  │  resize5.jpg
│  │  │  scream.jpg
│  │  │  voc.names
│  │  │  
│  │  └─labels
│  │          make_labels.py
│  │          
│  ├─include
│  │  └─armor_detect
│  │          .rm.h.swp
│  │          armor_detector.h
│  │          armor_detect_node.hpp
│  │          getStream.hpp
│  │          rm.h
│  │          SentryDetector.h
│  │          test.h
│  │          
│  ├─lib
│  │      libdarknet.so
│  │      libyolo.so
│  │      
│  ├─model
│  │      debug.yaml
│  │      opencv_detect_config.xml
│  │      sentry_detect_config.xml
│  │      s_template.bmp
│  │      template.bmp
│  │      test_yolov2-tiny-voc_final.weights
│  │      voc.data
│  │      voc.names
│  │      yolov2-tiny-voc.cfg
│  │      yolov2-tiny-voc_150000.weights
│  │      yolov2-tiny-voc_200000.weights
│  │      yolov2-tiny-voc_final.weights
│  │      
│  ├─msg
│  │      pose.msg
│  │      rect_data.msg
│  │      status.msg
│  │      
│  └─src
│          armor_detector.cpp
│          armor_detector2.cpp
│          armor_detect_node.cpp
│          getStream.cpp
│          SentryDetector.cpp
│          test.cpp
│          
├─pose_solver
│  │  CMakeLists.txt
│  │  package.xml
│  │  
│  ├─include
│  │  └─pose_solver
│  │          pnp_solver.h
│  │          pose_node.h
│  │          trans_pose.h
│  │          
│  ├─msg
│  │      pose.msg
│  │      rect_data.msg
│  │      status.msg
│  │      
│  └─src
│          pnp_solver.cpp
│          pose_node.cpp
│          trans_pose.cpp
│          
├─serial
│  │  CMakeLists.txt
│  │  package.xml
│  │  
│  ├─include
│  │  └─serial
│  │          serial.h
│  │          serial_node.h
│  │          
│  ├─lib
│  ├─msg
│  │      pose.msg
│  │      status.msg
│  │      
│  └─src
│          serial.cpp
│          serial_node.cpp
│          
└─source
        FrameWork.jpg
        Infantry.jpg
 ```
