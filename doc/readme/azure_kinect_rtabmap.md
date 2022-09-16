# AzureKinectでRTABMapを実行するための注意点
- 入力するRGB画像と距離画像の縦横サイズを一致させること
- 「/rgb/image_raw」 と「/depth_to_rgb/image_raw」がheight: 720、width: 1280で一致

```python
ros_melodic_sobit_gpu sobits@:~$ rostopic echo /rgb/image_raw --noarr -n 1
header: 
  seq: 0
  stamp: 
    secs: 1634014441
    nsecs: 767586222
  frame_id: "rgb_camera_link"
height: 720
width: 1280
encoding: "bgra8"
is_bigendian: 0
step: 5120
data: "<array type: uint8, length: 3686400>"
---
ros_melodic_sobit_gpu sobits@:~$ rostopic echo /depth_to_rgb/image_raw --noarr -n 1
header: 
  seq: 7
  stamp: 
    secs: 1634014444
    nsecs:   1141244
  frame_id: "rgb_camera_link"
height: 720
width: 1280
encoding: "32FC1"
is_bigendian: 0
step: 5120
data: "<array type: uint8, length: 3686400>"
---
```

---

- [Topに戻る](https://github.com/TeamSOBITS/sobit_navigation_stack)