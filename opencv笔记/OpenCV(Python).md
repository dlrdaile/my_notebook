# OpenCV(Python)

## 图像的输入输出

### 图像

#### 读取静态图像:`imread(文件名,读取模式)`

* 读取模式:`cv2.IMREAD_***`

  <img src="file:///home/dllr/.config/weixin/wechat/users/wxid_lrqo8yv8sgan22/message/cache/9e20f478899dc29eb19741386f9343c8/image/2022-08/42_1661339022_hd" alt="img" style="zoom: 67%;" /><img src="/home/dllr/Desktop/notebook/opencv笔记/OpenCV(Python).assets/43_1661339049_hd" alt="img" style="zoom: 67%;" />

  #### 写入静态图像`imwrite(文件路径，写入的图像frame)`

  > 注意文件路径中写明了图像的保存后缀

  ### 视频

  #### 读取视频`cv2.VideoCapture(视频文件路径)`

  ```python
  import cv2
  
  videoCapture = cv2.VideoCapture('MyInputVid.avi') #读取视频文件
  fps = videoCapture.get(cv2.CAP_PROP_FPS) #获取视频的fps
  size = (int(videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)),
          int(videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT))) #获取视频的frame尺寸
  videoWriter = cv2.VideoWriter(
      'MyOutputVid.avi', cv2.VideoWriter_fourcc('I','4','2','0'), fps, size) #写入视频，注意此处需要说明视频的编码方式
  
  success, frame = videoCapture.read()
  while success: # Loop until there are no more frames.
      videoWriter.write(frame)
      success, frame = videoCapture.read()
  ```

  #### 读取摄像头

  ###### 单个摄像头的读取

  > 和上述方法相近，只是将`cv2.VideoCapture`中的文件路径改为系统识别的**设备号或者设备编号**

  ###### 多个摄像头同步读取

  ![](/home/dllr/Desktop/notebook/opencv笔记/OpenCV(Python).assets/036-i.jpg)

  #### 写入视频

  ##### 编码方式

  <img src="/home/dllr/Desktop/notebook/opencv笔记/OpenCV(Python).assets/53_1661339658_hd" alt="img" style="zoom:50%;" />

  <img src="/home/dllr/Desktop/notebook/opencv笔记/OpenCV(Python).assets/54_1661339881_hd" alt="img" style="zoom:50%;" />

  ```python
  import cv2
  
  cameraCapture = cv2.VideoCapture(0)
  fps = 30  # An assumption
  size = (int(cameraCapture.get(cv2.CAP_PROP_FRAME_WIDTH)),
          int(cameraCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
  videoWriter = cv2.VideoWriter(
      'MyOutputVid.avi', cv2.VideoWriter_fourcc('M','J','P','G'), fps, size)
  
  success, frame = cameraCapture.read()
  numFramesRemaining = 10 * fps - 1  # 10 seconds of frames
  while numFramesRemaining > 0:
      if frame is not None:
          videoWriter.write(frame)
      success, frame = cameraCapture.read()
      numFramesRemaining -= 1
  ```

  ### 显示图像或者视频

  ```python
  import cv2
  
  clicked = False
  def onMouse(event, x, y, flags, param):
      global clicked
      if event == cv2.EVENT_LBUTTONUP:
          clicked = True
  
  cameraCapture = cv2.VideoCapture(0)
  cv2.namedWindow('MyWindow')
  cv2.setMouseCallback('MyWindow', onMouse)
  
  print('Showing camera feed. Click window or press any key to stop.')
  success, frame = cameraCapture.read()
  while cv2.waitKey(1) == -1 and not clicked:
      if frame is not None:
          cv2.imshow('MyWindow', frame)
      success, frame = cameraCapture.read()
  
  cv2.destroyWindow('MyWindow')
  ```

  ## OpenCv中的颜色模型

  ### 灰度

  ### RGB

  ### HSV