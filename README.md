# calibCamera
这是一段用于相机标定的代码（源文件、头文件）

代码中的路径需要根据下图，对应实际路径进行修改
![image](https://user-images.githubusercontent.com/62756096/225047640-01788309-e7ea-424f-beaa-50e09da003ba.png)

具体用法：
1、若没有相机内参文件需要用改代码进行标定

  请将用于标定的图像存放在对应“相机标定图像文件路径”变量表示路径下，并将图像文件名称按照想要标定的相机名称进行命名，最后会输出对应相机名称的相机内参文本文件到内参文本文件路径下，并将内参   数据返回给类成员变量"Intrinsic"
  
2、若已有相机内参文件

  将相机内参文本文件按照相机名称存放在对应"相机标定文本文件路径"下，且文件只能出现"空格、数字以及换行符"。最后会将内参数据返回给类成员变量"Intrinsic"用于后续计算。
  
相机内参文本文件路径字符串变量用来查找对应输入cameraName即相机文件同名的文本文件是否存在
- 若存在则从中读取内参文件返回给内参矩阵的 Mat变量，代码结束
- 若不存在，则去存放相机所拍摄的标定图像文件的路径查找对应的标定图像文件是否存在同名标定图像文件
- - 若存在，则开始进行标定并返回标定结果存放相机名称同名标定文本文件于标定文本文件路径下
- - 若不存在，则返回标定失败
