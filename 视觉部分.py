# 作者：张天鹏 刘胜昔
# 创建：2022-06-13
# 更新：2022-07-14
# 用意：OpenMV入门学习
#OpenMV坐标系
#（原点）--------X轴
#  |
#  |
#  |
#  |
#  |
#  Y
#  轴
# 颜色追踪时需要控制环境光线稳定，避免识别标志物的色彩阈值发生改变
import sensor, image, time,math
import ustruct
from pyb import UART, LED
blue_threshold  = (3, 38, -20, 42, -47, -16)#设置蓝色阈值
#yellow_threshold  = (54, 78, -3, 44, 33, 127)#设置黄色阈值（原数据）
yellow_threshold  = (25, 80, 64, -24, 47, 108)#设置黄色阈值（改后）


sensor.reset() #初始化摄像头
sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
sensor.set_auto_gain(False) #使用颜色识别时需要关闭自动自动增益
sensor.set_auto_whitebal(False)#使用颜色识别时需要关闭自动自动白平衡
clock = time.clock() #追踪帧率


uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )


def sending_data(color,cx,cy):
    global uart;
    data = ustruct.pack("<bbbbbb",
    0x2c,0x12,int(color),int(cx),int(cy),0x5b)

    uart.write(data);
    for i in data:
        print("data的内容是：    ",hex(i))

while(True):

    #img = sensor.snapshot() # 从感光芯片获得一张图像
    img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1.0)#从感光芯片获得一张图像并且进行畸变校正
    blue_blobs = img.find_blobs([blue_threshold],x_stride=2, y_stride=5, pixels_threshold=10 )
    yellow_blobs = img.find_blobs([yellow_threshold], x_stride=5, y_stride=5, pixels_threshold=36 )


    #image.find_blobs(thresholds, roi=Auto, x_stride=2, y_stride=1, invert=False, area_threshold=10, pixels_threshold=10, merge=False, margin=0, threshold_cb=None, merge_cb=None)
    #thresholds为颜色阈值，是一个元组，需要用括号［ ］括起来
    #roi设置颜色识别的视野区域，roi是一个元组， roi = (x, y, w, h)，代表从左上顶点(x,y)开始的宽为w高为h的矩形区域，roi不设置的话默认为整个图像视野
    #x_stride 就是查找的色块的x方向上最小宽度的像素
    #y_stride 就是查找的色块的y方向上最小宽度的像素
    #invert 反转阈值，把阈值以外的颜色作为阈值进行查找
    #area_threshold 面积阈值，如果色块被框起来的面积小于这个值，会被过滤掉
    #pixels_threshold 像素个数阈值，如果色块像素数量小于这个值，会被过滤掉
    #merge 合并，如果设置为True，那么合并所有重叠的blob为一个。
    #注意：这会合并所有的blob，无论是什么颜色的。如果你想混淆多种颜色的blob，只需要分别调用不同颜色阈值的find_blobs。

    #这个函数返回一个列表，
    #［0］代表识别到的目标颜色区域左上顶点的x坐标
    #［1］代表识别到的目标颜色区域左上顶点的y坐标
    #［2］代表目标区域的宽
    #［3］代表目标区域的高
    #［4］代表目标区域像素点的个数
    #［5］代表目标区域的中心点x坐标
    #［6］代表目标区域中心点y坐标
    #［7］代表目标颜色区域的旋转角度（是弧度值，浮点型，列表其他元素是整型）
    #［8］代表与此目标区域交叉的目标个数
    #［9］代表颜色的编号（它可以用来分辨这个区域是用哪个颜色阈值threshold识别出来的）


    if blue_blobs:
        color_status = ord('B')
        for r in blue_blobs:
    #画矩形：image.draw_rectangle(rect_tuple, color=White)的格式是 (x, y, w, h)
    #画圆image.draw_circle(x, y, radius, color=White) 在图像中画一个圆。x,y是圆心坐标，radius是圆的半径
    #画十字：image.draw_cross(x, y, size=5, color=White) 在图像中画一个十字，x,y是坐标，size是两侧的尺寸
    #写字：image.draw_string(x, y, text, color=White) 在图像中写字8x10的像素,x,y是坐标。使用\n, \r, and \r\n会使光标移动到下一行。text是要写的字符串。
            img.draw_rectangle((r[0],r[1],r[2],r[3]),color=(255,255,255))
            img.draw_cross(r[5], r[6],size=2,color=(255,255,255))
            img.draw_string(r[0], (r[1]-10), "blue", color=(0,0,255))
            print("中心X坐标",r[5],"中心Y坐标",r[6],"识别颜色类型","蓝色")
            sending_data(color_status,r[5],r[6])

    elif yellow_blobs:
        color_status = ord('Y')
        for y in yellow_blobs:
            img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
            img.draw_cross(y[5], y[6],size=2,color=(255,255,255))
            img.draw_string(y[0], (y[1]-10), "yellow", color=(255,255,0))
            print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","黄色")
            sending_data(color_status,y[5],y[6])
    else:
        color_status = ord('A')
        sending_data(color_status,0,0)


