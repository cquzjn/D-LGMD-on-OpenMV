import sensor, image, pyb, os, time
import Mavlink_Trans
#import led_control as LED


#LED.Hello_LED(8)

sensor.reset() # 复位并初始化传感器。
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种

sensor.set_framesize(sensor.QQQVGA) # or sensor.QQVGA (or others)
#设置图像像素大小
sensor.set_auto_gain(False,40)
current_exposure_time_in_microseconds = sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(3000))



sensor.skip_frames(time = 1000) # 让新的设置生效。

Mav_Data = Mavlink_Trans.Mavlink_STRUCT()
clock = time.clock() # 跟踪FPS帧率

Im_Col = sensor.width()    #sensor.width()
Im_haf_col = int(Im_Col/2)
Im_Row = sensor.height()    #sensor.height()
Im_haf_row = int(Im_Row/2)

# 从主帧缓冲区的RAM中取出以分配第二帧缓冲区。
# 注意，在使用image 类的difference 方法后，该对象会被差分后的结果覆盖。
#extra_fb1 = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)    # 开辟FrameBuFfer。t-1
extra_fb1 = sensor.alloc_extra_fb(Im_Col, Im_Row, sensor.GRAYSCALE)    # 开辟FrameBuFfer。t-1
extra_fb2 = sensor.alloc_extra_fb(Im_Col, Im_Row, sensor.GRAYSCALE)    # 开辟FrameBuFfer。t-2
imgT0_1 = sensor.alloc_extra_fb(Im_Col, Im_Row, sensor.GRAYSCALE)    # 开辟FrameBuFfer。用于中转图像
imgT0_2 = sensor.alloc_extra_fb(Im_Col, Im_Row, sensor.GRAYSCALE)    # 开辟FrameBuFfer。用于中转图像
FFI = sensor.alloc_extra_fb(Im_Col, Im_Row, sensor.GRAYSCALE)    # 开辟FrameBuFfer。用于存FFI
Layer_G = image.Image(Im_Col, Im_Row, sensor.GRAYSCALE)  #开辟FrameBuFfer。用于暂存S层和最后存G层

Frame_count = 0
args_kernel_E = [
0,	0,	0,	0,	0,	0,	0,\
0,	1,	3,	0,	3,	1,	0,\
0,	3,	4,	12,	4,	3,	0,\
0,	0,	12,	27,	12,	0,	0,\
0,	3,	4,	12,	4,	3,	0,\
0,	1,	3,	0,	3,	1,	0,\
0,	0,	0,	0,	0,	0,	0
]
args_kernel_I = [
3,	5,	6,	6,	6,	5,	3,\
5,	7,	9,	0,	9,	7,	5,\
6,	9,	0,	0,	0,	9,	6,\
6,	0,	0,	0,	0,	0,	6,\
6,	9,	0,	0,	0,	9,	6,\
5,	7,	9,	0,	9,	7,	5,\
3,	5,	6,	6,	6,	5,	3
]
args_kernal_G = [
1,	1,	1,\
1,	1,	1,\
1,	1,	1]

Para_G_Scale = 2
Glayer_Threhold = 15        #白天20，夜间10

while(1):
    clock.tick()                    # Update the FPS clock.
    Frame_count +=1

    img = sensor.snapshot()         # Take a picture and return the image.
    imgT0_1.replace(img)    #暂存旧的图像
    #
    Diff_1 = img.difference(extra_fb1)    # "abs(NEW-OLD)" 帧差图像存入img内存。(P层t0)
    FFI_temp = Diff_1.copy()
    #
    if Frame_count>=3:
        #获取P层
        imgT0_2.replace(extra_fb1)#暂存旧的 t0-1 图像
        Diff_2 = extra_fb1.difference(extra_fb2)    #只有在这里，fb2表示旧的t0-1，也即t0-2.(P层t0-1)
        extra_fb2.replace(imgT0_2) #记录新的 t0-1 的图像

        #Layer_E = Diff_1.LGMD(3,args_kernel_E,0.01)
        #Layer_I = Diff_2.LGMD(3,args_kernel_I,0.01)
        #Layer_S = Layer_E.sub(Layer_I)


        #Layer_S = Diff_1.morph(3,args_kernel_E,0.01)
        Layer_S = Diff_1.LGMD(Diff_2,3,args_kernel_E,args_kernel_I,0.01)

        #计算G层2
        Layer_G.replace(Layer_S)
        Layer_S.morph(1,args_kernal_G,Para_G_Scale)
        Layer_G=Layer_G.mul(Layer_S)
        Layer_G.binary([(0,Glayer_Threhold)],invert=True)  #设置 zero 为True来使阈值像素为零，并使不在阈值列表中的像素保持不变

        img.replace(Layer_S)
        print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
       # Mav_Data.transfer(FFIout=0, LGMDout= 0 , AvoidingMode=0, DNS_Up=0, DNS_Down=0, DNS_Left=0, DNS_Right=0)

    extra_fb1.replace(imgT0_1)  #记录 t0 的图像
