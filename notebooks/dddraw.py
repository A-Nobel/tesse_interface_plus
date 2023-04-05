#-*-coding:utf-8-*-
from tesse.env import Env
from tesse.msgs import *
import time
import matplotlib.pyplot as plt
import defusedxml.ElementTree as ET
import numpy as np
import socket
import time
import threading
import matplotlib.pyplot as plt
import random
import math
import pygame,random
from pygame.locals import *
from pygame import font
# import pygame
import rospy
from std_msgs.msg import  String

# 判断点是否在矩形内的函数
def is_point_in_rect(t, p):
    rect = pygame.Rect(t[0], t[1], t[2], t[3])
    return rect.collidepoint(p[0], p[1])

# 订阅ROS话题，接收物体二维坐标和颜色信息
def callback(data):
    global pos_color_list
    global each_room_objects
    global total_objects 
    temp_room_objects = [0] * room_counts
    
    if(len(data.data)<5):
        return
    # print(data.data)
    # 解析接收到的字符串消息
    data_list = data.data.split(';')
    
    pos_color_list = []
    for item in data_list:
        # 将每个物体信息转换为坐标和颜色
        info_list = item.split(',')
        if(len(info_list)<3):
            continue
        # print(len(info_list))
        color = [int(x) for x in info_list[2].split(" ")]
        # x -25  31.5 y 1.5 45
        # x -25  32 y 0 50
        pos = tuple([(float(x)) for x in info_list[3].split(" ")])
        pos = ((pos[0])*multi+plus,(pos[1])*multi)

        roomId = info_list[7]

        # roompos = tuple([(float(x)) for x in info_list[8].split(" ")])
        # roompos = ((pos[0])*multi+plus,(pos[1])*multi)

        for i in range(room_counts):
            if(is_point_in_rect(t_list[i],pos)):
                temp_room_objects[i] += 1
                break
        pos_color_list.append((pos, color,roomId))
    for i in range(room_counts):
        each_room_objects[i] = temp_room_objects[i]
    total_objects = len(pos_color_list)

def run():
    global pos_color_list,total_objects
    global each_room_objects
    
    # 初始化Pygame
    print("init pygame")
    pygame.init()
    pygame.font.init()

    myfont = font.Font(None,fontSize)
    myfont2 = font.Font(None,20)

    # 创建屏幕对象
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROS物体坐标和颜色")

    # ROS节点初始化
    rospy.init_node('object_display')
    # 待生成方块的坐标和颜色

    rospy.Subscriber('/hydra_dsg_visualizer/object_info2', String, callback)
    # 游戏循环
    running = True
    print("ready running pygame")
    while running:
        # 处理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # 按下 Esc退出
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        # 绘制背景
        screen.fill(BG_COLOR)
        pygame.draw.rect(screen, color1,t1 )
        pygame.draw.rect(screen, color2,t2 )
        pygame.draw.rect(screen, color3,t3 )
        pygame.draw.rect(screen, color4,t4 )
        pygame.draw.rect(screen, color5,t5 )
        # 更新屏幕
        # 绘制所有物体方块
        for box in pos_color_list:
            pos, color,roomId = box
            pygame.draw.rect(screen, color, (pos[0] - BOX_SIZE // 2, pos[1] - BOX_SIZE // 2, BOX_SIZE, BOX_SIZE))
            screen.blit(myfont2.render(roomId, True, (255,255,0)), (pos[0], pos[1]))
        # 更新屏幕
        strforscore = ("Total Obj: "+str(total_objects)+"\nRoom1: "+str(each_room_objects[0])+"\nRoom2: "+str(each_room_objects[1])+"\nRoom3: "+str(each_room_objects[2])+"\nRoom4: "+str(each_room_objects[3])+"\nRoom5: "+str(each_room_objects[4]))
        lines = strforscore.splitlines()
        for i, l in enumerate(lines):
            screen.blit(myfont.render(l, True, colors[i]), (0, 0 + fontSize*i))
        pygame.display.flip()
    # 退出Pygame
    pygame.quit()


if __name__ == '__main__':
    # 开始时间
    BEGIN_TIME = time.time()
    env = Env()


    # 生成物体的线程变量
    room_counts = 5
    room_obj_real_counts = [0]*room_counts

  
    # GUI线程
    each_room_objects = [0] * room_counts
    total_objects = 0
    # 偏移量
    multi = 18
    plus = 500
    # 画图 x_min,y_min,x,y(x_min,y_min)为靠近原点的点
    t = tuple(int(x*multi+plus) for x in (-16,9.5,15.5,12))
    t1 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
    t = tuple(int(x*multi+plus) for x in (-14.5,27.5,12,12))
    t2 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))

    t = tuple(int(x*multi+plus) for x in (4.5,27.5,3,8))
    t3 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
    # t = tuple(int(x*multi+plus) for x in (-20.70,-23.30,24.72,27.25))

    # t = tuple(int(x*multi+plus) for x in (7.60,12.7,27.6,35.3))
    t = tuple(int(x*multi+plus) for x in (7.60,27.6,5.1,7.7))
    t4 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
    t = tuple(int(x*multi+plus) for x in (-20.70,24.72,2.6,2.5))
    t5 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
    t_list = [t1,t2,t3,t4,t5]
    pos_color_list = []
    fontSize =50
    # 定义颜色
    colorForFont = (255,0,0)
    color1 = (22,22,22)
    color2 = (33,33,220)
    color3 = (22,202,22)
    color4 = (202,22,22)
    color5 = (202,202,22)
    colors = [colorForFont,color1,color2,color3,color4,color5]
    # 定义屏幕尺寸和背景颜色
    SCREEN_WIDTH = 1200
    SCREEN_HEIGHT = 1200
    BG_COLOR = (255, 255, 255)
    # 定义物体方块尺寸和默认颜色
    BOX_SIZE = 10
    DEFAULT_COLOR = (255, 0, 0)

    # # GUI线程开始
    # reward_gui_server = threading.Thread(target=run, args=())
    # reward_gui_server.start()



    run()

    exit('end')
  






# # 初始化Pygame
# print("init pygame")
# pygame.init()
# pygame.font.init()

# myfont = pygame.font.Font(None,fontSize)

# # 创建屏幕对象
# screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
# pygame.display.set_caption("ROS物体坐标和颜色")

# # ROS节点初始化
# rospy.init_node('object_display')
# # 待生成方块的坐标和颜色
# print("init ros node finished")
# rospy.Subscriber('/incremental_dsg_builder_node/pgmo/object_info', String, callback)
# # 游戏循环
# running = True
# print("ready running pygame")
# while running:
#     # 处理事件
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         # 按下 Esc退出
#         if event.type == pygame.KEYDOWN:
#             if event.key == pygame.K_ESCAPE:
#                 running = False
#     # 绘制背景
#     screen.fill(BG_COLOR)
#     pygame.draw.rect(screen, color1,t1 )
#     pygame.draw.rect(screen, color2,t2 )
#     pygame.draw.rect(screen, color3,t3 )
#     # 更新屏幕
#     # 绘制所有物体方块
#     for box in pos_color_list:
#         pos, color = box
#         pygame.draw.rect(screen, color, (pos[0] - BOX_SIZE // 2, pos[1] - BOX_SIZE // 2, BOX_SIZE, BOX_SIZE))
#     # 更新屏幕
#     strforscore = ("Total Obj: "+str(total_objects)+"\nRoom1: "+str(each_room_objects[0])+"\nRoom2: "+str(each_room_objects[1])+"\nRoom3: "+str(each_room_objects[2]))
#     lines = strforscore.splitlines()
#     for i, l in enumerate(lines):
#         screen.blit(myfont.render(l, True, colors[i]), (0, 0 + fontSize*i))
#     pygame.display.flip()
# # 退出Pygame
# pygame.quit()


# gui_thread = threading.Thread(target=run, args=())
# gui_thread.start()

# gui_thread.join()
# udp_done_server.join()
# spawn_thread.join()
# actions_thread.join()

# exit('Bye')








