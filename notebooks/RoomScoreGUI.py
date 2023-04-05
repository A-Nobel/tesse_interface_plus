#-*-coding:utf-8-*-
import pygame,sys,random
from pygame.locals import *
import pygame
import rospy
from std_msgs.msg import  String

r1 =0
r2 =0
r3 =0
box_list = []
totalOb =0
fontSize =50
multi = 18
plus = 500
colorT = (255,0,0)
color1= (22,22,22)
color2=(33,33,220)
color3=(22,202,22)

# 定义屏幕尺寸和背景颜色
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 1200
BG_COLOR = (255, 255, 255)

# 定义方块尺寸和默认颜色
BOX_SIZE = 10
DEFAULT_COLOR = (255, 0, 0)
colors = [colorT,color1,color2,color3]
t = tuple(int(x*multi+plus) for x in (-14.5,27.5,12,12))
t1 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
t = tuple(int(x*multi+plus) for x in (-16,9.5,15.5,12))
t2 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
t = tuple(int(x*multi+plus) for x in (4.5,27.5,3,8))
t3 = tuple((t[0],t[1]-plus,t[2]-plus,t[3]-plus))
# 判断点是否在矩形内的函数
def is_point_in_rect(t, p):
    rect = pygame.Rect(t[0], t[1], t[2], t[3])
    return rect.collidepoint(p[0], p[1])

# 订阅ROS话题，接收物体二维坐标和颜色信息
def callback(data):
    global box_list
    global r1,r2,r3
    global totalOb 
    tr1 = 0
    tr2 = 0
    tr3 = 0

    if(len(data.data)<5):
        return
    # 解析接收到的字符串消息
    data_list = data.data.split(';')
    box_list = []
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
        if(is_point_in_rect(t1,pos)):
            tr1+=1
        elif(is_point_in_rect(t2,pos)):
            tr2+=1
        elif(is_point_in_rect(t3,pos)):
            tr3+=1
        box_list.append((pos, color))
    r1=tr1
    r2=tr2
    r3=tr3

    totalOb = len(box_list)

def run():
    global box_list,totalOb
    global r1,r2,r3
    # 初始化Pygame
    pygame.init()
    pygame.font.init()

    myfont = pygame.font.Font(None,fontSize)

    # 创建屏幕对象
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROS物体坐标和颜色")

    # ROS节点初始化
    rospy.init_node('object_display')
    # 待生成方块的坐标和颜色


    rospy.Subscriber('/incremental_dsg_builder_node/pgmo/object_info', String, callback)
    # 游戏循环
    running = True

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
        pygame.draw.rect(screen, color1, t1 )
        pygame.draw.rect(screen, color2,t2 )
        pygame.draw.rect(screen, color3,t3 )
        # 更新屏幕
        # 绘制所有物体方块
        for box in box_list:
            pos, color = box
            pygame.draw.rect(screen, color, (pos[0] - BOX_SIZE // 2, pos[1] - BOX_SIZE // 2, BOX_SIZE, BOX_SIZE))
        # 更新屏幕
        strforscore = ("Total Obj: "+str(totalOb)+"\nRoom1: "+str(r1)+"\nRoom2: "+str(r2)+"\nRoom3: "+str(r3))
        lines = strforscore.splitlines()
        for i, l in enumerate(lines):
            screen.blit(myfont.render(l, True, colors[i]), (0, 0 + fontSize*i))
        pygame.display.flip()
    # 退出Pygame
    pygame.quit()


if __name__ == '__main__':
    run()
# y（40.5-44.5）x（13.5-31.5）zoulang1
# bangongshi 1 full of gongwei x(13.5 - 29.5) y(29.5 40.5) + x(13.5 - 23) y(22.5 30)
# secret office x(23 29.5) y (22.5 30)
# meeting room x(7.5 13.5) y(27.5 35.5)  x 7.65-9.6-11.38-13.124 y35.07 32.9  men x13.5 y 31.5
# toilet x(4.5 7.5) y(27.5 35.5)

# close office 1 x(-14.5, -2.5)y(27.5,39.5)
#close office 2 x(-0.5 -16) y(9.5 21.5 )
# -0.5 21.5
''' -8.7 -11.2 18.4 21.5

-10  20,  -1.5 20, -1.5 11.5, -10 11.5  ,-14.5 11.5,-14.5 20
name x_0 y_0 x_l y_l
office 1 -14.5,27.5,12,12
office 2 -16,9.5,15.5,12
toilet 4.5,27.5,3,8
'''
