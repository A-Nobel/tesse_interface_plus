#-*-coding:utf-8-*-
import rospy
from std_msgs.msg import String

rospy.init_node("test_sub", anonymous=True)#初始化节点 名称：test
def call_back_String(msg):
    print(msg)
    print("Received")

def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/incremental_dsg_builder_node/pgmo/obj',String, call_back_String)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException:
        pass