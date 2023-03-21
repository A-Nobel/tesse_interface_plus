#-*-coding:utf-8-*-
import rospy
from std_msgs.msg import String

def call_back_String(msg):
    print(msg.data)
    print("Received")

def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/incremental_dsg_builder_node/pgmo/object_info',String, call_back_String)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        print(""+str(e))