import rospy
from std_msgs.msg import String

def publish_positions():
    pub = rospy.Publisher('angles', String, queue_size=10)
    rospy.init_node('stepper_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        positions = "100,-200,-300"  # Example positions for stepper motors
        rospy.loginfo(positions)
        pub.publish(positions)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_positions()
    except rospy.ROSInterruptException:
        pass
