import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np


# write a publisher that publises the obstacles in the world as a float32multiarrar of size 16 

def pub_obstacles():   
    pub = rospy.Publisher('obstacles', Float32MultiArray, queue_size=10)
    rospy.init_node('obstacle_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        obstacles = Float32MultiArray()
        obstacles.data = np.random.rand(16)*10  # put a range for values
        pub.publish(obstacles)    



if __name__ == '__main__':
    try:
        pub_obstacles()
    except rospy.ROSInterruptException:
        pass
