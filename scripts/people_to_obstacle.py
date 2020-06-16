#!/usr/bin/env python

import rospy, math, tf
#import roslib
import rostopic
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point32
#import geometry_msgs.msg
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
#import genpy
#from bson import Binary


from people_msgs.msg import People, Person

def callback(data):
    obstacle_array = ObstacleArrayMsg()
    
    obstacle_array.header = data.header

    for person in data.people:
        obstacle_msg = ObstacleMsg()
        
        obstacle_msg.id = int(person.name)
        obstacle_msg.radius = 0.5
        obstacle_msg.polygon.points = [Point32()]
        obstacle_msg.polygon.points[0].x = person.position.x
        obstacle_msg.polygon.points[0].y = person.position.y
        obstacle_msg.polygon.points[0].z = person.position.z
        
        yaw = math.atan2(person.velocity.x, person.velocity.y)
        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        #obstacle_msg.orientation = Quaternion(*q)
        obstacle_msg.orientation.x = q[0]
        obstacle_msg.orientation.y = q[1]
        obstacle_msg.orientation.z = q[2]
        obstacle_msg.orientation.w = q[3]
        obstacle_msg.velocities.twist.linear.x = person.velocity.x 
        obstacle_msg.velocities.twist.linear.y = person.velocity.y
        obstacle_msg.velocities.twist.linear.z = 0
        obstacle_msg.velocities.twist.angular.x = 0
        obstacle_msg.velocities.twist.angular.y = 0
        obstacle_msg.velocities.twist.angular.z = 0

        obstacle_array.obstacles.append(obstacle_msg)    
    obstacle_publisher.publish(obstacle_array)



if __name__ == '__main__':

    rospy.init_node('people_to_obstacle')
    obstacle_publisher = rospy.Publisher("~/robot_0/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=0)
    rospy.Subscriber("people", People, callback)
    rospy.spin()
    
