# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

rospy.sleep(1)

# Hier wird der Funktion der Mittelpunkt der BB übergeben
def bbox_center(data):
    # Ausgabe wie groß X aktuell ist
    x = data.x
    z = data.z
    print(x)

    rospy.loginfo(data)

    # for i in data.bounding_boxes:
    #     rospy.loginfo(
    #          "x: {}, y: {}, z: {}".format(
    #             i.x, i.y, i.z
    #         )
    #     )

    #Einteilung der BB Winkel in 5 Bereiche:
    move_cmd = Twist()
    if z < 500: # Wenn Person zu nah erkannt wird
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        print('Person zu nah')
    else:
        if x <128: # weit links
            move_cmd.linear.x =  0.0
            move_cmd.angular.z = 1.0
            print('Person ganz weit links, fahre nach links')
        elif x >129 and x < 256: # links
            move_cmd.linear.x =  0.15
            move_cmd.angular.z = 1.0
            print('Person links, fahre nach links')
        elif x >257 and x < 384: # mittig, fährt nur gerade aus
            move_cmd.linear.x =  0.2
            move_cmd.angular.z = 0.0
            print('Person mittig, fahre geradeaus')
        elif x >385 and x < 512: # rechts
            move_cmd.linear.x =  0.15
            move_cmd.angular.z = -1.0
            print('Person rechts, fahre nach rechts')
        elif x >513 and x < 640: # weit rechts
            move_cmd.linear.x =  0.0
            move_cmd.angular.z = -1.0
            print('Person ganz weit rechts, fahre nach rechts')
        else: # wenn kein x gefunden wird, stehen bleiben
            move_cmd.linear.x =  0.0
            move_cmd.angular.z = 0.0
            print('keine Person gefunden')

    now = rospy.Time.now()
    rate = rospy.Rate(10)

    # Solange X vorhanden ist, sollen Nachrichten geschickt werden
    #while rospy.loginfo(data) != 0:
    while rospy.Time.now() < now + rospy.Duration.from_sec(0.1):
        # if bbox_center == 0:
            # move_cmd.linear.x = 0.0
            # move_cmd.angular.z = 0.0
        pub_vel.publish(move_cmd)
        rate.sleep()
        # else:
            # pub_vel.publish(move_cmd)
            # rate.sleep()

rospy.init_node('person_drive', anonymous=True)
pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size=5)
rospy.Subscriber('cmd_vel', Twist, queue_size=5)
rospy.Subscriber('/person_tracking/bbox_center', Point, bbox_center, queue_size=5)

rospy.spin()