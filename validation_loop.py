import roslaunch
import rospy
try:
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/anshul/catkin_ws/src/carla_ros-bridge/client_cmcdot.launch"])
    launch.start()
    rospy.loginfo("started")

    rospy.sleep(60)
    # 3 seconds later

except rospy.ROSInternalException:
    print("killing roslaunch")
    launch.shutdown()

finally:
    launch.shutdown()
