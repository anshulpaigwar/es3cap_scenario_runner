#!/usr/bin/env python
"""
Follow a ego vehicle in carla using pure pursuit algorithm
author: Anshul paigwar

To detect if two object would collide read the link below
https://gamedev.stackexchange.com/questions/97337/detect-if-two-objects-are-going-to-collide
https://cocalc.com/projects/9288750d-aa44-43a9-8416-730920bddba8/files/Detect%20collision%20of%20to%20moving%20circles.sagews?session=default
https://codepen.io/sveinatle/pen/OPqLKE?editors=011

"""
import os
import errno
import numpy as np
import math
import matplotlib.pyplot as plt
import argparse
from argparse import RawTextHelpFormatter



import rospy
import tf
from tf import TransformListener
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import Object, ObjectArray


# from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker
from message_filters import TimeSynchronizer, Subscriber
from e_motion_perception_msgs.msg import FloatOccupancyGrid

from scenario_runner_cmcdot import *

# pub = rospy.Publisher('test_riskGrid', FloatOccupancyGrid, queue_size=10)


grid_x_min = -10
grid_y_min = -30
resolution = 0.1
agent_frame_id = "test_vehicle"





class scenario_params(object):
    def __init__(self):
        self.ego_vehicle_vel = 0
        self.other_vehicle_vel = 0






class vehicle(object):

    def __init__(self, agent_frame_id):
        self.frame_id = agent_frame_id
        self.l = 0
        self.w = 0
        self.h = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel = 0
        self.pose = PoseStamped()
        self.cmd_vel_subscriber = rospy.Subscriber('/carla/objects', ObjectArray, self.obj_callback)

    def obj_callback(self, objectArray):
        # print(objectArray)
        obj = [object for object in objectArray.objects if object.header.frame_id == self.frame_id]
        if not obj:
            return
        vehicle_obj = obj[0]
        self.pose.pose = vehicle_obj.pose
        self.pose.header.frame_id = vehicle_obj.header.frame_id
        self.l = vehicle_obj.shape.dimensions[0]
        self.w = vehicle_obj.shape.dimensions[1]
        self.h = vehicle_obj.shape.dimensions[2]
    	# print(self.l, self.w, self.h, self.frame_id)

        self.vel_x = vehicle_obj.twist.linear.x
        self.vel_y = vehicle_obj.twist.linear.y
        self.vel = math.sqrt(self.vel_x**2 + self.vel_y**2)
        # print(self.vel)
        self.box_coord = np.array([[self.l/2, -self.w/2], [self.l/2, self.w/2], [-self.l/2, self.w/2], [-self.l/2, -self.w/2]])




class grid(object):

    def __init__(self, x_min, y_min, res,  target, zoe, collision):
        self.target = target
        self.zoe = zoe
        self.grid_x_min = x_min
        self.grid_y_min = y_min
        self.res = res
        self.tf_listener = tf.TransformListener()
        self.grid_sub = rospy.Subscriber('/zoe/risk_grid', FloatOccupancyGrid, self.callback)
        self.trace = []
        self.collision = collision
    	self.prev_collision_status = 0
        self.prev_time_stamp = 0.0
        self.skips = 0.0



    def callback(self, risk_grid):

        # tranform the target vehicle in the frame of risk_grid (zoe/base_link)
        (trans,rot) = self.tf_listener.lookupTransform(risk_grid.header.frame_id, self.target.pose.header.frame_id, rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion (rot)

        # origin of grid is at (grid_x_min,grid_y_min) with respect to zoe/base_link
        cx = trans[0] - self.grid_x_min
        cy = trans[1] - self. grid_y_min
        trans_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        center = np.array([cx, cy]) # center of the target vehicle w.r.t grid origin

        box_coord = np.dot(self.target.box_coord, trans_matrix) + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        box_coord = box_coord/self.res # bounding box coordinate cell number

        xmax,ymax = box_coord.max(axis=0).astype(int)
        xmin,ymin = box_coord.min(axis=0).astype(int)


        # print("zoe:", self.zoe.vel)
        # print("target", self.target.vel)
        # print(cx, cy ,xmax,ymax,xmin,ymin)
        # print(self.target.l, self.target.w)


        # check that bounding box is not out of grid index
        if((xmin < 0) or (ymin < 0) or (ymax > risk_grid.info.height) or (xmax > risk_grid.info.width)):
            return

        if(risk_grid.header.stamp.to_sec() == self.prev_time_stamp):
            self.skips +=1
            return

        risk_arr = (np.array(risk_grid.data)*255).astype('uint8')
        risk_arr = risk_arr.reshape(risk_grid.info.height, risk_grid.info.width, risk_grid.nb_channels)

        max_risk_1sec = risk_arr[ymin:ymax, xmin:xmax,0].max()
        max_risk_2sec = risk_arr[ymin:ymax, xmin:xmax,1].max()
        max_risk_3sec = risk_arr[ymin:ymax, xmin:xmax,2].max()


        # else:
        #     center = (center/self.res).astype(int)
        #     # print(xmax,ymax,xmin,ymin)
        #     # print("size zero")
        #     max_risk_1sec = risk_arr[center[1], center[0],0]
        #     max_risk_2sec = risk_arr[center[1], center[0],1]
        #     max_risk_3sec = risk_arr[center[1], center[0],2]


        will_collide, time = compute_collision_risk(self.zoe, self.target)



    	if self.prev_collision_status == 0:
    		self.prev_collision_status = self.collision.status

        self.trace.append([risk_grid.header.stamp.to_sec(),
                    self.zoe.vel, self.target.vel,
                    max_risk_1sec, max_risk_2sec, max_risk_3sec,
                    -self.grid_x_min, -self.grid_y_min, cx, cy, self.prev_collision_status, will_collide, time])

        self.prev_time_stamp = risk_grid.header.stamp.to_sec()

        # test_grid = risk_grid
        # test_arr = np.zeros_like(risk_arr)
        # test_arr[ymin:ymax,xmin:xmax,:] = 1
        # test_arr = test_arr.flatten().tolist()
        # test_grid.data = test_arr
        # pub.publish(test_grid)





class CollisionCheck(object):
    def __init__(self):
        self.status = False
        self.grid_sub = rospy.Subscriber('collision_check', Bool, self.callback)

    def callback(self,msg):
        self.status = msg.data





def compute_collision_risk(zoe , target):
    a_x = zoe.pose.pose.position.x
    a_y = zoe.pose.pose.position.y
    a_vx = zoe.vel_x
    a_vy = zoe.vel_y

    b_x = target.pose.pose.position.x
    b_y = target.pose.pose.position.y
    b_vx = target.vel_x
    b_vy = target.vel_y

    # print("zoe:", zoe.vel)
    # print("target", target.vel)

    will_collide = False
    collision_dist = abs(math.sqrt((zoe.l/2 + target.w/2)**2 + (zoe.w/2 + target.l/2)**2))
    # collision_dist = zoe.l/2 + target.l/2



    # time to reach the minimum distance
    denominotor = (a_vx**2 - 2*a_vx*b_vx + b_vx**2 + a_vy**2 - 2*a_vy*b_vy + b_vy**2)
    if denominotor == 0:
        minDist_time = -1
        return will_collide, minDist_time

    minDist_time = -(a_x*a_vx - a_vx*b_x - (a_x - b_x)*b_vx + a_y*a_vy - a_vy*b_y - (a_y - b_y)*b_vy)/denominotor


    # dx = a_x-b_x
    # dy = a_y-b_y
    # dvx = a_vx - b_vx
    # dvy = a_vy - b_vy

    # A = (dvx**2 + dvy**2)
    # if A == 0:
	# minDist_time = -1
	# return will_collide, minDist_time

    # B = 2*(dx * dvx + dy * dvy)
    # C = dx**2 + dy**2 - collision_dist**2
    # collision_time = (-2*B - math.sqrt(B**2 - 4 *A *C)) / (2*A)
    # # print(collision_time)


    # print(collision_dist, math.sqrt((a_x -b_x)**2 + (a_y-b_y)**2),collision_time,minDist_time )

    t = minDist_time
    minDist = math.sqrt((t*a_vx - t*b_vx + a_x - b_x)**2 + (t*a_vy - t*b_vy + a_y - b_y)**2)
    # print("minidist: ", minDist)
    if minDist < collision_dist:
        will_collide = True
    return will_collide, minDist_time













def listener():

    rospy.init_node('risk_grid_valid', anonymous=True)
    # rate = rospy.Rate(10) # 10hz

    ARGUMENTS.scenario = "PedestrianCrossingRisk"
    # ARGUMENTS.scenario = "RiskEstimation"
    runner = ScenarioRunner(ARGUMENTS)

    params = scenario_params()
    collision = CollisionCheck()
    zoe = vehicle(agent_frame_id= "zoe/zoe_odom_origin")
    target = vehicle(agent_frame_id= "pedestrian_0")
    # target = vehicle(agent_frame_id= "test_vehicle_0")
    risk_grid = grid(-10,-30,0.1,target, zoe, collision)

    all_trace_dir = "/home/anshul/enable-s3/traces/risk_grid_pedestrian/"


    for variation in range(1):
        files = os.listdir(all_trace_dir)
        new_trace_dir = all_trace_dir + "%06d/" % len(files)


        uuidx = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuidx)
        launchx = roslaunch.parent.ROSLaunchParent(uuidx, ["/home/anshul/catkin_ws/src/chroma-irt/zoe_ros_apps/launch/cmcdot_carla.launch"])
        launchx.start()
        rospy.loginfo("cmcdot started")
        time.sleep(3)


        for trace_num in range(10):
            params.ego_vehicle_vel = 8.2 #+ 0.5* float(ARGUMENTS.var1)
            # params.other_vehicle_vel = 8.7
            params.other_vehicle_vel = 2.8 #params.ego_vehicle_vel -1  #+0.25 * float(ARGUMENTS.var2)
            # scenario = Recorder(risk_grid)
	    # append the parameters used for generating the trace so that trace can be reproduced
            risk_grid.trace = [[0,params.ego_vehicle_vel,params.other_vehicle_vel,0,0,0,0,0,0,0,0,0,0]]
            risk_grid.skips = 0
            risk_grid.prev_collision_status = 0

            runner.run(ARGUMENTS, params)

            print("/n Number of timestep skipped %d"%risk_grid.skips)

            if ARGUMENTS.save:
                try:
                    os.makedirs(new_trace_dir)
                except OSError as e:
                    if e.errno != errno.EEXIST:
                        raise

                trace_path = new_trace_dir + "%06d.txt" % trace_num
                np.savetxt(trace_path, risk_grid.trace,fmt='%.4f', delimiter=' ')

        launchx.shutdown()
        time.sleep(2)
        rospy.loginfo("cmcdot killed")


if __name__ == '__main__':
    listener()






        # # TODO create an object that will be called everytime I recive status value
        # collision_check = run_scenario(params)
        # # trace = np.around(np.array(scenario.trace),3)


    # while not rospy.is_shutdown():
    #     will_collide, time = compute_collision_risk(zoe, target)
    #     # if will_collide:
    #     #     print("cars will collide within {:0.2f} sec".format(time))
    #     # elif time != -1:
    #     #     print("cars will not collide, minimum distance within {:0.2f} sec".format(time))
    #     rate.sleep()




            # if collision_check == True:
            #     print("It is a Failure")
            #     risk_grid.trace[-1][-1] = 1


    # vehicle_marker_sub = message_filters.Subscriber('vehicles', MarkerArray)
    # risk_grid_sub = message_filters.Subscriber('/zoe/risk_grid', FloatOccupancyGrid)
    #
    # ts = TimeSynchronizer([vehicle_marker_sub,risk_grid_sub],10)
    # ts.registerCallback(callback)



        # obj_risk_list = []
        # for i in range(xmin,xmax):
        #     for j in range(ymin, ymax):
        #
        #         # risk = risk_grid.data[i*num_col + j]
        #         obj_risk_list.append(risk)


        # print(self.target.l, self.target.w, self.target.h)
        # print(center, yaw)
