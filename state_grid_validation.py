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
# from PIL import Image
import cv2



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

pub = rospy.Publisher('test_riskGrid', FloatOccupancyGrid, queue_size=10)


grid_x_min = -28
grid_y_min = -28
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
        self._margin = 0.6 #(meters)
        self._focus_margin = 2
        self.cmd_vel_subscriber = rospy.Subscriber('/carla/objects', ObjectArray, self.obj_callback)

    def obj_callback(self, objectArray):
        # print(objectArray)
        obj = [object for object in objectArray.objects if object.header.frame_id == self.frame_id]
        if not obj:
            return
        vehicle_obj = obj[0]
        self.pose.pose = vehicle_obj.pose
        self.pose.header.frame_id = vehicle_obj.header.frame_id
        self.l = vehicle_obj.shape.dimensions[0] + self._margin
        self.w = vehicle_obj.shape.dimensions[1] + self._margin
        self.h = vehicle_obj.shape.dimensions[2] + self._margin

        self.vel_x = vehicle_obj.twist.linear.x
        self.vel_y = vehicle_obj.twist.linear.y
        self.vel = math.sqrt(self.vel_x**2 + self.vel_y**2)
        self.box_coord = np.array([[self.l/2, -self.w/2], [self.l/2, self.w/2], [-self.l/2, self.w/2], [-self.l/2, -self.w/2]])
        l = self.l + self._focus_margin
        w = self.w + self._focus_margin
        h = self.h + self._focus_margin
        self.focus_coord = np.array([[l/2, -w/2], [l/2, w/2], [-l/2, w/2], [-l/2, -w/2]])




class grid(object):

    def __init__(self, x_min, y_min, res,  target, zoe):
        self.target = target
        self.zoe = zoe
        self.grid_x_min = x_min
        self.grid_y_min = y_min
        self.res = res
        self.tf_listener = tf.TransformListener()
        self.grid_sub = rospy.Subscriber('/zoe/state_grid', FloatOccupancyGrid, self.callback)
        self.trace = []
        self.gt_focus = [] # Ground Truth Focus
        self.cmcdot_focus = [] # output from cmcdot Focus




    def callback(self, state_grid):

        # tranform the target vehicle in the frame of state_grid (zoe/base_link)
        (trans,rot) = self.tf_listener.lookupTransform(state_grid.header.frame_id, self.target.pose.header.frame_id, rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion (rot)

        # origin of grid is at (grid_x_min,grid_y_min) with respect to zoe/base_link
        cx = trans[0] - self.grid_x_min
        cy = trans[1] - self. grid_y_min
        trans_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        center = np.array([cx, cy]) # center of the target vehicle w.r.t grid origin

        box_coord = np.dot(self.target.box_coord, trans_matrix) + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        box_coord = box_coord/self.res # bounding box coordinate cell number

        xmax_box,ymax_box = box_coord.max(axis=0).astype(int)
        xmin_box,ymin_box = box_coord.min(axis=0).astype(int)




        focus_coord = np.dot(self.target.focus_coord, trans_matrix) + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        focus_coord = focus_coord/self.res # bounding box coordinate cell number

        xmax_focus,ymax_focus = focus_coord.max(axis=0).astype(int)
        xmin_focus,ymin_focus = focus_coord.min(axis=0).astype(int)



        # print("zoe:", self.zoe.vel)
        # print("target", self.target.vel)
        # print(cx, cy ,xmax,ymax,xmin,ymin)

        # check that bounding box is not out of grid index
        if((xmin_focus < 0) or (ymin_focus < 0) or (xmax_focus > state_grid.info.height) or (ymax_focus > state_grid.info.width)):
            # print("target is out of grid")
            self.info = None
            return

        state_arr = (np.array(state_grid.data)*255).astype('uint8')
        state_arr = state_arr.reshape(state_grid.info.height, state_grid.info.width, state_grid.nb_channels)

        test_grid = state_grid
        test_arr = np.zeros_like(state_arr)
        # test_arr[ymin_focus:ymax_focus, xmin_focus:xmax_focus, 2] = 1

        if self.target.vel > 0.3 :
            test_arr[ymin_box:ymax_box, xmin_box:xmax_box, 1] = 255
        else:
            test_arr[ymin_box:ymax_box, xmin_box:xmax_box, 0] = 255

        self.gt_focus.append(test_arr[ymin_focus:ymax_focus, xmin_focus:xmax_focus,:3])
        self.cmcdot_focus.append(state_arr[ymin_focus:ymax_focus, xmin_focus:xmax_focus,:3])

        self.trace.append([state_grid.header.stamp.to_sec(),
                    self.zoe.vel, self.target.vel,
                    -self.grid_x_min, -self.grid_y_min, cx, cy])


        # test_arr = test_arr.flatten().tolist()
        # test_grid.data = test_arr
        # pub.publish(test_grid)







def listener():

    rospy.init_node('risk_grid_valid', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ARGUMENTS.scenario = "DynamicObjectDetect"
    runner = ScenarioRunner(ARGUMENTS)

    params = scenario_params()
    zoe = vehicle(agent_frame_id= "zoe/zoe_odom_origin")
    target = vehicle(agent_frame_id= "test_vehicle")
    state_grid = grid(-28,-28,0.1,target, zoe)

    all_trace_dir = "/home/anshul/enable-s3/traces/state_grid/"

    for i in range(7,15):

        files = os.listdir(all_trace_dir)

        params.ego_vehicle_vel = 8
        params.other_vehicle_vel =10

        # empty the trce informations stored
        state_grid.trace = []
        state_grid.gt_focus = []
        state_grid.cmcdot_focus = []

        runner.run(params)
        collision_check = runner.collision_check

        if ARGUMENTS.save:

            # We check how many number of folders are present in the trace dir then make a new folder with a higher Number
            # So that all the trace we record are in new folder
            new_trace_dir = all_trace_dir + "%06d/" % len(files)
            gt_focus_dir = new_trace_dir + "/true_focus/"
            cmcdot_focus_dir = new_trace_dir + "/cmcdot_focus/"

            try:
                os.makedirs(gt_focus_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
            try:
                os.makedirs(cmcdot_focus_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise

            np.savetxt(new_trace_dir + "data.txt", state_grid.trace,fmt='%.4f', delimiter=' ')

            for a in range(len(state_grid.gt_focus)):
                cv2.imwrite(gt_focus_dir + "%03d.png"%a, state_grid.gt_focus[a])
                cv2.imwrite(cmcdot_focus_dir + "%03d.png"%a, state_grid.cmcdot_focus[a])




if __name__ == '__main__':
    listener()



# Saving image using pillow:
                # pil_img = Image.fromarray(state_grid.gt_focus[a])
                # pil_img.save(gt_focus_dir + "%03d.png"%a)
                # pil_img = Image.fromarray(state_grid.cmcdot_focus[a])
                # pil_img.save(cmcdot_focus_dir + "%03d.png"%a)





    # while not rospy.is_shutdown():
    #     will_collide, time = compute_collision_risk(zoe, target)
    #     # if will_collide:
    #     #     print("cars will collide within {:0.2f} sec".format(time))
    #     # elif time != -1:
    #     #     print("cars will not collide, minimum distance within {:0.2f} sec".format(time))
    #     rate.sleep()







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





# class Recorder(object):
#     def __init__(self,risk_grid):
#         self.grid = risk_grid
#         self.trace = []
#         self.grid_sub = rospy.Subscriber('collision_check', Bool, self.callback)
#
#     def callback(self,msg):
#
#         self.status = msg.data
#         if self.grid.info == None:
#             return
#         trace_seq = self.grid.info + [self.status]
#         # formatted_trace = [ '%.2f' % elem for elem in trace_seq ]
#         # print(formatted_trace)
#         # self.trace.append(formatted_trace)
#         self.trace.append(trace_seq)









############################# IMPORTANT ######################################



def compute_collision_risk(zoe , target):
    a_x = zoe.pose.pose.position.x
    a_y = zoe.pose.pose.position.y
    a_vx = zoe.vel_x
    a_vy = zoe.vel_y

    b_x = target.pose.pose.position.x
    b_y = target.pose.pose.position.y
    b_vx = target.vel_x
    b_vy = target.vel_y

    print("zoe:", zoe.vel)
    print("target", target.vel)

    will_collide = False
    collision_dist = math.sqrt((zoe.l/2 + target.w/2)**2 + (zoe.w/2 + target.l/2)**2)

    # time to reach the minimum distance
    denominotor = (a_vx**2 - 2*a_vx*b_vx + b_vx**2 + a_vy**2 - 2*a_vy*b_vy + b_vy**2)
    if denominotor == 0:
        minDist_time = -1
        return will_collide, minDist_time

    minDist_time = -(a_x*a_vx - a_vx*b_x - (a_x - b_x)*b_vx + a_y*a_vy - a_vy*b_y - (a_y - b_y)*b_vy)/denominotor

    t = minDist_time
    minDist = math.sqrt((t*a_vx - t*b_vx + a_x - b_x)**2 + (t*a_vy - t*b_vy + a_y - b_y)**2)
    if minDist < collision_dist:
        will_collide = True
    return will_collide, minDist_time
