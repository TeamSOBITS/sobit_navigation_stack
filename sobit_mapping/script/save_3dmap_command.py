#!/usr/bin/env python
# coding: utf-8

import subprocess
import datetime
import rospy
import roslib.packages

print "作った地図を保存するときは[Enter]キーを押してください。"
Enter_key_trig = str(raw_input("\n\nWhen you save this map, press the 'Enter' key! : "))

map_dir_path = rospy.get_param('/save_3dmap_command/map_save_path')
map_name_3d = "map_3d_" + str(datetime.datetime.today().month) + "_" + str(datetime.datetime.today().day) + "_" + str(datetime.datetime.today().hour) + "_" + str(datetime.datetime.today().minute) + ".bt"
cmd_3d = "rosrun octomap_server octomap_saver -f " + map_dir_path + map_name_3d

map_name_2d_multi = "map_2d_multi_" + str(datetime.datetime.today().month) + "_" + str(datetime.datetime.today().day) + "_" + str(datetime.datetime.today().hour) + "_" + str(datetime.datetime.today().minute) + ".bt"
cmd_3d_tmp = "rosrun octomap_server octomap_saver -f " + map_dir_path + map_name_2d_multi 
cmd_2d_multi = "rosrun octomap_server octomap_server_multilayer " + map_dir_path + map_name_2d_multi

map_name_2d = "map_2d_" + str(datetime.datetime.today().month) + "_" + str(datetime.datetime.today().day) + "_" + str(datetime.datetime.today().hour) + "_" + str(datetime.datetime.today().minute)
cmd_2d = "rosrun map_server map_saver -f " + map_dir_path + map_name_2d

print "\n\n" + map_dir_path + map_name_3d
print "\n\nPlease wait...\n\n"
subprocess.call(cmd_3d, shell=True)# Do the command

print "\n\n" + map_dir_path + cmd_2d_multi
print "\n\nPlease wait...\n\n"
subprocess.call(cmd_3d_tmp, shell=True)# Do the command
subprocess.call(cmd_2d_multi, shell=True)# Do the command

print "\n\n" + map_dir_path + map_name_2d
print "\n\nPlease wait...\n\n"
subprocess.call(cmd_2d, shell=True)# Do the command

print "\n\nOK.finished. Please check it out!"

print "\n\n 閉じるときは❌ボタンで。"
while True:
	rospy.sleep(10)#sleep
