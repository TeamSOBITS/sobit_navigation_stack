#!/usr/bin/env python
# coding: utf-8

import subprocess
import datetime
import rospy
import roslib.packages

print "作った地図を保存するときは[Enter]キーを押してください。"
Enter_key_trig = str(raw_input("\n\nWhen you save this map, press the 'Enter' key! : "))

map_dir_path = rospy.get_param('map_save_path')
map_name = "map_" + str(datetime.datetime.today().month) + "_" + str(datetime.datetime.today().day) + "_" + str(datetime.datetime.today().hour) + "_" + str(datetime.datetime.today().minute)

cmd = "rosrun map_server map_saver -f " + map_dir_path + map_name

print "\n\n" + map_dir_path + map_name
print "\n\nPlease wait...\n\n"

subprocess.call(cmd, shell=True)# Do the command

print "\n\n" + map_dir_path + map_name
print "\n\nOK.finished. Please check it out!"

print "\n\n 閉じるときは❌ボタンで。"
while True:
	rospy.sleep(10)#sleep
