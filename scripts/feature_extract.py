#!/usr/bin/python
#! -*- encoding: utf-8 -*-

# Python implementation of the bash script written by Romuald Perrot
# Created by @vins31
# Modified by YuChen
#
# this script is for easy use of I23dSFM
#
# usage : python i23dsfm.py image_dir output_dir
#
# image_dir is the input directory where images are located
# output_dir is where the project must be saved
#
# if output_dir is not present script will create it
#

# Indicate the i23dSFM binary directory
OPENMVG_SFM_BIN = "/home/chenyu/projects/XFeatures/build/ext/openMVG/Linux-x86_64-Release"
I23DSFM_SFM_BIN = "/home/chenyu/projects/GraphSfM/build/Linux-x86_64-Release"
VIEWING_GRAPH_BIN = "/home/chenyu/projects/XFeatures/bin"

# Indicate the i23dSFM camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/chenyu/projects/openMVG/src/openMVG/exif/sensor_width_database"

import commands
import os
import subprocess
import sys
import time
import shutil

if len(sys.argv) < 3:
    print ("Usage: %s image_dir output_dir" % sys.argv[0])
    sys.exit(1)

input_dir = sys.argv[1]
output_dir = sys.argv[2]

matches_dir = os.path.join(output_dir, "matches")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

print ("Using input  dir  : ", input_dir)
print ("      output_dir : ", output_dir)

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

file_path = output_dir + "/sfm_time_record.txt"
flog = open(file_path, 'w')


flog.write("before intrinsics analysis \t\t" + time.asctime(time.localtime(time.time())) + '\n')
print ("\033[31m step 1 : Before intrinsics analysis\033[00m " + time.asctime(time.localtime(time.time())))
pIntrisics = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_SfMInit_ImageListing"), 
"-e","1", "-i", input_dir, "-g","1","-o", matches_dir, "-d", camera_file_params] )
pIntrisics.wait()


print ("\033[31m step 2 : Before compute features\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before compute features \t\t" + time.asctime(time.localtime(time.time())) + '\n')
pFeatures = subprocess.Popen([os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  
                             "-i", matches_dir + "/sfm_data.json", 
                             "-o", matches_dir, 
                             "-m", "SIFT", 
                             "-p", "HIGH" ])
pFeatures.wait()

sift_list = os.path.join(matches_dir, "sift_list")
pList = os.popen("ls -d " + matches_dir + "/*.desc > " + sift_list).read()

print ("\033[31m finish time\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("finish time \t\t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
flog.close()
