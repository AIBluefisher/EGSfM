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

OPENMVG_SFM_BIN = "EGSfM/build/ext/openMVG/Linux-x86_64-Release"
I23DSFM_SFM_BIN = "GraphSfM/build/Linux-x86_64-Release"
SIMILARITY_SEARCH_BIN = "EGSfM/build/bin"
VIEWING_GRAPH_MATCH_EXE = "EGSfM/bin/vg_matching"

# Indicate the i23dSFM camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "EGSfM/ext/openMVG/openMVG/exif/sensor_width_database"

import commands
import os
import subprocess
import sys
import time
import shutil

if len(sys.argv) < 3:
    print ("Usage %s image_dir output_dir" % sys.argv[0])
    sys.exit(1)

input_dir = sys.argv[1]
output_dir = sys.argv[2]
matches_dir = os.path.join(output_dir, "matches")
reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

print ("Using input dir  : ", input_dir)
print ("      output_dir : ", output_dir)

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

file_path = output_dir + "/sfm_timerecord.txt"
flog=open(file_path,'w')


flog.write("before intrinsics analysis \t\t" + time.asctime(time.localtime(time.time())) + '\n')
print ("\033[31m step 1 : Before intrinsics analysis\033[00m " + time.asctime(time.localtime(time.time())))
pIntrisics = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_SfMInit_ImageListing"), "-e","1", "-i", input_dir, "-g","1","-o", matches_dir, "-d", camera_file_params] )
pIntrisics.wait()

# start=time.time()
print ("\033[31m step 2 : Before compute features\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before compute features \t\t" + time.asctime(time.localtime(time.time())) + '\n')
pFeatures = subprocess.Popen([os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT", "-p", "HIGH" ])
# pFeatures = subprocess.Popen([os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT", "-p", "HIGH" ])
pFeatures.wait()

# end=time.time()
# print("sift spend time:"+str(end-start))
# start=end

print ("\033[31m step 3 : Before Similarity search\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before Similarity search \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
if not os.path.exists(matches_dir+"/vocab_out/match.out"):
  sift_list = os.path.join(matches_dir, "sift_list")
  pList = os.popen("ls -d " + matches_dir + "/*.desc > " + sift_list).read()
  pSearch = subprocess.Popen([os.path.join(SIMILARITY_SEARCH_BIN, "image_search"), sift_list, matches_dir+"/vocab_out", "6", "8", "1"])
  pSearch.wait()


print ("\033[31m step 4 : before compute matches\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before compute matches \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
pMatches = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeMatches"), "-i", matches_dir+"/sfm_data.json", "-o", matches_dir,"-G","0"] )
pMatches.wait()

# print("match spend time:"+str(time.time()-start))


# Create the reconstruction if not present
if not os.path.exists(reconstruction_dir):
    os.mkdir(reconstruction_dir)

print ("\033[31m step 5 : Before Incremental reconstruction\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before Incremental reconstruction \t" + time.asctime(time.localtime(time.time())) + '\n')
pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_IncrementalSfM"),  "-i", matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir, "-r", "1"] )
pRecons.wait()

print ("\033[31m step 6 : Before colorize structure\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before colorize structure \t\t" + time.asctime(time.localtime(time.time())) + '\n')
pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  "-i", reconstruction_dir+"/sfm_data.json", "-o", os.path.join(reconstruction_dir,"colorized.ply")] )
pRecons.wait()

# optional, compute final valid structure from the known camera poses
# print ("\033[31m step 7 : Before structure from known poses\033[00m " + time.asctime(time.localtime(time.time())))
# flog.write("before structure from known posses \t" + time.asctime(time.localtime(time.time())) + '\n')
# pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeStructureFromKnownPoses"),  "-i", reconstruction_dir+"/sfm_data.json", "-m", matches_dir, "-f", os.path.join(matches_dir, "matches.f.txt"), "-o", os.path.join(reconstruction_dir,"robust.json")] )
# pRecons.wait()

# print ("\033[31m step 8 : Before write ply files\033[00m " + time.asctime(time.localtime(time.time())))
# flog.write("before write ply files \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
# pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  "-i", reconstruction_dir+"/robust.json", "-o", os.path.join(reconstruction_dir,"robust_colorized.ply"), "-p", os.path.join(reconstruction_dir,"photo_align.ply"), "-m", reconstruction_dir] )
# pRecons.wait()

print ("\033[31m finish time\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("finish time \t\t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
flog.close()

