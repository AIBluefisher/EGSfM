#!/usr/bin/python
#! -*- encoding: utf-8 -*-

# Python implementation of the bash script written by Romuald Perrot
# Created by @vins31
# Modified by Chenyu
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

I23DSFM_SFM_BIN = "EGSfM/i23dSFM/build/Linux-x86_64-Release"
EGSfM_BIN = "EGSfM/bin"

# Indicate the i23dSFM camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "EGSfM/ext/openMVG/openMVG/exif/sensor_width_database"

import commands
import os
import subprocess
import sys
import time
import shutil

input_dir = sys.argv[1]
output_dir_name = sys.argv[2]
max_image_number = sys.argv[3]
# img_format = sys.argv[4]
completeness_ratio = "0.7"
matches_dir = os.path.join(output_dir_name, "matches")
blocks_path = matches_dir + "/blocks_path.txt"
cluster_dirs_file = os.path.join(input_dir, "clusters.txt")
matches_list_dirs_file = os.path.join(input_dir, "matches_list.txt")
# reconstruction_dir = os.path.join(output_dir_name, "reconstruction_sequential")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")


print ("Using input dir  : ", input_dir)
print ("      output_dir : ", output_dir_name)
# Create the ouput/matches folder if not present
if not os.path.exists(output_dir_name):
  os.mkdir(output_dir_name)
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

file_path = output_dir_name + "/sfm_timerecord.txt"
flog=open(file_path,'w')



################################################### Image Listing ##########################################
flog.write("Before Intrinsics Analysis \t\t" + time.asctime(time.localtime(time.time())) + '\n')
print ("\033[31m step 0 : Before Intrinsics Analysis\033[00m " + time.asctime(time.localtime(time.time())))
pIntrisics = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_SfMInit_ImageListing"), 
                                "-e","1", "-i", input_dir, 
                                "-g","1","-o", matches_dir, 
                                "-d", camera_file_params] )
pIntrisics.wait()


start = time.time()
print ("\033[31m step 1 : Before Compute Features\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before Compute Features \t\t" + time.asctime(time.localtime(time.time())) + '\n')
pFeatures = subprocess.Popen([os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeFeatures"),  
                            "-i", matches_dir+"/sfm_data.json", 
                            "-o", matches_dir, 
                            "-m", "SIFT", 
                            "-p", "HIGH" ])
pFeatures.wait()
end = time.time()
print("sift spend time:"+str(end-start))
start = end


print ("\033[31m step 2 : before compute matches\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before compute matches \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
pMatches = subprocess.Popen([os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeMatches"), 
                            "-i", matches_dir+"/sfm_data.json", 
                            "-o", matches_dir
                            ] )
pMatches.wait()

print("match spend time:"+str(time.time()-start))


################################################### Graph Cluster######################################
flog.write("Before Graph Cluster \t\t" + time.asctime(time.localtime(time.time())) + '\n')
print ("\033[31m step 3: Before Graph Cluster\033[00m " + time.asctime(time.localtime(time.time())))
if os.path.isfile(matches_dir + "/match.out"):
  shutil.move(matches_dir + "/match.out", input_dir+"/match.out")
gc = subprocess.Popen([os.path.join(EGSfM_BIN, "image_cluster"), 
                       "--abs_imglist_path=" + input_dir + "/image_list.txt", 
                       "--weight_file=" + input_dir + "/match.out", 
                       "--cluster_option=expansion", 
                       "--max_cluster_size=" + max_image_number, 
                       "--completeness_ratio=" + completeness_ratio, 
                       "--matches_dir=" + matches_dir, 
                       "--output_dir=" + output_dir_name,
                       "--copy_images=false"] )
gc.wait()


################################################Incremental SfM################################################

# Create the reconstruction if not present
f = open(cluster_dirs_file, 'r')
m = open(matches_list_dirs_file, 'r')
while True:
  line = f.readline()
  match_line = m.readline()
  if not line : break
  if not match_line : break

  reconstruction_dir_partial = line.strip('\n')
  matches_file = match_line.strip('\n')

  if not os.path.exists(reconstruction_dir_partial):
    os.mkdir(reconstruction_dir_partial)

  output_dir = reconstruction_dir_partial
  reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
  
  if not os.path.exists(reconstruction_dir):
    os.mkdir(reconstruction_dir)

  print ("\033[31m step 4 : Before Incremental Reconstruction\033[00m " + time.asctime(time.localtime(time.time())))
  flog.write("before Incremental reconstruction \t" + time.asctime(time.localtime(time.time())) + '\n')
  pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_IncrementalSfM"),  
                            "-i", output_dir+"/partial_sfm_data.json", 
                            "-m", matches_dir, 
                            "-o", reconstruction_dir, 
                            "-r", "1", "-t", matches_file] )
  pRecons.wait()
  (filepath, tmpfilename) = os.path.split(matches_file)
  shutil.move(matches_file, reconstruction_dir_partial + "/" + tmpfilename)

  # print ("\033[31m step 5 : Before Colorize Structure\033[00m " + time.asctime(time.localtime(time.time())))
  # flog.write("Before Colorize Structure \t\t" + time.asctime(time.localtime(time.time())) + '\n')
  # pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  "-i", reconstruction_dir+"/sfm_data.json", "-o", os.path.join(reconstruction_dir,"colorized.ply")] )
  # pRecons.wait()

  # optional, compute final valid structure from the known camera poses
  print ("\033[31m step 5 : Before Structure From Known Poses\033[00m " + time.asctime(time.localtime(time.time())))
  flog.write("Before Structure From Known Posses \t" + time.asctime(time.localtime(time.time())) + '\n')
  pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeStructureFromKnownPoses"),  
                            "-i", reconstruction_dir+"/sfm_data.json", 
                            "-m", matches_dir, 
                            "-f", os.path.join(matches_dir, "matches.f.txt"), 
                            "-o", os.path.join(reconstruction_dir,"robust.json")] )
  pRecons.wait()

  # print ("\033[31m step 7 : Before Write Ply Files\033[00m " + time.asctime(time.localtime(time.time())))
  # flog.write("Before Write Ply Files \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
  # pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  "-i", reconstruction_dir+"/robust.json", "-o", os.path.join(reconstruction_dir,"robust_colorized.ply"), "-p", os.path.join(reconstruction_dir,"photo_align.ply"), "-m", reconstruction_dir] )
  # pRecons.wait()


#############################################Point Cloud Alignment##############################################
print ("\033[31m step 6 : Before Point Cloud Alignment\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before Point Cloud Alignment \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
aRecons = subprocess.Popen( [os.path.join(EGSfM_BIN, "sfm_align"), 
                            "--sfm_datas_path=" + os.path.join(input_dir, "sfm_datas.txt"), 
                            "--img_path=" + input_dir,
                            "--use_common_cameras=true",
                            "--use_common_structures=false",
                            "--perform_global_ba=false",
                            "--output_dir=" + output_dir_name])
aRecons.wait()

print ("\033[31m step 7 : Before Write Ply Files\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before Write Ply Files \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
wRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  
                            "-i", output_dir_name + "/final_sfm_data.json", 
                            "-o", os.path.join(output_dir_name,"robust_colorized.ply")] )
wRecons.wait()


print ("\033[31m Finish Time\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Finish Time \t\t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
flog.close()

