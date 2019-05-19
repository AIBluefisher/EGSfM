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
SIMILARITY_SEARCH_BIN = "/home/chenyu/projects/XFeatures/build/bin"
VIEWING_GRAPH_BIN = "/home/chenyu/projects/XFeatures/bin"

# Indicate the i23dSFM camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/chenyu/projects/openMVG/src/openMVG/exif/sensor_width_database"

import commands
import os
import subprocess
import sys
import time
import shutil

if len(sys.argv) < 4:
    print ("Usage: %s image_dir output_dir community_number" % sys.argv[0])
    sys.exit(1)

input_dir = sys.argv[1]
output_dir = sys.argv[2]
community_number = sys.argv[3]

matches_dir = os.path.join(output_dir, "matches")
reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")
# match_pairs = input_dir + "/match_pairs" # mirror similarity
# match_pairs = matches_dir + "/fisher_vector_out/fisher_similarity.out" # fisher distance
match_pairs = matches_dir + "/vocab_out/match.out" # BoW similarity

print ("Using input dir  : ", input_dir)
print ("      output_dir : ", output_dir)

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

file_path = output_dir + "/sfm_time_record.txt"
flog=open(file_path,'w')


flog.write("before intrinsics analysis \t\t" + time.asctime(time.localtime(time.time())) + '\n')
print ("\033[31m step 1 : Before intrinsics analysis\033[00m " + time.asctime(time.localtime(time.time())))
pIntrisics = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_SfMInit_ImageListing"), 
"-e","1", "-i", input_dir, "-g","1","-o", matches_dir, "-d", camera_file_params] )
pIntrisics.wait()


print ("\033[31m step 2 : Before compute features\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before compute features \t\t" + time.asctime(time.localtime(time.time())) + '\n')
# pFeatures = subprocess.Popen([os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT", "-p", "HIGH" ])
pFeatures = subprocess.Popen([os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  
                             "-i", matches_dir + "/sfm_data.json", 
                             "-o", matches_dir, 
                             "-m", "SIFT", 
                             "-p", "HIGH" ])
pFeatures.wait()

if len(sys.argv) == 4:
    print ("\033[31m step 3 : Before BoW Similarity search\033[00m " + time.asctime(time.localtime(time.time())))
    flog.write("Before BoW Similarity search \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
    sift_list = os.path.join(matches_dir, "sift_list")
    pList = os.popen("ls -d " + matches_dir + "/*.desc > " + sift_list).read()
    if not os.path.exists(matches_dir+"/vocab_out/match.out"):
      pSearch = subprocess.Popen([os.path.join(SIMILARITY_SEARCH_BIN, "image_search"), 
                                  sift_list, 
                                  matches_dir + "/vocab_out", "6", "8", "1"])
      pSearch.wait()

elif len(sys.argv) == 5:
    gmm_file = sys.argv[4]
    print ("\033[31m step 3 : Before Fisher Vector Similarity search\033[00m " + time.asctime(time.localtime(time.time())))
    flog.write("Before Fisher Vector Similarity search \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
    if not os.path.exists(matches_dir + "/fisher_vector_out/fisher_similarity.out"):
      pFSearch = subprocess.Popen([os.path.join(VIEWING_GRAPH_BIN, "fisher_vector_similarity"),
                                  "--filename=" + gmm_file,
                                  "--output_dir=" + matches_dir + "/fisher_vector_out",
                                  "--metric_option=2"])
      pFSearch.wait()

print ("\033[31m step 4 : Before Viewing Graph matches\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before Viewing Graph matches \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
pMatches = subprocess.Popen([os.path.join(VIEWING_GRAPH_BIN, "vg_matching"),
                            "--matches_dir=" + matches_dir,
                            "--sfm_data_file=" + matches_dir + "/sfm_data.json",
                            "--similarity_match_file=" + match_pairs,
                            "--community_number=" + community_number,
                            "--loop_consistency_enable=true",
                            "--rank_edges_enable=false",
                            "--retain_singleton_nodes_enable=false",
                            "--strong_triplet_expansion_enable=true",
                            "--graph_reinforcement_enable=true"])
pMatches.wait()


# Create the reconstruction if not present
if not os.path.exists(reconstruction_dir):
    os.mkdir(reconstruction_dir)

print ("\033[31m step 5 : Before Incremental reconstruction\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before Incremental reconstruction \t" + time.asctime(time.localtime(time.time())) + '\n')
pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_IncrementalSfM"), 
                            "-i", matches_dir + "/sfm_data.json",
                            "-m", matches_dir, 
                            "-o", reconstruction_dir, 
                            "-r", "1"] )
pRecons.wait()

print ("\033[31m step 6 : Before colorize structure\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("before colorize structure \t\t" + time.asctime(time.localtime(time.time())) + '\n')
pRecons = subprocess.Popen( [os.path.join(I23DSFM_SFM_BIN, "i23dSFM_main_ComputeSfM_DataColor"),  
                            "-i", reconstruction_dir + "/sfm_data.json", 
                            "-o", os.path.join(reconstruction_dir,"colorized.ply")] )
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

