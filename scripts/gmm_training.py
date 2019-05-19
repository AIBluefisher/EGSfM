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

# Indicate the viewing graph binary directory
VIEWING_GRAPH_BIN = "/home/chenyu/projects/XFeatures/bin"

import commands
import os
import subprocess
import sys
import time
import shutil

if len(sys.argv) < 4:
    print ("Usage: %s sift_lists output_dir filename" % sys.argv[0])
    sys.exit(1)

sift_lists = sys.argv[1]
output_dir = sys.argv[2]
gmm_file = sys.argv[3]

print ("Using sift_lists  : ", sift_lists)
print ("      output_dir : ", output_dir)

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)

file_path = output_dir + "/gmm_training_time_record.txt"
flog = open(file_path, 'w')

fs = open(sift_lists, 'r')
descs_file = open(output_dir + '/descs_file.txt', 'w')
while True:
    sift_list = fs.readline()
    if not sift_list : break
    sift_list = sift_list.strip('\n')

    f = open(sift_list, 'r')
    while True:
        desc = f.readline()
        if not desc : break
        #desc = desc.strip('\n')
        descs_file.write(desc)
    f.close()
fs.close()
descs_file.close()


print ("\033[31m Before GMM Training\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("Before GMM Training \t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
pGMM = subprocess.Popen([os.path.join(VIEWING_GRAPH_BIN, "fisher_vector_trainer"),
                        "--filename=" + output_dir + '/descs_file.txt',
                        "--output_dir=" + output_dir,
                        "--gmm_file=" + gmm_file])
pGMM.wait()

print ("\033[31m finish time\033[00m " + time.asctime(time.localtime(time.time())))
flog.write("finish time \t\t\t\t" + time.asctime(time.localtime(time.time())) + '\n')
flog.close()