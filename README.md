# Graph Structure from Motion (GraphSfM)
**This project is not maintained anymore.**

## 1. Overview of GraphSfM

Our Structure from Motion approach, named **```Graph Structure from Motion```**, is aimed at large scale 3D reconstruction. Besides, we aimed at exploring the computation ability of computer and making SfM easily transferred to distributed system. Our work is partially based on an early version of [OpenMVG](https://github.com/openMVG/openMVG).

In our work, 3D reconstruction is deemed as a ```divide-and-conquer``` problem. Our adaptive graph cluster algorithm divides images into different clusters, while images with high relativity remained in the same group. The strong/weak spanning tree (ST) conditions enhance the connectivity between clusters, and make
multiple point clouds alignment more robust. After the completion of local SfM in all clusters, an elaborate graph initialization and MST construction algorithm is designed to accurately merge clusters, and cope well with drift problems. The two proposed graph-based algorithms make SfM more efficient and robust - the graph cluster algorithm accelerate the SfM step while guarantee the robustness of clusters merging, and the MST construction makes point clouds alignment as accurate as possible. Our approach can reconstruct large scale data-set in one single machine with very high accuracy and efficiency.


## 2. How to Build

### 2.1 Required
```
## Global Required(For development: GLOG, GFlags, GTest)
sudo apt-get install libgoogle-glog-dev libgflags-dev libgtest-dev

cd /usr/src/gtest
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo cp libgtest* /usr/lib/
cd ..
sudo rm -rf build

git clone https://github.com/gflags/gflags
cd gflags
mkdir build && cd build
cmake ..
make -j4
sudo make install

## Eigen 3
hg clone https://bitbucket.org/eigen/eigen
mkdir eigen_build && cd eigen_build
cmake . ../eigen
make && sudo make install
cd ..

## Ceres-Solver (Required by both COLMAP and TheiaSfM and OpenMVG)
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make
sudo make install

## Required by openMVG
sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz

## Required by COLMAP
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libcgal-qt5-dev


## Configure and compile COLMAP (Optional)
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build
cd build
cmake ..
make
sudo make install

#GLFW3 (Optional)
sudo apt-get -y install freeglut3-dev libglew-dev libglfw3-dev
```
### 2.2 Build
```bash
cd EGSfM
mkdir build && cd build
cmake -D CUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
make -j4
```

## 3. Usage

All the parameters required for running your sfm tasks should be provided in a `yaml` file.
There is template named `config.yaml` in `app` subfolder. Then you may run the command
```sh
./bin/sfm_reconstruction --config_filename=$yourpath/config.yaml
```

However, the old version to run sfm tasks is also provided is `scripts` subfolder:

For small scale reconstruction, just use the incremental SfM pipeline of [OpenMVG](https://github.com/openMVG/openMVG)
```bash
python scripts/sfm/openmvg_sequential_pipeline.py $image_dir $output_dir
```

For large scale reconstruction, our GSfM is highly recommended.
```bash
python scripts/sfm/sfm_divide_and_conquer_pipeline.py $image_dir $output_dir $max_img_num
```

- ```$image_dir```:   The directory that stores images
- ```$output_dir```:  The directory that stores the reconstruciton results
- ```$max_img_num```: The maximum image number in each cluster. For example, ```80~120```.

## 4. License

BSD 3-Clause License

Copyright (c) 2019, 陈煜
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

