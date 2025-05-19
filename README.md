# stella_plp_vslam
stella_plp_vslam updating codes

# run
``` bash
cd  ~/codes/slam_projects/stella_plp_vslam
conda deactivate
./build/run_euroc_slam -v ./orb_vocab/orb_vocab.dbow2 -d /media/longxiaoze/datasets/datasets_ubuntu/slam_datasets/euroc_datas/MH_01_easy/mav0 -c ./example/euroc/EuRoC_mono.yaml
./build/run_euroc_slam_line -v ./orb_vocab/orb_vocab.dbow2 -d /media/longxiaoze/datasets/datasets_ubuntu/slam_datasets/euroc_datas/MH_01_easy/mav0 -c ./example/euroc/EuRoC_mono.yaml
```
# evaluation
## Evaluation with [EVO tool](https://github.com/MichaelGrupp/evo)

``` bash
evo_ape tum /data/TUM_RGBD/rgbd_dataset_freiburg3_structure_texture_far/groundtruth.txt ./keyframe_trajectory.txt -p --plot_mode=xy -a --verbose -s
# evaluation between PLP-SLAM and stella_plp_slam
evo_ape tum ./keyframe_trajectory-plp.txt ./keyframe_trajectory.txt -p --plot_mode=xy -a --verbose -s
```

Important flags:
```
--align or -a = SE(3) Umeyama alignment (rotation, translation)
--align --correct_scale or -as = Sim(3) Umeyama alignment (rotation, translation, scale)
--correct_scale or -s = scale alignment
```
![ape-plp-stellaPLP](https://github.com/Longxiaoze/stella_plp_vslam/blob/main/docs/ape-plp-stellaPLP.png)


# build

## 3rd build
``` bash
conda deactivate 
sudo apt-get -y install libyaml-cpp-dev 
sudo apt-get -y install libgoogle-glog-dev
sudo apt-get install libsuitesparse-dev
# OPENCV 3.4.16
sudo apt-get install -y  build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libswscale-dev libtiff5-dev libgtk2.0-dev pkg-config
wget https://github.com/opencv/opencv/archive/refs/tags/3.4.16.zip
unzip 3.4.16.zip
rm 3.4.16.zip
wget https://github.com/opencv/opencv_contrib/archive/refs/tags/3.4.16.zip
unzip 3.4.16.zip
rm 3.4.16.zip
cd opencv-3.4.16/
mkdir build
cd build
cmake -D OPENCV_EXTRA_MODULES_PATH= ../../opencv_contrib-3.4.16/modules ..
make -j12
sudo make install

# g2o.zip
# Dbow2.zip

# Pangolin
sudo apt install doxygen
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81
# need to change 3rd/Pangolin/CMakeLists.txt line 24  from set (CMAKE_CXX_STANDARD 14) to set (CMAKE_CXX_STANDARD 17)
mkdir build
cd build
cmake .. -Wno-dev
make -j12
sudo make install
```

## stella_plp_vslam build
``` bash
mkdir build
cd build
# cmake     -DBUILD_WITH_MARCH_NATIVE=OFF     -DUSE_PANGOLIN_VIEWER=ON     -DUSE_SOCKET_PUBLISHER=OFF     -DUSE_STACK_TRACE_LOGGER=ON     -DBOW_FRAMEWORK=DBoW2     -DBUILD_TESTS=OFF     ..
cmake ..     -DBUILD_WITH_MARCH_NATIVE=OFF     -DUSE_PANGOLIN_VIEWER=ON     -DUSE_SOCKET_PUBLISHER=OFF     -DUSE_STACK_TRACE_LOGGER=ON     -DBOW_FRAMEWORK=DBoW2     -DBUILD_TESTS=OFF     -Wno-dev     -DCXSPARSE_INCLUDE_DIR=/usr/include/suitesparse     -DGLOG_INCLUDE_DIR=/usr/include     -DGLOG_LIBRARY=/usr/lib/x86_64-linux-gnu/libglog.so     -DGLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION=OFF

make -j12
```

