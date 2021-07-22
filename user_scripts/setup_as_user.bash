#!/bin/bash

# pull directory of current folder.
APP_DIRECTORY=/home/hello-robot/catkin_ws/src/robot-camera-streaming-demo-with-aws-kvswebrtc-kvs
touch ~/.bash_aliases
echo "source $APP_DIRECTORY/user_scripts/utility_bash_functions" >> ~/.bash_aliases
echo "source ~/.bash_aliases" >> ~/.bashrc
# Setup sample ros application
if [ -d $APP_DIRECTORY ]; then
    mkdir -p $APP_DIRECTORY
fi


# clone ros images to rtsp stream rospackage. This needs gstreamer plugins setup in root script
git clone https://github.com/konduri/ros_rtsp.git  $APP_DIRECTORY/deps/ros_rtsp

# Install dependencies
pip install roslibpy
# rosdep install --from-paths $APP_DIRECTORY/src -r -y --skip-keys roslibpy-pip
# catkin_make

## Setup webrtc here
cd $APP_DIRECTORY
#git clone --recursive https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c.git

git clone https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c.git
cd $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c;git checkout 54c4138; git submodule update --recursive

mkdir $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c/build
cd $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c/build
cmake ..
make



# Setup KVS sink
cd $APP_DIRECTORY
git clone https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp.git
mkdir $APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/build
cd $APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/build
cmake ..  -DBUILD_GSTREAMER_PLUGIN=TRUE
make

export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/open-source/local/lib
