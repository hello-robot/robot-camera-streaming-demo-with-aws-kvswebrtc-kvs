## Objective:

This README file showcases a guide for a Stretch robot utilizing [Amazon Kinesis Video Streams](https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/what-is-kinesis-video.html), and [Amazon Kinesis Video Streams with WebRTC](https://docs.aws.amazon.com/kinesisvideostreams-webrtc-dg/latest/devguide/what-is-kvswebrtc.html). The robot's image ROS-topic is converted to an RTSP stream using the [ROS-RTSP ROS-package](https://github.com/CircusMonkey/ros_rtsp). This work is based off Amazon's [AWS KVS workshop](https://github.com/aws-samples/robot-camera-streaming-demo-with-aws-kvswebrtc-kvs).

![](readmeimages/KVS.gif)

## Setup
Have an Ubuntu 18.04 machine with ROS Melodic installed and AWS credential setup as env variables on the local machine. Specifically
* AWS_REGION
* AWS_ACCESS_KEY_ID
* AWS_SECRET_ACCESS_KEY
* AWS_SESSION_TOKEN (optional)

### Create a new IAM user

Create a new IAM user for your Stretch robot.
1.  Sign in to the AWS Management Console and open the IAM console at https://console.aws.amazon.com/iam/

2. In the navigation panel, choose **Users** and then choose **Add users**.

3. Type *Stretch* followed by its id number for the user name.  This is the sign-in name for AWS and will help keep track of your device in the AWS console.

4. Select **Programmatic access** to create an access key for the stretch robot.
![image](readmeimages/add_user_step_1.png)

5. Click on **Next:Permissions**

6. On the **Set permissions page**, select the **Attach existing policies directly** tab. In the search toolbar, type *kinesis* to see the list of policies that is relevant to KVS. Select the policy name, **AmazonKinesisVideoStreamsFullAccess**.
![image](readmeimages/add_user_step_2.png)

7. Click on **Next:Tags**. Currently, there is no need to attach a tag to this policy.

8. Click on **Next:Review** to see all of the choices you made up to this point. When you are ready to proceed, choose **Create user**.

9. Once the IAM user is added, the following page will provide you the robot's Access Key ID and Secret access key. To view the users' access keys (access key IDs and secret access keys), choose **Show** next to each password and access key that you want to see. To save the access keys, choose **Download .csv** and then save the file to a safe location.

**IMPORTANT** This is your only opportunity to view or download the secret access keys, and you must provide this information to your users before they can use the AWS API. Save the user's new access key ID and secret access key in a safe and secure place. **You will not have access to the secret keys again after this step.**
![image](readmeimages/add_user_step_3.png)

For further information about creating an IAM user in your AWS account, please check out [Creating IAM users](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_users_create.html).


## Application setup

Clone this [repository](https://github.com/hello-robot/robot-camera-streaming-demo-with-aws-kvswebrtc-kvs.git) provided in this workshop onto the development environment, preferably under /home/hello-robot/catkin_ws/src/

```
cd /home/hello-robot/catkin_ws/src
git clone https://github.com/hello-robot/stretch_robomaker_video_streaming.git
```

Run the following command to install corresponding [libararies, software](setup_with_sudo.bash) and the [robot and kvs applications](user_scripts/setup_as_user.bash)
```
cd stretch_robomaker_video_streaming/user_scripts
sudo bash setup_with_sudo.bash; bash setup_as_user.bash
```

Once the above command is completed, change the sample application code present in the sample application source code present in your environment. You can open the file with the following commands if you used the default setup

```
cd /home/hello-robot/catkin_ws/src/stretch_robomaker_video_streaming/amazon-kinesis-video-streams-webrtc-sdk-c/samples/
gedit kvsWebRTCClientMasterGstreamerSample.c
```

The code is change is shown in before and after pictures below
![before_image](readmeimages/webrtc_before.png)
![after_image](readmeimages/webrtc_after.png)

You can find the file with the code to copy [here](rtsp_command.txt) , with the current code as
```
                pipeline = gst_parse_launch(
                        "rtspsrc location=rtsp://0.0.0.0:8554/back short-header=TRUE ! rtph264depay ! "
                        "video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
                        "appsink sync=TRUE emit-signals=TRUE name=appsink-video",
                        &error);
```

## Update ROS RTSP Configuration File

Edit the stream_setyp.yaml file to the appropriate camera topic that will be broadcasted.
```
cd /home/hello-robot/catkin_ws/src/stretch_robomaker_video_streaming/deps/ros_rtsp/config
gedit stream_setup.yaml
```
Simply change the source to match Stretch's camera plugin name, **/camera/color/image_raw_upright_view**.

![image](readmeimages/modified_stream_setup.png)

## Launch Applications

### Launch ROS Application
To Open a new terminal and run the following launch file command

```
roslaunch stretch_deep_perception stretch_detect_faces.launch
```

In another terminal run the following launch file command
```
roslaunch stretch_core upright_camera_view.launch
```
This should setup an rviz setup like the image below. If the camera feed in the bottom left corner in the rviz window is not shown, simply click on the *Add* button and include the *Camera* to the display. You then select the same *Image Topic* you defined in RTSP configuration file. ![image](readmeimages/rviz_bringup.png).

Run the following command in another terminal to provide a real-time video feed of the Stretch robot's camera.
```
roslaunch ros_rtsp rtsp_streams.launch
```
### Set AWS credentials
Include your aws credentials (access key, secret access key, and default region) to the *creds_from_default_file_stretch()* function in the [utility_bash_function](utility_bash_functions) file. This file is located in /home/hello-robot/catkin_ws/src/stretch_robomaker_video_streaming/user_scripts

![image](readmeimages/set_credentials.png)

Then in a separate terminal setup your credentials by running the following command.
```
creds_from_default_file_stretch
```

### Launch Webrtc application

Once the credentials are setup, you can setup the webrtc application by running the following *IN THE SAME TERMINAL WHERE YOU SETUP CREDENTIALS*
```
# build code again since we modified it with new gst-pipeline

echo "building webrtc code"
APP_DIRECTORY=/home/hello-robot/catkin_ws/src/stretch_robomaker_video_streaming
mkdir $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c/build
cd $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c/build
cmake ..
make

# Set environment variable for log level
export AWS_KVS_LOG_LEVEL=3

# move to location of built code
echo "launching webrtc app"
cd $APP_DIRECTORY/amazon-kinesis-video-streams-webrtc-sdk-c/build/samples

# Create signalling channel
echo "create signalling channel, in case it doesnt exist"
aws kinesisvideo create-signaling-channel --channel-name robot_webrtc_stream
sleep 3

# Launch the application
./kvsWebrtcClientMasterGstSample robot_webrtc_stream
```

This will create a KVS webrtc connection between the robot and your browser. You can view it on the Console page by selecting the corresponding signaling channel clicking on Media playback viewer.

![kvs_webrc_image](readmeimages/webrtc.gif)

### Launch KVS video stream application
Setup the KVS stream from the same terminal from which you have the credentials using the following commands
```
# set env vars to recognize the kvs plugin
APP_DIRECTORY=/home/hello-robot/catkin_ws/src/stretch_robomaker_video_streaming
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$APP_DIRECTORY/amazon-kinesis-video-streams-producer-sdk-cpp/open-source/local/lib

export AWS_KVS_LOG_LEVEL=3

# Make create stream api everytime for simplicity. It just fails if it already exists.
echo "Creating a new stream if it doesnt exist"
aws kinesisvideo create-stream --stream-name "robot_kvs_stream" --data-retention-in-hours "120"
sleep 3
echo "setting up kvs streaming"
gst-launch-1.0 -v rtspsrc location=rtsp://0.0.0.0:8554/back drop-on-latency=true use-pipeline-clock=true do-retransmission=false latency=0 ! rtph264depay ! h264parse ! kvssink stream-name="robot_kvs_stream" storage-size=512 aws-region=$AWS_REGION
```

You can then view the current and historical data that is streamed from the robot on the AWS console by selecting the corresponding video stream on the kvs page, as shown in the gif below.

![kvs_image](readmeimages/video_stream.gif)


All the commands for launching applications are available from this [file](user_scripts/utility_bash_functions)

## Clean up

You can kill the processes by running `Ctrl+C` on all the tabs. The Webrtc process does not die untill the ROS application is killed.
