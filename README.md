# data_logger_bag

This package contains utilities to write to bag files and then convert to hdf5 format. 

Specifically, this package contains two main core functions:

1. The ability for folks to programmatically start/stop recording bag files using the commandline rosbag interface rather than the rosbag API. It also allows changing record topics and naming of bag files using a custom message that can be published from any node.

2. Utilities that automatically converts rosbag files based on message type into hdf5 format. Also some basic utilities that can then load the hdf5 file into python using pytables.

For more details on how to install and tutorials on how to run the package please see the [wiki](https://github.com/si-machines/data_logger_bag/wiki)


-----------------------------------------------------------------------


This package (despite its name) now contains the ability to launch three primary nodes: the data logger (which records and converts the data from each trial run), the environment setup (which gets the object positions and allows the user to reset Sawyer into its initial position before each trial), and the camera feed. Look at the launch file study_bringup.launch.

In order to run everything in unison, start up three separate terminal instances in intera (be sure you are connected to Sawyer) and run the following commands:

*(TERMINAL 1)*
>> roslaunch data_logger_bag study_bringup.launch

*(TERMINAL 2)*
>> rostopic pub /C6_Task_Description data_logger_bag/LogControl "{taskname: '', actionType: '', skillName: 'REACHING', topics: '', playback: false, runName: 'RUN0', userID: 'USER0'}"
* (make sure to specify skillName, userID, runName - the above arguments are only examples)

*(TERMINAL 3)*
>> rostopic pub -1 /data_logger_flag std_msgs/Bool "data: true"
* (toggle data from true/false in order to start/stop publishing)

Once the launch file is started, you will receive a prompt asking whether you want to move Sawyer back to its initial position. After answering the prompt, two video feeds will appear with different views of the experiment.

When this is done, you should have two files for this trial - RUNO.bag and RUNO.h5 in the above example. This should be located in a directory with a similar structure:
>> /home/[user]/data/robot/REACHING/