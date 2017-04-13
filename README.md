# vive_image_view

test for HTC_Vive - ROS visual connection

# Assume
* OpenVR is compiled on /home/user/openvr
* Steam and SteamVR are installed
* No additional environment variables needed on launching (such as $openvr,$steam,$steamvr,$LD_LIBRARY_PATH)
* Device access permission is allowd on hidraw* 
* OpenVR sample can be executed with no problem (latest steam may be unstable now(2016/10) ...)

#　Usage
```
$ roscore
$ rosrun usb_cam usb_cam_node
$ roslaunch vive_image_view normal.launch IMAGE:=/usb_cam/image_raw
```
or
```
$ roscore
$ (launch multisense)
$ roslaunch vive_image_view multisense.launch
```

![tksg](https://github.com/ishiguroJSK/vive_image_view/blob/readme-img/tksg.png "TKSG")
