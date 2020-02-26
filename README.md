# moveit_ws

This is a backed up place for the ROS files I'm working with. Openni_camera is a package that introduces a driver for the Kinect camera in the lab. Find_object is a package that works with the driver to identify objects and locations in pixel coordinates. When I was attempting to get the Kinect to work I came across the package image_common. I'm not sure if I'm using it anywhere but included in case it is used or will be used in the future.

trav_controllers is my package that uses Moveit, which needs to be downloaded and compiled to have the controller work. See the Moveit website for more information. trav_controllers contains the pick and place controller that I developed using the move_group_python_interface. I'm hoping to improve this controller by adding camera input from the above packages and having the robot locate an object through the camera and pick it up to deliver it to a specified location.
