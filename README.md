# Perception and Controll Of BlueROV2
 This program uses the camera integrated in the BLueROV2 and the MAVLink protocol to obtain the distance from a triangular marker to the ROV and controll the vehicle to aproxximate the object.
 
 # Install OpenCV and OpenCV_contrib on ubuntu
 
 https://gist.github.com/Mahedi-61/804a663b449e4cdb31b5fea96bb9d561
 
 Download and Run Script.
 
 # Install gstreamer libraries to access the BlueROV2 camera
 
 # Running the code
  Launch the python script and then start the c++ code. The python script will wait for the c++ connection to the socket. 
  After they are connected the proframm will be sending the IMU data from python to c++, the c++ program will get the live feed from the vehicle and analyse the image to get the center the distances to the object. After that, the final distances are sent through the socket to the python script and applied to the position controllers. 
  

