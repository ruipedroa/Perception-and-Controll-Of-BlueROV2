#
#JAMESON LICENSE
#
#This code is publicly available to anyone who wishes to use it.
#If someday you have the fortune to pass by me, and you think it was useful,
#I only ask that you pay me a glassh of Jameson.
#
#Code created by Rui P. Alves
#Faculty of Engeneering of University of Porto
#


import time
import socket
import sys
import math
from pymavlink import mavutil


# Wait for server connection
def wait_conn(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

#Send attitude onfroamtion to C++ code
def send_control(roll, pitch, yaw):	 
 	  connection.sendall(str(roll)+ ' ' +str(pitch)+ ' ' + str(yaw))


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('localhost', 8888)
sock.bind(server_address)

# Listen for incoming connection
sock.listen(1)
connection, client_address = sock.accept()
print('Accept client connection')

# Create the connection
#  Companion is already configured to allow script connections under the port 9000
# Note: The connection is done with 'udpout' and not 'udpin'.
#  You can check in http:192.168.1.2:2770/mavproxy that the communication made for 9000
#  uses a 'udp' (server) and not 'udpout' (client).
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')

# Send a ping to start connection and wait for any reply.
#  This function is necessary when using 'udpout',
#  as described before, 'udpout' connects to 'udpin',
#  and needs to send something to allow 'udpin' to start
#  sending data.
wait_conn(master)


kpz=0.2833
kiz=0.0505 # integral gain
ipz=0  # error integral initialization
dt=0.01
kpx = 0.3524  #x position gains
kix = 0.0327
kpy = 0.4159 #y position gains
kiy = 0.0304
kpYaw = 0.2263 #yaw position gains 
kiYaw = 0.0338
ipx=0
ipy=0
ipYaw=0
while True:
    try:
	message=master.recv_match(type='ATTITUDE').to_dict() # Gets attitude information(Roll, Pitch, Yaw)

	send_control(message['roll'], message['pitch'], message['yaw']) #Send attitude onfroamtion to C++ code
	print('Enviou dados')
	data= connection.recv(4096)  #Receives the x,y,z data of the relative position 
	#print(data)
	if not data:continue
	# Pass the data from string to float
	Position= [float(x) for x in data.split(' ')]
	Ex=Position[0]
	Ey=Position[1]
	Ez=Position[2]
    Eyaw= float(message['yaw'])
	
    # If the Z error is less then |0.1| then pass to the horizontal controller
	if Ez > 0.1  or Ez < -0.1:
	# 
	# Vertical Position Controller
        	ipz = ipz + dt * Ez 
            #anti windup
        	if Ez > 1.5 :
            		ipz=0
        	elif Ez < -1.5 :
            		ipz=0
    
		wref=kpz*Ez + kiz*ipz
		wref= -wref
    
        	if wref > 0 :
            		zvel = 300 * wref + 550
        	elif wref < 0 :
            		zvel= 300 * wref + 450
       		elif wref == 0 :
            		zvel=500

		master.mav.manual_control_send(
            		master.target_system,
            		0,
            		0,
            		zvel,
            		0,
            		0)
        	

        #Horizontal Position Controller
    	else :
        	ipx= ipx+dt*Ex
        	ipy= ipy+dt*Ey
        	ipYaw = ipYaw +dt*Eyaw
    
    		if Ex > 1.5 :
            		ipx =0
    		elif Ex < -1.5 :
            		ipx=0
    
    		if Ey > 1.5 :
            		ipy =0
    		elif Ey < -1.5 :
            		ipy=0
     
    		if Eyaw> math.pi/3 :
            		ipYaw=0
    		elif Eyaw < -math.pi/3 :
            		ipYaw=0
        
    		uref= -kpx*Ex-kix*ipx;
    		vref= -kpy*Ey -kiy*ipy;
    		rref= -kpYaw*Eyaw -kiYaw*ipYaw;
    
    		uref=-uref
    		vref=-vref
    		rref=-rref
    
    		if uref > 0 :
            		uvel = 600 * uref + 100
    		elif uref < 0 :
            		uvel= 600 * uref -100
    		elif uref == 0 :
            		uvel=0
    
    		if vref > 0 :
            		vvel = 600 * vref + 100
    		elif vref < 0 :
            		vvel= 600 * vref -100
    		elif vref == 0 :
            		vvel=0
        
    		if rref > 0 :
            		rvel = 600 * rref + 100
    		elif rref < 0 :
            		rvel= 600 * rref -100
    		elif rref == 0 :
            		rvel=0    
    
  		
    		master.mav.manual_control_send(
           		master.target_system,
           		uvel,
           		vvel,
           		500,
           		rvel,
           		0)
    	#time.sleep(0.01)
		
    except:
	pass
   # time.sleep(0.01)


