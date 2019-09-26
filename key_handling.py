#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
import rospy
import std_msgs.msg
from std_msgs.msg import String
import roslib
import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist,PoseArray
import sys, select, termios, tty
x=0
y=0
z=0

Kp=0.9919
Ki=0.09940
Kd=0.047


Kpr=3
Kir=0.0001
Kdr=0.813


Kpp=4.5
Kip=0
Kdp=0.3

Kpy=0.5
Kiy=0
Kdy=0.005

errSumz,errSumr,errSump,errSumy = (0,0,0,0)
lastTimez,lastTimer,lastTimep,lastTimey=(0,0,0,0)
outputz,lastErrorz,errSumz=(0,0,0)
outputr,lastErrorr,errSumr=(0,0,0)
outputp,lastErrorp,errSump=(0,0,0)
outputy,lastErrory,errSumy=(0,0,0)
initialWaypoint=[0.0,0.0,0.0]
roll,pitch,yaw,altitude=(0,0,0,0)

class send_data():
    """docstring for request_data"""
    def __init__(self):

        lastTimex,lastTimey=(0,0)
        outputx,lastErrorx,errSumx=(0,0,0)
        outputy,lastErrory,errSumy=(0,0,0)
        rospy.init_node('drone_server')
        self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
        self.debug1 = rospy.Publisher('/debug1',String,queue_size = 1)
	self.debug2 = rospy.Publisher('/debug2',String,queue_size = 1)
        rospy.Subscriber('/input_key', Int16, self.indentify_key )
        rospy.Subscriber('/temp',std_msgs.msg.String,self.callback)
        rospy.Subscriber('/whycon/poses',PoseArray,self.getPos)
        
        self.key_value =-1
        self.cmd = PlutoMsg()
        self.cmd.rcRoll =1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw =1500
        self.cmd.rcThrottle =1500
        self.cmd.rcAUX1 =1500
        self.cmd.rcAUX2 =1500
        self.cmd.rcAUX3 =1500
        self.cmd.rcAUX4 =1000

    def arm(self):
        self.cmd.rcRoll=1500
        self.cmd.rcYaw=1500
        self.cmd.rcPitch =1500
        self.cmd.rcThrottle =1000
        self.cmd.rcAUX4 =1500
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def disarm(self):
        self.cmd.rcThrottle =1300
        self.cmd.rcAUX4 = 1200
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)
    
    def indentify_key(self, msg):
        self.key_value = msg.data
	
    def forward(self):
        self.cmd.rcPitch =1600
        self.command_pub.publish(self.cmd)
    def backward(self):
        self.cmd.rcPitch =1400
        self.command_pub.publish(self.cmd)
    def left(self):
        self.cmd.rcRoll =1600
        self.command_pub.publish(self.cmd)  
    def right(self):
        self.cmd.rcRoll =1400
        self.command_pub.publish(self.cmd)
    def reset(self):
        self.cmd.rcRoll =1500
        self.cmd.rcThrottle=1500
        self.cmd.rcPitch =1500
        self.cmd.rcYaw = 1500
        self.command_pub.publish(self.cmd)
    def increase_height(self):
 	'''       
	if(self.cmd.rcThrottle<1500):
            self.cmd.rcThrottle = 1500

        elif (self.cmd.rcThrottle>1900):
            self.cmd.rcThrottle=1800
        '''                
                       
                        
        
        self.cmd.rcThrottle = 1500
                        
        self.command_pub.publish(self.cmd)
    def decrease_height(self):
        self.cmd.rcThrottle-=50
        self.command_pub.publish(self.cmd)

    def control_drone(self):
        while True:
            if self.key_value == 0:         
                self.disarm()
            elif self.key_value == 70:
                self.arm()
                #self.test.publish("throttle value is %s "%self.cmd.rcThrottle)
            elif self.key_value == 10:
                self.forward()
            elif self.key_value == 20:
                self.reset()
            elif self.key_value == 30:
                self.left()
            elif self.key_value == 40:
                self.right()
            elif self.key_value == 80:
                self.reset()
            elif self.key_value == 50:
                self.increase_height()
                #self.test.publish("after increasing throttle value is %s "%self.cmd.rcThrottle)
                
            elif self.key_value == 60:
                self.decrease_height()
            elif self.key_value == 110:
                self.backward()
            self.command_pub.publish(self.cmd)

    def callback(self, data):
	    global roll,pitch,yaw,altitude

            str1=data.data
            roll1,pitch1,yaw1,altitude1=str1.split()
            roll=float(roll1)
            pitch=float(pitch1)
            yaw=float(yaw1)
            altitude=float(altitude1)
	    
            
    #position of the drone        
    def getPos(self,coord):
        global x,y,z
        
        y = -coord.poses[0].position.x
        x = -coord.poses[0].position.y
        z = coord.poses[0].position.z   
        #self.test.publish("co ordinates are %s  %s  %s "%(x,y,z))
        
    def takeoff(self):
        global x,y,z,initialWaypoint,lastErrorz,errSumz,lastTimez,roll,altitude
	height = 65
        #print z
	#time.sleep(1)
        #Initial waypoint
	initialWaypoint = [x,y,altitude]
	initialYaw = 0
	
        while True:
	    if	(x!=0) and (y!=0) and (altitude!=0):			
            	initialWaypoint=[x,y,altitude]
 		#initialYaw = yaw
		
	    	break
	    else:
		pass
	
        count=0    
        #Drone hold waypoint after take-off
        holdWaypoint = [initialWaypoint[0],initialWaypoint[1],height] 
	
	print "Initial and final waypoint are %s , %s"%(initialWaypoint,holdWaypoint)
	print "yaw is %s"%initialYaw
        if(count==0):
            lastErrorz=height - altitude
            lastErrorr=holdWaypoint[0]-initialWaypoint[0]
            lastErrorp=holdWaypoint[1]-initialWaypoint[1]
            count=count+1
	
        self.cmd.rcThrottle = 1500

        self.command_pub.publish(self.cmd)	
	time.sleep(1)
	self.command_pub.publish(self.cmd)
	
	
        self.arm()
	#self.cmd.rcThrottle = 1999

        self.command_pub.publish(self.cmd)	
	
	
	#print "Error z is %s"%errorz
	
        #while(math.sqrt(((z-holdWaypoint[2])**2)+((z-holdWaypoint[2])**2)+((z-holdWaypoint[2])**2))>0.2):  
	while True:
	    
	       
	    
	    self.computePid(height,altitude,x,holdWaypoint[0],y,holdWaypoint[1])

	    #Variable to hold the count for decreasing height
	    i = 0
	    #Key d to disarm immediately in case the drone is about to crash	    
            if(self.key_value == 0):
		while(True):	
			if(i>=10):
				break	
			#Decrease height slowly 
			self.decrease_height()
			i = i + 1
			time.sleep(0.1)

			
		self.disarm()
		self.disarm()
		
	    #print "Error z is %s"%(z-holdWaypoint[2])
	'''
	print "resetting............."
	self.reset()
	self.command_pub.publish(self.cmd)		
        '''
	
            
            
    def computePid(self,cz,pz,cr,pr,cp,pp): 
        global lastErrorz,errSumz,lastTimez,outputz,outputr,lastErrorr,errSumr,lastTimer,outputp,lastErrorp,errSump,lastTimep,outputy,lastErrory,errSumy,lastTimey
        
    	#Calculate time 
        lastTimez = time.time() 
        outputz,lastErrorz,errSumz,lastTimez =self.compute(pz,cz,lastErrorz,errSumz,lastTimez)
	self.cmd.rcThrottle += int(outputz*0.33)
	#print "error is %s"%(cz-pz)
	if(self.cmd.rcThrottle>=1990):
		self.cmd.rcThrottle = 1980

	elif(self.cmd.rcThrottle<=1500):
		self.cmd.rcThrottle = 1510
	else:
        	pass
		
        #self.test.publish("throttle value is %s "%self.cmd.rcThrottle)
        #self.test.publish("error out is %s "%outputz)
	#print "Altitude: %s"%altitude
	self.cmd.rcAUX1 =1500
        self.cmd.rcAUX2 =1500
        self.cmd.rcAUX3 =1500
        self.cmd.rcAUX4 =1500
	#self.command_pub.publish(self.cmd)
        #rospy.sleep(0.013)   
	
	
        lastTimer = time.time() 
        outputr,lastErrorr,errSumr,lastTimer =self.compute1(cr,pr,lastErrorr,errSumr,lastTimer)
	self.cmd.rcRoll = 1500 + int(outputr)   
        #self.debug1.publish(outputr)
	#self.debug2.publish(cr-pr)
	print "roll error , sum ,output  is %s\t %s \t %s" %((pr-cr),errSumr,outputr)
	if(self.cmd.rcRoll>=1590):
		self.cmd.rcRoll = 1580
	elif(self.cmd.rcRoll<=1410):
		self.cmd.rcRoll = 1420
	else:
		pass

	print "roll is %s\n" %(cr-pr)
	self.cmd.rcAUX1 =1500
        self.cmd.rcAUX2 =1500
        self.cmd.rcAUX3 =1500
        self.cmd.rcAUX4 =1500
	#self.command_pub.publish(self.cmd)
	
        #rospy.sleep(0.013)   
	

	lastTimep = time.time() 
	outputp,lastErrorp,errSump,lastTimep =self.compute2(cp,pp,lastErrorp,errSump,lastTimep)
	self.cmd.rcPitch =1500 + int(outputp)
	
	#print outputp
        #print "current %s\n" %cp    
	print "error, output is %s\t %s" %((pp-cp),outputp)

	if(self.cmd.rcPitch>=1590):
		self.cmd.rcPitch = 1580
	elif(self.cmd.rcPitch<=1410):
		self.cmd.rcPitch = 1420
	else:
		pass
	
       
        self.cmd.rcAUX1 =1500
        self.cmd.rcAUX2 =1500
        self.cmd.rcAUX3 =1500
        self.cmd.rcAUX4 =1500
	
	
	self.command_pub.publish(self.cmd)
        #Desired time delay
        rospy.sleep(0.040)   
    	

    def compute(self,Input,Setpoint,lastError,errSum,lastTime):
             #Time delay
             time.sleep(0.001)

             timeChange=0
             error=0
             dError=0
             output=0

             #Calculate time
             now = time.time()
         
             #Time change used for differential component
             timeChange = now-lastTime
             
             error = Setpoint - Input
        
             #errSum gives the integral component       
             errSum += error*timeChange
        
             #Differential component
             dError = (error - lastError)/(timeChange)
          
             #Store the error   
             lastError = error
             lastTime = now
             proportional = Kp*error
             integral = Ki*errSum
             differential =  Kd*dError
         
             #Output of the PID controller which is given to the drone
             output = proportional + integral + differential
             #prints values of PID in console
             #print "Proportional, Integral and Differential %s,%s,%s\n"%(proportional,integral,differential)
             #print "output is %s\n"%(output)
             return output,lastError,errSum,lastTime 

    def compute1(self,Input,Setpoint,lastError,errSum,lastTime):
             #Time delay
             time.sleep(0.0001)

             timeChange=0
             error=0
             dError=0
             output=0

             #Calculate time
             now = time.time()
         
             #Time change used for differential component
             timeChange = now-lastTime
             
             error = Setpoint - Input
        
             #errSum gives the integral component       
             errSum += error*timeChange
        
             #Differential component
             dError = (error - lastError)/(timeChange)
          
             #Store the error   
             lastError = error
             lastTime = now
             proportional = Kpr*error
             integral = Kir*errSum
             differential =  Kdr*dError
         
             #Output of the PID controller which is given to the drone
             output = proportional + integral + differential
             #prints values of PID in console
             print "Proportional, Integral and Differential %s %s %s\n"%(proportional,integral,differential)
    
             return output,lastError,errSum,lastTime  

    def compute2(self,Input,Setpoint,lastError,errSum,lastTime):
             #Time delay
             time.sleep(0.0001)

             timeChange=0
             error=0
             dError=0
             output=0

             #Calculate time
             now = time.time()
         
             #Time change used for differential component
             timeChange = now-lastTime
             
             error = Setpoint - Input
        
             #errSum gives the integral component       
             errSum += error*timeChange
        
             #Differential component
             dError = (error - lastError)/(timeChange)
          
             #Store the error   
             lastError = error
             lastTime = now
             proportional = Kpp*error
             integral = Kip*errSum
             differential =  Kdp*dError
         
             #Output of the PID controller which is given to the drone
             output = proportional + integral + differential
             #prints values of PID in console
             #print "Proportional, Integral and Differential %s,%s,%s\n"%(proportional,integral,differential)
    
             return output,lastError,errSum,lastTime   


    def compute3(self,Input,Setpoint,lastError,errSum,lastTime):
             #Time delay
             time.sleep(0.001)

             timeChange=0
             error=0
             dError=0
             output=0

             #Calculate time
             now = time.time()
         
             #Time change used for differential component
             timeChange = now-lastTime
             
             error = Setpoint - Input
        
             #errSum gives the integral component       
             errSum += error*timeChange
        
             #Differential component
             dError = (error - lastError)/(timeChange)
          
             #Store the error   
             lastError = error
             lastTime = now
             proportional = Kpp*error
             integral = Kip*errSum
             differential =  Kdp*dError
         
             #Output of the PID controller which is given to the drone
             output = proportional + integral + differential
             #prints values of PID in console
             #print "Proportional, Integral and Differential %s,%s,%s\n"%(proportional,integral,differential)
    
             return output,lastError,errSum,lastTime   	 
    
                       
if __name__ == '__main__':
    test = send_data()
   
    while not rospy.is_shutdown():
        
        
        '''
        test.arm()
        time.sleep(1)
        test.arm()
        '''
	#test.control_drone()
        test.takeoff()
	
       	#rospy.spin()
        #sys.exit(1)




