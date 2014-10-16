## This is a unit test for the purpose of ensuring the gains used in the OmniGrip are fine.
#  Author: Michael Lin michael.lin.yang@berkeley.edu
from __future__ import division
import numpy as np

OmniGrip_radius  = 55                             #OmniGrip lever length average (in mm)
#encoder_range    = 2000                           #Full swing encoder count
encoder_range    = 1750                           #Full swing encoder count
swing_range      = (np.pi/180)*(25/encoder_range) #Conversion ratio from encoder reading to radians
max_volts        = 12                             #Max voltage that powers the DC motor
motor_resistance = 2.7                            #RE-Max motor resistance
k_torque         = 23.2                           #Torque constant in mNm/rad
capstan_radius   = 4.5                            #Radius of the capstan attached to motor axle (in mm)
maxPWMcounter    = 256                            #For 100% PWM duty cycle

encoder_range2 = encoder_range*2/5

conv_factor = swing_range*(motor_resistance/(k_torque*max_volts))*(capstan_radius/OmniGrip_radius)*maxPWMcounter
#conv_factor = swing_range*(motor_resistance/(k_torque*max_volts))*(capstan_radius/OmniGrip_radius)*encoder_range

##Safety parameters
maxOKPWM = maxPWMcounter*0.42

##Gain set
#gain_set = [(40, 200), (60, 250), (80, 290)]
#gain_set = [(210, 825), (280, 1100), (350, 1375)] #in mNm/rad
ref1, ref2 = 320, 1400
gain_set = [(ref1*.75, ref2*.75), (ref1, ref2), (ref1*1.25, ref2*1.25)] #in mNm/rad

def maxPWMtest(gains):
  for k1, k2 in gains:
    maxPWM = (encoder_range*conv_factor*k1 + encoder_range2*conv_factor*k2) 
    print "({0},{1}) max PWM: {2}".format(k1,k2,maxPWM)
    if (maxPWM > maxOKPWM):
      print "Gain set k1:{0} k2:{1} are too large".format(k1, k2)
      print "Force by k1: {0}".format(encoder_range*conv_factor*k1)
      print "Force by k2: {0}".format(encoder_range2*conv_factor*k2)

#Run tests here
maxPWMtest(gain_set)


#Deprecated function
def maxPWMtest_wrong(gains):
  for k1, k2 in gains:
    K1 = k1*encoder_range/maxPWMcounter
    K2 = k2*encoder_range/maxPWMcounter
    #K1 = k1
    #K2 = k2
    maxPWM = (1200*conv_factor*K1 + (1200*3/5)*conv_factor*K2) 
    print "({0},{1}) max PWM: {2}".format(K1,K2,maxPWM)
    if (maxPWM > maxOKPWM):
      print "Gain set k1: {0} k2: {1} are too large".format(K1,K2)
      print "Force by k1: {0}".format(encoder_range*conv_factor*K1)
      print "Force by k2: {0}".format(encoder_range2*conv_factor*K2)
