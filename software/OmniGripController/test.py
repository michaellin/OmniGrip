from __future__ import division
import numpy as np

maxGripEncCount = 1200   

#Conversions from encoder reading to desired PWM output
L_OmniGrip       = 55                                   
encoder_range    = maxGripEncCount                      
swing_range      = (25/encoder_range)*(np.pi/180)       
max_volts        = 12                                   
motor_resistance = 2.7                                  
K_torque         = 23.2                                 
capstan_radius   = 4.5                                  
L_daV            = 41                                   
K_daV            = 5.4/L_daV                            
maxPWM           = 256

## Conversion factor calculation
#  I_eff = PWM*V_max/R_motor
#  F_eff = I_eff*K_torque/capstan_radius
#
#  angle_reading_radian = input_encoder_reading*(25/encoder_range)*(pi/180)    25 is max swing in degrees
#  F_des = K_stiff*angle_reading_radian/gripper_radius
#
#  F_eff = F_des
#  (PWM*K_torque*V_max)/(R_motor*capstan_radius) = (encoder_reading*(25*pi/180)*K_stiff)/(encoder_range*gripper_radius)
#  Solve for PWM
#
 
conv_factor = swing_range*(motor_resistance/(K_torque*max_volts))*(capstan_radius/L_OmniGrip)*maxPWM    
print conv_factor
