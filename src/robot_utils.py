import wpilib
import math
class robot_utils():
    
    def cookJoystickInputs4(self, jsv):
        if(jsv < 0):
            jsv = jsv * -1
            return -(math.sqrt(jsv))
        elif(jsv > 0):
            return (math.sqrt(jsv))
        else: 
            return 0
    
    def cookJoystickInputs2(self, jsv):
        if(jsv < 0):
            return -(jsv * jsv)
        elif(jsv > 0):
            return (jsv * jsv)
        else:
            return 0
               
    def Efficiency(self, jsv):
        return jsv * .875
    
    def cookThrottleInputs(self, ths):
        return ((-ths+1)/2)
     

    

