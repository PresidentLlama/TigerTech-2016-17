import wpilib
from wpilib.doublesolenoid import DoubleSolenoid


class DualMotorGearbox():
    def __init__(self, channel1, channel2):
        self.motor1 = wpilib.VictorSP(channel1)
        self.motor2 = wpilib.VictorSP(channel2)
        self.isInverted = False
        
    def pidWrite(self, output):
        self.motor1.set(output)
        self.motor2.set(output)
        
        
    def set(self, value):
        if(self.isInverted == True):
            self.motor1.set(-value)
            self.motor2.set(-value)
        else:
            self.motor1.set(value)
            self.motor2.set(value)
            
    def set1(self, value):
        if(self.isInverted == True):
            self.motor1.set(-value)
        else:
            self.motor1.set(value)
    
    def set2(self, value):
        if(self.isInverted == True):
            self.motor2.set(-value)
        else:
            self.motor2.set(value)
    
    def setInverted(self, bool):
        self.isInverted = bool
        
    def get(self):
        return self.motor1.get()
    
    def getInverted(self):
        return self.isInverted

class InvertedDualMotorGearbox():
    def __init__(self, channel1, channel2, bool1, bool2):
        if(bool1 == True):
            self.motor1 = wpilib.VictorSP(channel1)
            self.motor1.setInverted(True)
            self.motor2 = wpilib.VictorSP(channel2)
        else:
            self.motor2 = wpilib.VictorSP(channel2)
            self.motor2.setInverted(True)
            self.motor1 = wpilib.VictorSP(channel1)
        
    def pidWrite(self, output):
        self.motor1.set(output)
        self.motor2.set(output)
        
        
    def set(self, value):
            self.motor1.set(value)
            self.motor2.set(value)
            
    def set1(self, value):
        self.motor1.set(value)
    
    def set2(self, value):
        self.motor2.set(value)
              
class TripleMotorGearbox(): 
    def __init__(self, channel1, channel2, channel3,channel4):
        
        self.motor1 = wpilib.VictorSP(channel1)
        self.motor2 = wpilib.VictorSP(channel2)
        self.motor3 = wpilib.VictorSP(channel3)
        self.PID = wpilib.PIDController(0,0,0,channel4,self.motor1)
        
        self.isInverted = False
    def get_PID(self):
        return self.PID
    def pidWrite(self, output):
        self.motor1.set(output)
        self.motor2.set(output)
        self.motor3.set(output)
        
        
    def set(self, value):
        if(self.isInverted == True):
            self.motor1.set(-value)
            self.motor2.set(-value)
            self.motor3.set(-value)
        else:
            self.motor1.set(value)
            self.motor2.set(value)
            self.motor3.set(value)
            
    def set1(self, value):
        if(self.isInverted == True):
            self.motor1.set(-value)
        else:
            self.motor1.set(value)
    
    def set2(self, value):
        if(self.isInverted == True):
            self.motor2.set(-value)
        else:
            self.motor2.set(value)
            
    def set3(self, value):
        if(self.isInverted == True):
            self.motor3.set(-value)
        else:
            self.motor3.set(value)
    
    def setInverted(self, bool):
        self.isInverted = bool
        
    def get(self):
        return self.motor1.get()
    
    def getInverted(self):
        return self.isInverted

class DualPiston():

    def __init__(self, fwc1, rvc1, fwc2, rvc2):
        self.DS1 = wpilib.DoubleSolenoid(fwc1,rvc1)
        self.DS2 = wpilib.DoubleSolenoid(fwc2,rvc2)
        
    def set(self, value):
        self.DS1.set(value)
        self.DS2.set(value)
    
    def get(self):
        return self.DS1.get()



