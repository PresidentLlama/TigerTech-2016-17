#!/usr/bin/env python3
import wpilib
import ctre
from robot_systems import TripleMotorGearbox
from robot_systems import DualMotorGearbox
from robotpy_ext.common_drivers.navx import AHRS
from wpilib.doublesolenoid import DoubleSolenoid
from networktables import NetworkTable
from robotpy_ext.autonomous import AutonomousModeSelector
from robot_utils import robot_utils
from network_table import Network
import network_table
import math
import time

class MyRobot(wpilib.IterativeRobot):
    
    def robotInit(self):
        
        #Magic Numbers
        
        self.rotateToAngleRate = 0
        self.ControlSwitch = "GM"
        
        #Sensors 
        
        self.LeftEncoder = wpilib.Encoder(0, 1)
        self.LeftEncoder.reset()
   
        self.RightEncoder = wpilib.Encoder(2, 3, True)
        self.RightEncoder.reset()
        
        self.LeftEncoder.setDistancePerPulse((4*math.pi)/256)
        self.RightEncoder.setDistancePerPulse((4*math.pi)/256)
        
        self.Gyro = AHRS.create_spi()
        self.Gyro.reset()
        wpilib.CameraServer.launch()
        self.Sonar = wpilib.AnalogInput(0)
        self.GS1 = wpilib.AnalogInput(1)
        self.GS2 = wpilib.AnalogInput(2)
        
        #wpilib.CameraServer.launch('vision.py:main')
        
        #Motors
        
        self.LDT = TripleMotorGearbox(5, 6, 7, self.LeftEncoder)
        self.RDT = TripleMotorGearbox(0, 1, 2, self.RightEncoder)
        self.RDT.setInverted(True)
        
        self.LowIntake = wpilib.VictorSP(9)
        
        self.Winch = wpilib.VictorSP(3)
        
        self.Agitator = wpilib.VictorSP(10)
        self.Conveyor = wpilib.VictorSP(4)
        self.Shooter = ctre.CANTalon(25)
        
        #Controllers
        
        self.LeftJoystick = wpilib.Joystick(0)
        self.RightJoystick = wpilib.Joystick(1)
        self.XboxController = wpilib.Joystick(2)
        
        #Pneumatics
         
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.PSV = self.Compressor.getPressureSwitchValue()
        self.GearMech = wpilib.DoubleSolenoid(4, 3)
        self.TopFlap = wpilib.DoubleSolenoid(1, 2)
        self.Compressor.start()
        
        #Smart Dashboard
        
        self.nt = network_table.Network()
        self.SD = NetworkTable.getTable("SmartDashboard")
     
        self.SD.putBoolean("   Is Gear Mech Out?", False)
        
        self.SD.putNumber("Gyro", self.Gyro.getAngle())
    
        #Autonomous
        
        self.components = {

           'LDT': self.LDT,
           'RDT': self.RDT,
           'SD':self.SD,
           'LowIntake': self.LowIntake,
           'LeftEncoder': self.LeftEncoder,
           'RightEncoder': self.RightEncoder,
           'Sonar': self.Sonar,
           'Gyro': self.Gyro,
           'GearMech': self.GearMech,
           'Top Flap': self.TopFlap,
           'GS1': self.GS1,
           'GS2': self.GS2}
        
        self.Auton = AutonomousModeSelector('autonomous', self.components)
    
    def autonomousPeriodic(self):
        self.SD.putNumber("Gyro", self.Gyro.getYaw())
        self.Auton.run()
       
    
    def teleopInit(self):
        
        self.motorUpdatePeriod = 0.005
        self.networkUpdatePeriod = 0.25
  
        self.LeftEncoder.reset()
        self.RightEncoder.reset()
    def teleopPeriodic(self):
        
        self.Gyro.reset()
        
        while self.isEnabled():
            
            #self.SD.putNumber("Am i Alive?/Angle ", self.Gyro.getYaw())
            self.SD.putNumber("Gyro", self.Gyro.getYaw())
            self.SD.putNumber(" Sonar Voltage", self.Sonar.getVoltage())
            self.SD.putString("Operator Stage", self.ControlSwitch)
            self.SD.putNumber("Voltage 1", self.GS1.getAverageVoltage())
            self.SD.putNumber("Voltage 2", self.GS2.getAverageVoltage())
            
            
            
            #Drive Train 
            
            
            
            
            self.LJV = self.LeftJoystick.getY()
            self.RJV = self.RightJoystick.getY()
            
            
            # FORWARD
            if(self.LJV > 0.03 and self.RJV > 0.03):
                Lvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.LeftJoystick.getY()))
                Rvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.RightJoystick.getY()))
                self.LDT.set(-Lvalue)
                self.RDT.set(-Rvalue)
            # BACKWARDS
            elif(self.LJV < -0.03 and self.RJV < -0.03):
                Lvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.LeftJoystick.getY()))
                Rvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.RightJoystick.getY()))
                self.LDT.set(-Lvalue)
                self.RDT.set(-Rvalue)
            # Turning With Both Joysticks
            elif((self.LJV > 0.03 and self.RJV < -0.03) or (self.LJV < -0.03 and self.RJV > 0.03)):
                Lvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.LeftJoystick.getY()))
                Rvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.RightJoystick.getY()))
                self.LDT.set(-Lvalue/1.46)
                self.RDT.set(-Rvalue/1.46)
            # Spinning Only Left Joystick
            elif((self.LJV > 0 or self.LJV < 0) and (self.RJV > -0.15 and self.RJV < 0.15)):
                Lvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.LeftJoystick.getY()))
                Rvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.RightJoystick.getY()))
                self.LDT.set(-Lvalue/1.25)
                self.RDT.set(-Rvalue/1.25)
            # Spinning Only Right Joystick
            elif((self.RJV > 0 or self.RJV < 0) and (self.LJV > -0.15 and self.LJV < 0.15)):
                Lvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.LeftJoystick.getY()))
                Rvalue = robot_utils.Efficiency(self, robot_utils.cookJoystickInputs2(self, self.RightJoystick.getY()))
                self.LDT.set(-Lvalue/1.25)
                self.RDT.set(-Rvalue/1.25)
            else:
                self.LDT.set(0)
                self.RDT.set(0)
                
            #MICRO_CONTROLS
            while(self.LeftJoystick.getRawButton(5)):
                try:
                    self.LDT.set(.17)
                    self.RDT.set(-.17)

                except TypeError:
                    pass
            while(self.LeftJoystick.getRawButton(4)):
                try:
                    self.LDT.set(-.17)
                    self.RDT.set(.17)
                except TypeError:
                    pass
          
            while(self.LeftJoystick.getRawButton(2)):
                try:
                    self.LDT.set(-.17)
                    self.RDT.set(-.17)
                except TypeError:
                    pass
          
            while(self.LeftJoystick.getRawButton(3)):
                try:
                    self.LDT.set(.17)
                    self.RDT.set(.17)
                except TypeError:
                    pass
                
            #MACRO_CONTROLS
            while(self.RightJoystick.getRawButton(5)):
                try:
                    self.LDT.set(.3)
                    self.RDT.set(-.3)

                except TypeError:
                    pass 
            while(self.RightJoystick.getRawButton(4)):
                try:
                    self.LDT.set(-.3)
                    self.RDT.set(.3)

                except TypeError:
                    pass 
            while(self.RightJoystick.getRawButton(3)):
                try:
                    self.LDT.set(.3)
                    self.RDT.set(.3)

                except TypeError:
                    pass 
            while(self.RightJoystick.getRawButton(2)):
                try:
                    self.LDT.set(-.3)
                    self.RDT.set(-.3)

                except TypeError:
                    pass
            
            if(self.XboxController.getRawButton(5)):
                self.LowIntake.set(0.875)
            elif(self.XboxController.getRawButton(6)):
                self.LowIntake.set(-0.875)
            else:
                self.LowIntake.set(0)
            
            
            
            
            
            
            if(self.XboxController.getRawButton(7)):
                self.ControlSwitch = "SH"
            elif(self.XboxController.getRawButton(8)):
                self.ControlSwitch = "GM"
            
            if(self.ControlSwitch == "GM"):
                
 
                if(self.XboxController.getRawButton(2)):
                    self.TopFlap.set(DoubleSolenoid.Value.kForward)
                    
                if(self.XboxController.getRawButton(4)):
                    self.TopFlap.set(DoubleSolenoid.Value.kReverse)
                    
                if(self.XboxController.getRawButton(1)):
                    self.GearMech.set(DoubleSolenoid.Value.kForward)
                    
                if(self.XboxController.getRawButton(3)):
                    self.GearMech.set(DoubleSolenoid.Value.kReverse)
                    
                #Winch
                if(self.XboxController.getY()>.5):
                    self.Winch.set(.5)   
                elif(self.XboxController.getY()<-.5):
                    self.Winch.set(-.5)
                else:
                    self.Winch.set(0)
            elif(self.ControlSwitch == "SH"):
                
                #Agitator
                
                if(self.XboxController.getY() > 0.5):
                    self.Agitator.set(0.875)
                elif(self.XboxController.getY() < -0.5):
                    self.Agitator.set(-0.875)
                else:
                    self.Agitator.set(0)
                    
                #Conveyor
                
                if(self.XboxController.getRawButton(1)):
                    self.Conveyor.set(0.875)
                else:
                    self.Conveyor.set(0)
                    
                #Shooter
                
                if(self.XboxController.getRawButton(3)):
                    self.Shooter.set(0.5)
                else:
                    self.Shooter.set(0)
                
    
                
            
                
            
            
            
                
            
            
            
                
                
            
        wpilib.Timer.delay(self.motorUpdatePeriod)
            
    
    def disabledInit(self):
        pass

    
    def disabledPeriodic(self):
        pass
    
    def testInit(self):
        pass
      
    def testPeriodic(self):
      pass

    
        
    
        #pass
    
if __name__ == "__main__":
    wpilib.run(MyRobot)


