from robotpy_ext.autonomous import StatefulAutonomous, timed_state, state
from wpilib import DoubleSolenoid
import time
class Example(StatefulAutonomous):
    
    MODE_NAME = 'SUPER_SIMPLE_AUTO'
    
    def initialize(self):
        pass 
    
    # initial move forward(put in to prevent initial sonar values from messing with the initial speed)
    @timed_state(duration=1, next_state = 'move_forward', first= True)
    def creep(self):
        self.LDT.set(.3)
        self.RDT.set(.31)
        
    # move at that speed until we reach a certain distance 
    @state
    def move_forward(self):
        while(self.Sonar.getAverageVoltage()>1.6):
            self.LDT.set(.3)
            self.RDT.set(.31)
        self.next_state('move_forward2')
        
    # once we reach the distance move at a slower speed until we are at the wall
    @state
    def move_forward2(self):
        while(self.Sonar.getAverageVoltage()>.45):
            self.LDT.set(.17)
            self.RDT.set(.173)
        self.next_state('wait')
            
    # wait a few secs while at the wall
    @timed_state(duration=2, next_state = 'drop')
    def wait(self):
        self.LDT.set(0)
        self.RDT.set(0)
    
      
    @state
    def drop(self): 
        # if the peg is in release the gear and then start wait_again 
        if(self.GS1.getAverageVoltage() > 1.35 or self.GS2.getAverageVoltage() > 1.35):
            self.GearMech.set(DoubleSolenoid.Value.kReverse)
            self.next_state('wait_again')
            
        # else start back_up2
        else:
            self.next_state('back_up2')
            
    # wait .2 seconds        
    @timed_state(duration = .2, next_state= 'back_up')
    def wait_again(self):
        self.LDT.set(0)
        self.RDT.set(0)
    
    #jerk backwards
    @timed_state(duration=.2, next_state ='stop')
    def back_up(self):
        self.LDT.set(-.17)
        self.RDT.set(-.171)
    @timed_state(duration=.5,next_state='back_up2')
    def stop(self):
        self.LDT.set(0)
        self.RDT.set(0)
    
    #backup slowly at first then increase your speed
    @state
    def back_up2(self):
        while(self.Sonar.getAverageVoltage()<.5):
            self.LDT.set(-.17)
            self.RDT.set(-.171)
        self.next_state('back_up3')
    @timed_state(duration=.7, next_state ='close')
    def back_up3(self):
        self.LDT.set(-.3)
        self.RDT.set(-.31)
    
    #close the gear mech and stop moving backwards    
    @state
    def close(self):
        self.LDT.set(0)
        self.RDT.set(0)
        self.GearMech.set(DoubleSolenoid.Value.kForward)
        self.next_state('turn')
    
    #turn right
    @timed_state(duration=1, next_state= 'turn2')
    def turn(self):
        self.LDT.set(.6)
        self.RDT.set(.2)
    
    #turn left
    @timed_state(duration= 1, next_state = 'straight')
    def turn2(self):
        self.LDT.set(.2)
        self.RDT.set(.55)
    
    #go straight
    @timed_state(duration = 2)
    def straight(self):
        self.LDT.set(.17)
        self.RDT.set(.173)
        
    #stop (robot is over the line)
    @state
    def end(self):
        self.LDT.set(0)
        self.RDT.set(0)
    