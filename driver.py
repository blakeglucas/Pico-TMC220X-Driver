from machine import Pin
import math
from . import registers
import time
from .uart import TMC_UART
import uasyncio
from .utils import set_bit, clear_bit, Enum

def mean(obj: list):
    return sum(obj) / len(obj)

class Direction(Enum):
    CCW = 0
    CW = 1

class MovementStyle(Enum):
    ABSOLUTE = 0
    RELATIVE = 1

class SpreadOrStealth(Enum):
    StealthChop = 0
    SpreadCycle = 1

class TMC220X(object):
    _direction = Direction.CW
    _stop = False

    _msres = -1
    _stepsPerRevolution = 0

    _currentPos = 0                 # current position of stepper in steps
    _targetPos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _maxSpeed = 1.0                 # the maximum speed in steps per second
    _maxSpeedHoming = 500           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _accelerationHoming = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _stepInterval = 0               # the current interval between two steps
    _minPulseWidth = 1              # minimum allowed pulse with in microseconds
    _lastStepTime = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _sg_threshold = 100             # threshold for stallguard
    _movement_style = MovementStyle.ABSOLUTE

    def __init__(self, pin_step: int | str, pin_dir: int | str, pin_en: int | str, baud = 115200, serialport = 0, tx: int | Pin | None = None, rx: int | Pin | None = None):
        self.uart = TMC_UART(serialport, baud, tx, rx)
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self._pin_en = pin_en
        self.pin_step = Pin(self._pin_step, Pin.OUT)
        self.pin_dir = Pin(self._pin_dir, Pin.OUT)
        self.pin_en = Pin(self._pin_en, Pin.OUT, pull=Pin.PULL_UP)
        self.pin_dir.value(self._direction)
        uasyncio.run_until_complete(self.__async_init())
        self.uart.flushSerialBuffer()

    def __del__(self):
        self.setMotorEnabled(False)
    
    async def __async_init(self):
        '''
        Any asynchronous setup calls go here
        '''
        await self.readStepsPerRevolution()
        await self.clearGSTAT()

    @property
    def direction(self):
        return self._direction
    
    @direction.setter
    def direction(self, d):
        self._direction = d
        self.pin_dir.value(self._direction)
    
    def setMovementStyle(self, style: MovementStyle):
        self._movement_style = style

    def setMotorEnabled(self, en):
        '''
        TMC220X EN pin is active LOW! This reads opposite of the value passed, so we invert it
        '''
        self.pin_en.value(not en)

    def getCurrentPosition(self):
        return self._currentPos
    
    def setCurrentPosition(self, newPos):
        self._currentPos = newPos
    
    def reverseDirection(self):
        self.direction = not self.direction

    def setDirection(self, d: Direction):
        self.direction = d

    async def getDirection_reg(self):
        gconf = await self.uart.read_int(registers.GCONF)
        return (gconf & registers.shaft)
    
    async def setDirection_reg(self, direction):
        '''
        Sets the motor shaft direction to the given value in the GCONF register: 0 = CCW; 1 = CW
        '''    
        gconf = await self.uart.read_int(registers.GCONF)
        if(direction):
            gconf = set_bit(gconf, registers.shaft)
        else:
            gconf = clear_bit(gconf, registers.shaft)
        await self.uart.write_reg_check(registers.GCONF, gconf)

    async def getIScaleAnalog(self):
        gconf = await self.uart.read_int(registers.GCONF)
        return (gconf & registers.i_scale_analog)
    
    async def clearGSTAT(self):
        '''
        Clears GSTAT register address
        '''
        gstat = await self.uart.read_int(registers.GSTAT)
        gstat = set_bit(gstat, registers.reset)
        gstat = set_bit(gstat, registers.drv_err)
        await self.uart.write_reg_check(registers.GSTAT, gstat)
    
    async def setIScaleAnalog(self,en):        
        gconf = await self.uart.read_int(registers.GCONF)
        if(en):
            gconf = set_bit(gconf, registers.i_scale_analog)
        else:
            gconf = clear_bit(gconf, registers.i_scale_analog)
        await self.uart.write_reg_check(registers.GCONF, gconf)

    async def getVSense(self):
        '''
        Returns which sense resistor voltage is used for current scaling:
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)
        return (chopconf & registers.vsense)
    
    async def setVSense(self,en):
        '''
        Sets which sense resistor voltage is used for current scaling:
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)
        if(en):
            chopconf = set_bit(chopconf, registers.vsense)
        else:
            chopconf = clear_bit(chopconf, registers.vsense)
        await self.uart.write_reg_check(registers.CHOPCONF, chopconf)

    async def getInternalRSense(self):
        '''
        Returns which sense resistor voltage is used for current scaling:
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        '''
        gconf = await self.uart.read_int(registers.GCONF)
        return (gconf & registers.internal_rsense)
    
    async def setInternalRSense(self,en):
        '''
        Sets which sense resistor voltage is used for current scaling:
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        '''
        gconf = await self.uart.read_int(registers.GCONF)
        if(en):
            gconf = set_bit(gconf, registers.internal_rsense)
        else:
            gconf = clear_bit(gconf, registers.internal_rsense)
        await self.uart.write_reg_check(registers.GCONF, gconf)

    async def setIRun_Ihold(self, IHold, IRun, IHoldDelay):
        '''
        sets the current scale (CS) for Running and Holding
        and the delay, when to be switched to Holding current
        IHold = 0-31; IRun = 0-31; IHoldDelay = 0-15
        '''
        ihold_irun = 0
        
        ihold_irun = ihold_irun | IHold << 0
        ihold_irun = ihold_irun | IRun << 8
        ihold_irun = ihold_irun | IHoldDelay << 16
        await self.uart.write_reg_check(registers.IHOLD_IRUN, ihold_irun)

    async def setCurrent(self, run_current, hold_current_multiplier = 0.5, hold_current_delay = 10, Vref = 1.2):
        '''
        sets the current flow for the motor
        run_current in mA
        check whether Vref is actually 1.2V
        '''
        CS_IRun = 0
        Rsense = 0.11
        Vfs = 0

        if(await self.getVSense()):
            Vfs = 0.180 * Vref / 2.5
        else:
            Vfs = 0.325 * Vref / 2.5
            
        CS_IRun = 32.0*1.41421*run_current/1000.0*(Rsense+0.02)/Vfs - 1

        CS_IRun = min(CS_IRun, 31)
        CS_IRun = max(CS_IRun, 0)
        
        CS_IHold = hold_current_multiplier * CS_IRun

        CS_IRun = round(CS_IRun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        await self.setIRun_Ihold(CS_IHold, CS_IRun, hold_current_delay)

    async def getSpreadCycle(self):
        '''
        Returns whether spreadcycle (1) is active or stealthchop (0)
        '''
        gconf = await self.uart.read_int(registers.GCONF)
        return (gconf & registers.en_spreadcycle)

    async def setSpreadCycle(self, en_spread: SpreadOrStealth):
        '''
        Enables spreadcycle (1) or stealthchop (0)
        '''
        gconf = await self.uart.read_int(registers.GCONF)
        if(en_spread):
            gconf = set_bit(gconf, registers.en_spreadcycle)
        else:
            gconf = clear_bit(gconf, registers.en_spreadcycle)
        await self.uart.write_reg_check(registers.GCONF, gconf)

    async def getInterpolation(self):
        '''
        Returns whether the TMC inbuilt interpolation is active
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)
        if(chopconf & registers.intpol):
            return True
        else:
            return False

    async def setInterpolation(self, en):
        '''
        Enables the tmc inbuilt interpolation of the steps to 256 microsteps
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)

        if(en):
            chopconf = set_bit(chopconf, registers.intpol)
        else:
            chopconf = clear_bit(chopconf, registers.intpol)
        await self.uart.write_reg_check(registers.CHOPCONF, chopconf)

    async def getMicroSteppingResolution(self):
        '''
        Returns the current native microstep resolution (1-256)
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)
        msresdezimal = chopconf & (registers.msres0 | registers.msres1 | registers.msres2 | registers.msres3)
        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal
        self._msres = int(math.pow(2, msresdezimal))
        return self._msres

    async def setMicrosteppingResolution(self, msres):
        '''
        Sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)
        '''
        chopconf = await self.uart.read_int(registers.CHOPCONF)
        chopconf = chopconf & (~registers.msres0 | ~registers.msres1 | ~registers.msres2 | ~registers.msres3) #setting all bits to zero
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = int(chopconf) & int(4043309055)
        chopconf = chopconf | msresdezimal <<24
        
        await self.uart.write_reg_check(registers.CHOPCONF, chopconf)
        await self.setMStepResolutionRegSelect(True)
        await self.readStepsPerRevolution()

    async def setMStepResolutionRegSelect(self, en):
        '''
        Sets the register bit 'mstep_reg_select' to 1 or 0 depending to the given value.
        This is needed to set the microstep resolution via UART
        This method is called by 'setMicrosteppingResolution'
        '''
        gconf = await self.uart.read_int(registers.GCONF)
        
        if(en == True):
            gconf = set_bit(gconf, registers.mstep_reg_select)
        else:
            gconf = clear_bit(gconf, registers.mstep_reg_select)

        await self.uart.write_reg_check(registers.GCONF, gconf)

    async def readStepsPerRevolution(self):
        '''
        Returns how many steps are needed for one revolution
        '''
        self._stepsPerRevolution = 200 * (await self.getMicroSteppingResolution())
        return self._stepsPerRevolution
    
    async def getInterfaceTransmissionCounter(self):
        '''
        Reads the interface transmission counter from the tmc register
        This value is increased on every succesfull write access
        Can be used to verify a write access
        '''
        ifcnt = await self.uart.read_int(registers.IFCNT)
        return ifcnt
    
    async def getTStep(self):
        '''
        Returns the current stallguard result
        Its will be calculated with every fullstep
        Higher values mean a lower motor load
        '''
        tstep = await self.uart.read_int(registers.TSTEP)
        return tstep
    
    async def getStallguard_Result(self):
        '''
        Returns the current stallguard result
        Its will be calculated with every fullstep
        Higher values means a lower motor load
        '''
        sg_result = await self.uart.read_int(registers.SG_RESULT)
        return sg_result
    
    async def setStallguard_Threshold(self, threshold):
        '''
        Sets the register bit 'SGTHRS' to to a given value
        This is needed for the stallguard interrupt callback
        SG_RESULT becomes compared to the double of this threshold.
        SG_RESULT ≤ SGTHRS*2
        '''
        await self.uart.write_reg_check(registers.SGTHRS, threshold)
    
    async def setCoolStep_Threshold(self, threshold):
        '''
        This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output. (unsigned)      
        '''
        await self.uart.write_reg_check(registers.TCOOLTHRS, threshold)

    async def setStallguard_Callback(self, pin_stallguard, threshold, my_callback, min_speed = 2000):
        '''
        Sets a function to call back when the driver detects a stall via stallguard
        High value on the diag pin can also mean a driver error
        '''
        await self.setStallguard_Threshold(threshold)
        await self.setCoolStep_Threshold(min_speed)
        p25 = Pin(pin_stallguard, Pin.IN, Pin.PULL_DOWN)
        p25.irq(trigger=Pin.IRQ_RISING, handler=my_callback)

    async def getCoolSense_Threshold(self):
        coolthrs = await self.uart.read_int(registers.TCOOLTHRS)
        return coolthrs

    async def getPWM_Threshold(self):
        return await self.uart.read_int(registers.TPWMTHRS)

    async def setVActual(self, v):
        return await self.uart.write_reg_check(registers.VACTUAL, v)

    async def getMicrostepCounter(self):
        '''
        Returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A
        '''
        mscnt = await self.uart.read_int(registers.MSCNT)
        return mscnt
    
    async def getMicrostepCounterInSteps(self, offset=0):
        '''
        Returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A
        '''
        mscnt = await self.getMicrostepCounter()
        step = (mscnt - 64) * self._msres * 4 / 1024
        step = (4*self._msres) - step - 1
        step = round(step)
        return step + offset
    
    def setMaxSpeed(self, speed):
        '''
        Sets the maximum motor speed in steps per second
        '''
        if (speed < 0.0):
           speed = -speed
        if (self._maxSpeed != speed):
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if (self._n > 0):
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.computeNewSpeed()

    def getMaxSpeed(self):
        '''
        Returns the maximum motor speed in steps per second
        '''
        return self._maxSpeed
    
    def setAcceleration(self, acceleration):
        '''
        Sets the motor acceleration/decceleration in steps per sec per sec
        '''
        if (acceleration == 0.0):
            return
        if (acceleration < 0.0):
          acceleration = -acceleration
        if (self._acceleration != acceleration):
            # Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
            # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.computeNewSpeed()

    def getAcceleration(self):
        '''
        Returns the motor acceleration/decceleration in steps per sec per sec
        '''
        return self._acceleration
    
    def stop(self):
        self._stop = True

    async def runToPositionSteps(self, steps, movement_style = None):
        '''
        Runs the motor to the given position with acceleration and deceleration.
        Blocks the code until finished or stopped from a different thread!
        Returns true when the movement if finished normally and false when the movement was stopped
        '''
        if movement_style is not None:
            this_movement_style = movement_style
        else:
            this_movement_style = self._movement_style

        if this_movement_style == MovementStyle.RELATIVE:
            self._targetPos = self._currentPos + steps
        else:
            self._targetPos = steps

        self._stop = False
        self._stepInterval = 0
        self._speed = 0.0
        self._n = 0
        self.computeNewSpeed()
        while (self.__run() and not self._stop):
            await uasyncio.sleep_ms(0)
        return not self._stop
    
    def runToPositionRevolutions(self, revolutions, movement_style = None):
        return self.runToPositionSteps(round(revolutions * self._stepsPerRevolution), movement_style)
    
    def __run(self):
        if (self.runSpeed()): #returns true, when a step is made
            self.computeNewSpeed()
            #print(self.getStallguard_Result())
            #print(self.getTStep())
        return (self._speed != 0.0 and self.distanceToGo() != 0)
    
    def distanceToGo(self):
        return self._targetPos - self._currentPos
    
    def computeNewSpeed(self):
        '''
        returns the calculated current speed depending on the acceleration
        this code is based on: 
        'Generate stepper-motor speed profiles in real time' by David Austin
        
        https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
        '''
        distanceTo = self.distanceToGo() # +ve is clockwise from curent location     
        stepsToStop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16      
        if (distanceTo == 0 and stepsToStop <= 1):
            # We are at the target and its time to stop
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return
        
        if (distanceTo > 0):
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) or self._direction == Direction.CCW):
                    self._n = -stepsToStop # Start deceleration
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) and self._direction == Direction.CW):
                    self._n = -self._n # Start accceleration
        elif (distanceTo < 0):
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= -distanceTo) or self._direction == Direction.CW):
                    self._n = -stepsToStop # Start deceleration
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((stepsToStop < -distanceTo) and self._direction == Direction.CCW):
                    self._n = -self._n # Start accceleration
        # Need to accelerate or decelerate
        if (self._n == 0):
            # First step from stopped
            self._cn = self._c0
            self.pin_step.off()
            if(distanceTo > 0):
                self.setDirection(Direction.CW)     # type:ignore
            else:
                self.setDirection(Direction.CCW)    # type:ignore
        else:
            # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
            self._cn = max(self._cn, self._cmin)
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if (self._direction == 0):
            self._speed = -self._speed

    def runSpeed(self):
        if (not self._stepInterval):
            return False
        
        curtime = time.ticks_us()
        
        if (curtime - self._lastStepTime >= self._stepInterval):
            if not self._stop:
                if (self._direction == 1): # Clockwise
                    self._currentPos += 1
                else: # Anticlockwise 
                    self._currentPos -= 1
                self.makeAStep()
                
                self._lastStepTime = curtime # Caution: does not account for costs in step()
                return True
        else:
            return False
        
    def makeAStep(self):
        self.pin_step.on()
        time.sleep_us(1)
        self.pin_step.off()
        time.sleep_us(10)