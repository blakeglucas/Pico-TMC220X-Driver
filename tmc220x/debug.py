from .driver import TMC220X
from . import registers

async def readDRVSTATUS(drv: TMC220X):
    '''
    Reads register address DRVSTATUS and prints the current settings
    '''
    print('TMC220X: ---')
    print('TMC220X: DRIVER STATUS:')
    drvstatus = await drv.uart.read_int(registers.DRVSTATUS)
    if(drvstatus & registers.stst):
        print('TMC220X: Info: motor is standing still')
    else:
        print('TMC220X: Info: motor is running')

    if(drvstatus & registers.stealth):
        print('TMC220X: Info: motor is running on StealthChop')
    else:
        print('TMC220X: Info: motor is running on SpreadCycle')

    cs_actual = drvstatus & registers.cs_actual
    cs_actual = cs_actual >> 16
    print('TMC220X: CS actual: '+str(cs_actual))

    if(drvstatus & registers.olb):
        print('TMC220X: Warning: Open load detected on phase B')
    
    if(drvstatus & registers.ola):
        print('TMC220X: Warning: Open load detected on phase A')
    
    if(drvstatus & registers.s2vsb):
        print('TMC220X: Error: Short on low-side MOSFET detected on phase B. The driver becomes disabled')

    if(drvstatus & registers.s2vsa):
        print('TMC220X: Error: Short on low-side MOSFET detected on phase A. The driver becomes disabled')

    if(drvstatus & registers.s2gb):
        print('TMC220X: Error: Short to GND detected on phase B. The driver becomes disabled. ')
    
    if(drvstatus & registers.s2ga):
        print('TMC220X: Error: Short to GND detected on phase A. The driver becomes disabled. ')
    
    if(drvstatus & registers.ot):
        print('TMC220X: Error: Driver Overheating!')
    
    if(drvstatus & registers.otpw):
        print('TMC220X: Warning: Driver Overheating Prewarning!')
    
    print('---')
    return drvstatus

async def readGCONF(drv: TMC220X):
    '''
    Reads register address GCONF and prints the current settings
    '''
    print('TMC220X: ---')
    print('TMC220X: GENERAL CONFIG')
    gconf = await drv.uart.read_int(registers.GCONF)

    if(gconf & registers.i_scale_analog):
        print('TMC220X: Driver is using voltage supplied to VREF as current reference')
    else:
        print('TMC220X: Driver is using internal reference derived from 5VOUT')
    if(gconf & registers.internal_rsense):
        print('TMC220X: Internal sense resistors. Use current supplied into VREF as reference.')
        print('TMC220X: VREF pin internally is driven to GND in this mode.')
        print('TMC220X: This will most likely destroy your driver!!!')
        raise SystemExit
    else:
        print('TMC220X: Operation with external sense resistors')
    if(gconf & registers.en_spreadcycle):
        print('TMC220X: SpreadCycle mode enabled')
    else:
        print('TMC220X: StealthChop PWM mode enabled')
    if(gconf & registers.shaft):
        print('TMC220X: Inverse motor direction')
    else:
        print('TMC220X: normal motor direction')
    if(gconf & registers.index_otpw):
        print('TMC220X: INDEX pin outputs overtemperature prewarning flag')
    else:
        print('TMC220X: INDEX shows the first microstep position of sequencer')
    if(gconf & registers.index_step):
        print('TMC220X: INDEX output shows step pulses from internal pulse generator')
    else:
        print('TMC220X: INDEX output as selected by index_otpw')
    if(gconf & registers.mstep_reg_select):
        print('TMC220X: Microstep resolution selected by MSTEP register')
    else:
        print('TMC220X: Microstep resolution selected by pins MS1, MS2')
    
    print('TMC220X: ---')
    return gconf

async def readGSTAT(drv: TMC220X):
    '''
    Reads register address GSTAT and prints the current settings
    '''
    print('TMC220X: ---')
    print('TMC220X: GSTAT')
    gstat = await drv.uart.read_int(registers.GSTAT)
    if(gstat & registers.reset):
        print('TMC220X: The Driver has been reset since the last read access to GSTAT')
    if(gstat & registers.drv_err):
        print('TMC220X: The driver has been shut down due to overtemperature or short circuit detection since the last read access')
    if(gstat & registers.uv_cp):
        print('TMC220X: Undervoltage on the charge pump. The driver is disabled in this case')
    print('TMC220X: ---')
    return gstat

async def readIOIN(drv: TMC220X):
    '''
    Reads register address IOIN and prints the current settings
    '''
    print('TMC2209: ---')
    print('TMC2209: INPUTS')
    ioin = await drv.uart.read_int(registers.IOIN)
    if(ioin & registers.io_spread):
        print('TMC2209: spread is high')
    else:
        print('TMC2209: spread is low')

    if(ioin & registers.io_dir):
        print('TMC2209: dir is high')
    else:
        print('TMC2209: dir is low')

    if(ioin & registers.io_step):
        print('TMC2209: step is high')
    else:
        print('TMC2209: step is low')

    if(ioin & registers.io_enn):
        print('TMC2209: en is high')
    else:
        print('TMC2209: en is low')
    
    print('TMC2209: ---')
    return ioin

async def readCHOPCONF(drv: TMC220X):
    '''
    Reads register address CHOPCONF and prints the current settings
    '''
    print('TMC2209: ---')
    print('TMC2209: CHOPPER CONTROL')
    chopconf = await drv.uart.read_int(registers.CHOPCONF)
    
    print('TMC2209: native '+str(await drv.getMicroSteppingResolution())+' microstep setting')
    
    if(chopconf & registers.intpol):
        print('TMC2209: interpolation to 256 microsteps')
    
    if(chopconf & registers.vsense):
        print('TMC2209: 1: High sensitivity, low sense resistor voltage')
    else:
        print('TMC2209: 0: Low sensitivity, high sense resistor voltage')

    print('TMC2209: ---')
    return chopconf