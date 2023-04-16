from micropython import const
import uasyncio

from TMC220X.debug import *
from TMC220X.driver import TMC220X, MovementStyle

X_RES = const(1)
X_SPEED = const(1000)

async def main():
    x_motor = TMC220X(6, 7, 8)
    await readDRVSTATUS(x_motor)
    await x_motor.setCurrent(100)
    x_motor.setAcceleration(1000)
    x_motor.setMaxSpeed(X_SPEED)
    await x_motor.setMicrosteppingResolution(X_RES)
    x_motor.setMotorEnabled(True)

    while True:
        try:
            await x_motor.runToPositionInSteps(20, MovementStyle.RELATIVE)
            await uasyncio.sleep(2)
        except KeyboardInterrupt:
            break
    x_motor.setMotorEnabled(False)


uasyncio.run(main())