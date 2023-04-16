'''
This is just the base firmware code from my project AutoLaserPro
'''

from micropython import const
import random
import uasyncio

from TMC220X.debug import *
from TMC220X.driver import TMC220X, MovementStyle

X_RES = const(1)
Y_RES = const(1)

X_MIN = const(-50 * X_RES)
X_MAX = const(50 * X_RES)

Y_MIN = const(-20 * Y_RES)
Y_MAX = const(30 * Y_RES)

X_MAX_SPEED = const(40)
Y_MAX_SPEED = const(18)

MIN_DELAY = const(0.5)
MAX_DELAY = const(4)


async def main():
    x_motor = TMC220X(6, 7, 8)
    await readDRVSTATUS(x_motor)
    await x_motor.setCurrent(100)
    x_motor.setAcceleration(1000)
    x_motor.setMaxSpeed(X_MAX_SPEED)
    await x_motor.setMicrosteppingResolution(X_RES)

    y_motor = TMC220X(18, 19, 20, serialport=1, tx=4, rx=5)
    await readDRVSTATUS(y_motor)
    await y_motor.setCurrent(100)
    y_motor.setAcceleration(1000)
    y_motor.setMaxSpeed(Y_MAX_SPEED)
    await y_motor.setMicrosteppingResolution(Y_RES)

    x_motor.setMotorEnabled(True)
    y_motor.setMotorEnabled(True)

    await x_motor.runToPositionSteps(0, MovementStyle.ABSOLUTE)
    await y_motor.runToPositionSteps(0, MovementStyle.ABSOLUTE)

    while True:
        try:
            new_x_pos = random.randint(X_MIN, X_MAX)
            new_y_pos = random.randint(Y_MIN, Y_MAX)
            print(new_x_pos, new_y_pos)
            await uasyncio.gather(x_motor.runToPositionSteps(new_x_pos, MovementStyle.ABSOLUTE), y_motor.runToPositionSteps(new_y_pos, MovementStyle.ABSOLUTE))
            delay = random.random() * MAX_DELAY + MIN_DELAY
            await uasyncio.sleep(delay)
        except KeyboardInterrupt:
            break
    x_motor.setMotorEnabled(False)
    y_motor.setMotorEnabled(False)


uasyncio.run(main())