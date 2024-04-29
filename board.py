import time
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg
from rpi_ws281x import PixelStrip
from rpi_ws281x import Color as PixelColor

ADC_BAT_ADDR = 0
SERVO_ADDR = 21
MOTOR_ADDR = 31
SERVO_ADDR_CMD = 40

motor_speeds = [0, 0, 0, 0]
servo_angles = [0, 0, 0, 0, 0, 0]
servo_pulse = [0, 0, 0, 0, 0, 0]

I2C_ADDR = 0x7a
i2c = 1

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)


def setMotor(index, speed):
    if index < 1 or index > 4:
        raise AttributeError(f"Invalid motor number: {index}")
    if index == 2 or index == 4:
        speed = speed
    else:
        speed = -speed
    index -= 1
    speed = 100 if speed > 100 else speed
    speed = -100 if speed < -100 else speed
    reg = MOTOR_ADDR + index

    with SMBus(i2c) as bus:
        try:
            msg = i2c_msg.write(I2C_ADDR, [reg, speed.to_bytes(1, "little", signed=True)[0]])
            bus.i2c_rdwr(msg)
            motor_speeds[index] = speed
        except:
            msg = i2c_msg.write(I2C_ADDR, [reg, speed.to_bytes(1, "little", signed=True)[0]])
            bus.i2c_rdwr(msg)
            motor_speeds[index] = speed
    
    return motor_speeds[index]
 

def getMotor(index):
    if index < 1 or index > 4:
        raise AttributeError(f"Invalid motor number: {index}")
    
    index -= 1
    return motor_speeds[index]


def setBuzzer(new_state):
    GPIO.setup(31, GPIO.OUT)
    GPIO.output(31, new_state)


def getBattery():
    ret = 0
    with SMBus(i2c) as bus:
        try:
            msg = i2c_msg.write(I2C_ADDR, [ADC_BAT_ADDR,])
            bus.i2c_rdwr(msg)
            read = i2c_msg.read(I2C_ADDR, 2)
            bus.i2c_rdwr(read)
            ret = int.from_bytes(bytes(list(read)), "little")
        except:
            msg = i2c_msg.write(I2C_ADDR, [ADC_BAT_ADDR,])
            bus.i2c_rdwr(msg)
            read = i2c_msg.read(I2C_ADDR, 2)
            bus.i2c_rdwr(read)
            ret = int.from_bytes(bytes(list(read)), "little")
   
    return ret


if __name__ == "__main__":
    setBuzzer(1)
    time.sleep(1)
    setBuzzer(0)
    for i in range(1, 5):
        setMotor(i, 60)
        print(getBattery())
        print(getMotor(i))

    time.sleep(5)
    for i in range(1, 5):
        setMotor(i, 0)


