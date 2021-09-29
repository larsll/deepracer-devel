from time import sleep
import os
from BMI160_i2c import Driver
from BMI160_i2c import definitions

print('Trying to initialize the sensor...')
sensor = Driver(0x68, 1) # change address if needed
print('Initialization done')

sensor.setFullScaleAccelRange(definitions.ACCEL_RANGE_4G, 4.0)
sensor.setFullScaleGyroRange(definitions.GYRO_RANGE_250, 250.0)

sensor.autoCalibrateXAccelOffset(0)
sensor.autoCalibrateYAccelOffset(0)
sensor.autoCalibrateZAccelOffset(-1)

sleep(0.3)
sensor.setAccelOffsetEnabled(True)

while True:
  data = sensor.getMotion6()
  # fetch all gyro and acclerometer values
  print({
    'gx': '{:+.0f}'.format(data[0] / 0x8000 * 250),
    'gy': '{:+.0f}'.format(data[1] / 0x8000 * 250),
    'gz': '{:+.0f}'.format(data[2] / 0x8000 * 250),
    'ax': '{:+.2f}'.format(data[3] / 0x8000 * 4),
    'ay': '{:+.2f}'.format(data[4] / 0x8000 * 4),
    'az': '{:+.2f}'.format(data[5] / 0x8000 * 4)
  })
  
