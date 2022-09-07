#!/bin/bash

/opt/aws/deepracer/util/pwm.sh enable
/opt/aws/deepracer/util/pwm.sh led on
sleep 1
/opt/aws/deepracer/util/pwm.sh led off
/bin/chgrp -R i2c /sys$1
/bin/chmod g+w /sys$1/*/duty_cycle /sys$1/*/enable /sys$1/*/period /sys$1/*/polarity /sys$1/*/uevent
