import maestro as m
import time

SERVO_IZQ=4
SERVO_DER=5

s = m.Controller()
target = 1200 * 4
right_target = 1350
left_target = 1500 - (1500 - right_target) / 1.6

#right_target = target
#left_target = target

print right_target
print left_target

s.setTarget(SERVO_IZQ, int(left_target))
s.setTarget(SERVO_DER, int(right_target))
time.sleep(5) # sleep 5 seconds
s.setTarget(4,1500)
s.setTarget(5,1500)
time.sleep(1)
s.setTarget(4,0)
s.setTarget(5,0)

