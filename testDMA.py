from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

factory = PiGPIOFactory()

# servo = Servo(17,pin_factory=factory)


# while True:
#     servo.min()
#     sleep(2)
#     servo.mid()
#     sleep(2)
#     servo.max()
#     sleep(2)


#servo = AngularServo(17, min_angle=90, max_angle=-90,pin_factory=factory)
servo = Servo(17,pin_factory=factory,max_pulse_width=2/1000)

# while True:
#     servo.angle = -90
#     sleep(2)
#     servo.angle = -45
#     sleep(2)
#     servo.angle = 0
#     sleep(2)
#     servo.angle = 45
#     sleep(2)
#     servo.angle = 90
#     sleep(2)

servo.min()
sleep(2)
servo.max()

