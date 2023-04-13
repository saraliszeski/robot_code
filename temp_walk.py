from math import sin, cos
from pylx16a.lx16a import *
import time

LX16A.initialize("COM5")

try:
    servo1 = LX16A(1)#knee
    servo3 = LX16A(3)#hip2
    servo2 = LX16A(2)#knee
    servo4 = LX16A(4)#knee
    servo5 = LX16A(5)#knee
    servo6 = LX16A(6)#hip
    servo1.set_angle_offset(-30.00)
    servo1.set_angle_limits(0, 180)
    servo2.set_angle_limits(30, 150)
    servo3.set_angle_limits(40, 160)
    servo4.set_angle_limits(20, 220)
    servo5.set_angle_limits(20, 178)
    servo6.set_angle_limits(100, 230)
    
    print("moving")
    # servo1.move(120)
    servo1.move(98)
    servo2.move(30.0)
    servo3.move(120)
    # servo4.move(120)
    servo4.move(140)
    servo5.move(140)
    servo6.move(130)
    
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Timeout or disconnect error. Exiting...")
    quit()

except ServoChecksumError as e:
    print(f"Servo {e.id_} is not responding. Bad Checksum. Exiting...")
    quit()
    
except ServoArgumentError as e:
    print(f"Servo {e.id_} is not responding. Bad Argument. Exiting...")
    quit()

except ServoLogicalError as e:
    print(f"Servo {e.id_} is not responding. Bad Argument. Exiting...")
    quit()

t = 0
while True:
    #motor 4: 138 to 115, (move to 138 first)
    try: 
        # servo1.move(sin(0.5*t) * 90 + 90)
        i=-2.3
        j = -1.5
        k=-2.8
        l = -7.00
        m = -0.5
        factor = 0.35
        for step in range(int(2*3.14/factor)*10): #want to move from 42 to 120
            # servo6.move(sin(0.55*k)*9+131)
            servo2.move(cos(factor*i) * 15 + 65)
            # servo5.move(cos(0.25*l) * 20 + 125)
            time.sleep(0.005)
            i+=0.1
            l += 0.1
        time.sleep(0.25)
        for step in range(int(2*3.14/factor)*10): #want to move from 42 to 120
            # servo6.move(sin(0.55*k)*9+131)
            # servo2.move(sin(0.25*i) * 20 + 85)
            servo5.move(sin(factor*l) * 15 + 125)
            time.sleep(0.005)
            i+=0.1
            l += 0.1
        time.sleep(0.25)
        # while l < 12.4:
        #     # servo1.move(sin(j)*19 +75)
        #     servo5.move(cos(0.5*l) * 20 + 125)
        #     # servo4.move(sin(m) * 19 + 126.5)
        #     time.sleep(0.005)
        #     j+=0.1
        #     k+=0.1
        #     l+=0.1
            # m+= 0.1
        # time.sleep(0.25)
        i = -3.48
        j = -0.5
        # while j < 5.8: #want to move from 82 to 138c
        #     servo5.move(sin(i) * 9 + 110)
        #     servo4.move(sin(j) * 9 + 126.5)
        #     time.sleep(0.05)
        #     i+=0.1
        #     j+=0.1
        # servo3.move(sin(0.5*t) * 55 + 100)
        time.sleep(0.5)
        t += 0.1
    except ServoError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        quit()
    
