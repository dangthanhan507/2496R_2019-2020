import numpy as np
import matplotlib.pyplot as plt

v_max = 300
accel_max = 100
target = 2000

v_current = 0
accel_current = 0
pos_current = 0

t1 = v_max / accel_max
t2 = (target/v_max - v_max/accel_max)+t1
t3 = t1 + t2

def f(x):
    if(x <= t1):
        if(accel_max * x > v_max):
            return v_max
        else:
            return accel_max * x
    elif(x <= t2):
        return v_max
    elif(x <= t3):
        return v_max - accel_max * (x-t2)


vf = np.vectorize(f)


t0 = np.arange(0,t3,.1)

plt.title('Motion Profiling Test')
plt.xlabel('Time')
plt.ylabel('Velocity(cm/s)')    
plt.axis([0,t3+5,0,v_max+50])

plt.plot(t0, vf(t0), 'r-')

plt.show()
