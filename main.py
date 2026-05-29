import kg_robot as kgr
import time
import numpy as np
import sched
import serial
import os
import datetime
import random

savestring = "GDa"
with open("Data/filename.txt", "w") as f:
    f.write(savestring)

urnie = kgr.kg_robot(port=30010, db_host="169.254.53.12")
urnie.set_tcp([0, 0, 0.1640, 0, 0, 0])
# print(urnie.getl())
# urnie.close()
# exit()

# Set at height just before fingertip makes contact with force plate
# defaultpose = [0.292042, -0.174061, 0.118453, 2.25038, -2.18194, -0.00928678]
# defaultpose = [0.290023, -0.19214, 0.0874012+0.01, 2.25038, -2.18195, -0.00929016]
# defaultpose = [0.35031, -0.177798, 0.117821, 2.26559, -2.17504, -0.0158261]
# defaultpose = [0.284625, -0.173465, 0.0862045, 2.26555, -2.17506, -0.0158023]
# defaultpose = [0.284626, -0.173463, 0.0904381, 2.26556, -2.17506, -0.0158294]
# defaultpose = [0.350342, -0.178356, 0.122167, 2.2656, -2.17502, -0.0158225]
# defaultpose = [0.350333, -0.178363, 0.117765, 2.26557, -2.17503, -0.0158209]
defaultpose = [0.288454, -0.165453, 0.0860761, 2.26561, -2.17504, -0.0157469]

urnie.movel(defaultpose, vel=0.05, acc=0.05)
# urnie.close()
# exit()
# defaultpose = urnie.getl()
scheduler = sched.scheduler(time.time, time.sleep)


# In total time 'duration', move down to 'depth', pause for 'pauseduration', and return to starting point
# Angles are specified in radians by anglex & angley
def parameter_move(duration, t0, defaultpose, x, y, anglex, angley, depth, pauseduration):
    t = time.time() - t0
    if t < (duration-pauseduration)/2.0:
        currentdepth = depth*t*(2/(duration-pauseduration))
    elif (duration-pauseduration)/2.0 <= t <= (duration+pauseduration)/2.0:
        currentdepth = depth
    else:
        currentdepth = depth-depth*(t-(duration+pauseduration)/2.0)*(2/(duration-pauseduration))

    now = datetime.datetime.now()
    f.write(now.strftime("%m/%d/%Y, %H:%M:%S:%f")+", "+str(x)+", "+str(y)+", "+str(currentdepth)+", "+str(anglex)+", "+str(angley)+"\n")

    npose = np.add(defaultpose, [x, y, -currentdepth, anglex, angley, 0])
    # pass to UR5
    urnie.servoj(npose, vel=0.5, control_time=0.05)


def schedule_it(dt, callable, *args):
    for i in range(int(args[0]/dt)):
        scheduler.enter(i*dt, 1, callable, args)


def run_given_params(x, y, xangle, yangle, duration, pauseduration, depth):
    urnie.movel(np.add(defaultpose, [x, y, 0, xangle, yangle, 0]), vel=0.05, acc=0.05)
    t0 = time.time()
    # initialise scheduler
    schedule_it(0.05, parameter_move, duration, t0, defaultpose, x, y, xangle, yangle, depth, pauseduration)
    # run scheduler calling servoj
    scheduler.run()
    time.sleep(1)


def collectlocalization(n):
    for i in range(n):
        print(i)
        x = 20*random.random() - 10
        y = 20 * random.random() - 10
        depthoffset = 12.5 - (12.5 ** 2 - (x ** 2 + y ** 2)) ** 0.5
        run_given_params(0.001*x, 0.001*y, 0, 0, 3, 1, 0.005+0.001*depthoffset)



os.system("LogArduino.ttl")
os.system("LogScales.ttl")
time.sleep(3)

with open("Data/"+savestring+"_positions.txt", "w") as f:

    # collectlocalization(1000)

    # for i in range(10):
    #     print(i)
    #     for j in range(5):
    #         print(j)
    #         #  x, y, Xangle, Yangle, Duration, PauseDuration, Depth
    #         run_given_params(0, 0, 0, 0, 6, j, 0.001)
    #         run_given_params(0, 0, 0, 0, 6, j, 0.002)
    #         run_given_params(0, 0, 0, 0, 6, j, 0.003)
    #         run_given_params(0, 0, 0, 0, 6, j, 0.004)
    #         run_given_params(0, 0, 0, 0, 6, j, 0.005)
    #
    #         run_given_params(0, 0, 0, 0, 4, j/2, 0.001)
    #         run_given_params(0, 0, 0, 0, 4, j/2, 0.002)
    #         run_given_params(0, 0, 0, 0, 4, j/2, 0.003)
    #         run_given_params(0, 0, 0, 0, 4, j/2, 0.004)
    #         run_given_params(0, 0, 0, 0, 4, j/2, 0.005)

    for i in range(5):
        print(i)
        for j in range(3):
            print(j)
            #  x, y, Xangle, Yangle, Duration, PauseDuration, Depth
            run_given_params(0, 0, 0, 0, 12, 2*j, 0.001)
            run_given_params(0, 0, 0, 0, 12, 2*j, 0.003)
            run_given_params(0, 0, 0, 0, 12, 2*j, 0.005)
            run_given_params(0, 0, 0, 0, 12, 2*j, 0.008)

            run_given_params(0, 0, 0, 0, 8, 2*j, 0.001)
            run_given_params(0, 0, 0, 0, 8, 2*j, 0.003)
            run_given_params(0, 0, 0, 0, 8, 2*j, 0.005)
            run_given_params(0, 0, 0, 0, 8, 2*j, 0.008)

os.system("taskkill /IM ttermpro.exe")
urnie.movel(np.add(defaultpose, [0, 0, 0.005, 0, 0, 0]), vel=0.05, acc=0.05)

urnie.close()