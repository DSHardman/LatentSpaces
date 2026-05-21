import kg_robot as kgr
import time
import numpy as np
import sched
import serial
import os
import datetime

savestring = "H1a"
with open("Data/filename.txt", "w") as f:
    f.write(savestring)

urnie = kgr.kg_robot(port=30010, db_host="169.254.53.12")
urnie.set_tcp([0, 0, 0.1640, 0, 0, 0])
# print(urnie.getl())
# urnie.close()
# exit()

# Set at height just before fingertip makes contact with force plate
# defaultpose = [0.20578, -0.458081, 0.100106, 2.25037, -2.18194, -0.00930407]
defaultpose = [0.290023, -0.19214, 0.0874012, 2.25038, -2.18195, -0.00929016]

urnie.movel(defaultpose, vel=0.05, acc=0.05)
# urnie.close()
# exit()
# defaultpose = urnie.getl()
scheduler = sched.scheduler(time.time, time.sleep)


# In total time 'duration', move down to 'depth', pause for 'pauseduration', and return to starting point
# Angles are specified in radians by anglex & angley
def parameter_move(duration, t0, defaultpose, anglex, angley, depth, pauseduration):
    t = time.time() - t0
    if t < (duration-pauseduration)/2.0:
        currentdepth = depth*t*(2/(duration-pauseduration))
        npose = np.add(defaultpose, [0, 0, -currentdepth, anglex, angley, 0])
    elif (duration-pauseduration)/2.0 <= t <= (duration+pauseduration)/2.0:
        currentdepth = depth
        # npose = np.add(defaultpose, [0, 0, -depth, anglex, angley, 0])
    else:
        currentdepth = depth-depth*(t-(duration+pauseduration)/2.0)*(2/(duration-pauseduration))

    now = datetime.datetime.now()
    f.write(now.strftime("%m/%d/%Y, %H:%M:%S:%f")+", "+str(0)+", "+str(0)+", "+str(currentdepth)+", "+str(anglex)+", "+str(angley)+"\n")

    npose = np.add(defaultpose, [0, 0, -currentdepth, anglex, angley, 0])
    # pass to UR5
    urnie.servoj(npose, vel=0.5, control_time=0.05)


def schedule_it(dt, callable, *args):
    for i in range(int(args[0]/dt)):
        scheduler.enter(i*dt, 1, callable, args)


def run_given_params(xangle, yangle, duration, pauseduration, depth):
    urnie.movel(np.add(defaultpose, [0, 0, 0, xangle, yangle, 0]), vel=0.05, acc=0.05)
    t0 = time.time()
    # initialise scheduler
    schedule_it(0.05, parameter_move, duration, t0, defaultpose, xangle, yangle, depth, pauseduration)
    # run scheduler calling servoj
    scheduler.run()
    time.sleep(1)


os.system("LogArduino.ttl")
os.system("LogScales.ttl")
time.sleep(3)

with open("Data/"+savestring+"_positions.txt", "w") as f:
    # Xangle, Yangle, Duration, PauseDuration, Depth
    for i in range(10):
        print(i)
        for j in range(5):
            print(j)
            run_given_params(0, 0, 6, j, 0.001)
            run_given_params(0, 0, 6, j, 0.002)
            run_given_params(0, 0, 6, j, 0.003)
            run_given_params(0, 0, 6, j, 0.004)
            run_given_params(0, 0, 6, j, 0.005)

            run_given_params(0, 0, 4, j/2, 0.001)
            run_given_params(0, 0, 4, j/2, 0.002)
            run_given_params(0, 0, 4, j/2, 0.003)
            run_given_params(0, 0, 4, j/2, 0.004)
            run_given_params(0, 0, 4, j/2, 0.005)

    # run_given_params(0.4, 0, 4, 2, 0.02)

os.system("taskkill /IM ttermpro.exe")
urnie.movel(np.add(defaultpose, [0, 0, 0.005, 0, 0, 0]), vel=0.05, acc=0.05)

urnie.close()