import kg_robot as kgr
import time
import numpy as np
import sched

urnie = kgr.kg_robot(port=30010, db_host="169.254.46.50")

fingertip_tcp = [0, 0, 0.1640, 0, 0, 0]
urnie.set_tcp(fingertip_tcp)

# print(urnie.getl())
# print(urnie.getj())
# return

defaultpose = [0, 0, 0, 0, 0, 0]  # Set at height just before fingertip makes contact with force plate
scheduler = sched.scheduler(time.time, time.sleep)


# In total time 'duration', move down to 'depth', pause for 'pauseduration', and return to starting point
# Angles are specified in radians by anglex & angley
def parameter_move(duration, t0, defaultpose, anglex, angley, depth, pauseduration):
    t = time.time() - t0
    if t < (duration-pauseduration)/2.0:
        npose = np.add(defaultpose, [0, 0, -depth*t*(2/(duration-pauseduration)), anglex, angley, 0])
    elif (duration-pauseduration)/2.0 <= t <= (duration+pauseduration)/2.0:
        npose = np.add(defaultpose, [0, 0, -depth, anglex, angley, 0])
    else:
        npose = np.add(defaultpose, [0, 0, -depth + depth*(t-(duration+pauseduration)/2.0)*(2/(duration-pauseduration)),
                                     anglex, angley, 0])
    # pass to UR5
    urnie.servoj(npose, vel=50, control_time=0.05)


def schedule_it(dt, callable, *args):
    for i in range(int(args[0]/dt)):
        scheduler.enter(i*dt, 1, callable, args)


def run_given_params(xangle, yangle, duration, pauseduration, depth):
    urnie.movel(np.add(defaultpose, [0, 0, 0, xangle, yangle, 0]))
    t0 = time.time()
    # initialise scheduler
    schedule_it(0.05, parameter_move, duration, t0, defaultpose, xangle, yangle, depth, pauseduration)
    # run scheduler calling servoj
    scheduler.run()


run_given_params(0, 0, 10, 2, 0.01)

urnie.close()
