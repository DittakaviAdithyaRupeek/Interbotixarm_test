#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
import matplotlib.animation as animation

MAX_LEN = 50

# based on 9 motors present in the Interbotix WX250s Robotic Arm
q0 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # waist
q1 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # shoulder
q2 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # elbow
q3 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # forearm_roll
q4 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # wrist_angle
q5 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # wrist_rotate
q6 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # gripper
q7 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # left_finger
q8 = deque(np.zeros(MAX_LEN), maxlen=MAX_LEN)    # right_finger

def init():
    line0.set_ydata([np.nan] * len(x))
    line1.set_ydata([np.nan] * len(x))
    line2.set_ydata([np.nan] * len(x))
    line3.set_ydata([np.nan] * len(x))
    line4.set_ydata([np.nan] * len(x))
    line5.set_ydata([np.nan] * len(x))
    line6.set_ydata([np.nan] * len(x))
    line7.set_ydata([np.nan] * len(x))
    line8.set_ydata([np.nan] * len(x))

    return line0, line1, line2, line3, line4, line5, line6, line7, line8

def joint_state_callback(data):
    # rospy.loginfo(data.effort)
    q0.popleft()
    try:
        q0.append(data.effort[0])
    except:
        q0.append(np.pi)
    q1.popleft()
    try:
        q1.append(data.effort[1])
    except:
        q1.append(np.pi)
    
    q2.popleft()
    try:
        q2.append(data.effort[2])
    except:
        q2.append(np.pi)

    q3.popleft()
    try:
        q3.append(data.effort[3])
    except:
        q3.append(np.pi)

    q4.popleft()
    try:
        q4.append(data.effort[4])
    except:
        q4.append(np.pi)
    
    q5.popleft()
    try:
        q5.append(data.effort[5])
    except:
        q5.append(np.pi)
    
    q6.popleft()
    try:
        q6.append(data.effort[6])
    except:
        q6.append(np.pi)
    
    q7.popleft()
    try:
        q7.append(data.effort[7])
    except:
        q7.append(np.pi)
    
    q8.popleft()
    try:
        q8.append(data.effort[8])
    except:
        q8.append(np.pi)

def animate(i):
    line0.set_ydata(q0)
    line1.set_ydata(q1)
    line2.set_ydata(q2)
    line3.set_ydata(q3)
    line4.set_ydata(q4)
    line5.set_ydata(q5)
    line6.set_ydata(q6)
    line7.set_ydata(q7)
    line8.set_ydata(q8)
    
    return line0, line1, line2, line3, line4, line5, line6, line7, line8

rospy.init_node("wx250s_current_visualizer", anonymous=False)
rospy.Subscriber("/wx250s/joint_states", JointState, joint_state_callback, queue_size=10)

fig, ax = plt.subplots(3,3)

x = np.arange(0, MAX_LEN)

ax[0,0].set_ylim(-1000, 1000)
ax[0,0].set_xlim(0, MAX_LEN-1)
ax[0,1].set_ylim(-1000, 1000)
ax[0,1].set_xlim(0, MAX_LEN-1)
ax[0,2].set_ylim(-1000, 1000)
ax[0,2].set_xlim(0, MAX_LEN-1)
ax[1,0].set_ylim(-1000, 1000)
ax[1,0].set_xlim(0, MAX_LEN-1)
ax[1,1].set_ylim(-1000, 1000)
ax[1,1].set_xlim(0, MAX_LEN-1)
ax[1,2].set_ylim(-1000, 1000)
ax[1,2].set_xlim(0, MAX_LEN-1)
ax[2,0].set_ylim(-1000, 1000)
ax[2,0].set_xlim(0, MAX_LEN-1)
ax[2,1].set_ylim(-1000, 1000)
ax[2,1].set_xlim(0, MAX_LEN-1)
ax[2,2].set_ylim(-1000, 1000)
ax[2,2].set_xlim(0, MAX_LEN-1)

line0, = ax[0,0].plot(x, np.random.randint(0, 10, MAX_LEN))
line1, = ax[0,1].plot(x, np.random.randint(0, 10, MAX_LEN))
line2, = ax[0,2].plot(x, np.random.randint(0, 10, MAX_LEN))
line3, = ax[1,0].plot(x, np.random.randint(0, 10, MAX_LEN))
line4, = ax[1,1].plot(x, np.random.randint(0, 10, MAX_LEN))
line5, = ax[1,2].plot(x, np.random.randint(0, 10, MAX_LEN))
line6, = ax[2,0].plot(x, np.random.randint(0, 10, MAX_LEN))
line7, = ax[2,1].plot(x, np.random.randint(0, 10, MAX_LEN))
line8, = ax[2,2].plot(x, np.random.randint(0, 10, MAX_LEN))

plt.xlabel('Seconds ago')

ani = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True, save_count=10)
plt.show()

while not rospy.is_shutdown():
    rospy.spin()