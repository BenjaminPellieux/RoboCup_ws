import matplotlib.pyplot as plt

import numpy as np

from matplotlib.widgets import Button, Slider
from std_msgs.msg import String,Int32,Int32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension

import rospy
from std_msgs.msg import String


pub = rospy.Publisher('chatter', String, queue_size=10)
pub_2 = rospy.Publisher('data', Float64MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

fig, ax = plt.subplots()


ax_Tx = fig.add_axes([0.25, 0.9, 0.65, 0.03])
ax_Ty = fig.add_axes([0.25, 0.8, 0.65, 0.03])
ax_Tz = fig.add_axes([0.25, 0.7, 0.65, 0.03])
ax_Rx = fig.add_axes([0.25, 0.6, 0.65, 0.03])
ax_Ry = fig.add_axes([0.25, 0.5, 0.65, 0.03])
ax_Rz = fig.add_axes([0.25, 0.4, 0.65, 0.03])

Tx_slider = Slider(
    
    ax=ax_Tx,

    label='Translation selon X',
    valmin=-1,
    valmax=1,
    valinit=0,
)
Ty_slider = Slider(
    
    ax=ax_Ty,

    label='Translation selon Y',
    valmin=0,
    valmax=2,
    valinit=0.75,
)

Tz_slider = Slider(
    
    ax=ax_Tz,

    label='Translation selon Z',
    valmin=-1,
    valmax=1,
    valinit=0,
)
Rx_slider = Slider(
    
    ax=ax_Rx,

    label='Rotation selon X',
    valmin=-180,
    valmax=180,
    valinit=90,
)
Ry_slider = Slider(
    
    ax=ax_Ry,

    label='Rotation selon Y',
    valmin=-180,
    valmax=180,
    valinit=0,
)
Rz_slider = Slider(
    
    ax=ax_Rz,

    label='Rotation selon Z',
    valmin=-180,
    valmax=180,
    valinit=0,
)
fig.subplots_adjust(left=0.85,bottom=0.85);


def update(val):
   
   Tx=Tx_slider.val
   Ty=Ty_slider.val
   Tz=Tz_slider.val
   Rx=Rx_slider.val
   Ry=Ry_slider.val
   Rz=Rz_slider.val
   print(Tx,Ty,Tz,Rx,Ry,Rz)
   

#print (Tx)

Tx_slider.on_changed(update)
Ty_slider.on_changed(update)
Tz_slider.on_changed(update)
Rx_slider.on_changed(update)
Ry_slider.on_changed(update)
Rz_slider.on_changed(update)

while not rospy.is_shutdown():
        
        hello_str = "hello world %s" % Tx_slider.val
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        coord=Float64MultiArray()
        coord.data =[Tx_slider.val,Ty_slider.val,Tz_slider.val,Rx_slider.val,Ry_slider.val,Rz_slider.val]
        pub_2.publish(coord)
        rate.sleep()
        print("On est dans la boucle ou on est sensé publier")
        plt.pause(0.05)
        


plt.draw()
