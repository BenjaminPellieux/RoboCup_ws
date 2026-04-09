import matplotlib.pyplot as plt

import numpy as np

from matplotlib.widgets import Button, Slider
from std_msgs.msg import String,Int32,Int32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension

import rospy
from std_msgs.msg import String

Rb = 0

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
ax_Gr = fig.add_axes([0.25, 0.3, 0.65, 0.03])





# Créez l'axe pour le bouton
ax_button = plt.axes([0.25, 0.2, 0.65, 0.03])
button = Button(ax_button, 'Reboot')
Tx_slider = Slider(
    
    ax=ax_Tx,

    label='axe1',
    valmin=0,
    valmax=1,
    valinit=0.5,
)
Ty_slider = Slider(
    
    ax=ax_Ty,

    label='axe2',
    valmin=0,
    valmax=1,
    valinit=0.5,
)

Tz_slider = Slider(
    
    ax=ax_Tz,

    label='axe3',
    valmin=0,
    valmax=1,
    valinit=0.5,
)
Rx_slider = Slider(
    
    ax=ax_Rx,

    label='axe4',
    valmin=0,
    valmax=1,
    valinit=0.75,
)
Ry_slider = Slider(
    
    ax=ax_Ry,

    label='axe5',
    valmin=0,
    valmax=1,
    valinit=0.5,
)
Rz_slider = Slider(
    
    ax=ax_Rz,

    label='axe6',
    valmin=0,
    valmax=1,
    valinit=0.5,
)
Gr_slider = Slider(
    
    ax=ax_Gr,

    label='Pince',
    valmin=0.45,
    valmax=1,
    valinit=0.5,
)

# Fonction de mise à jour pour vérifier la valeur de reboot_status
def check_reboot_status():
    global reboot_status
    while True:
        if reboot_status == 1:
            print("Reboot status has been updated to 1")
            break
        time.sleep(0.1)  # Pause pour éviter une boucle trop rapide


def update(val):
   
   Tx=Tx_slider.val
   Ty=Ty_slider.val
   Tz=Tz_slider.val
   Rx=Rx_slider.val
   Ry=Ry_slider.val
   Rz=Rz_slider.val
   Gr=Gr_slider.val
   print(Tx,Ty,Tz,Rx,Ry,Rz,Gr)
   
def reboot_callback(event):
   global Rb
   Rb = 1




   



Tx_slider.on_changed(update)
Ty_slider.on_changed(update)
Tz_slider.on_changed(update)
Rx_slider.on_changed(update)
Ry_slider.on_changed(update)
Rz_slider.on_changed(update)
Gr_slider.on_changed(update)



button.on_clicked(reboot_callback)
while not rospy.is_shutdown():
        
        hello_str = "hello world %s" % Tx_slider.val
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        coord=Float64MultiArray()
        
        if Rb == 1:
            # Publier sur un topic ROS ou effectuer d'autres actions
            print("Reboot initiated")
            coord.data =[Tx_slider.val,Ty_slider.val,Tz_slider.val,Rx_slider.val,Ry_slider.val,Rz_slider.val,Gr_slider.val,Rb]
            pub_2.publish(coord)
            rate.sleep()
            print("On est dans la boucle ou on est sensé publier")
            print (Rb)
            plt.pause(1)
        
            # Réinitialiser Rb si nécessaire
            Rb = 0
        
            
        coord.data =[Tx_slider.val,Ty_slider.val,Tz_slider.val,Rx_slider.val,Ry_slider.val,Rz_slider.val,Gr_slider.val,Rb]
        pub_2.publish(coord)
        rate.sleep()
        print("On est dans la boucle ou on est sensé publier")
        print (Rb)
        plt.pause(0.05)
        


plt.draw()
