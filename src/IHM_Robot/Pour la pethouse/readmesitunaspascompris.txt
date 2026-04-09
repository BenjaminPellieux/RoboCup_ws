Bonjour !

donc si tu veux faire fonctionner mon html
tu dois faire 
roscore (pour juste les codes arduino)
ou 
roslaunch cv_camera cv_camera.launch (qui possede déjà un roscore)(camera1)
et
roslaunch cv_camera2 cv_camera.launch (camera 2)
Téleverse ton code arduino sur l'arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0 

APRES

Dans un autre terminal : 

roslaunch rosbridge_server rosbridge_websocket.launch

et sur un navigateur tu tape http://127.0.0.1//camera_2.html

cela peut arriver que tu n'aies pas le même adresse ip, dans ces cas la faudra que tu regardes tes adresses ip

le fichier camera_2 et le favicon.ico il faut les mettre dans / var/ www / html

