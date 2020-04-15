#! /usr/bin/env python
# ------------------------------------------------
# importacion de las librerias necesarias para utilizar python y los mensajes y servicios de ROS

import rospy
from geometry_msgs.msg import Twist  # Mensajes de geometria para la velocidad
from turtlesim.msg import Pose  # Mensajes de pose para la posicion
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
theta = 0


# ------------------------------------------------
# se hace el callback a raiz de haber obtenido la pose, es decir, se asignan las variables x, y y theta con los datos de posicion actual recibidos desde ROS
def poseCallback(pose_message):
    global x
    global y
    global z
    global theta

    x = round(pose_message.x, 4)
    y = round(pose_message.y, 4)
    theta = pose_message.theta


# ------------------------------------------------
# funcion go to goal con las coordenadas de la meta argumentos y el topico de comando de velocidad (del tipo twist) como salida
def go_to_goal(xgoal, ygoal):
    global x
    global y
    global theta
    # bandera =1500
    bandera2=0
    pi = 3.141592

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    # *************CODIGO NUEVO INICIA***********************
    #Hacemos que la tortuga gire al angulo deseado antes de que comience a moverse, tiene 3 segundos para girar al angulo deseado
    desired_angle_goal = math.atan2(ygoal - y, xgoal - x)
    velocity_message.linear.x = 0
    velocity_message.angular.z = desired_angle_goal - theta
    velocity_publisher.publish(velocity_message)
    time.sleep(3)
    # *************CODIGO NUEVO TERMINA***********************

    # ------------------------------------------------
    # calculo de las velocidades lineal y angular (escaladas por factores kv y ka) a partir de la trigonometria entre las posiciones actual y deseada
    while (True):
        kv = 0.5
        distance = abs(math.sqrt(((xgoal - x) ** 2) + ((ygoal - y) ** 2)))
        linear_speed = kv * distance  # Me dice que tan lejos estoy, entre mas lejos mas rapido

        ka = 6
        desired_angle_goal = math.atan2(ygoal - y, xgoal - x)  # angulo que quiere
        dtheta = desired_angle_goal - theta  # angulo que quiero - angulo que tengo
        angular_speed = ka * (dtheta)  # Me dice que tanto me hace falta girar

        # ------------------------------------------------
        # # escritura de las velocidades calculadas en el topico que se va a publicar
        # if (bandera2 == 0):
        #     bandera2 = 1
        #     print('theta', theta, 'dtheta', dtheta)
        #     time.sleep(3)

        # Publico los valores encontrados en consola, osea se imprimen x y z
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print('x=', x, 'y=', y, 'theta', theta, 'dtheta', dtheta)

        if (distance < 0.1):  # Si la tortuga esta a 0.01 que se detenga.
            break


# ------------------------------------------------

if __name__ == '__main__':
    try:

        rospy.init_node('turtlebot_controller', anonymous=True)

        # ------------------------------------------------
        # publicacion del topico de velocidad (cmd_vel)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # ------------------------------------------------
        # suscripcion al topico de posicion (Pose)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        # ------------------------------------------------
        # llamado de la funcion principal

        #*************CODIGO NUEVO INICIA***********************
        #Cada punto reprenta una esquina del cuadrado
        go_to_goal(1.0, 1.0)
        # print('salio')
        # time.sleep(3)
        go_to_goal(9.0, 1.0)
        # print('salio')
        # time.sleep(3)
        go_to_goal(9.0, 9.0)
        # print('salio')
        # time.sleep(3)
        go_to_goal(1.0, 9.0)
        # print('salio')
        # time.sleep(3)
        go_to_goal(1.0, 1.0)
        # time.sleep(3)

        #*************CODIGO NUEVO TERMINA***********************


        # setDesiredOrientation(math.radians(90))
    # ------------------------------------------------
    except rospy.ROSInterruptException:
        pass