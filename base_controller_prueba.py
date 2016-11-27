#!/usr/bin/python

# system imports
import time
# micromaestro imports
import maestro as m
# ros specific imports
import rospy
from geometry_msgs.msg import Twist

# Crear objeto MicroMaestro
s = m.Controller()

# Asignacion de canales
servo_izq = 4
servo_der = 5

# Variables y constantes para la logica de control
AVANZANDO=1
PARADO=2
GIRANDO=3
RETROCEDIENDO=4
estado=2

# Tiempos de movimiento y pausa para estabilizacion de lectura
avance = 0.5
turning = 0.75
stopped= 0.5
min_stop_time= 0.5

# Paro los motores, en el caso de que estuvieran activos
s.setTarget(servo_izq,0)
s.setTarget(servo_der,0)

def parar():
	s.setTarget(servo_izq, 0)
	s.setTarget(servo_der, 0)
def retroceder():
	s.setTarget(servo_izq, -1)
	s.setTarget(servo_der, -1)
def avanzar():
	s.setTarget(servo_izq, 1)
	s.setTarget(servo_der, 1)
def girar(direccion):
	s.setTarget(servo_izq, 1*direccion)
	s.setTarget(servo_der, -1*direccion)

def esperar():
	time.sleep(min_stop_time)

def speed_callback(data):
	global estado
	if (data.angular.z == 0):
		if (data.linear.x > 0):
			if(estado != AVANZANDO):
				parar()
				esperar()
				avanzar()
				estado = AVANZANDO
		elif (data.linear.x < 0):
			if(estado != RETROCEDIENDO):
				parar()
				esperar()
				retroceder()
				estado = RETROCEDIENDO
		elif (data.linear.x == 0):
			parar()
			estado = PARADO

	elif (data.angular.z > 0):
		parar()
		esperar()
		girar(-1)
		estado = GIRANDO

	elif (data.angular.z < 0):
		parar()
		esperar()
		girar(1)
		estado = GIRANDO

def main():

	rospy.init_node('base_controller', anonymous=False)
	rospy.Subscriber("turtle1/cmd_vel",Twist ,speed_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
