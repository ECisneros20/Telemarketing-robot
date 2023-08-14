#!/usr/bin/env python3
import rospy
import xml.etree.ElementTree as ET
import numpy as np

velocidad_requerida=rospy.get_param('AutomatizacionCreacionDeMundo/speed_dynamic_obstacles')
tiempo_de_giro=rospy.get_param('AutomatizacionCreacionDeMundo/turning_time')
cantidad_de_actores=int(rospy.get_param('AutomatizacionCreacionDeMundo/number_dynamic_obstacles'))
nombre_archivo=rospy.get_param('AutomatizacionCreacionDeMundo/nombre_archivo')
tipo_movimiento=rospy.get_param('AutomatizacionCreacionDeMundo/tipo_movimiento')


tree = ET.parse('/home/zetans/ros_ws/src/Telemarketing_Robot/telemarketing_gazebo/worlds/Testing_Procedure/'+tipo_movimiento+'/'+str(cantidad_de_actores)+'_Actores/'+nombre_archivo+'.world')
root=tree.getroot()

actores=root[0]

if tipo_movimiento!="0_Aleatorio":
    for i in range(1,1+cantidad_de_actores):
        elementos_waypoints = actores[i][2][0]
        pos_inicial=elementos_waypoints[0][1].text.split(" ")
        pos_inicial_x=float(pos_inicial[0])
        pos_inicial_y=float(pos_inicial[1])

        pos_final=elementos_waypoints[1][1].text.split(" ")
        pos_final_x=float(pos_final[0])
        pos_final_y=float(pos_final[1])

        distancia=np.sqrt((pos_inicial_x-pos_final_x)**2+(pos_inicial_y-pos_final_y)**2)
        tiempo=distancia/velocidad_requerida
        sumando=tiempo_de_giro

        for waypoint in elementos_waypoints:
            time=waypoint[0]
            valor_tiempo=float(time.text)
            if valor_tiempo!=0:
                sumando=sumando*-1+tiempo+tiempo_de_giro#en cada iteracion se suma tiempo o tiempo_de_giro
                time.text=str(float(valor_tiempo_anterior)+sumando)
                print(time.text)
            valor_tiempo_anterior=time.text
else:
    for i in range(1,1+cantidad_de_actores):#en caso se busque el mundo aleatorio en el cual el codigo es distinto
        plugin = actores[i][3]
        plugin[6].text = str(velocidad_requerida)

tree.write('/home/zetans/ros_ws/src/Telemarketing_Robot/telemarketing_gazebo/worlds/Testing_Procedure/'+tipo_movimiento+'/'+str(cantidad_de_actores)+'_Actores/'+nombre_archivo+'.world')
