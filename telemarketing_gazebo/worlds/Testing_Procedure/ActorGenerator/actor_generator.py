import xml.etree.ElementTree as ET
import xml.dom.minidom
import os
import math

posicion_inicial_x=1
posicion_inicial_y=-7.5
velocidad_requerida=0.6
distancia=3
direccion=90
tiempo_de_giro=0.5

#Hallando los tiempos para el script del actor
direccion_rad=direccion*math.pi/180
pos_final_x=posicion_inicial_x+distancia*math.cos(direccion_rad)
pos_final_y=posicion_inicial_y+distancia*math.sin(direccion_rad)
tiempo=distancia/velocidad_requerida
sumando=tiempo_de_giro
tiempo_actual=0
valores_tiempo=list()
for i in range(5):
    if i!=0:
        sumando=sumando*-1+tiempo+tiempo_de_giro#en cada iteracion se suma tiempo o tiempo_de_giro
        tiempo_actual=valor_tiempo_anterior+sumando
    valor_tiempo_anterior=tiempo_actual
    valores_tiempo.append(tiempo_actual)

#Hallando las posiciones del script
pos_inicial_text_f=str(posicion_inicial_x)+" "+str(posicion_inicial_y)+" 0 0 0 "+str(direccion_rad)
pos_inicial_text_r=str(posicion_inicial_x)+" "+str(posicion_inicial_y)+" 0 0 0 "+str(direccion_rad+math.pi)

pos_final_text_f=str(pos_final_x)+" "+str(pos_final_y)+" 0 0 0 "+str(direccion_rad)
pos_final_text_r=str(pos_final_x)+" "+str(pos_final_y)+" 0 0 0 "+str(direccion_rad+math.pi)
posiciones=[pos_inicial_text_f,pos_final_text_f,pos_final_text_r,pos_inicial_text_r,pos_inicial_text_f]

print(valores_tiempo)
print(posiciones)

print(os.getcwd())
path='/home/zetans/ros_ws/src/Telemarketing_Robot/telemarketing_gazebo/worlds/Testing_Procedure/ActorGenerator/ActorGenerado.world'
file = ET.Element("ActorCreado")

actor = ET.SubElement(file,'actor')
actor.attrib["name"] = "actor0"

skin = ET.SubElement(actor, 'skin')
filename =ET.SubElement(skin, "filename")
filename.text = "walk.dae"

animation = ET.SubElement(actor, 'animation')
animation.attrib["name"] = "walking"
filename =ET.SubElement(animation, "filename")
filename.text = "walk.dae"
interpolate_x =ET.SubElement(animation, "interpolate_x")
interpolate_x.text = "true"

script = ET.SubElement(actor, 'script')
trayectory =ET.SubElement(script, "trajectory")
trayectory.attrib["id"] = "0"
trayectory.attrib["type"] = "walking"

#waypoints
waypoints=list()
for i in range(5):
    waypoints.append(ET.SubElement(trayectory, "waypoint"))
    time =ET.SubElement(waypoints[i], "time")
    time.text = str(valores_tiempo[i])
    pose =ET.SubElement(waypoints[i], "pose")
    pose.text = posiciones[i]

#Creating the file
if os.path.exists(path):
  os.remove(path)
xmlstr=xml.dom.minidom.parseString(ET.tostring(file)).toprettyxml()
print(xmlstr,file=open(path,'w'))
