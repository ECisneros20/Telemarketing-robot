import time
import serial
from time import sleep
from omxplayer.player import OMXPlayer
from pathlib import Path

#Variables
isAudioPlayed = False
next="0"
playing = "0"
#Videos con audio
v_w_audio=[]
v_positions = {
    "0": (0,6),  #Feliz animacion
    "1": (10,17),  #Triste animacion
    "2": (20,25),  #Feliz 1
    "3": (30,35),  #Feliz 2
    "4": (40,45),  #Idea 1
    "5": (50,55),  #Idea 2
    "6": (60,65),  #Idea 3
    "7": (70,75),  #Triste 1
    "8": (80,85),  #Triste 2
    "9": (90,95),  #Triste 3
    "a": (100,105),  #Triste 4
}

#Videos
video_path = Path("/home/pi/Telemarketing-robot/telemarketing_raspberry/Videos/Video_Telemarketing_v1/Video_Telemarketing_v1.mp4")
player = OMXPlayer(video_path, args = ['--display=7','--orientation=0','--loop'],dbus_name='org.mpris.MediaPlayer2.omxplayer1')

#Control the displaying video
def video_handler(command):
    global playing, player, isAudioPlayed, next
    if len(command) == 1 : 
        #Select next video
        if isAudioPlayed == True :
           next=="0"
        else:
            next=command  
        print(next)
        player.set_position(v_positions[next][0])
        playing = next
        if next in v_w_audio:
            isAudioPlayed=True
        else:
            isAudioPlayed=False
        print(isAudioPlayed)           

serial_port = serial.Serial(
port="/dev/ttyS0",
baudrate=115200,
bytesize=serial.EIGHTBITS,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
timeout=0,)

sleep(1)
print("Interfaz de pantallas")

while True:  
    #UART
    if serial_port.inWaiting() > 0:
        data = serial_port.read(1)
        data=data.decode() 
        print(data)
        video_handler(data)
    #Video Loop
    position = player.position()
    print(position)
    if (position> v_positions[playing][1]+0.3 or position< v_positions[playing][0]-0.3):
        video_handler(playing)