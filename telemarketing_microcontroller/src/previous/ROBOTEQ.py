import serial, time

serialPort = 'COM17' #cambiar

roboteq = serial.Serial(
    port=serialPort,
    baudrate=9600,
    bytesize=serial.SEVENBITS,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE)

time.sleep(2)

def ReadBattery():
    roboteq.write(b'?E\r\n')
    roboteq.read(3) #echo
    raw=roboteq.read(2)
    raw=int(raw,16)
    roboteq.close()
    roboteq.open()
    voltage=55*raw/256
    voltage=round(voltage,2)
    return voltage

def setMotors(motor,speed,clockwise=True):
    if(not(clockwise)):
        motor=motor.lower()
    speed=speed*127/100
    hex_num = hex(int(speed))
    speedHex= '{:02X}'.format(int(hex_num, 16))
    value='!'+motor+str(speedHex)
    roboteq.write(value.encode())
    roboteq.write(b'\r\n')
    roboteq.close()
    roboteq.open()

while(True):
    option=input("¿Qué deseas hacer? (M o B): ")
    if option=='M':
        while(True):
            raw_motor=input("Ingresa el motor y velocidad (A100):")
            if raw_motor=="0":
                break
            setMotors(raw_motor[0],int(raw_motor[1:]),True)
    elif option=='B':
        voltage=ReadBattery()
        print(f"El voltaje de las baterías es {voltage}v")
    else:
        print("Error.")
        #roboteq.close()
