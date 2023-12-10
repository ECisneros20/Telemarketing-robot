import serial, time

class roboteq:

    def __init__(self, serialPort = "COM17"):
        self.voltage = 0
        self.roboteq = serial.Serial(
            port = serialPort,
            baudrate = 9600,
            bytesize = serial.SEVENBITS,
            parity = serial.PARITY_EVEN,
            stopbits = serial.STOPBITS_ONE)
        time.sleep(2)

    def ReadBattery(self):
        self.roboteq.write(b"?E\r\n")
        self.roboteq.read(3) #echo
        raw = self.roboteq.read(2)
        raw = int(raw,16)
        self.roboteq.close()
        self.roboteq.open()
        self.voltage = 55 * raw / 256
        self.voltage = round(self.voltage, 2)
        return self.voltage

    def setMotors(self, motor, speed, clockwise = True):
        if (not(clockwise)):
            motor = motor.lower()
        speed = speed * 127 / 100
        hex_num = hex(int(speed))
        speedHex = "{:02X}".format(int(hex_num, 16))
        value = "!" + motor + str(speedHex)
        self.roboteq.write(value.encode())
        self.roboteq.write(b"\r\n")
        self.roboteq.close()
        self.roboteq.open()

while (True):
    driver = roboteq(serialPort = "COM17")
    option = input("¿Qué deseas hacer? (M o B): ")
    if option == "M":
        while (True):
            raw_motor = input("Ingresa el motor y velocidad (A100):")
            if raw_motor == "0":
                break
            driver.setMotors(raw_motor[0], int(raw_motor[1:]), True)
    elif option == "B":
        voltage = driver.ReadBattery()
        print(f"El voltaje de las baterías es {voltage}v")
    else:
        print("Error.")
