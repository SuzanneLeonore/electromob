import socket
import time

from rtde_receive import RTDEReceiveInterface

rtde_r = RTDEReceiveInterface("10.2.30.60")
print("DO0 :", rtde_r.getDigitalOutState(0))

robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot.connect(("10.2.30.60", 30002))

# Script complet URScript : allume DO0
script = """
def open():
  set_standard_digital_out(0, False)
end
open()
"""
robot.send(script.encode('utf8'))
time.sleep(2)
robot.close()
