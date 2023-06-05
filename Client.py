import socket
import cv2
import pygame
import time
import json
# ---------------------------------------------------------
# Initialization
# ---------------------------------------------------------
def main(): 
    # Create a TCP/IP socket------------------------------------------------------------------------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening----------------------------------------------------------
    server_address = ('192.168.0.11', 10000)
    print('connecting to {} port {}'.format(*server_address))
    sock.connect(server_address)
    pygame.init()
    pygame.joystick.init()
    # Count Joysticks
    joystick_count = pygame.joystick.get_count()
    print('Found ' + str(joystick_count) + ' joysticks.')
    if joystick_count < 1:
        print('No joysticks found. Terminating.')
        exit(0)

    joystick = pygame.joystick.Joystick(0)
    joystick_name = joystick.get_name()

    print('Connected to joystick named ' + joystick_name)
    # ---------------------------------------------------------
    # Execution in Cycle
    # ---------------------------------------------------------

    while True:
        JLX = int(joystick.get_axis(0)*255)
        JLY = -int(joystick.get_axis(1)*255)
        JRX = int(joystick.get_axis(2)*255)
        JZ = int((joystick.get_axis(5)-joystick.get_axis(4))/2*255)
        A = joystick.get_button(0)
        pygame.event.pump()
        
        message=json.dumps({"JLX":JLX,"JLY":JLY,"JRX":JRX,"JZ":JZ,"A":A})+'\n'
        #print(message)
        sock.send(message.encode())
        time.sleep(0.001)
        data=sock.recv(1024)
        json_data=json.loads(data.decode())
        Altura=json_data.get("Altura")
        
        print("altura =" + str(Altura) + "m")
        
    # ---------------------------------------------------------
    # Termination
    # ---------------------------------------------------------
if __name__ == "__main__":
    main()