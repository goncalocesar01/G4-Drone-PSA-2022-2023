import cv2
import pygame
import time
# ---------------------------------------------------------
# Initialization
# ---------------------------------------------------------
def main(): 
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
        JLX = joystick.get_axis(0)
        JLY = -joystick.get_axis(1)
        JRX = -joystick.get_axis(3)
        JRY = joystick.get_axis(2)
        JZ = joystick.get_axis(5)-joystick.get_axis(4)
        A = joystick.get_button(0)

        pygame.event.pump()
        
        print('JLX ' + str(JLX) + '; JLY ' + str(JLY) + '; JRX ' + str(JRX) + '; JRY ' + str(JRY) + '; JZ' + str(JZ) + '; A ' + str(A))

        # Message Frequency
        time.sleep(0.1)

    # ---------------------------------------------------------
    # Termination
    # ---------------------------------------------------------

    pygame.quit()
if __name__ == "__main__":
    main()