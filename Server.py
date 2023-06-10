import socket
import time
import json
import serial

def main():
    # Create axis TCP/IP socket-----------------------------------------------------------------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port---------------------------------------------------------------------------------------
    server_address = ('192.168.1.9', 10000)
    print('starting up on {} port {}'.format(*server_address))
    sock.bind(server_address)

    # Listen for incoming connections-----------------------------------------------------------------------------------
    sock.listen(1)
    serial2=serial.Serial('COM4',115200)
    connection, client_address = sock.accept()
    while True:
        # Wait for axis connection
        
        
        data= connection.recv(5000).decode()
        if not connection.fileno():
                break
        if not data:
            break
            # Receber os dados completosunsupported operand type(s) for +=: 'dict' and 'str'
        jsondata = json.loads(data)
        JLX=jsondata["JLX"]
        JLY=jsondata["JLY"]
        JRX=jsondata["JRX"]
        JZ=jsondata["JZ"]
        A=jsondata["A"]
       
        #print(JLX,JLY,JRX,JZ,A)
        commands=f"{JLX},{JLY},{JRX},{JZ},{A}\n"
        serial2.write(commands.encode())
        time.sleep(0.1)    
        if serial2.in_waiting > 0:
            # Ler a mensagem da porta serial
            Altura = serial2.readline().decode().rstrip()
        
               # Exibir a mensagem
        connection.send(Altura.encode())

    connection.close()
        
if __name__ == "__main__":
    main()
