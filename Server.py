import socket
import time
import json
import serial

def main():
    # Create axis TCP/IP socket-----------------------------------------------------------------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port---------------------------------------------------------------------------------------
    server_address = ('192.168.0.11', 10000)
    print('starting up on {} port {}'.format(*server_address))
    sock.bind(server_address)

    # Listen for incoming connections-----------------------------------------------------------------------------------
    sock.listen(1)
    serial2=serial.Serial('COM4',115200)
    while True:
        # Wait for axis connection
        connection, client_address = sock.accept()
        while client_address != "":
            # Receber o tamanho dos dados
            data_length = int(connection.recv(1024).decode())
            # Receber os dados completos
            data = connection.recv(data_length).decode()       
            data = connection.recv(5000)
            data = json.loads(data.decode())
            JLX=data.get("JLX")
            JLY=data.get("JLX")
            JRX=data.get("JLX")
            JZ=data.get("JLX")
            A=data.get("JLX")
            time.sleep(0.0001)
            #print(JLX,JLY,JRX,JZ,A)
            commands=f"{JLX},{JLY},{JRX},{JZ},{A}\n"
            serial2.write(commands.encode())
            
            if serial2.in_waiting > 0:
            # Ler a mensagem da porta serial
                Altura = serial2.readline().decode().rstrip()
            # Exibir a mensagem
                sock.send(Altura.encode())
            if not connection.fileno():
                break
        
if __name__ == "__main__":
    main()
