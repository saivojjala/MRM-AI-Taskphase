import socket 

clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
clientsocket.connect((socket.gethostname(), 2002)) 

from_server = clientsocket.recv(1024)
print(from_server)

