import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind((socket.gethostname(), 2002))
serversocket.listen(5)

while True:
    clientsocket, address = serversocket.accept()
    clientsocket.send("121311231232132421AIANDAUTOMATION12131243214325124")
    

