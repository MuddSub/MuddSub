import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = socket.gethostname()

port = 12345

server_socket.bind((host, port))

server_socket.listen(5)

print(f"Listening on port {port}...")

while True:
    #Establish a connection
    client_socket, addr = server_socket.accept()
    print(f"Got a connection from {addr}")
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        print("Received:", data)
    client_socket.close()
    print("Connection with client closed")

