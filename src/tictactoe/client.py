import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        message = input("Enter a command (type 'exit' to quit): ")
        if message == "exit":
            break
        s.sendall(message.encode())
        data = s.recv(1024)
        print(f"Received {data.decode()}")

print("Connection closed.")
