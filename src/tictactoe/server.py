import socket
from basicTicTacToe import TicTacToe

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    game = TicTacToe()
    with conn:
        print(f"Connected by {addr}")
        while True:
            return_data = ""
            data = conn.recv(1024)
            if not data:
                break
            if data[0] == "mode":
                game.setMode(data[1])
            if data[0] == "piece":
                game.setPiece(data[1])
            if data[0] == "player":
                game.playerMove(data[1]/3, data[1]%3)
                if (game.checkWin()):
                    return_data = "Player wins"
                    for i in range(3):
                        print(game.board[i])
                if (game.checkTie()):
                    return_data = "Tie"
                    for i in range(3):
                        print(game.board[i])
                return_data += "\nGame Over"
            if data[0] == "puppet":
                game.computerMove(spot = data[1])
                if (game.checkWin()):
                    return_data = "Computer wins"
                    for i in range(3):
                        print(game.board[i])
                if (game.checkTie()):
                    return_data = "Tie"
                    for i in range(3):
                        print(game.board[i])
                return_data += "\nGame Over"
            
            conn.sendall(return_data)
