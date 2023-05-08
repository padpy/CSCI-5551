import random

# Constants
MIN = -1000
MAX = 1000

def generate_values(board, piece):
    values = []
    for i in range(9):
        if board[i//3][i%3] == " ":
            board[i//3][i%3] = piece
            score = heuristic(board, piece)
            values.append(score)
            board[i//3][i%3] = " "
        else:
            values.append(None)
    return values

def heuristic(board, piece):
    rows = [set(row) for row in board]
    cols = [set(col) for col in zip(*board)]
    diag1 = set(board[i][i] for i in range(3))
    diag2 = set(board[i][2-i] for i in range(3))
    num_wins = sum(1 for line in rows + cols + [diag1, diag2] if n in line and len(line) > 1)
    return num_wins

def minimax(depth, nodeIndex, maximizingPlayer, values, alpha, beta):
    # Terminating condition. i.e
    # leaf node is reached
    if depth == 3:
        score = heuristic(board, piece)
        return score

    values = generate_values(board, piece)
    if maximizingPlayer:
        best = MIN
        # Recur for left and right children
        for i in range(len(values)):
            val = minimax(depth + 1, nodeIndex * 2 + i, False, values, alpha, beta)
            best = max(best, val)
            alpha = max(alpha, best)
            # Alpha Beta Pruning
            if beta <= alpha:
                break

        return best
    else:
        best = MAX
        # Recur for left and
        # right children
        for i in range(len(values)):
            val = minimax(depth + 1, nodeIndex * 2 + i, True, values, alpha, beta)
            best = min(best, val)
            beta = min(beta, best)

            # Alpha Beta Pruning
            if beta <= alpha:
                break
        return best

class TicTacToe:
    board = [[0,0,0],[0,0,0],[0,0,0]] #game board
    empty = [[1,1], [1,2], [1,3], [2,1], [2,2], [2,3], [3,1], [3,2], [3,3]] #list of empty spaces
    mode = "dumb" #"bigbro" or "dumb" or "puppet"
    piece = "X"	#computer player
    piece_oppo = "O" #human player
    
    def __init__(self,_mode, _piece):
        self.setMode(_mode)
        
        self.setPiece(_piece)
            
    def setMode(self, _mode):
        if (_mode != "dumb" and _mode != "bigbro" and _mode != "puppet"):
            return -1;
        else:
            self.mode = _mode
            
    def setPiece(self, _piece):
        if (_piece != "O" and _piece != "X"):
            return -1;
        else:
            self.piece = _piece
            self.piece_oppo = "O" if _piece == "X" else "X"
            
    def playerMove(self, row, col):
        location = [row, col]
        if location in self.empty:
            self.empty.remove(location)
            self.board[row-1][col-1] = self.piece_oppo
        else:
            print("Invalid")
            
    def computerMove(self, spot = 0):
        if self.mode == "dumb":
            randLocation = random.choice(self.empty) #get random empty space
            print(randLocation)
            self.empty.remove(randLocation) #remove from list of available spaces
            self.board[randLocation[0]-1][randLocation[1]-1] = self.piece # -1 to convert to board form 
            return randLocation  #this is what we 1will send to the robot for its arm to move to
        
        if self.mode == "bigbro":
            return minimax(depth + 1, nodeIndex * 2 + i, False, values, alpha, beta)
            
        if self.mode == "puppet":
            self.board(spot) = self.piece

    def checkWin(self, piece):
        board = self.board
        for i in range(3):
            if board[i][0] == board[i][1] == board[i][2] == piece:
                return True
            if board[0][i] == board[1][i] == board[2][i] == piece:
                return True
        if board[0][0] == board[1][1] == board[2][2] == piece:
            return True
        if board[0][2] == board[1][1] == board[2][0] == piece:
            return True
        
    def checkTie(self):
        if (self.empty == []):
            return True

def main():
    game = TicTacToe("dumb", "X")
    while True: #game loop        
        print("Current board")
        for i in range(3):
            print(game.board[i])
        if (game.piece == "X"):
            print("Your turn")
            row = int(input("Enter row "))
            col = int(input("Enter column "))
            game.playerMove(row, col)
            if (game.checkWin()):
                print("Player wins")
                for i in range(3):
                    print(game.board[i])
                print("Game Over")
                break
            if (game.checkTie()):
                print("Tie")
                for i in range(3):
                    print(game.board[i])
                print("Game Over")
                break
        else:                               #if game piece is O
            print("Computer's turn")
            game.computerMove(game.piece)
            if (game.checkWin() or game.checkTie()):
                print("Computer wins")
                for i in range(3):
                    print(game.board[i])
                print("Game Over")
                break
            if (game.checkTie()):
                print("Tie")
                for i in range(3):
                    print(game.board[i])
                print("Game Over")
                break
if __name__ == '__main__':
    main()
