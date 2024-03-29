
def minimax(depth, nodeIndex, maximizingPlayer,
            values, alpha, beta):
    # Terminating condition. i.e
    # leaf node is reached
    if depth == 3:
        return values[nodeIndex]
 
    if maximizingPlayer:      
        best = MIN
        # Recur for left and right children
        for i in range(0, 2):         
            val = minimax(depth + 1, nodeIndex * 2 + i,
                          False, values, alpha, beta)
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
        for i in range(0, 2):
            val = minimax(depth + 1, nodeIndex * 2 + i,
                            True, values, alpha, beta)
            best = min(best, val)
            beta = min(beta, best)
 
            # Alpha Beta Pruning
            if beta <= alpha:
                break
        return best
		
class State:
	board = [[0,0,0],[0,0,0],[0,0,0]]
	turn = "X"

class TicTacToe:
	empty = [[1,1], [1,2], [1,3], [2,1], [2,2], [2,3], [3,1], [3,2], [3,3]]
	mode = "dumbass" #"bigbro" or "dumbass"
	piece = 0	#X goes first
	
	def init(_mode, _piece):
		if (_mode != "dumbass" || _mode != "bigbro"):
			return -1;
		else:
			mode = _mode
		
		if (_piece != "O" || _piece != "X"):
			return -1;
		else:
			piece = _piece

	def addpiece(row, col, piece):
		if board[row][col] != 0:
			board[row][col] = piece
		else:
			return -1;
	
	def clearboard():
		board = [[0,0,0],[0,0,0],[0,0,0]]
		empty = [[1,1], [1,2], [1,3], [2,1], [2,2], [2,3], [3,1], [3,2], [3,3]]
	
	def random_move():
		return empty[random.rand_int(0, empty.size()-1)]
	
	def nextmove():
		if mode == "dumbass":
			random_move()
		else mode == "bigbro":
			minimax(depth = 3, maximizingPlayer = True)
	
	def check(direction, slot = -1):
		win = true
		if (directon == "row"):
			for i in range(0,3):
				win = (board[slot][i] != 0)
		elif (direction == "col"):
			for i in range(0,3):
				win = (board[i][slot] != 0)
		elif (direction == "diag"):
			for i in range(0,3):
				win = (board[i][i] != 0)
		elif (direction == "antidiag"):
			for i in range(0,3):
				win = (board[2-i][i] != 0)
		return win
	
	def iswinningmove(row, col):
		win = check("row", row) || check("col", col)
		if (row == col) || (row + col == 2):
			win = win || check("diag") || check("antidiag")
		return win
			
	#corner, middle, or center
