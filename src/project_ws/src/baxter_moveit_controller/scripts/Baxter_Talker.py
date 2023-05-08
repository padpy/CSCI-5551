#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def checkWin(board):
    for i in range(1, 3):  # 1 is computer and 2 is human
        for j in range(3):
            if board[j][0] == board[j][1] == board[j][2] == i:  #checks all rows and columns for a 1 or 2 win
                return True, i
            if board[0][j] == board[1][j] == board[2][j] == i:
                return True, i
        if board[0][0] == board[1][1] == board[2][2] == i:   #checks both diagonals for a 1 or 2 win
            return True, i
        if board[0][2] == board[1][1] == board[2][0] == i:
            return True, i
    return False, 0

def checkTie(board):
    for i in range(3):  #check if board is full
        for j in range(3):
            if board[i][j] == 0:
                return False
    return True

def miniMax(board, score, isMaxing):
    #https://www.geeksforgeeks.org/minimax-algorithm-in-game-theory-set-4-alpha-beta-pruning/
    if checkTie(board):
        return [-1, -1], 0     #if tie dont add or subtract from score
    if checkWin(board)[0]:
        player = checkWin(board)[1]
        if player == 1:    #if computer wins
            return [-1, -1], 1 
        else:              #if human wins
            return [-1, -1], -1
    bestScore = 0
    bestMove = [-1, -1]  #should never return this as it was already checked if game was won or lost would be good to add a condition to check if this is returned
    if isMaxing:
        for i in range(3):
            for j in range(3):
                if board[i][j] == 0:        # checks next possible move for computer
                    board[i][j] = 1
                    score = miniMax(board, score, False)[1]  
                    board[i][j] = 0  
                    if score > bestScore:     # if better than last computer score then make that move
                        bestScore = score
                        bestMove = [i,j]
        return bestMove, bestScore
    else:             #minimizing compareing human score to computer score
        for i in range(3):
            for j in range(3):
                if board[i][j] == 0:    # checks next possible move for human
                    board[i][j] = 2
                    score = miniMax(board, score, True)[1] 
                    board[i][j] = 0  
                    if score < bestScore:       # if human score beats the computer score then the computer should make that move instead
                        bestScore = score
                        bestMove = [i,j]
        return bestMove, bestScore

def convertCoordinateToLocation(coordinate):
    for i in range(3):
        for j in range(3):
            if (coordinate == [i,j]):
                return i*3+j+1             #returns 1 - 9 for location related to board


def getNextMove(board):
    score = 0
    if (checkWin(board)[0] or checkTie(board)):
        return 0 #send message to baxter to stop it does not matter who wins baxter just needs to stop
    next_move = miniMax(board, score, True) #returns best move for current board
    location = convertCoordinateToLocation(next_move[0]) #converts coordinate to location
    return location #send message to baxter to move to locatio

def getNextMoveRandom(board):
    empty = []
    for i in range(3):
        for j in range(3):
            if board[i][j] == 0:
                empty.push((i*3)+j+1)
    randLocation = random.choice(self.empty) #get random empty space
    return randLocation  #this is what we 1will send to the robot for its arm to move to

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    count = 0
    rospy.init_node('tictactoe', anonymous = True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        board = [[1,1,2],[1,0,0],[1,2,1]] #temp board for testing this will be replaced with vision function call to get board
        next_move = getNextMove(board)
        rospy.loginfo(next_move) 
        pub.publish(next_move)  #0 stop baxter, 1-9 move to location
        rate.sleep()
        count = count + 1
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
