#!/usr/bin/env python3
import rospy
from NextMove import getNextMove
# from std_msg.msg import SOMEARRAYCLASS
from std_msgs.msg import (
    UInt16,
    UInt16MultiArray
)

class MoveServer:
    def __init__(self) -> None:
        rospy.init_node("next_move_server")
        self.pub = rospy.Publisher('/tictactoe/move', UInt16)
        rospy.Subsciber('/tictactoe/board', UInt16, self.board_state_callback)

    def next_move(self, board):
        mov_val = getNextMove(board)
        return mov_val

    def board_state_callback(self, msg):
        board = msg.data 

        if len(board) != 9:
            return
        move_msg = UInt16()
        move_msg.data = getNextMove([board[0:3], board[3:6], board[6:9]])
        self.pub(move_msg)
        rospy.loginfo(f'Told Baxter to play {move_msg.data}')

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    move_server = MoveServer()
    move_server.spin() 