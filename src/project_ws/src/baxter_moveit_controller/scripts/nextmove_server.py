
import rospy
from srv import t3, t3Response
from . import NextMove

def nextmove(req):
    if req.mode == "minimax":
        return t3Response(NextMove.GetNextMove(req.board))
    else:
        return t3Response(NextMove.GetNextMoveRandom(req.board))

def nextmove_server():
    rospy.init_node('t3_server')
    s = rospy.Service('get_next_move', t3, nextmove)
    rospy.spin()
    
if __name__ == "__main__":
    nextmove_server()