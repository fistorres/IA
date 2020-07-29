import random
from PacInv import *

def generateboard(O,P,X,Y,Pos):
    Obs = []
    Gums = []
    while len(Obs) < O:
        o_x = random.randrange(1,X+1)
        o_y = random.randrange(1,Y+1)
        if (o_x,o_y) not in Obs:
            Obs.append((o_x,o_y))

    while len(Gums) < P:
        p_x = random.randrange(1,X+1)
        p_y = random.randrange(1,Y+1)
        if (p_x,p_y) not in Obs \
           and (p_x,p_y) not in Gums:
            Gums.append((p_x,p_y))
            

    return ProblemaPacInv(X,Y,Gums,Obs,Pos)



def h1(problem,node):
    state = node.state
    heu = problem.h1(state)
    return heu

