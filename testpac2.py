#########PROJECT TESTER
from search2_2 import *
from PacInv import *
from copy import deepcopy
from generator import h1

#########TESTING PROBLEM AND STATES
##w = ProblemaPacInv(4,4,[(1,3),(2,3),(3,3),(3,2),(4,2)],[],(1,1))
w = ProblemaPacInv(8,6,[(8,4),(8,5),(8,6),(7,6),(6,6)],[],(3,4))
##w2 = deepcopy(w)
##p2 = w2.initial_state
##print('Touched gums: ', p2.touched_gum)
##print('Discarded positions: ', p2.disc_pos)
##print('Hash is: ', hash(p2))
##print('Position is: ', p2.pos)
##
##w2.display(p2)
##
##p3 = w2.result(p2,(1,1))
##print('Touched gums: ', p3.touched_gum)
##print('Discarded positions: ', p3.disc_pos)
##print('Path cost is: ', w2.path_cost(0,p2,(1,1),p3))
##print('Hash is: ', hash(p3))
##print('Position is: ', p3.pos)
##
##w2.display(p3)
##
##p4 = w2.result(p3,(1,1))
##print('Touched gums: ', p4.touched_gum)
##print('Discarded positions: ', p4.disc_pos)
##print('Path cost is: ', w2.path_cost(0,p3,(1,1),p4))
##print('Hash is: ', hash(p4))
##print('Position is: ', p4.pos)
##
##w2.display(p4)

#########APPLYING SEARCH TO PROBLEM 
print("RUNNING SEARCH ALG!")

###depth is working
l = depth_first_graph_search(w)
##breadth is working
##l = breadth_first_search(w)

## iterative deepening is working
##l = iterative_deepening_search(w)

""" Best first """

## uniform cost not working!!
##l = uniform_cost_search(w)


""" Heuristica """

## A* not implemented h(n)
##l = astar_search(w)

## greedy not implemented h(n)
##l = greedy_best_first_graph_search(w)


for i in l[0].path():
	w.display(i.state)
	print("\n !!!")

