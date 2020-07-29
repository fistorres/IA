#########PROJECT PROBLEM CLASS AND STATE CLASS
from search2_2 import *
from copy import deepcopy

class ProblemaPacInv(Problem):
    def __init__(self, x, y, p, o, pacPos = None):
        """
        P an O are list
        x and y are int, the dimensions of the game board 
        pacPos is position of pac
        """
        if type(o) == int:
            Obs = []
            while len(Obs) < o:
                o_x = random.randrange(1,x+1)
                o_y = random.randrange(1,y+1)
                if (o_x,o_y) not in Obs:
                    Obs.append((o_x,o_y))
            o = deepcopy(Obs)
            
        if type(p) == int:
            Gums = []
            while len(Gums) < p:
                p_x = random.randrange(1,x+1)
                p_y = random.randrange(1,y+1)
                if (p_x,p_y) not in o \
                and (p_x,p_y) not in Gums:
                    Gums.append((p_x,p_y))
            p = deepcopy(Gums)
            
        while pacPos == None:
             p_x = random.randrange(1,x+1)
             p_y = random.randrange(1,y+1)
             if (p_x,p_y) not in o \
            and (p_x,p_y) not in p:
                pacPos = (p_x,p_y)
            
        able_gums = 0
        for i in p:
            if not self.unable_gum(i, o, x, y):
                able_gums += 1
                
        if able_gums < 3:
            raise Exception('Ups talvez queira adicionar mais pastilhas')
        else:         
            self.x = x
            self.y = y
            self.obstacles = o 
            self.gums = p
            self.prox = [(x,y) for x in range(-1,2) for y in range(-1,2)]
            self.initial_state = self.result(PacGameState(pacPos))
            super().__init__(self.initial_state)

    def unable_gum(self, gum, obstacles, width, height):
        surround_by = 0
        for x in range(-1, 2):
            for y in range(-1, 2):
                relPos = (gum[0] + x, gum[1] + y)
                if relPos in obstacles or relPos[0] > width or relPos[y] > height:
                    surround_by += 1  
        if surround_by == 8:
            print(relPos)
            return True
        return False
        

    def actions(self,state):
        """ Returns all the the legal actions given a state.
            Requires: state as instance of PacGameState
        """
        #Only legal actions
        actions = deepcopy(self.prox)
        actions.remove((0,0))
        position = state.pos
        temp = deepcopy(actions)
        for action in temp:
            newpos = (position[0]+action[0], \
                      position[1]+action[1])
            if newpos in self.obstacles:
                actions.remove(action)
            elif newpos[0] > self.x or newpos[1] > self.y or \
                 newpos[0] < 1 or newpos[1] < 1:
                actions.remove(action)

        return actions
 
    def result(self, state, action=(0,0)):
        """ Considers a state and an action and returns the new resulting state
            Requires: state as instance of PacGameState, action as tuple.
            Ensures: new instance of PacGameState
        """
        # (0,0) to check if PacMan starts close to any gums
        if action in self.actions(state) or action == (0,0):
            newposition = (state.pos[0]+action[0], \
                            state.pos[1]+action[1])
            
            # update touched_gums, position and disc_pos
            newdisc_pos = state.disc_pos
            newtouched_gum = deepcopy(state.touched_gum)

            # self.prox are the proximities, relative to pac. Newloc is the absolute location on the board
            for prox in self.prox:
                newloc = (newposition[0]+prox[0], \
                           newposition[1]+prox[1])
                if newloc in self.gums:
                    
                    if prox == (0,0): #gum is stepped on. (touch=2, because Im using 2 as synonym to popped)
                            if newloc not in newtouched_gum["2"]:
                                newtouched_gum["2"].append(newloc)
                            
                            if newloc in newtouched_gum["1"]:
                                newtouched_gum["1"].remove(newloc)
                                
                    elif newloc in list(newtouched_gum.values())[0] \
                         or newloc in list(newtouched_gum.values())[1] :
                        
                        if newloc in newtouched_gum["1"]:
                            newtouched_gum["2"].append(newloc)
                            newtouched_gum["1"].remove(newloc)
                            newdisc_pos[prox] = "X"
                    else:
                        newtouched_gum["1"].append(newloc)
                        newdisc_pos[prox] = "X"             
        else:
            print("Action not recognized")
            return state

        return PacGameState(newposition,newtouched_gum,newdisc_pos)

    def path_cost(self, c, state1, action, state2):
        """ Calculates the cost of an action given an inicial state
            and the resulting state of that action.
            Requires: state1 and state2 as instances of PacGameState
       """
        popped1 = len(state1.touched_gum["2"])
        popped2 = len(state2.touched_gum["2"])
        blastedcost = (popped2 - popped1) * 10
        
        #cost of moving up/sideway
        movcost = 0
        if action[1] == 1:
            movcost += 3
        if action[1] == 0 and action != (0,0):
            movcost += 1

        return blastedcost + movcost + c
 



    def display(self,state):
        """
        Displays the game board given a state
        Requires: state as instance of PacGameState
        """
        poppedgums = state.touched_gum["2"]
        l = []
        for y in range(1,self.y+1):
            sti = ""
            for x in range(1,self.x+1):
                # estou a assumir que nao ha pastilhas onde há obstaculos \
                # e que na função geradora geramos os obstaculos primeiro
                if (x,y) in self.obstacles:
                    sti += "O" 
                elif (x,y) in self.gums and (x,y) not in poppedgums:
                    sti += "P" 
                elif (x,y) == state.pos:
                    sti += "@" 
                else:
                    sti += "." 
            l.append(sti)

        l.reverse()
        for i in l:
            for x in i:
                print(x,end="   ")
            print("\n")
                            
    def goal_test(self,state) :
        """
        Checks to see if a state is a final state
        Requires: state as instance of PacGameState()
        Ensures: True if state is a fial state
        """
        c = 0
        for i in estado.disc_pos.values():
            if i == "X":
                c += 1
        if c == 7:
            return True
        return False

    
                    
                
class PacGameState():
    
    refboard = {}
    ref = [(x,y) for x in range(-1,2) for y in range(-1,2)]
    ref.remove((0,0))
    for i in ref:
        refboard[i] = ""
        
    # t_gum is dic in wich the key is the number of touched
    def __init__(self,position = (1,1), touched_gum = {"1":[],"2":[]},
                 discarded_positions = deepcopy(refboard)):
        
        self.touched_gum = touched_gum
        self.disc_pos = deepcopy(discarded_positions)
        self.pos = position

    def __lt__(self,o):
        return self
 
    def __eq__(self, o):
        return self.touched_gum == o.touched_gum and self.disc_pos == o.disc_pos and self.pos == o.pos

    def __hash__(self):
        return hash(((str(self.touched_gum)),(str(self.pos)),(str(self.disc_pos))))
    
    def __str__(self):
        return str(self.pos) 
