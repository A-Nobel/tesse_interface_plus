import numpy as np
import itertools
import matplotlib.pyplot as plt
import random
import math

class roomNode():
    def __init__(self,coord,slope,timecost,changes,number,timestep,name,vec):
        self.coord = coord
        self.x = coord[0]
        self.y = coord[1]
        self.slope = slope
        self.timecost = timecost # time used to observe the room
        self.changes = changes
        self.number = number
        self.timestep = timestep
        self.name = name
        self.vec = vec

class robot():
    def __init__(self,coord,roomR):
        self.coord = coord
        self.x = coord[0]
        self.y = coord[1]
        self.roomR = None

class gameEnv():
    def __init__(self):
        self.counts = 0
        self.rooms = []
        self.robot = None
        self.totalReward = 0
        self.time = 0

    def reset(self):
        self.counts = 0
        self.rooms = []
        self.robot = None
        self.totalReward = 0
        self.time = 0
        self.rooms.append(roomNode((10,10),slope=0.3,changes = 0,number = 0,timestep = 0,name='office1',vec=1))
        self.rooms.append(roomNode((60,20),slope=3,changes = 0,number = 0,timestep = 0,name='office2',vec=2))
        # change the following code into parameter value format
        # self.rooms.append(roomNode((80,25),3,6*6,0,1,0,2,'lab',self.room_lab))
        # self.rooms.append(roomNode((10,50),5,7*7,0,2,0,1.5,'reading room',self.room_reading))
        # self.rooms.append(roomNode((30,70),7,8*8,0,3,0,1.4,'office',self.room_office))
        # self.rooms.append(roomNode((75,60),1,5*5,0,4,0,3,'storage room',self.room_storage))
        self.actions = len(self.rooms)

        randomRoom = random.choice(self.rooms)
        self.robot = robot(randomRoom.coord,randomRoom.x,randomRoom.y,randomRoom.name)
        a = randomRoom.vec
        b = randomRoom.timestep
        c = randomRoom.changes
        observation = np.array([a,b,c])

        return observation

    
    def newPosition(self):
        iterables = [ range(self.sizeX), range(self.sizeY)]
        points = []
        for t in itertools.product(*iterables):
            points.append(t)
        currentPositions = []
        for objectA in self.rooms:
            if (objectA.x,objectA.y) not in currentPositions:
                currentPositions.append((objectA.x,objectA.y))
        for pos in currentPositions:
            points.remove(pos)
        location = np.random.choice(range(len(points)),replace=False)
        return points[location]

        
    def step(self,action):
        '''
        pre state is [prechanges,pretimes,preroomname]
        action happen
        room timestep ++
        room reward ++
        robot go and ob the romm and get reward
        the room reward = 0
        now observed state is [change,time,roomname]
        '''
        
        roomDes = self.rooms[action]
        robotNow  = self.robot
        
        # Compute action timecost
        timestep = abs(robotNow.x - roomDes.x) + abs(robotNow.y - roomDes.y) + roomDes.timecost
        
        # Update all rooms properties
        for room in self.rooms:
            room.timestep += timestep
            room.changes += round(room.slope*timestep/100)

        # Reset the robot
        robotNow.x = roomDes.x
        robotNow.y = roomDes.y
        robotNow.roomR = roomDes.name
        
        # Compute the reward using log2 and pathcost
        if roomDes.changes > 1 :
            reward = 10*math.log(roomDes.changes*10,2) - timestep//5
        else :
            reward = - timestep//5
        # Count the total reward and overall time cost
        self.totalReward += reward
        self.time += timestep
        
        # Reset the room
        roomDes.changes = 0
        roomDes.timestep = 0
        
        # Get observation
        a = roomDes.vec
        b = roomDes.timestep
        c = roomDes.changes
        observation = np.array([a,b,c])

        # return observation,reward,done
        return observation , reward , False


def main():
    env = gameEnv(100,100)
    env.reset()
    # import sys
    # for i in range(5):
    #     print("\r", end="")
    #     print(str(i)*10, end='')
    #     sys.stdout.flush()
    TL = []
    RL = []
    while(True):
        state,reward,done = env.step(random.randrange(4))
        # birth_year = input("hhh")
        print("x: %d y: %d "%(env.robot.x,env.robot.y))
        print("reward: %d"%env.totalReward)
        TL.append(env.time)
        RL.append(env.totalReward)
        if(env.time>50000):
            break


    with open("./randomreward.txt", 'w') as rr:
        for i in range(0,len(TL)-1):
            rr.write(str(TL[i])+' '+str(RL[i])+'\n')

    TL = []
    RL = []
    env.reset()
    i=0
    while(True):
        if(i>=5):
            i=0
        state,reward,done = env.step(i)
        # birth_year = input("hhh")
        print("x: %d y: %d "%(env.robot.x,env.robot.y))
        print("reward: %d"%env.totalReward)
        TL.append(env.time)
        RL.append(env.totalReward)
        i+=1
        if(env.time>50000):
            break


    with open("./nochange.txt", 'w') as rr:
        for i in range(0,len(TL)-1):
            rr.write(str(TL[i])+' '+str(RL[i])+'\n')

if __name__== "__main__" :
    main()

