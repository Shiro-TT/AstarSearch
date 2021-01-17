import pandas as pd
import numpy as np     # installed with matplotlib
import pygame
import matplotlib.pyplot as plt
import math
import itertools 
import sys
sys.setrecursionlimit(100000)

def readInputFromFile(filename):
    file = open(filename, "r")
    # Read space limit
    str = file.read()
    file.close()
    return str

#direct distance of A and B
def distanceFromAtoB(A,B):
    if (A[0]==B[0] and A[1]==B[1]):
        return 0
    d = math.sqrt((B[0]-A[0])**2+(B[1]-A[1])**2)
    return d

class AstarGraph(object):
    def __init__(self, input):
        inputLine = input.splitlines()
        str = inputLine[0].split(",")
        self.length = (int)(str[0]) + 1
        self.width = (int)(str[1]) + 1
        self.Maze = np.zeros((self.length,self.width))
        for i in range(self.width):
            self.Maze[0][i]=-1
            self.Maze[self.length-1][i]=-1
        for i in range(self.length):
            self.Maze[i][0]=-1
            self.Maze[i][self.width-1]=-1
        self.barriers = []  #chứa tất cả các điểm là khung
        self.numberOfStopPoint = 0
    #start Point
    def getStartPoint(self):
        for i in range(self.length-1):
            for j in range(self.width-1):
                if self.Maze[i][j] == 1:
                    return (i,j)
        return None
    #end Point
    def getEndPoint(self):
        for i in range(self.length-1):
            for j in range(self.width-1):
                if self.Maze[i][j] == 2:
                    return (i,j)
        return None
    #stop Point
    def getStopPoint(self):
        n=[]
        for i in range(self.length-1):
            for j in range(self.width-1):
                if self.Maze[i][j] == 3:
                    n.append((i,j))
        return n
    #print
    def printMaze(self):
        print(self.Maze)
    #return Maze
    def getMaze(self):
        return Maze
    #check if point i is in the Maze
    def inMaze(self,i):
        if (i[1]<=0 or i[1]>=self.width-1):
            return False
        if (i[0]<=0 or i[0]>=self.length-1):
            return False
        return True

    #Read input to get speacialLocation and mark it in the Maze matrix: startpoint , endpoint ,stoppoint , barrier
    def getSpecialLocations(self,input):
        inputLine = input.splitlines()
        locations = inputLine[1].split(",")
        locationsLength = len(locations)
        if locationsLength % 2 == 0:
            numberOfLocation = locationsLength/2
        if numberOfLocation <2:
            print("Input file missing data")
            return 0
        else:
            #Get startLocations and endLocations
            # and Add special Location to the Maze  
            startLocation_l = (int)(locations[0])
            startLocation_w = (int)(locations[1])
            endLocation_l = (int)(locations[2])
            endLocation_w = (int)(locations[3])
            #1 ~ startLocation
            self.Maze[startLocation_l][startLocation_w] = 1
            #2 ~ endLocation
            self.Maze[endLocation_l][endLocation_w] = 2
            #Get stopLocation
            if numberOfLocation > 2:
                count = 4
                index = 0
                while count < locationsLength:
                    stopLocation_l = (int)(locations[count])
                    stopLocation_w = (int)(locations[count+1])
                    #3 ~ stopLocation
                    self.Maze[stopLocation_l][stopLocation_w] = 3
                    self.numberOfStopPoint = self.numberOfStopPoint + 1
                    count = count + 2
                    index = index + 1
        return 1

    #draw polygon to Maze
    def drawPolygon(self,input):
        inputLine = input.splitlines()
        if len(inputLine) <= 2 :    #if there're no input of Polygon:
            return
        numberOfPolygon = (int)(inputLine[2])
        polygonArray = []   #tao mang 2 chieu luu cac diem cua moi da giac len 1 dong
        for i in range(numberOfPolygon):
            polygonArray.append(inputLine[3+i])
            point = polygonArray[i].split(",")

            l = int(len(point)/2)
            A = (((int)(point[(int)(len(point)-2)]),(int)(point[(int)(len(point)-1)])))
            B = ((int)(point[0]),(int)(point[1]))
            self.conectToPointInMaze(A,B)
            for i in range(l-1):
                A = (((int)(point[i*2]),(int)(point[i*2+1])))
                B = (((int)(point[i*2+2]),(int)(point[i*2+3])))
                self.conectToPointInMaze(A,B)

    #conect 2 point A and B and mark it in Maze
    def conectToPointInMaze(self,A,B):
        if (A not in self.barriers):
            self.Maze[A[0]][A[1]] = -2
            self.barriers.append(A)
        I = A
        T = A
        min = 2*distanceFromAtoB(I,B) + 1000000 # so cuc lon
        while (I[0]!=B[0] or I[1]!=B[1]):
            for i in self.get_vertex_neighbours(I):
                diem1 =  distanceFromAtoB(i, B)
                if diem1 < min:
                    if self.Maze[i[0]][i[1]] > 0:
                        continue
                    ## only create wall on the blank position
                    min = diem1
                    T = i
            I = T
            if (I[0]==B[0] and I[1]==B[1]):
                break
            self.Maze[I[0]][I[1]] = -2
            self.barriers.append(I)
            min = distanceFromAtoB(I,B)

    #caculate heuristic
    def heuristic_directDistance(self, start, end):    #heuristic : directDistance
        return distanceFromAtoB(start,end)

    #Get 8 point nearby
    def get_vertex_neighbours(self,position):
        n = []
        #Move allow
        for dx , dy in [(0,-1),(0,1),(-1,0),(1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            x = position[0] + dx
            y = position[1] + dy
            if self.inMaze((x,y)) == False:
                continue
            n.append((x,y))
        return n

    #cost move from A to B (A next to B)
    def move_cost(self, A, B):
        for barrier in self.barriers:
            if B in barrier:
                return inf
        if A[0]==B[0] or A[1]==B[1]:
                return 1
        return 1.5

#a star search : return path to 2 Point (startPoint to endPoint) with map Graph (don't care about the stopPoint)
def aStarSearchAlogrithm(startPoint,endPoint,graph):
    G = {}
    F = {}
    G[startPoint] = 0
    F[startPoint] =  graph.heuristic_directDistance(startPoint,endPoint)
    #initialze openList and closeList
    open_list = []
    close_list = []
    cameFrom = {}

    open_list.append(startPoint)

    #Loop to find endpoint
    while len(open_list)>0:
        current = None
        currentFcore = None
        for position in open_list:
            if current is None or F[position] < currentFcore:
                currentFcore = F[position]
                current = position
        #found the goal
        if current == endPoint:
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            #return the reverse path and cost
            return path[::-1],F[endPoint]

        #pop current off openList, add to closeList
        open_list.remove(current)
        close_list.append(current)

        for neighbour in graph.get_vertex_neighbours(current):
            if neighbour in close_list:     #neighbour opened
                continue
            if neighbour in graph.barriers: #neighbour is barrrier
                continue
            M = graph.move_cost(current,neighbour)
            #kiểm tra đi chéo có vào hình không
            if M == 1.5:    #đi chéo
                index = ((neighbour[0] - current[0], neighbour[1] - current[1]))
                if index == ((-1,1)):
                    temp1 =  ((current[0] ,current[1]+1))
                    temp2 =  ((current[0] -1,current[1]))
                elif index == ((1,1)):
                    temp1 =  ((current[0] ,current[1]+1))
                    temp2 =  ((current[0] +1,current[1]))
                elif index == ((1,-1)):
                    temp1 =  ((current[0] ,current[1]-1))
                    temp2 =  ((current[0] +1,current[1]))
                else:   #index == ((-1,-1))
                    temp1 =  ((current[0] -1,current[1]))
                    temp2 =  ((current[0] ,current[1]-1))
                if graph.inMaze(temp1)==False or graph.inMaze(temp2)==False:
                    continue    
                if graph.Maze[temp1[0] ,temp1[1]] == -2 and graph.Maze[temp2[0] ,temp2[1]] == -2:
                    continue
            candidateG = G[current] + M
            if neighbour not in open_list:
                open_list.append(neighbour)
            elif candidateG >= G[neighbour]:    #way not optimize
                continue
            #Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic_directDistance(neighbour,endPoint)
            F[neighbour] = G[neighbour] + H
    return None,-1

#a star search : return path from startPoint to endPoint with map Graph (before reach the endPoint, they've to pass all the stopPoint, if 1 or more spoint can't reach, return None)
def aStarSearch(matrixA):
    #tìm các điểm đặc biệt
    begin = matrixA.getStartPoint()
    end = matrixA.getEndPoint()
    #print("begin ",begin," end ",end)
    numSP = matrixA.numberOfStopPoint
    #print("Number of stopPoint ",numSP)

    #tìm đường đi
    #neu khong co diem don
    if numSP == 0:
        path,cost = aStarSearchAlogrithm(begin,end,matrixA)
    #neu co 1 diem don
    elif numSP == 1:
        path = []
        stopPositions = matrixA.getStopPoint()
        stop = stopPositions[0]
        #print("stopPoint ",stop)

        path1 ,cost1 = aStarSearchAlogrithm(begin,stop,matrixA)
        if path1 != None:
            path2 = None
            cost2 = -1
            path2 ,cost2 = aStarSearchAlogrithm(stop,end,matrixA)
            if path2 == None:
                path = None
            else:
                path2.pop(0)
                path = path1 + path2
                cost = cost1 + cost2
        else:
            path = None
    #neu co nhieu diem don
    else:
        #print all stopPoint
        stopPositions = matrixA.getStopPoint()
        #print("stopPoint ",stopPositions)
        minPath = []
        minCost = -1
     
        for stopList in itertools.permutations(stopPositions):
            stopListTemp = list(stopList)
            #thuc hien tim duong di theo thu tu cua stopListTemp
            path = []
            cost = 0
            stopListTemp.append(end)
            x = begin
            while stopListTemp != []:
                stop = stopListTemp[0]
                path1 ,cost1 = aStarSearchAlogrithm(x,stop,matrixA) 
                if path1 != None:
                    #xoa phan tu trung o dau 
                    if path != []:
                        path1.pop(0)
                    path = path + path1
                    cost = cost + cost1
                    stopListTemp.pop(0)
                    x = stop
                else:  #nếu không tìm được đường đi
                    path = None
                    break
            if path != None:
                if cost < minCost or (cost > 0 and minCost == -1):
                    minPath = path
                    minCost = cost
        #ket qua
        path = minPath
        cost = minCost
    #in ra kết quả
    if path == None or path == []:
        return None,-1
    else:
        return path,cost

def drawOnPyGame(path,cost, matrixMaze):
    pygame.init()
    gameDisplay = pygame.display.set_mode((1000,600))
    pygame.display.set_caption("A star search")

    smallfont = pygame.font.SysFont("comicsansms",25)
    mediumfont = pygame.font.SysFont("comicsansms",45)
    largefont = pygame.font.SysFont("comicsansms",80)
    x= 30
    y= 30
    width = 30
    height = 30
    vel = 5
    gameExit = False
    count=0
    while not gameExit:
        count += 1
        pygame.time.delay(1000)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                gameExit = True
        if count == 2:
            gameExit = True
        for xM in range(matrixMaze.length):
            for yM in range(matrixMaze.width):
                #Polygon
                if matrixMaze.Maze[xM][yM] == -2:
                    pygame.draw.rect(gameDisplay,(56,4,75),(x*xM,y*yM,width,height))
                #outside Wall
                if matrixMaze.Maze[xM][yM] == -1:
                    pygame.draw.rect(gameDisplay,(54,54,54),(x*xM,y*yM,width,height))
                #Blank postion
                if matrixMaze.Maze[xM][yM] == 0:
                    pygame.draw.rect(gameDisplay,(255,255,255),(x*xM,y*yM,width,height))
                #StartPoint Position
                if matrixMaze.Maze[xM][yM] == 1:
                    pygame.draw.rect(gameDisplay,(0,166,173),(x*xM,y*yM,width,height))
                    startPointText = smallfont.render(" S",True, (0,0,0))
                    gameDisplay.blit(startPointText,[x*xM,y*yM])
                #EndPoint Position
                if matrixMaze.Maze[xM][yM] == 2:
                    pygame.draw.rect(gameDisplay,(255,0,0),(x*xM,y*yM,width,height))
                    endPointText = smallfont.render(" E",True, (0,0,0))
                    gameDisplay.blit(endPointText,[x*xM,y*yM])
                #StopPoint postion
                if matrixMaze.Maze[xM][yM] == 3:
                    pygame.draw.rect(gameDisplay,(255,255,0),(x*xM,y*yM,width,height))
                    stopPointText = smallfont.render("SP",True, (0,0,0))
                    gameDisplay.blit(stopPointText,[x*xM,y*yM])
        #Draw result
        if path == None or path ==[]:
            textNoFound = mediumfont.render("No way to go",True, (255,0,0))
            gameDisplay.blit(textNoFound,[690,0])
            pygame.display.update()
            pygame.time.delay(200)
        else:
            textFound = mediumfont.render("Found the way",True, (255,0,0))
            gameDisplay.blit(textFound,[690,0])
            text = smallfont.render("Cost: ",True, (255,255,255))
            gameDisplay.blit(text,[800,100])
            for pos in path:
                pygame.draw.rect(gameDisplay,(0,174,114),(x*pos[0],y*pos[1],width,height))
                pygame.display.update()
                pygame.time.delay(150)
            textCost = smallfont.render("Cost: "+ str(cost),True, (255,255,255))
            gameDisplay.blit(textCost,[800,100])
            pygame.display.update()
            pygame.time.delay(200)
    pygame.quit()

def main():
    print("A star Search Alogrihm")
    
    fileInputName = "input.txt"

    #đọc dữ liệu
    inputL = readInputFromFile(fileInputName)
    #nạp dữ liệu
    matrixA = AstarGraph(inputL)
    matrixA.getSpecialLocations(inputL)
    matrixA.drawPolygon(inputL)

    #tìm đường đi
    pathResult, costResult = aStarSearch(matrixA)
    #in kết quả
    drawOnPyGame(pathResult,costResult,matrixA)
   
main()
