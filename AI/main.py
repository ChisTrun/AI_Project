# Import zone
import pygame
from priorityqueue import *
import numpy as np
from enum import Enum
import time
import copy

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128) 
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PINK = (255, 192, 203)
BLOCK_SIZE = 10
CROSS_COST = 14
STRAIGHT_COST = 10


class NodeStatus(Enum):
    BLOCKED = 1
    OPEN = 2
    CLOSE = 3
    START = 4
    END = 5
    NONE = 6
    DONE = 7

# Class implement zone
class Node:
    def __init__(self,x,y,g,h,status):
        self.x = x
        self.y = y
        self.h = h
        self.g = g
        self.f = g + h
        self.status = status
        self.parent = None

    def Display(self):
        print("(",self.x,",",self.y,")",end="")

class Map:
    def __init__(self,width, height):
        self.nodes = []
        for x in range(width):
            col = []
            for y in range(height):
                col.append(Node(x,y,0,0,NodeStatus.NONE))
            self.nodes.append(col)
    
    def Display(self):
        for col in self.nodes:
            for node in col:
                node.Display()
            print("")

    def GetNeighbours(self, node):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1),(1, 1),(-1,-1),(1,-1),(-1,1)]
        neighbours = []
        curX = node.x
        curY = node.y
        for i, (dx, dy) in enumerate(directions):
            nx, ny = curX + dx, curY + dy
            if 0 <= nx < len(self.nodes) and 0 <= ny < len(self.nodes[0]):
                neighbour = self.nodes[nx][ny]
                if  neighbour.status != NodeStatus.BLOCKED and neighbour.status != NodeStatus.OPEN and neighbour.status != NodeStatus.CLOSE and neighbour.status != NodeStatus.START:
                    if i in [0,1,2,3]:
                        neighbour.g = node.g + STRAIGHT_COST
                    else :
                        neighbour.g = node.g + CROSS_COST
                    neighbours.append(neighbour)
        return neighbours
    
    def SetBarricade(self,Vertices) :
        for i in range(len(Vertices) - 1):
            if i == 0 :
                nodes = AtarFindEdge(self,Vertices[i].x,Vertices[i].y,Vertices[len(Vertices)-1].x, Vertices[len(Vertices) - 1].y,EuclideanHeuristic)
                for node in nodes:
                    self.nodes[node.x][node.y].status = NodeStatus.BLOCKED
            nodes = AtarFindEdge(self,Vertices[i].x,Vertices[i].y,Vertices[i+1].x, Vertices[i+1].y,EuclideanHeuristic)
            for node in nodes:
                node.Display()
                self.nodes[node.x][node.y].status = NodeStatus.BLOCKED
            
            
    
    
    
        
# Will be implement
def EuclideanHeuristic(curNode,targetNode):
    return  np.sqrt(pow((curNode.x - targetNode.x),2) + pow((curNode.y - targetNode.y),2) ) * CROSS_COST

def ManhattanHeuristic(curNode,targetNode):
    return  (abs(curNode.x - targetNode.x) + abs(curNode.y - targetNode.y) ) * STRAIGHT_COST

# Pygame setup
def drawGrid(map):
    for row in map.nodes:
        for node in row:
            rect = pygame.Rect(node.x * BLOCK_SIZE, BLOCK_SIZE *(len(map.nodes[0]) - 1 - node.y )  , BLOCK_SIZE, BLOCK_SIZE)
            # if node.blocked:
            #     pygame.draw.rect(SCREEN, RED, rect)
            match node.status :
                case NodeStatus.START:
                    pygame.draw.rect(SCREEN, BLUE , rect)
                case NodeStatus.END:
                    pygame.draw.rect(SCREEN,ORANGE, rect)
                case NodeStatus.BLOCKED:
                    pygame.draw.rect(SCREEN, BLACK, rect)
                case NodeStatus.CLOSE:
                    pygame.draw.rect(SCREEN, RED, rect)
                case NodeStatus.OPEN:
                    pygame.draw.rect(SCREEN, GREEN, rect)
                case NodeStatus.DONE:
                    pygame.draw.rect(SCREEN, YELLOW, rect)
                
            pygame.draw.rect(SCREEN, GRAY, rect, 1)


def AtarFindEdge(map,startX, startY, endX, endY, heuristicFunction):
    copyMap = copy.deepcopy(map)
    startNode = copyMap.nodes[startX][startY]
    targetNode = copyMap.nodes[endX][endY]
    startNode.status = NodeStatus.START
    targetNode.status = NodeStatus.END
    open = PriorityQueue()
    close = PriorityQueue()
    open.insert(startNode)
    searching = True
    nodes = []
    while searching:
        currentNode = open.delete()
        close.insert(currentNode)
        currentNode.status = NodeStatus.CLOSE
        currentNode.Display()
        if currentNode == targetNode: 
            searching = False
            while currentNode != startNode:
                nodes.append(currentNode)
                # nodes.append(Node(currentNode.x +1,currentNode.y,0,0,NodeStatus.NONE))
                currentNode = currentNode.parent
        else:
            Neighbours = []
            Neighbours = copyMap.GetNeighbours(currentNode)
            for node in Neighbours:
                node.parent = currentNode
                node.h = heuristicFunction(node,targetNode)
                node.f = node.g + node.h
                node.status = NodeStatus.OPEN
                open.insert(node)
    nodes.append(startNode)
    return nodes

def AStar(map, startX, startY, endX, endY, heuristicFunction):
    startNode = map.nodes[startX][startY]
    targetNode = map.nodes[endX][endY]
    startNode.status = NodeStatus.START
    targetNode.status = NodeStatus.END
    open = PriorityQueue()
    close = PriorityQueue()
    open.insert(startNode)
    showing = True
    searching = True
    
    while showing:
        drawGrid(map)
        if searching:
            time.sleep(0.001)
            currentNode = open.delete()
            close.insert(currentNode)
            if currentNode != startNode:
                currentNode.status = NodeStatus.CLOSE
            if currentNode == targetNode: 
                searching = False
                print(currentNode.f)
                currentNode.status = NodeStatus.END
                while currentNode.parent != startNode:
                    currentNode.parent.status = NodeStatus.DONE
                    currentNode = currentNode.parent
            else:
                Neighbours = []
                Neighbours = map.GetNeighbours(currentNode)
                for node in Neighbours:
                    node.parent = currentNode
                    node.h = heuristicFunction(node,targetNode)
                    node.f = node.g + node.h
                    node.status = NodeStatus.OPEN
                    open.insert(node)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                showing = False

        pygame.display.update()



def main():
    map = Map(50,50)
    startX = 3
    startY = 10
    endX = 40
    endY = 20
    
    map.SetBarricade([Node(20,24,0,0,NodeStatus.NONE),Node(20,10,0,0,NodeStatus.NONE),Node(7,10,0,0,NodeStatus.NONE)])
    # map.SetBarricade([Node(1,1,0,0,NodeStatus.NONE),Node(3,3,0,0,NodeStatus.NONE)])
    global SCREEN, CLOCK
    pygame.init()
    SCREEN = pygame.display.set_mode((len(map.nodes) * BLOCK_SIZE , len(map.nodes[0]) * BLOCK_SIZE))
    CLOCK = pygame.time.Clock()
    SCREEN.fill(WHITE)


    AStar(map,startX,startY,endX,endY,EuclideanHeuristic)
    # AStar(map,startX,startY,endX,endY,ManhattanHeuristic)
   

#Run 
main()

