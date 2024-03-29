# Import zone
import pygame
from priorityqueue import *
import numpy as np
from enum import Enum
import time


BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128) 
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PINK = (255, 192, 203)
BLOCK_SIZE = 20
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


def AStart(map, startX, startY, endX, endY, heuristicFunction):
    startNode = map.nodes[startX][startY]
    targetNode = map.nodes[endX][endY]
    # map.nodes[4][5].status = NodeStatus.BLOCKED
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
            time.sleep(0.1)
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

    map = Map(30,30)
    startX = 3
    startY = 10
    endX = 27
    endY = 27
    
    # map.nodes[5][7].status = NodeStatus.BLOCKED
    # map.nodes[5][6].status = NodeStatus.BLOCKED
    # map.nodes[5][9].status = NodeStatus.BLOCKED
    # map.nodes[5][10].status = NodeStatus.BLOCKED
    # map.nodes[5][11].status = NodeStatus.BLOCKED
    # map.nodes[5][12].status = NodeStatus.BLOCKED
    # map.nodes[5][13].status = NodeStatus.BLOCKED
    # map.nodes[5][14].status = NodeStatus.BLOCKED
    
    # map.nodes[6][7].status = NodeStatus.BLOCKED
    # map.nodes[6][6].status = NodeStatus.BLOCKED
    # map.nodes[6][9].status = NodeStatus.BLOCKED
    # map.nodes[6][10].status = NodeStatus.BLOCKED
    # map.nodes[6][11].status = NodeStatus.BLOCKED
    # map.nodes[6][12].status = NodeStatus.BLOCKED
    # map.nodes[6][13].status = NodeStatus.BLOCKED
    # map.nodes[6][14].status = NodeStatus.BLOCKED
    
    # map.nodes[8][7].status = NodeStatus.BLOCKED
    # map.nodes[8][6].status = NodeStatus.BLOCKED
    # map.nodes[8][9].status = NodeStatus.BLOCKED
    # map.nodes[8][10].status = NodeStatus.BLOCKED
    # map.nodes[8][11].status = NodeStatus.BLOCKED
    # map.nodes[8][12].status = NodeStatus.BLOCKED
    # map.nodes[8][13].status = NodeStatus.BLOCKED
    # map.nodes[8][14].status = NodeStatus.BLOCKED
    
    # map.nodes[7][7].status = NodeStatus.BLOCKED
    # map.nodes[7][6].status = NodeStatus.BLOCKED
    # map.nodes[7][9].status = NodeStatus.BLOCKED
    # map.nodes[7][10].status = NodeStatus.BLOCKED
    # map.nodes[7][11].status = NodeStatus.BLOCKED
    # map.nodes[7][12].status = NodeStatus.BLOCKED
    # map.nodes[7][13].status = NodeStatus.BLOCKED
    # map.nodes[7][14].status = NodeStatus.BLOCKED
    
    # map.nodes[5][8].status = NodeStatus.BLOCKED
    # map.nodes[6][8].status = NodeStatus.BLOCKED
    # map.nodes[7][8].status = NodeStatus.BLOCKED
    # map.nodes[8][8].status = NodeStatus.BLOCKED
    
    
    

    global SCREEN, CLOCK
    pygame.init()
    SCREEN = pygame.display.set_mode((len(map.nodes) * BLOCK_SIZE , len(map.nodes[0]) * BLOCK_SIZE))
    CLOCK = pygame.time.Clock()
    SCREEN.fill(WHITE)


    AStart(map,startX,startY,endX,endY,ManhattanHeuristic)
   

#Run 
main()

