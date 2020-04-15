import os
import re
import random
import time
import math

def test():
    timeStart = time.time()
    points = Dijkstra('v.txt', 'e.txt')
    timeEnd = time.time()
    timeStart1 = time.time()
    AStar('v.txt', 'e.txt', points[0], points[1])
    timeEnd1 = time.time()
    print('Dijkstra run     ', timeEnd-timeStart, 's')
    print('A* algorithm run ',timeEnd1-timeStart1, 's', '\n' )


def calculateDistanceInGrid(square1, square2):
    x1 = square1%10*10
    y1 = square1//10*10
    x2 = square2%10*10
    y2 = square2//10*10

    if x1==x2 and y1==y2:
        minDis = 0
    elif x1==x2:
        minDis = abs(y1-y2)-10
    elif y1==y2:
        minDis = abs(x1-x2)-10
    elif x1<x2 and y1<y2:
        x1 = x1 + 10
        y1 = y1 + 10
        minDis = math.sqrt((x1-x2)**2+(y1-y2)**2)
    elif x1>x2 and y1>y2:
        x2 = x2 + 10
        y2 = y2 + 10
        minDis = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    elif x1<x2 and y1>y2:
        x1 = x1 + 10
        y2 = y2 + 10
        minDis =math.sqrt((x1-x2)**2+(y1-y2)**2)
    elif x1>x2 and y1<y2:
        y1 = y1 + 10
        x2 = x2 + 10
        minDis = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return minDis

def chooseHeuristic(square1, square2):
    #return calculateDistanceInGridWithHigerCost()
    return calculateDistanceInGrid(square1,square2)

def AStar(verticesFile, edgesFile, startPoint, endPoint):
    numberOfSteps = 0
    openlist = {}
    closelist = []
    father = {}
    #sotre vertices
    vertices = {}
    #store edges, edged is an array[100][100], the value of edges[1][2] is distance from vertex1 to vertex2
    maxNum = float('inf')
    #edges = [[maxNum]*1000 for i in range(1000)]
    #Put the two files in the same directory as the .py file
    currentPath = os.path.abspath(os.path.dirname(__file__))
    pathV = os.path.join(currentPath, verticesFile)
    cnt = 0
    with open(pathV, 'r', encoding='utf-8') as vFile:
        for line in vFile.readlines():
            if line.startswith('#') or line.startswith(' '):
                continue
            else:
                p = re.split(r'[\,\n]',line)
                x = int(p[0])
                positionInGraph = int(p[1])
                vertices[x] = positionInGraph
                father[x] = -1
                cnt = cnt + 1

    edges = [[maxNum]*cnt for i in range(cnt)]
    pathE = os.path.join(currentPath, edgesFile)
    with open(pathE, 'r', encoding='utf-8') as eFile:
        # cntEdges = 0
        for line in eFile.readlines():
            if line.startswith('#') or line.startswith(' '):
                continue
            else:
                p = re.split(r'[\,\n]',line)
                x = int(p[0])
                y = int(p[1])
                distanceXY = int(p[2])
                edges[x][x] = 0
                edges[y][y] = 0
                edges[x][y] = distanceXY
                edges[y][x] = distanceXY
    #A*
    openlist[startPoint] = [0,0]
    while openlist:
        numberOfSteps = numberOfSteps + 1
        minf = maxNum
        indexOfMinF = 0
        for k, v in openlist.items():
            numberOfSteps = numberOfSteps + 1
            if v[0]+v[1]<minf:
                minf = v[0]+v[1]
                indexOfMinF = k
        poi = indexOfMinF
        if poi==endPoint:
            break
        poi_g = openlist[poi][0]
        openlist.pop(poi)
        closelist.append(poi)

        g = h =0
        for i in range(cnt):
            numberOfSteps = numberOfSteps + 1
            if edges[poi][i]!=maxNum and i not in closelist:
                g = poi_g+edges[poi][i]
                h = chooseHeuristic(vertices[endPoint],vertices[i])
                #h=0
                if i not in openlist:
                    father[i] = poi
                    openlist[i] = [g, h]
                else:
                    if openlist[i][0]>g:
                        openlist[i][0] = g
                        father[i] = poi
    #print path
    shortestPath = []
    shortestValue = 0
    j = endPoint
    cntPath = 1
    while father[j]!=-1:
        shortestValue = edges[j][father[j]] + shortestValue
        shortestPath.append(j)
        j = father[j]
        cntPath = cntPath+1
    shortestPath.append(startPoint)
    shortestPath.reverse()
    print('A*: The shortest distance from point '+str(startPoint)+' to '+str(endPoint)+' is '+str(shortestValue))
    print('A*: The shortest path is ', end='')
    for i in range(cntPath-1):
        print(str(shortestPath[i])+'->', end='')
    print(shortestPath[cntPath-1])
    print('A*: The number of step is ', numberOfSteps, '\n')






def Dijkstra(verticesFile, edgesFile):
    numberOfSteps = 0
    #if current value is the shortest value, book[x]=1
    book = []
    distance = []
    father = []
    #sotre vertices, eg. vertices[0] = {20,10} x=20, y=10
    vertices = {}
    #store edges, edged is an array[100][100], the value of edges[1][2] is distance from vertex1 to vertex2
    maxNum = float('inf')
    #Put the two files in the same directory as the .py file
    currentPath = os.path.abspath(os.path.dirname(__file__))
    pathV = os.path.join(currentPath, verticesFile)
    cnt = 0
    with open(pathV, 'r', encoding='utf-8') as vFile:
        for line in vFile.readlines():
            if line.startswith('#') or line.startswith(' '):
                continue
            else:
                p = re.split(r'[\,\n]',line)
                vertices[p[0]] = p[1]
                book.append(0)
                distance.append(maxNum)
                father.append(p[0])
                cnt = cnt + 1

    edges = [[maxNum]*cnt for i in range(cnt)]
    pathE = os.path.join(currentPath, edgesFile)
    with open(pathE, 'r', encoding='utf-8') as eFile:
        for line in eFile.readlines():
            if line.startswith('#') or line.startswith(' '):
                continue
            else:
                p = re.split(r'[\,\n]',line)
                x = int(p[0])
                y = int(p[1])
                distanceXY = int(p[2])
                edges[x][x] = 0
                edges[y][y] = 0
                edges[x][y] = distanceXY
                edges[y][x] = distanceXY
    #generate start point and end point randomly
    startPoint = random.randint(0, cnt-1)
    endPoint = random.randint(0, cnt-1)
    # startPoint = 557
    # endPoint = 340
    book[startPoint] = 1
    distance[startPoint] = 0
    for i in range(cnt):
        distance[i] = edges[startPoint][i];
        if distance[i]<maxNum:
            father[i] = startPoint
    father[startPoint] = -1
    #Dijkstra
    for i in range(cnt):
        Min = maxNum
        tempIndex = 0
        for j in range(cnt):
            if book[j]==0 and distance[j]<Min:
                Min = distance[j]
                tempIndex = j
        book[tempIndex] = 1
        for j in range(cnt):
            numberOfSteps = numberOfSteps + 1
            if book[j]==0 and edges[tempIndex][j]!=maxNum and (distance[j]>distance[tempIndex]+int(edges[tempIndex][j])):
                distance[j] = distance[tempIndex]+edges[tempIndex][j]
                father[j] = tempIndex
                numberOfSteps = numberOfSteps + 1

    #print path
    shortestPath = []
    shortestValue = distance[endPoint]
    j = endPoint
    cntPath = 1
    while father[j]!=-1:
        shortestPath.append(j)
        j = father[j]
        cntPath = cntPath+1
    shortestPath.append(startPoint)
    shortestPath.reverse()
    print('Dijkstra: The shortest distance from point '+str(startPoint)+' to '+str(endPoint)+' is '+str(shortestValue))
    print('DijkstraThe shortest path is ', end='')
    for i in range(cntPath-1):
        print(str(shortestPath[i])+'->', end='')
    print(shortestPath[cntPath-1])
    print('Dijkstra: The number of step is ', numberOfSteps, '\n')
    return [startPoint, endPoint]



test()