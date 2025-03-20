import numpy as np
import math
from enum import Enum
from queue import PriorityQueue
from sklearn.cluster import KMeans
import os
import warnings


warnings.filterwarnings("ignore", message="KMeans is known to have a memory leak on Windows with MKL")

os.environ["OMP_NUM_THREADS"] = "2"  


class Direction(Enum):
    RIGHT = 0
    UP = np.deg2rad(90)
    LEFT = np.deg2rad(180)
    DOWN = np.deg2rad(270)

class Robot:
    
    def __init__(self, visibleAngle=120, visibleDistance=40):
        # Convert visible angle to radians
        self.visibleAngle = math.radians(visibleAngle)
        self.visibleDistance = visibleDistance
        self.mapExtendSize = visibleDistance + 1
        self.map = np.full((self.mapExtendSize * 2 + 1, self.mapExtendSize * 2 + 1), -1)
        self.yInMap, self.xInMap = -1, -1
        self.field = None
        self.direction = Direction.RIGHT
        self.rayAngle, self.rayCount = self.DecideRayInfo()
        self.plot = None

    def DecideRayInfo(self):
        
        # Calculate ray angle and count for sensor simulation
        maxRayAngle = math.acos((2 * (self.visibleDistance ** 2) - 0.9) / (2 * (self.visibleDistance ** 2)))
        sectorCount = math.ceil(self.visibleAngle / maxRayAngle)
        rayAngle = self.visibleAngle / sectorCount
        return rayAngle, sectorCount + 1

    def MapExtend(self):
        # Extend internal map if robot nears map boundary
        if (self.yInMap + self.visibleDistance) > (self.map.shape[0] - 2):
            yRange = slice(0, self.map.shape[0], 1)
            yChange = True
        elif (self.yInMap - self.visibleDistance) < 1:
            yRange = slice(self.mapExtendSize, self.mapExtendSize + self.map.shape[0], 1)
            self.yInMap += self.mapExtendSize
            yChange = True
        else:
            yRange = slice(0, self.map.shape[0], 1)
            yChange = False

        if (self.xInMap + self.visibleDistance) > (self.map.shape[1] - 2):
            xRange = slice(0, self.map.shape[1], 1)
            xChange = True
        elif (self.xInMap - self.visibleDistance) < 1:
            xRange = slice(self.mapExtendSize, self.mapExtendSize + self.map.shape[1], 1)
            self.xInMap += self.mapExtendSize
            xChange = True
        else:
            xRange = slice(0, self.map.shape[1], 1)
            xChange = False

        if not (yChange or xChange):
            return
        else:
            tempMap = self.map.copy()
            newRows = self.map.shape[0] + self.mapExtendSize * int(yChange)
            newCols = self.map.shape[1] + self.mapExtendSize * int(xChange)
            self.map = np.full((newRows, newCols), -1)
            self.map[yRange, xRange] = tempMap.copy()

    def Move(self):
        
        # Mark current cell as visited
        self.map[self.yInMap, self.xInMap] = 1
        # Determine movement vector based on direction
        dy, dx = -int(np.sin(self.direction.value)), int(np.cos(self.direction.value))

        # Check if next cell is blocked (0 represents obstacle)
        if self.map[self.yInMap + dy, self.xInMap + dx] == 0:
            return

        self.yInMap += dy
        self.xInMap += dx

        self.field.RobotMove(self, dy, dx)
        self.MapExtend()
        self.map[self.yInMap, self.xInMap] = 2
        self.PerfectSee()

    def Turn(self, newDirection):
        
        self.direction = newDirection
        self.PerfectSee()
        

    def AssignField(self, fieldInstance):
        
        # Set robot to map center and assign field
        self.yInMap = int((self.map.shape[0] - 1) / 2)
        self.xInMap = int((self.map.shape[1] - 1) / 2)
        self.field = fieldInstance
        self.map[self.yInMap, self.xInMap] = 1
        self.LookAround()

    def PerfectSee(self):
        
        view = self.field.GiveView(self)
        # Slice internal map based on robot's current direction
        match self.direction:
            case Direction.UP:
                mapSlice = self.map[self.yInMap - self.visibleDistance:self.yInMap,
                                     self.xInMap - self.visibleDistance:self.xInMap + self.visibleDistance + 1]
            case Direction.RIGHT:
                mapSlice = self.map[self.yInMap - self.visibleDistance:self.yInMap + self.visibleDistance + 1,
                                     self.xInMap + 1:self.xInMap + 1 + self.visibleDistance]
            case Direction.DOWN:
                mapSlice = self.map[self.yInMap + 1:self.yInMap + 1 + self.visibleDistance,
                                     self.xInMap - self.visibleDistance:self.xInMap + self.visibleDistance + 1]
            case Direction.LEFT:
                mapSlice = self.map[self.yInMap - self.visibleDistance:self.yInMap + self.visibleDistance + 1,
                                     self.xInMap - self.visibleDistance:self.xInMap]
        # Update unknown cells (-1) with view data
        mask = (mapSlice == -1)
        mapSlice[mask] = view[mask]
        self.plot.UpdateMap(self.map)

    def GetAstarPath(self, targetY, targetX, minDist):
        
        # A* algorithm for shortest path planning
        hCost = lambda pos: abs(targetY - pos[0]) + abs(targetX - pos[1])
        initPos = (self.yInMap, self.xInMap)
        queue = PriorityQueue()
        queue.put((hCost(initPos), [initPos]))  #(h_cost, log[])
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        checked = set(initPos)
        
        while not queue.empty():
            cost, path = queue.get()
            if max(cost, len(path)) > minDist:
                return None
            lastPos = path[-1]
            if lastPos == (targetY, targetX):
                return path
            for d in directions:
                newPos = (lastPos[0] + d[0], lastPos[1] + d[1])
                if newPos not in checked and self.map[newPos] == 1:
                    newCost = hCost(newPos) + len(path)
                    queue.put((newCost, path + [newPos]))
                    checked.add(newPos)
        return None

    def GetFrontierPath(self):

        p1 = self.map[1:-1, 1:-1]
        p2 = self.map[0:-2, 1:-1]
        p4 = self.map[1:-1, 2:]
        p6 = self.map[2:, 1:-1]
        p8 = self.map[1:-1, 0:-2]
        neighbors = np.stack([p2, p4, p6, p8], axis=0)
        mask = np.any(neighbors == -1, axis=0)

        points = np.argwhere((p1 == 1) & mask) + 1  
        if len(points) == 0:
            return None  
    
        grid = np.zeros(self.map.shape, dtype=bool)
        grid[points[:, 0], points[:, 1]] = True
    
        visited = np.zeros_like(grid, dtype=bool)
        clusters = []
    
        def dfs(x, y, grid, visited):
            cluster = []
            stack = [(x, y)]
            visited[x, y] = True
            rows, cols = grid.shape
        
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < rows and 0 <= ny < cols:
                        if grid[nx, ny] and not visited[nx, ny]:
                            visited[nx, ny] = True
                            stack.append((nx, ny))
                
                for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < rows and 0 <= ny < cols:
                        if grid[nx, ny] and not visited[nx, ny]:

                            ix1, iy1 = cx + dx, cy       
                            ix2, iy2 = cx, cy + dy       
                            valid = False
                            if 0 <= ix1 < rows and 0 <= iy1 < cols:
                                if self.map[ix1, iy1] == 1:
                                    valid = True
                            if 0 <= ix2 < rows and 0 <= iy2 < cols:
                                if self.map[ix2, iy2] == 1:
                                    valid = True
                            if valid:
                                visited[nx, ny] = True
                                stack.append((nx, ny))
            return cluster

    
        for x in range(grid.shape[0]):
            for y in range(grid.shape[1]):
                if grid[x, y] and not visited[x, y]:
                    new_cluster = dfs(x, y, grid, visited)
                    clusters.append(new_cluster)
    
        # Division cluster for preventing large scale
        final_clusters = []
        for cluster in clusters:
            if len(cluster) > self.visibleDistance:
                coords = np.array(cluster)

                num_subclusters = int(np.ceil(len(cluster) / self.visibleDistance))
                kmeans = KMeans(n_clusters=num_subclusters, random_state=0).fit(coords)
                labels = kmeans.labels_
                for label in range(num_subclusters):
                    subcluster = coords[labels == label]
                    final_clusters.append(subcluster.tolist())
            else:
                final_clusters.append(cluster)

        
        
            
            
        clusterCentroid = [cluster[int(len(cluster)/2)] for cluster in final_clusters]
        hCost = lambda pos: abs(self.yInMap - pos[0]) + abs(self.xInMap - pos[1])
        sortedCentroids = sorted(clusterCentroid, key=lambda pos: hCost(pos))
        minDist = self.map.shape[0] * self.map.shape[1]
        shortestPath = None
        for pos in sortedCentroids:
            path = self.GetAstarPath(pos[0], pos[1], minDist)
            if path is not None:
                if hCost(pos) == len(path) - 1:
                    return path
                if len(path) < minDist:
                    shortestPath = path
                    minDist = len(path)

        return shortestPath

    def FollowPath(self, path):
        for i in range(len(path) - 1):
            currentPos = path[i]
            nextPos = path[i + 1]
            dy = nextPos[0] - currentPos[0]
            dx = nextPos[1] - currentPos[1]
            if ((dy != 0 and dx != 0) or (dy == 0 and dx == 0)):
                print("Exception in path")
                exit(0)
            if dy == 1:
                nextDirection = Direction.DOWN
            elif dy == -1:
                nextDirection = Direction.UP
            elif dx == 1:
                nextDirection = Direction.RIGHT
            elif dx == -1:
                nextDirection = Direction.LEFT
            if nextDirection != self.direction:
                self.Turn(nextDirection)
            self.Move()

    def OneSearch(self):

        path = self.GetFrontierPath()
        if path is None:
            return False

        self.FollowPath(path)
        self.LookAround()
        return True

    def FullSearch(self):
        while self.OneSearch():
            pass

    def LookAround(self):
        self.Turn(Direction.UP)
        self.Turn(Direction.DOWN)
        self.Turn(Direction.RIGHT)
        self.Turn(Direction.LEFT)
