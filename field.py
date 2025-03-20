import numpy as np
import math
from robot import Direction

def RoundHalfUp(n):
    # Round half up method
    if n >= 0:
        return math.floor(n + 0.5)
    else:
        return math.ceil(n - 0.5)

# wall: 0, path: 1

class FieldObject:
    def __init__(self, robot, yInField, xInField):
        self.robot = robot
        self.yInField = yInField
        self.xInField = xInField

class Field:
    def __init__(self, fieldArray):
        self.field = fieldArray.copy()
        self.robots = []

    def AppendRobot(self, robot, yInField, xInField):
        if self.field[yInField, xInField] != 1:
            return False
        newObject = FieldObject(robot, yInField, xInField)
        self.robots.append(newObject)
        robot.AssignField(self)
        return True

    def RobotMove(self, robot, y, x):
        # Find the robot object and update its field coordinates
        idx = next((idx for idx, obj in enumerate(self.robots) if obj.robot == robot), None)
        self.robots[idx].yInField += y
        self.robots[idx].xInField += x

    def ViewRecursion(self, robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, view, rayRad):
        # Recursive ray casting method
        y = 0
        x = 0
        realRayRad = rayRad + robot.direction.value
        lean = math.tan(realRayRad)

        # Escape range check
        if not (abs(rayRad) <= (robot.visibleAngle / 2)):
            return -1 * (robot.visibleAngle / 2)

        xDirect = int(np.sign(math.cos(realRayRad)))
        yDirect = int(-np.sign(math.sin(realRayRad))) #reverse y-sign between array index and coordinate

        if abs(lean) > 1:
            eStep = abs(1 / lean)
            if robot.direction in (Direction.RIGHT, Direction.LEFT):
                e = 0.5 * eStep
            if robot.direction in (Direction.UP, Direction.DOWN):
                e = 0.5 + eStep
            yMax = math.sin(realRayRad) * robot.visibleDistance
            
            while abs(y) < abs(yMax) - 1:
                if e <= 1:
                    e += eStep
                elif e > 1:
                    e = e - 1 + eStep
                    x += xDirect
                # Out of field check
                if not ((0 <= fieldOriginY + y < self.field.shape[0]) and (0 <= fieldOriginX + x < self.field.shape[1])):
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                # Detect wall and path
                if self.field[fieldOriginY + y, fieldOriginX + x] == 1:
                    view[viewOriginY + y, viewOriginX + x] = 1
                elif self.field[fieldOriginY + y, fieldOriginX + x] == 0:
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                y += yDirect
                if not ((0 <= fieldOriginY + y < self.field.shape[0]) and (0 <= fieldOriginX + x < self.field.shape[1])):
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                if self.field[fieldOriginY + y, fieldOriginX + x] == 1:
                    view[viewOriginY + y, viewOriginX + x] = 1
                elif self.field[fieldOriginY + y, fieldOriginX + x] == 0:
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                
        else:
            eStep = abs(lean)
            if robot.direction in (Direction.RIGHT, Direction.LEFT):
                e = 0.5 + eStep
            if robot.direction in (Direction.UP, Direction.DOWN):
                e = 0.5 * eStep
            xMax = abs(math.cos(rayRad)) * robot.visibleDistance
            while abs(x) < abs(xMax) - 1:
                if e <= 1:
                    e += eStep
                elif e > 1:
                    e = e - 1 + eStep
                    y += yDirect
                # Out of field check
                if not ((0 <= fieldOriginY + y < self.field.shape[0]) and (0 <= fieldOriginX + x < self.field.shape[1])):
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                # Detect wall and path
                if self.field[fieldOriginY + y, fieldOriginX + x] == 1:
                    view[viewOriginY + y, viewOriginX + x] = 1
                elif self.field[fieldOriginY + y, fieldOriginX + x] == 0:
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                x += xDirect
                if not ((0 <= fieldOriginY + y < self.field.shape[0]) and (0 <= fieldOriginX + x < self.field.shape[1])):
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
                if self.field[fieldOriginY + y, fieldOriginX + x] == 1:
                    view[viewOriginY + y, viewOriginX + x] = 1
                elif self.field[fieldOriginY + y, fieldOriginX + x] == 0:
                    return self.LookSideway(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad)
        return rayRad
        

    def LookSideway(self, robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, y, x, view, rayRad):
        # Process sideways view when an obstacle is hit
        if view[viewOriginY + y, viewOriginX + x] == -1:
            view[viewOriginY + y, viewOriginX + x] = 0
        else:
            return rayRad

        yDirect = 1 if y >= 0 else -1
        xDirect = 1 if x >= 0 else -1

        match robot.direction:
            case Direction.RIGHT:
                rad1 = -math.atan((y + 0.50001 * yDirect) / x) if x != 0 else -(np.deg2rad(90)) * yDirect
                rad2 = -math.atan((y - 0.50001 * yDirect) / (x + xDirect))
            case Direction.LEFT:
                rad1 = -math.atan((y + 0.50001 * yDirect) / x) if x != 0 else -(np.deg2rad(90)) * yDirect
                rad2 = -math.atan((y - 0.50001 * yDirect) / (x + xDirect)) + np.deg2rad(180)
            case Direction.UP:
                rad1 = -math.atan2(y, (x + 0.50001 * xDirect)) - np.deg2rad(90)
                rad2 = -math.atan2((y + yDirect), (x - 0.50001 * xDirect)) - np.deg2rad(90)
            case Direction.DOWN:
                rad1 = -math.atan2(y, (x + 0.50001 * xDirect)) + np.deg2rad(90)
                rad2 = -math.atan2((y + yDirect), (x - 0.50001 * xDirect)) + np.deg2rad(90)
 
        lowLimit1 = self.ViewRecursion(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, view, rad1)
        lowLimit2 = self.ViewRecursion(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, view, rad2)
        return max(lowLimit1, lowLimit2)

    def GiveView(self, robot):
        idx = next((idx for idx, obj in enumerate(self.robots) if obj.robot == robot), None)
        #Generate View, fiiled with field info
        match robot.direction:
            case Direction.RIGHT:
                view = np.full((robot.visibleDistance * 2 + 1, robot.visibleDistance), -1, dtype=np.int8)
                viewOriginY, viewOriginX = robot.visibleDistance, 0
                fieldOriginY = self.robots[idx].yInField
                fieldOriginX = self.robots[idx].xInField + 1
            case Direction.UP:
                view = np.full((robot.visibleDistance, robot.visibleDistance * 2 + 1), -1, dtype=np.int8)
                viewOriginY, viewOriginX = robot.visibleDistance - 1, robot.visibleDistance
                fieldOriginY = self.robots[idx].yInField - 1
                fieldOriginX = self.robots[idx].xInField
            case Direction.LEFT:
                view = np.full((robot.visibleDistance * 2 + 1, robot.visibleDistance), -1, dtype=np.int8)
                viewOriginY, viewOriginX = robot.visibleDistance, robot.visibleDistance - 1
                fieldOriginY = self.robots[idx].yInField
                fieldOriginX = self.robots[idx].xInField - 1
            case Direction.DOWN:
                view = np.full((robot.visibleDistance, robot.visibleDistance * 2 + 1), -1, dtype=np.int8)
                viewOriginY, viewOriginX = 0, robot.visibleDistance
                fieldOriginY = self.robots[idx].yInField + 1
                fieldOriginX = self.robots[idx].xInField

        startRad = -1 * (robot.visibleAngle / 2)
        lowLimit = -100
        for r in range(robot.rayCount):
            rayRad = startRad + r * robot.rayAngle
            if rayRad > lowLimit:
                leastRad = self.ViewRecursion(robot, fieldOriginY, fieldOriginX, viewOriginY, viewOriginX, view, rayRad)
                lowLimit = max(leastRad, lowLimit)
        return view.copy()
