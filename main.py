import robot
import field
from plot import Plot



import numpy as np
import cv2


IMG_DIR = r'.\sample_img\4.PNG'

if __name__ == '__main__':
    
    grayimg = cv2.imread(IMG_DIR, cv2.IMREAD_GRAYSCALE)
    field_array = (grayimg > 125).astype(np.int8)

    
    field_array = np.array(field_array)
    
    robot = robot.Robot()
    field = field.Field(field_array)
    
    plot = Plot(field.field, robot)     
    
    robot.plot = plot
    
    
    for i, j in np.ndindex(field.field.shape):
        if field.AppendRobot(robot, i, j):
            break
    robot.PerfectSee()
    robot.FullSearch()
    
    print("End")
    plot.End()
    