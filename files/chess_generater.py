import cv2 
import numpy as np

n_w = 8
n_h = 6
length = 100

width = (n_w+1)*100
height = (n_h+1)*100


backimg = np.ones((width+2*length,width+2*length),dtype = np.uint8)*255

image = np.zeros((width,height),dtype = np.uint8)

for j in range(height):
    for i in range(width):
        if((int)(i/length) + (int)(j/length))%2:
            image[i,j] = 255;

backimg[length:width+length, length*2:width] = image

cv2.imwrite("./chess.jpg",backimg)
#cv2.imshow("chess",image)
#cv2.waitKey(0)
