from audioop import minmax
import cv2
import numpy as np

img = cv2.imread('figuras.jpg')
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

lower_blue = np.array([36, 25, 25])
upper_blue = np.array([86, 255, 255])

mask = cv2.inRange(hsv, lower_blue, upper_blue)

res = cv2.bitwise_and(img, img, mask= mask)
min_x = 1000
max_x = 0
min_y = 1000
max_y = 0

for i in range(res.shape[0]):
    for j in range(res.shape[1]):
        pixel = res[i,j,:]
        if pixel[0] > 0:
            if pixel[1] > 0:
                if pixel[2] > 0:
                    if i > max_x:
                        max_x = i
                    if i < min_x:
                        min_x = i
                    if j > max_y:
                        max_y = j
                    if j < min_y:
                        min_y = j   

color = (0,0,255)
x1 = min_y
y1 = max_x
x2 = max_y
y2 = min_x

img_res = cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)

cv2.imshow('Verde detectado',img_res)
cv2.waitKey(0) #aprieta una tecla 
cv2.destroyAllWindows()
