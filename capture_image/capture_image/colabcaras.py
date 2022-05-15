import cv2
import numpy as np

img = cv2.imread('jos.jpg')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Cargamos los clasificadores
face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_eye.xml')

# Detectamos caras
faces = face_cascade.detectMultiScale(img_gray, 1.1, 5)

# Para cada cara detectada, dibujamos un rectángulo
for (x,y,w,h) in faces:
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    roi_color = img[y:y+h, x:x+w]

cv2.imshow('Imagen', img)
key = cv2.waitKey(0)
cv2.destroyAllWindows()