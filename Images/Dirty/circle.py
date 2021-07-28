# 노즐 홈 중심 찾기

import cv2
import numpy as np

img = cv2.imread('test.jpg')
# img = cv2.imread('test2.jpg')
# img = cv2.imread('test3.jpg')
# img = cv2.imread('test4.jpg')
# img = cv2.imread('dirty.jpg')
# img = cv2.imread('dirty2.jpg')
# img = cv2.imread('scale.jpg')

# Non-local Meaning Denoising
nmd = cv2.fastNlMeansDenoisingColored(img, dst=None, h=20, hColor=20, templateWindowSize=7, searchWindowSize=21)
cv2.imshow('NMD',nmd)

# color to grayscale
gray = cv2.cvtColor(nmd, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray', gray) 

# Thresholding
ret, thres = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
cv2.imshow('threshold', thres) 


# Hough Circle Transform
circles = cv2.HoughCircles(thres, cv2.HOUGH_GRADIENT, dp=1, minDist=3000, param1=100, param2=10, minRadius=5, maxRadius=20)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # 원 둘레에 초록색 원 그리기
        cv2.circle(img, (i[0], i[1]), i[2], (0,255,0), 2)
        # 원 중심점에 빨강색 원 그리기
        cv2.circle(img, (i[0], i[1]), 2, (0,0,255), 3)

text = "Center : X {0}, Y {1}, Radius {2}".format(circles[0][0][0], circles[0][0][1], circles[0][0][2])
cv2.putText(img, text, (circles[0][0][0], circles[0][0][1]+40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)

cv2.imshow('nozzle center', img)
cv2.waitKey(0)
cv2.destroyAllWindows()