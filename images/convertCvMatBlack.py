#!/usr/bin/env python
import os
import cv2
import numpy as np





#base_path = "/home/laus/uni/thesis/TowelGripper/catkin_ws/src/pick_point_finder/images/"
base_path = "/home/laus/uni/thesis/TowelCam/images/test"
imgs = os.listdir(base_path)


grey_pixel = np.zeros((1,1,3), np.uint8)
grey_pixel = (153, 153, 153)
black_pixel = np.zeros((1,1,3), np.uint8)
black_pixel = (0, 0, 0)
# pixel_val = [153 153 153]
print(grey_pixel)

# print(img[0,0])

for i in range(len(imgs)):
    print(i)
    print(os.path.join(base_path, imgs[i]))
    img_path = os.path.join(base_path, imgs[i])
    img = cv2.imread(img_path)
    #cv2.imshow("test", img)
    #cv2.waitKey(0)
    h,w,_ = img.shape
    for j in range(h):
        for k in range(w):
            #print(img[j,k])
            #print(img[j,k] == grey_pixel)
            #print(k)
            if (img[j,k] == grey_pixel).all():
                #print("hello")
                img[j,k] = black_pixel
                #print(img[j,k])
    # cv2.imshow("test", img)
    # cv2.waitKey(0)
    cv2.imwrite(img_path, img)
