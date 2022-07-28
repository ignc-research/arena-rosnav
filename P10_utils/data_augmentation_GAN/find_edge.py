
import numpy as np
import cv2
import os
import glob
from pathlib import Path

def edge(input):
    globPath = os.path.join(input, "*.png")
    input = glob.glob(globPath)
    for file in input:
        fileBase = os.path.basename(file)
        image = cv2.imread(file)
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        et, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)
        #cv2.imshow('Binary image', thresh)
        contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        bigger = max(contours, key=lambda item: cv2.contourArea(item))
        the_mask = np.zeros_like(image)
        cv2.drawContours(the_mask, [bigger], -1, (255, 255, 255), cv2.FILLED)
        res = image.copy()
        res[the_mask == 0] = 0
       # output_path = os.path.join(output, fileBase)
        cv2.imwrite( os.path.normpath(file), res)
        #cv2.imwrite(fileBase, res)
        print('--------------------------------------------------------------------------')
        print('file to remove unreachable area: ',fileBase )
#
