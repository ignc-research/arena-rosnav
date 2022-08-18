import cv2
import os
import math

maps = os.listdir("maps")
map_test = ""
for map_name in maps:
    image_path = f"maps/{map_name}/{map_name}.png"

    img = cv2.imread(image_path)
    height, width, channels = img.shape
    
    print(f"Map Name: {map_name}")
    map_test = map_name
    
    print(f"Height: {height} | Width: {width} | Channels: {channels}")
    width_padding = 150 - width
    height_padding = 150 - height
    image = cv2.copyMakeBorder(img, height_padding, 0, width_padding, 0, cv2.BORDER_CONSTANT)
    
    cv2.imwrite(image_path, image)
    
    print("After change:")
    img = cv2.imread(image_path)
    height, width, channels = img.shape

    print(f"Height: {height} | Width: {width} | Channels: {channels}")
    print()