import time

import cv2
import glob
import os
from argparse import ArgumentParser
from find_edge import edge

class Filter:
    def filterImages(self, inputs, output):

        for image in inputs:
            fileBase = os.path.basename(image)
            noisy_image = cv2.imread(image)
            noisy_image = cv2.fastNlMeansDenoising(noisy_image, None, 80, 9, 21)
            image = cv2.cvtColor(noisy_image, cv2.COLOR_BGR2GRAY)
            se = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 40))
            bg = cv2.morphologyEx(image, cv2.MORPH_DILATE, se)
            out_gray = cv2.divide(image, bg, scale=255)
            out_binary = cv2.threshold(out_gray, 0, 255, cv2.THRESH_OTSU)[1]
            if not output == None:
                output_path = os.path.join(output, fileBase)
                cv2.imwrite(output_path, out_binary)
                print("filterd file:", fileBase)
                print(
                    "----------------------------------------------------------------------------------"
                )


def main():

    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        help="path to the floor plan of your world. (in .pgm , .jpg or .png format)",
        required=True,
    )

    parser.add_argument(
        "--output_path",
        action="store",
        dest="output_path",
        help="output path where the generated images should be stored.",
        required=True,
    )

    args = parser.parse_args()
    path = os.path.join(args.image_path, "*.png")
    inputs = glob.glob(path)
    Filter().filterImages(inputs, args.output_path)
    print('--------------------------------------------------------------------------')
    print('start removing unreachable areas')
   # time.sleep(10)
    edge(args.output_path)

if __name__ == "__main__":
    main()

# Arguments example
# --image_path
# /home/nilou/Schreibtisch/git/IAS_Naviprediction/data_augmentation_GAN/output
# --output_path
# /home/nilou/Schreibtisch/git/IAS_Naviprediction/data_augmentation_GAN/output/filtered
