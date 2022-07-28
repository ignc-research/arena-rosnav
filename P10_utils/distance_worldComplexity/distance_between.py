# import the necessary packages
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2
import math
import ntpath


class Distance:
    def midpoint(self, ptA, ptB):
        "finds center points of boxes"
        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

    def pixelVal(self, pix, r1, s1, r2, s2):

        if 0 <= pix and pix <= r1:
            return (s1 / r1) * pix
        elif r1 < pix and pix <= r2:
            return ((s2 - s1) / (r2 - r1)) * (pix - r1) + s1
        else:
            return ((255 - s2) / (255 - r2)) * (pix - r2) + s2

    def find_Contours(self, path):
        "find edges of objects"
        img = cv2.imread(path)
        # convert image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # define color ranges
        low_yellow = (0, 0, 0)
        high_yellow = (240, 251, 255)
        low_gray = (0, 0, 0)
        high_gray = (245, 243, 242)
        # create masks
        yellow_mask = cv2.inRange(img, low_yellow, high_yellow)
        gray_mask = cv2.inRange(img, low_gray, high_gray)
        # combine masks
        combined_mask = cv2.bitwise_or(yellow_mask, gray_mask)
        # plt.imshow(combined_mask)
        # plt.show()
        # blur = cv2.GaussianBlur(gray, (5, 5), 100)
        kern_size = 3
        gray_blurred = cv2.medianBlur(combined_mask, kern_size)
        # gray_blurred = cv2.GaussianBlur(combined_mask, (3, 3), 5)
        threshold_lower = 30
        threshold_upper = 220
        edged = cv2.Canny(gray_blurred, threshold_lower, threshold_upper)
        # cv2.imshow('edged',edged)

        (cnt, hierarchy) = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # cv2.drawContours(rgb, cnt, -1, (0, 255, 0), 2)
        # cv2.imshow('map5', rgb)
        # cv2.waitKey(0)

        return cnt, img

    def image_feat(self, path, width):
        """
		# load the image, convert it to grayscale, and blur it slightly
		"""
        head, tail = ntpath.split(path)
        cnts, image = self.find_Contours(path)
        normDist, var, minimum, average = self.find_boxes(cnts, width, image, tail)
        if not normDist == None:
            normDist = float("{0:.1f}".format(normDist))
        else:
            normDist = []
        if not var == None:
            var = float("{0:.5f}".format(var))
        else:
            var = []
        return normDist, var, minimum, average

    def find_boxes(self, cnts, width, image, tail):
        "find bounding boxes of objects and calculates distances"
        (cnts, _) = contours.sort_contours(cnts)
        refD = None
        countRef = 0
        boxesList = []
        minimum = []
        mean = []
        dif = []
        for boxes in cnts:
            box = cv2.minAreaRect(boxes)
            box = cv2.boxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            boxesList.append(box)

        for ref in boxesList:
            temp = []
            # ------------------------------------------------------------------
            # print('------------------------------------------------------------------------------------------------')
            # print('file name:', tail)
            # print('reference Object:', countRef)
            # ------------------------------------------------------------------
            (tl, tr, br, bl) = ref
            (tlblX, tlblY) = self.midpoint(tl, bl)
            (trbrX, trbrY) = self.midpoint(tr, br)
            D = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            D = 2.54 * D * 10
            cX = np.average(ref[:, 0])
            cY = np.average(ref[:, 1])
            refObj = (ref, (cX, cY))
            countBox = 0
            if refD == None:
                refD = D / width

            for c in cnts:  # loop over the contours individually
                # ------------------------------------------------------------------
                # print('reference Object:' + str(countRef) + ' to object:', str(countBox))
                # ------------------------------------------------------------------
                box = cv2.minAreaRect(c)
                box = cv2.boxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
                box = np.array(box, dtype="int")
                box = perspective.order_points(box)
                cX = np.average(box[:, 0])
                cY = np.average(box[:, 1])
                orig = image.copy()
                cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 1)
                cv2.drawContours(orig, [refObj[0].astype("int")], -1, (0, 255, 0), 1)
                refCoords = np.vstack([refObj[0], refObj[1]])
                objCoords = np.vstack([box, (cX, cY)])
                countBox += 1
                # loop over the original points
                cv2.circle(
                    orig,
                    (int(refCoords[4][0]), int(refCoords[4][1])),
                    3,
                    (240, 0, 159),
                    -1,
                )
                cv2.circle(
                    orig,
                    (int(objCoords[4][0]), int(objCoords[4][1])),
                    3,
                    (240, 0, 159),
                    -1,
                )
                cv2.line(
                    orig,
                    (int(refCoords[4][0]), int(refCoords[4][1])),
                    (int(objCoords[4][0]), int(objCoords[4][1])),
                    (240, 0, 159),
                    1,
                )
                if not refD == 0:

                    distNumpy = (
                        math.dist(
                            (refCoords[4][0], refCoords[4][1]),
                            (objCoords[4][0], objCoords[4][1]),
                        )
                        / refD
                    )
                    distNumpy = 2.54 * distNumpy
                    # ------------------------------------------------------------------
                    # print('distance with numpy: ', distNumpy  )
                    # ------------------------------------------------------------------
                    Distance = (
                        dist.euclidean(
                            (refCoords[4][0], refCoords[4][1]),
                            (objCoords[4][0], objCoords[4][1]),
                        )
                        / refD
                    )
                    DistanceCm = 2.54 * Distance
                    DistanceCm = int(DistanceCm * 10 ** 1) / 10.0 ** 3
                    (mX, mY) = self.midpoint(
                        (refCoords[4][0], refCoords[4][1]),
                        (objCoords[4][0], objCoords[4][1]),
                    )
                    cv2.putText(
                        orig,
                        str(DistanceCm),
                        (int(mX), int(mY - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (240, 0, 159),
                        2,
                    )
                    # ------------------------------------------------------------------
                    # print('distance with scipy: ',str(DistanceCm))
                    # ------------------------------------------------------------------

                    # imS = cv2.resize(orig, (900, 900))
                    # cv2.imshow('image', imS)
                    # cv2.waitKey(20000)
                    temp.append(distNumpy)
            arrayTemp = np.array(temp)
            is_all_zero = np.all((arrayTemp == 0))
            if not is_all_zero:
                # ------------------------------------------------------------------
                # print('minimum', np.min([value for value in temp if value!=0]))
                # ------------------------------------------------------------------
                mean.append(np.mean([value for value in temp if value != 0]))
                minimum.append(np.min([value for value in temp if value != 0]))
            else:
                mean = None
                minimum = None
            dif.append(np.diff(temp))
            countRef += 1
        if not mean == None:
            # ------------------------------------------------------------------
            # print('mean²', np.mean(mean))
            # print('variance', np.var(mean))
            # ------------------------------------------------------------------
            x = []
            for i in mean:
                if i <= np.mean(mean):
                    x.append(x)
            # ------------------------------------------------------------------
            # print('length of mean', len(mean))
            # print('number of points less than mean', len(x))
            # print('normalized value of distance', len(x)/len(mean))
            # ------------------------------------------------------------------
            normDist = len(x) / len(mean)
            var = np.var(mean)
            average = float("{:.5f}".format(np.mean(mean)))
        else:
            average = None
            var = None
            normDist = None
        if not minimum == None:
            min = np.min(minimum)
            # ------------------------------------------------------------------
            # print('min²', np.min(minimum))
            # ------------------------------------------------------------------
        else:
            min = None

        return normDist, var, min, average
