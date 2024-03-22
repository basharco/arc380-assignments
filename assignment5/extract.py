import os

import numpy as np
import cv2

from ARC380_Assignment5_helper import capture_img


# VALUE FOR THRESHOLDING
THRESHOLD = 200 # 255 is white

# EPSILON FOR CONTOUR APPROXIMATION
EPS = 0.01

# THICKNESS FOR BOUNDING BOXES
LINE_THICKNESS = 2

# PADDING FOR SQUARE DETECTION
PADDING = 0.1

# FONT SIZE
FONTSCALE = 0.6

# SPACE BETWEEN TOP OF BOUNDING BOX AND LABEL
Y_DIST = 10


def extract_features(img: np.ndarray, path: str) -> list[dict[str, str | float | list[float]]]:
    """Extract features from an image.

    Args:
        img (np.ndarray): The image to extract features from.

    Returns:
        list[dict[str, str | float | np.ndarray]]: A list of dictionaryies--each dictionary belonging to a feature and
            containing relevant characterstic information. Values returned are "color", "shape", "size", "position",
            "orientation".

            Sample:
            [
                {
                    "color": [127, 50, 250],
                    "shape": "triangle",
                    "size": 50,
                    "position": [400, 110, 24],
                    "orientation": 45
                },
                {
                    "color": [255, 255, 255],
                    "shape": "rectangle",
                    "size": 100,
                    "position": [350, 180, 30],
                    "orientation": -1
                }
            ]
    """

    # convert image to grayscale
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, threshold_img = cv2.threshold(gray_img, THRESHOLD, 255, cv2.THRESH_BINARY)

    # 'contours' is a list containing groups of points. Each group outlines the boundaries (vertices) of a shape found
    # in the image.
    contours = cv2.findContours(threshold_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    features = []

    for i in range(1, len(contours)):
        contour = contours[i]
        approx_contour = cv2.approxPolyDP(contour, EPS * cv2.arclength(contour, True), True)
        moments = cv2.moments(approx_contour)

        # calculate contour's area/size
        size = moments['m00']

        # calculate contour's center/position
        center_x = int(moments['m10'] / size)
        center_y = int(moments['m01'] / size)

        # get color (BGR?) at center of contour
        color = img[center_y][center_x]

        # calculate and draw bounding box around contour
        x, y, w, h = cv2.boundingRect(approx_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), color, LINE_THICKNESS)

        shape = ""
        orientation = -1 # if contour is not a square, orientation is -1

        if len(contour) == 3:
            shape = "triangle"
        elif len(contour) == 4:
            ratio = float(w) / h

            if ratio >= 1 - PADDING and ratio <= 1 + PADDING:
                shape = "square"

                _, _, _, _, orientation = cv2.minAreaRect(approx_contour)
            else:
                shape = "rectangle"
        elif len(contour) == 5:
            shape = "pentagon"
        elif len(contour) == 6:
            shape = "hexagon"
        else:
            shape = "circle"

        features.append({
            "color": color,
            "shape": shape,
            "size": size, # in pixels?
            "position": [center_x, center_y], # in image coordinates?
            "orientation": orientation
        })

        cv2.putText(img, shape, (x, y - Y_DIST), cv2.FONT_HERSHEY_SIMPLEX, FONTSCALE, LINE_THICKNESS)
        cv2.imwrite(path, img)

        return features


if __name__ == '__main__':
    """Each time this script is executed, a new folder will be created inside of the './images' directory. This folder
        will contain two PNG files--the orignal captured image and the annotated version of the same image.
    """
    folder = os.path.join("./images", f"image-{len(os.listdir("./images")) + 1}")
    img = capture_img(save=True, str=os.path.join(folder, "original.png"))
    features = extract_features(img, os.path.join(folder, "annotated.png"))

    print(features)
