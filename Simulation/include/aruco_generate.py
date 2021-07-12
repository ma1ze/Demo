import os


# Specify the output directory (the folder name if it's under the current directory)
output_dir = '/home/'

import cv2
import cv2.aruco as aruco
import numpy as np


class CircularTag(object):
    def __init__(self, aruco_id, aruco_dict=aruco.DICT_4X4_50, \

                 id_color=(240, 176, 0), \



                 id_pos=(0.5, 0.9), \
                 id_font=cv2.FONT_HERSHEY_SIMPLEX, \
                 id_scale=1.2, \
                 id_thickness=2):
        # ArUco information
        self.aruco_dict = aruco.Dictionary_get(aruco_dict)
        self.aruco_id = aruco_id

        # Color information

        self.id_color = id_color  # color of the id number in the bottom of the image


        # Geometry information

        self.id_pos = id_pos  # Center of the id number text in the image, in form of the fraction
        self.id_font = id_font
        self.id_scale = id_scale
        self.id_thickness = id_thickness

        # Pixel size information
        self._circle_diameter = 500  # the pixel diameter of the circle (as well as the overall height/width)
        self._aruco_size = int((1.5 / 2.5) * self._circle_diameter)  # pixel length of ArUco marker
        self._circle_thickness = 2



        self._text1_offset = 0.025
        self._text2_offset = 0.045
        if self.aruco_id < 10:
            self._text_pos = (int(self.id_pos[0] * self._circle_diameter - self._text1_offset * self._circle_diameter),
                              int(self.id_pos[1] * self._circle_diameter))
        else:
            self._text_pos = (int(self.id_pos[0] * self._circle_diameter - self._text2_offset * self._circle_diameter),
                              int(self.id_pos[1] * self._circle_diameter))

    def generateImage(self):
        '''
        According to the parameters, generate and return
        the image in cv2 format of white background
        '''
        # Generate a white background image
        back_image = np.full((self._circle_diameter, self._circle_diameter, 3), 255, dtype=np.uint8)
        # Draw the inscribed circle
        circle_radius = self._circle_diameter / 2
        circle_center = (int(circle_radius), int(circle_radius))


        aruco_image = aruco.drawMarker(self.aruco_dict, self.aruco_id, self._aruco_size, back_image)
        # Convert gray image to color (BGR) image
        aruco_image = cv2.cvtColor(aruco_image, cv2.COLOR_GRAY2BGR)

        # Put aruco_image on the center of the background image
        top_left = int((self._circle_diameter - self._aruco_size) / 2)
        right_bottom = int((self._aruco_size + self._circle_diameter) / 2)
        back_image[top_left:right_bottom, top_left:right_bottom] = aruco_image

        # Draw the arrow on the top of ArUco code


        # put the id text on the bottom of ArUco code
        cv2.putText(back_image, str(self.aruco_id), self._text_pos, self.id_font, self.id_scale, self.id_color,
                    self.id_thickness)

        return back_image


if __name__ == "__main__":
    for _id in range(0, 50):
        marker = CircularTag(_id)
        image = marker.generateImage()
        image_name = "{:0>2d}.png".format(_id)
        image_path = os.path.join(output_dir, image_name)
        cv2.imwrite(image_path, image)

