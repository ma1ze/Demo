
import asyncio
import websockets
import cv2
import time
from enum import Enum
from Detector import Detector
import numpy as np
import threading
import sys
import os
class Mode(Enum):
    tracker = 1
    posShowing = 2
    stop = 3

def restart_program():
  python = sys.executable
  os.execl(python, python, * sys.argv)

mode = 0



# def controller(mode):
def printMode():
    global mode
    while True:
        print(mode)
        time.sleep(1)

async def time1(websocket, path):
    while True:
        global mode
        message = await websocket.recv()
        print("Data from html:", message)
        # print(mode)
        order = message.split("|")
        if order[0] == "camera" and order[1] == "on":
            mode = Mode.tracker
        elif order[0] == "posShow" and order[1] == "on":
            mode = Mode.posShowing
        elif order[0] =="stop" and order[1]=="on":
            mode = Mode.stop
        # mode chance

        if mode == Mode.tracker:
            print("Tracker working")
            detector = Detector(debug=True, Blur=False, Sharpen=False)
            id_count = np.zeros(50)
            while True:
                tag_list = detector.get_tag_list()
                for e in tag_list:
                    id_count[e._id] += 1

                if mode == Mode.stop:
                    break

        elif mode == Mode.posShowing:
            print("posShowing working")
            detector = Detector(debug=False, Blur=False, Sharpen=False)
            id_count = np.zeros(50)
            while True:
                tag_list = detector.get_tag_list()
                for e in tag_list:
                    id_count[e._id] += 1
                for tag in tag_list:
                    string = str(tag.world_position[0])+"|"+str(tag.world_position[1])+"|"+str(tag.world_position[2])
                    print(string)
                    await websocket.send(string)

        elif mode == Mode.stop:
            print("restarting....")
            restart_program()





start_server = websockets.serve(time1, "127.0.0.1", 9997)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()


########################
#
# import asyncio
# import websockets
# import cv2
# import time
# from enum import Enum
# from Detector import Detector
# import numpy as np
# import threading
#
# class Mode(Enum):
#     tracker = 1
#     posShowing = 2
#     stop = 3
#
# mode = 0
#
#
# # def controller(mode):
#
#
#
#
#
# async def time(websocket, path):
#     while True:
#         message = await websocket.recv()
#         print("Data from html:", message)
#         order = message.split("|")
#         if order[0] == "camera" and order[1] == "on":
#             mode = Mode.tracker
#         elif order[0] == "posShow" and order[1] == "on":
#             mode = Mode.posShowing
#         elif order[0] =="stop" and order[1]=="on":
#             mode = Mode.stop
#         # mode chance
#         if mode == Mode.tracker:
#             print("Tracker working")
#             detector = Detector(debug=True, Blur=False, Sharpen=False)
#             id_count = np.zeros(50)
#             while True:
#                 tag_list = detector.get_tag_list()
#                 for e in tag_list:
#                     id_count[e._id] += 1
#
#                 if mode == Mode.stop:
#                     break
#
#
#
#         elif mode == Mode.posShowing:
#             print("posShowing working")
#             detector = Detector(debug=False, Blur=False, Sharpen=False)
#             id_count = np.zeros(50)
#             while True:
#                 tag_list = detector.get_tag_list()
#                 for e in tag_list:
#                     id_count[e._id] += 1
#                 for tag in tag_list:
#                     string = str(tag.world_position[0])+"|"+str(tag.world_position[1])+"|"+str(tag.world_position[2])
#                     print(string)
#                     await websocket.send(string)
#
#
#         # elif mode == Mode.stop:
#
#
#
#
# start_server = websockets.serve(time, "127.0.0.1", 9997)
# asyncio.get_event_loop().run_until_complete(start_server)
# asyncio.get_event_loop().run_forever()

