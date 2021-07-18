
import asyncio
import websockets
import threading
import cv2
import time
from enum import Enum
from Detector import Detector
import numpy as np

class Mode(Enum):
    start = 0
    tracker = 1
    posShowing = 2
    gameMode = 3
    stop = 4

# global mode
mode = Mode.start
string = 'Null'

# def controller(mode):
async def time1(websocket, path):
    global mode,string
    while True:
        message = await websocket.recv()
        print("Data from html:", message)
        order = message.split("|")
        if order[0] == "camera" and order[1] == "on":
            mode = Mode.tracker
        elif order[0] == "posShow" and order[1] == "on":
            mode = Mode.posShowing
        elif order[0] =="stop" and order[1]=="on":
            mode = Mode.stop
        elif order[0] =="game" and order[1]=="on":
            mode = Mode.gameMode



def Controller():
    global mode,string
    detector = Detector(debug=True, Blur=False, Sharpen=False)
    while True:
        print("mode",mode)
        if mode == Mode.tracker:
            print("Tracker working")
            # detector = Detector(debug=True, Blur=False, Sharpen=False)
            id_count = np.zeros(50)
            while True:
                tag_list = detector.get_tag_list()
                for e in tag_list:
                    id_count[e._id] += 1

                if mode == Mode.stop:
                    cv2.destroyAllWindows()
                    mode = Mode.start
                    break

        elif mode == Mode.posShowing:
            print("posShowing working")
            # detector = Detector(debug=False, Blur=False, Sharpen=False)
            id_count = np.zeros(50)
            while True:
                tag_list = detector.get_tag_list()
                for e in tag_list:
                    id_count[e._id] += 1
                for tag in tag_list:
                    string = str(tag.world_position[0])+"|"+str(tag.world_position[1])+"|"+str(tag.world_position[2])
                    print(string)

                    # await websocket.send(string)
                if mode == Mode.stop:
                    cv2.destroyAllWindows()
                    mode = Mode.start
                    break

        # elif mode == Mode.gameMode:
        #
        #
        # elif mode == Mode.stop:
        #     print("Stopping......")







        # if mode == Mode.posShowing:
        #     await websocket.send(string)
    # mode chance
    # if mode == Mode.tracker:
    #     print("Tracker working")
    #     detector = Detector(debug=True, Blur=False, Sharpen=False)
    #     id_count = np.zeros(50)
    #     while True:
    #         tag_list = detector.get_tag_list()
    #         for e in tag_list:
    #             id_count[e._id] += 1
    #
    #         if mode == Mode.stop:
    #             break
    #
    #
    #
    # elif mode == Mode.posShowing:
    #     print("posShowing working")
    #     detector = Detector(debug=False, Blur=False, Sharpen=False)
    #     id_count = np.zeros(50)
    #     while True:
    #         tag_list = detector.get_tag_list()
    #         for e in tag_list:
    #             id_count[e._id] += 1
    #         for tag in tag_list:
    #             string = str(tag.world_position[0])+"|"+str(tag.world_position[1])+"|"+str(tag.world_position[2])
    #             print(string)
    #             await websocket.send(string)
    #
    #
    # elif mode == Mode.stop:
    #     print("Stopping......")



start_server = websockets.serve(time1, "127.0.0.1", 9996)
threading.Thread(target=Controller).start()
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
# loop = asyncio.get_event_loop()
# tasks = [printMode()]
# loop.run_until_complete()
# print('1')