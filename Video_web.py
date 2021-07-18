#!/usr/bin/env python

# WS server that sends camera streams to a web server using opencv


import asyncio
import websockets
import cv2
from threading import Thread
import time
from aruco_image import *




async def time(websocket, path):
    while True:

        camera = True
        if camera == True:
            cap = cv2.VideoCapture(0)
            # cap.set(3, 1280)
            # cap.set(4, 720)
            # cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            # cap.set(5, 60)
        try:
            while (cap.isOpened()):
                img, frame = cap.read()
                result = aruco_image(frame)
                frame = cv2.resize(result.getImg(), (640, 480))
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                man = cv2.imencode('.jpg', frame, encode_param)[1]
                x=result.getX()
                y=result.getY()
                z=result.getZ()
                string_send = str(x)+"|"+str(y)+"|"+str(z)

                # websocket.setblocking(0)

                # message = websocket.recv()
                # message = message.decode('utf-8')
                # print("Data from html:", message)
                # try:
                # message = await websocket.recv()
                # except:
                #     message = None
                # print(message)

                await websocket.send(string_send)
                await websocket.send(man.tobytes())
                message = await websocket.recv()
                print("Data from html:", message)

        except:

            pass

# def deal(websocket,path1):
#     while True:
#         try:
#             while True:
#                 message = websocket.recv()
#                 print(message)
#         except:
#             pass



start_server = websockets.serve(time, "127.0.0.1", 9997)
# start_server1 = websockets.serve(deal, "127.0.0.1", 9997)

asyncio.get_event_loop().run_until_complete(start_server)
# asyncio.get_event_loop().run_until_complete(start_server1)

asyncio.get_event_loop().run_forever()

