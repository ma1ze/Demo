import cv2

cap = cv2.VideoCapture(2)

print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  #set 3
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) #set 4
print(cap.get(cv2.CAP_PROP_FPS))  #set 5
print(cap.get(cv2.CAP_PROP_BRIGHTNESS)) #set 10
print(cap.get(cv2.CAP_PROP_CONTRAST)) #set 11
print(cap.get(cv2.CAP_PROP_SATURATION)) #set 12
print(cap.get(cv2.CAP_PROP_HUE)) #set 13
print(cap.get(cv2.CAP_PROP_EXPOSURE)) #set 15
#
# cap.set(3, 3000)
# cap.set(4, 3000)


while (cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()

#
# capture.set(CV_CAP_PROP_FRAME_WIDTH, 1080);//宽度
# capture.set(CV_CAP_PROP_FRAME_HEIGHT, 960);//高度
# capture.set(CV_CAP_PROP_FPS, 30);//帧数
# capture.set(CV_CAP_PROP_BRIGHTNESS, 1);//亮度 1
# capture.set(CV_CAP_PROP_CONTRAST,40);//对比度 40
# capture.set(CV_CAP_PROP_SATURATION, 50);//饱和度 50
# capture.set(CV_CAP_PROP_HUE, 50);//色调 50
# capture.set(CV_CAP_PROP_EXPOSURE, 50);//曝光 50
# */
# //打印摄像头参数
# printf("width = %.2f\n",capture.get(CV_CAP_PROP_FRAME_WIDTH));
# printf("height = %.2f\n",capture.get(CV_CAP_PROP_FRAME_HEIGHT));
# printf("fbs = %.2f\n",capture.get(CV_CAP_PROP_FPS));
# printf("brightness = %.2f\n",capture.get(CV_CAP_PROP_BRIGHTNESS));
# printf("contrast = %.2f\n",capture.get(CV_CAP_PROP_CONTRAST));
# printf("saturation = %.2f\n",capture.get(CV_CAP_PROP_SATURATION));
# printf("hue = %.2f\n",capture.get(CV_CAP_PROP_HUE));
# printf("exposure = %.2f\n",capture.get(CV_CAP_PROP_EXPOSURE));
