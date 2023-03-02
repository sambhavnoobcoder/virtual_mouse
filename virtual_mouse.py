import mediapipe as mp
import cv2
import pyautogui
import math
import numpy as np
import autopy
import time

class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=False, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def findHands(self, img, draw=True):    # Finds all hands in a frame
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)

        return img

    def findPosition(self, img, handNo=0, draw=True):   # Fetches the position of hands
        xList = []
        yList = []
        bbox = []
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax

            if draw:
                cv2.rectangle(img, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20),
                              (0, 255, 0), 2)

        return self.lmList, bbox

    def fingersUp(self):    # Checks which fingers are up
        fingers = []
        # Thumb
        if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # Fingers
        for id in range(1, 5):

            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        # totalFingers = fingers.count(1)

        return fingers

    def findDistance(self, p1, p2, img, draw=True,r=15, t=3):   # Finds distance between two fingers
        x1, y1 = self.lmList[p1][1:]
        x2, y2 = self.lmList[p2][1:]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
            cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)

        return length, img, [x1, y1, x2, y2, cx, cy]



# iny=0
# capture=cv2.VideoCapture(0)
# hand = mp.solutions.hands.Hands()
# drawing_utils=mp.solutions.drawing_utils
# screen_width,screen_height= pyautogui.size()
# fpsLimit = 1
# while True:
#     _,frames=capture.read()     #all frames that are read from cature vaiablr from the camera
#     frames=cv2.flip(frames,1)
#     frames_height,frames_width,_=frames.shape
#     rgb_color = cv2.cvtColor(frames , cv2.COLOR_BGR2RGB)
#     output = hand.process(rgb_color)
#     handy = output.multi_hand_landmarks
#     # print(handy)
#     if handy:
#         for h in handy:
#             drawing_utils.draw_landmarks(frames,h)
#             landmarks=h.landmark
#             for id,landmark in enumerate(landmarks):
#                 x=int(landmark.x*frames_width)
#                 y=int(landmark.y*frames_height)
#                 print(x,y)
#
                # if id ==8:
                #     cv2.circle(img=frames,center=(x,y),radius=10,color=(255,0,255))
                #     inx= screen_width/frames_width*x
                #     iny=screen_height/frames_height*y
                #     pyautogui.moveTo(inx,iny)
                # if id ==12:
                #     cv2.circle(img=frames,center=(x,y),radius=10,color=(255,0,255))
                #     thx= screen_width/frames_width*x
                #     thy=screen_height/frames_height*y
                #     if abs(thy - iny )<30:
                #         pyautogui.click()
                #         pyautogui.sleep(1)
                #         # print("click")
#     cv2.imshow("virtual mouse",frames)
#     cv2.waitKey(1)



### Variables Declaration
pTime = 0               # Used to calculate frame rate
width = 780            # Width of Camera
height = 640            # Height of Camera
frameR = 100            # Frame Rate
smoothening = 8         # Smoothening Factor
prev_x, prev_y = 0, 0   # Previous coordinates
curr_x, curr_y = 0, 0   # Current coordinates

cap = cv2.VideoCapture(0)   # Getting video feed from the webcam
cap.set(3, width)           # Adjusting size
cap.set(4, height)

detector = handDetector(maxHands=1)                  # Detecting one hand at max
screen_width, screen_height = autopy.screen.size()      # Getting the screen size
while True:
    success, img = cap.read()
    img=cv2.flip(img,1)
    img = detector.findHands(img)                       # Finding the hand
    lmlist, bbox = detector.findPosition(img)           # Getting position of hand

    if len(lmlist)!=0:
        x1, y1 = lmlist[8][1:]
        x2, y2 = lmlist[12][1:]

        fingers = detector.fingersUp()      # Checking if fingers are upwards
        cv2.rectangle(img, (frameR, frameR), (width - frameR, height - frameR), (255, 0, 255), 2)   # Creating boundary box
        if fingers[1] == 1 and fingers[2] == 0:     # If fore finger is up and middle finger is down
            x3 = np.interp(x1, (frameR,width-frameR), (0,screen_width))
            y3 = np.interp(y1, (frameR, height-frameR), (0, screen_height))

            curr_x = prev_x + (x3 - prev_x)/smoothening
            curr_y = prev_y + (y3 - prev_y) / smoothening

            autopy.mouse.move(curr_x, curr_y)    # Moving the cursor
            cv2.circle(img, (x1, y1), 7, (255, 0, 255), cv2.FILLED)
            prev_x, prev_y = curr_x, curr_y

        if fingers[1] == 1 and fingers[2] == 1:     # If fore finger & middle finger both are up
            length, img, lineInfo = detector.findDistance(8, 12, img)
            dbc,img,lineinfo=detector.findDistance(8,4,img) #if index finger and thumb are close initiate double click

            if length < 40:     # If both fingers are really close to each other
                cv2.circle(img, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                autopy.mouse.click()    # Perform Click
                # autopy.sleep(1)
                print ("click 2 ")
                time.sleep(.5)

            if dbc<40:
                cv2.circle(img, (lineinfo[3],lineinfo[5]),15,(255,0,0),cv2.FILLED)
                # autopy.mouse.click(autopy.mouse.Button.RIGHT)
                # autopy.mouse.click()
                # time.sleep(.5)
                # autopy.mouse.click()
                pyautogui.hotkey('command','o')
                time.sleep(.7)
                # pyautogui.click(curr_x,curr_y,clicks=2)
                # pyautogui.doubleClick()
                # pyautogui.doubleclick()
                print("click for double click")
                # time.sleep(.7)


    cTime = time.time()
    fps = 1/(cTime-pTime)
    pTime = cTime
    cv2.putText(img, str(int(fps)), (20, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
