import mediapipe as mp
import cv2
import pyautogui
iny=0
capture=cv2.VideoCapture(0)
hand = mp.solutions.hands.Hands()
drawing_utils=mp.solutions.drawing_utils
screen_width,screen_height= pyautogui.size()
fpsLimit = 1
while True:
    _,frames=capture.read()     #all frames that are read from cature vaiablr from the camera
    frames=cv2.flip(frames,1)
    frames_height,frames_width,_=frames.shape
    rgb_color = cv2.cvtColor(frames , cv2.COLOR_BGR2RGB)
    output = hand.process(rgb_color)
    handy = output.multi_hand_landmarks
    # print(handy)
    if handy:
        for h in handy:
            drawing_utils.draw_landmarks(frames,h)
            landmarks=h.landmark
            for id,landmark in enumerate(landmarks):
                x=int(landmark.x*frames_width)
                y=int(landmark.y*frames_height)
                print(x,y)

                if id ==8:
                    cv2.circle(img=frames,center=(x,y),radius=10,color=(255,0,255))
                    inx= screen_width/frames_width*x
                    iny=screen_height/frames_height*y
                    pyautogui.moveTo(inx,iny)
                # if id ==12:
                #     cv2.circle(img=frames,center=(x,y),radius=10,color=(255,0,255))
                #     thx= screen_width/frames_width*x
                #     thy=screen_height/frames_height*y
                #     if abs(thy - iny )<30:
                #         pyautogui.click()
                #         pyautogui.sleep(1)
                #         print("click")
    cv2.imshow("virtual mouse",frames)
    cv2.waitKey(1)
