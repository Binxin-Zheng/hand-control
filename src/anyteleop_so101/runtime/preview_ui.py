"""Vision-environment UI only; transforms presentation, never detector geometry."""
import cv2
import numpy as np


def hand_view(image, pixels, size=512):
    # Full original camera frame, letterboxed only to fit the pane.
    # Skeleton is drawn in original pixels before this uniform display resize.
    h,w=image.shape[:2]
    scale=min(size/w,size/h)
    resized=cv2.resize(image,(round(w*scale),round(h*scale)))
    result=np.full((size,size,3),(35,28,24),np.uint8)
    y=(size-resized.shape[0])//2;x=(size-resized.shape[1])//2
    result[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
    return result


def compose(robot_rgba, hand, opening, ratio, status, max_angle_deg=100., joint_angle_rad=None):
    canvas=np.full((650,1060,3),(24,20,17),np.uint8)
    left=np.full((512,512,3),(35,28,24),np.uint8) if robot_rgba is None else cv2.cvtColor(robot_rgba,cv2.COLOR_RGBA2BGR)
    canvas[65:577,12:524]=left;canvas[65:577,536:1048]=hand
    def text(label,xy,scale=.65,color=(220,225,230)):
        cv2.putText(canvas,label,xy,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
    text('SO101 GRIPPER',(25,37),.7);text('CAMERA',(550,37),.7)
    text(status,(550,605),.6,(160,225,180))
    text('Holding' if opening is None else f'{np.degrees(joint_angle_rad) if joint_angle_rad is not None else opening*max_angle_deg:.0f} deg  /  {opening*100:.0f}% open',(25,607),.8)
    cv2.rectangle(canvas,(25,624),(500,632),(65,55,45),-1)
    if opening is not None:cv2.rectangle(canvas,(25,624),(25+int(475*opening),632),(130,210,150),-1)
    text('Thumb = moving jaw | palm/fingers = fixed | Q quit',(550,634),.43,(140,150,160))
    return canvas
