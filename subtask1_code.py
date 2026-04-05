
import cv2
import numpy as np

cap = cv2.VideoCapture('OPTICAL_FLOW.mp4')

ret, frame1 = cap.read() #reads the first frame, ret= true if sucesfil, frame 1 is the avtual img data
if not ret:
    print("cant read video")
    exit()

gray1= cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY) #converts to grayscale, optical flow needs only brightenss not colour

corners= cv2.goodFeaturesToTrack(gray1, 100, 0.01, 7) #10 good feutre points, dist of 7, 0.01 min quality
corners= corners.reshape(-1, 2) #-1; how many ever rpws, 2: 2 columns, array with each row one point x,y

fps= cap.get(cv2.CAP_PROP_FPS) #gets the videos frames per second
if fps == 0 or np.isnan(fps):
    fps= 30 #if cant get, take 30

width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) #get the input video dimensions
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

out= cv2.VideoWriter( #output video with same dimentsions
    'foutput.mp4',
    cv2.VideoWriter_fourcc(*'mp4v'),
    int(fps),
    (width, height)
)

frame_count= 0
max_frames= 600

while frame_count < max_frames: #runs for unpto 600 frames

    ret, frame2= cap.read() #reads next frame from video
    if not ret: #if no more frames sstop loop
        break

    frame_count +=1
    print("Frame:", frame_count)

    gray2= cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY) #grayscale

    next_corners =[] #empty list to collect where points moved too

    #increase visibility, makes arrow bigger nad more visible
    scale= 15

    for corner in corners: #loops through every tracked feature point

        x, y = int(corner[0]), int(corner[1]) #x and y cordinated of this corner point

        win= 15 #window size- 15, 15 border aeound each point, so actual patch is 30 cross 30
        if (x-win< 0 or y-win< 0 or
            x+win>= gray1.shape[1] or y+win>= gray1.shape[0]):
            continue

        patch1= gray1[y-win:y+win, x-win:x+win]
        patch2= gray2[y-win:y+win, x-win:x+win]

        flow= cv2.calcOpticalFlowFarneback(
            patch1, patch2, None,
            0.5, 1, 15, 2, 5, 1.1, 0
        )

        dx= np.mean(flow[..., 0])
        dy= np.mean(flow[..., 1])

        x2= x + dx
        y2= y + dy

        magnitude= np.sqrt(dx**2 + dy**2)

        #low threshold
        if magnitude>0.3:

            #color based on motion
            if magnitude < 1:
                color= (255, 0, 0)
            elif magnitude < 3:
                color= (0, 255, 0)
            else:
                color= (0, 0, 255)

            p1= (x, y)
            p2= (int(x + scale*dx), int(y + scale*dy))

            cv2.arrowedLine(frame2, p1, p2, color, 2, tipLength=0.4)

            next_corners.append([x2, y2])

    #re-detect if lost
    if len(next_corners)< 20:
        corners=cv2.goodFeaturesToTrack(gray2, 100, 0.01, 7)
        if corners is not None:
            corners = corners.reshape(-1, 2)
        else:
            continue
    else:
        corners=np.array(next_corners)

    out.write(frame2)
    gray1=gray2.copy()

cap.release()
out.release()

print("saved as output.mp4")
