
import cv2
import numpy as np

cap = cv2.VideoCapture('OPTICAL_FLOW.mp4')

ret, frame1 = cap.read()
if not ret:
    print("Cannot read video")
    exit()

gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

# 🔥 MORE points = better coverage
corners = cv2.goodFeaturesToTrack(gray1, 100, 0.01, 7)
corners = corners.reshape(-1, 2)

# Video writer
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0 or np.isnan(fps):
    fps = 30

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

out = cv2.VideoWriter(
    'foutput.mp4',
    cv2.VideoWriter_fourcc(*'mp4v'),
    int(fps),
    (width, height)
)

frame_count = 0
max_frames = 300

while frame_count < max_frames:

    ret, frame2 = cap.read()
    if not ret:
        break

    frame_count += 1
    print("Frame:", frame_count)

    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    next_corners = []

    # 🔥 Increase visibility
    scale = 15

    for corner in corners:

        x, y = int(corner[0]), int(corner[1])

        # small window LK (simplified)
        win = 15
        if (x-win < 0 or y-win < 0 or
            x+win >= gray1.shape[1] or y+win >= gray1.shape[0]):
            continue

        patch1 = gray1[y-win:y+win, x-win:x+win]
        patch2 = gray2[y-win:y+win, x-win:x+win]

        flow = cv2.calcOpticalFlowFarneback(
            patch1, patch2, None,
            0.5, 1, 15, 2, 5, 1.1, 0
        )

        dx = np.mean(flow[..., 0])
        dy = np.mean(flow[..., 1])

        x2 = x + dx
        y2 = y + dy

        magnitude = np.sqrt(dx**2 + dy**2)

        # 🔥 LOW threshold
        if magnitude > 0.3:

            # color based on motion
            if magnitude < 1:
                color = (255, 0, 0)
            elif magnitude < 3:
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)

            p1 = (x, y)
            p2 = (int(x + scale*dx), int(y + scale*dy))

            cv2.arrowedLine(frame2, p1, p2, color, 2, tipLength=0.4)

            next_corners.append([x2, y2])

    # 🔥 Re-detect if lost
    if len(next_corners) < 20:
        corners = cv2.goodFeaturesToTrack(gray2, 100, 0.01, 7)
        if corners is not None:
            corners = corners.reshape(-1, 2)
        else:
            continue
    else:
        corners = np.array(next_corners)

    out.write(frame2)
    gray1 = gray2.copy()

cap.release()
out.release()

print("✅ Saved as output.mp4")
