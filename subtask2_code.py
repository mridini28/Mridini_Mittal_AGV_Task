import pybullet as p
import cv2
import numpy as np
import time

from simulation_setup import setup_simulation


def get_camera_image(car_id):
    pos, _= p.getBasePositionAndOrientation(car_id)
    camera_pos= [pos[0], pos[1], pos[2] + 0.5]
    target= [pos[0] + 2, pos[1], pos[2] + 0.5]
    view= p.computeViewMatrix(camera_pos, target, [0, 0, 1])
    proj= p.computeProjectionMatrixFOV(60, 1.0, 0.1, 100)
    img= p.getCameraImage(320, 240, view, proj)
    frame= np.reshape(img[2], (240, 320, 4))[:, :, :3]
    return frame


def get_optical_flow(prev, curr):
    prev_gray= cv2.cvtColor(prev,cv2.COLOR_BGR2GRAY)
    curr_gray= cv2.cvtColor(curr,cv2.COLOR_BGR2GRAY)
    pts= cv2.goodFeaturesToTrack(prev_gray,100,0.01,7)
    if pts is None:
        return None, None
    next_pts,status,_ = cv2.calcOpticalFlowPyrLK(prev_gray,curr_gray,pts,None)
    good_old= pts[status== 1]
    good_new= next_pts[status== 1]
    return good_old, good_new


def get_FOE(old_pts, new_pts):
    flows= new_pts - old_pts
    A,b= [],[]
    for i in range(len(old_pts)):
        x,y= old_pts[i]
        dx,dy= flows[i]
        A.append([-dy, dx])
        b.append(x * (-dy) - y * dx)
    A = np.array(A, dtype=np.float32)
    b = np.array(b, dtype=np.float32)
    foe, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return foe


def compute_potential_field(old_pts, new_pts, foe, goal=(160, 120)):
    flows = new_pts - old_pts
    mag = np.sqrt(flows[:, 0]**2 + flows[:, 1]**2)
    mask = mag > 4
    obstacle_pts = old_pts[mask]

    repulsive = np.array([0.0,0.0])
    for pt in obstacle_pts:
        diff= pt-foe
        dist= np.linalg.norm(diff) + 1e-5
        repulsive+= diff / (dist**2)

    attractive= np.array(goal, dtype=np.float32) - np.array(foe, dtype=np.float32)
    attractive= attractive / (np.linalg.norm(attractive) + 1e-5)

    total= attractive-0.5*repulsive
    return total


def decide_steering(old_pts, new_pts):
    if len(old_pts) < 4:
        return 0
    foe = get_FOE(old_pts, new_pts)
    gradient = compute_potential_field(old_pts, new_pts, foe)
    steer = float(np.clip(gradient[0] * 0.05, -0.4, 0.4))
    return steer


def move_car(car_id, steering_joints, motor_joints, steer):
    for j in steering_joints:
        p.setJointMotorControl2(car_id, j, p.POSITION_CONTROL, targetPosition=steer)
    for j in motor_joints:
        p.setJointMotorControl2(car_id, j, p.VELOCITY_CONTROL, targetVelocity=8, force=500)


if __name__ == "__main__":

    p.connect(p.GUI)

    car_id, steering_joints, motor_joints = setup_simulation()
    prev_frame = get_camera_image(car_id)

    while True:
        p.stepSimulation()

        curr_frame = get_camera_image(car_id)
        old_pts, new_pts = get_optical_flow(prev_frame, curr_frame)

        steer = 0
        if old_pts is not None:
            steer = decide_steering(old_pts, new_pts)

        move_car(car_id, steering_joints, motor_joints, steer)

        cv2.imshow("Camera", curr_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        prev_frame = curr_frame
        time.sleep(1/60)

    cv2.destroyAllWindows()
    p.disconnect()
