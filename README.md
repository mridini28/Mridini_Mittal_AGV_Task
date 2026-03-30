# Optical Flow
Monocular Images- images as seen by a single camera- 2D
(Force in robotics is the control command.)
Visual Potential Field: 2D map calculated from a camera image where low values are assigned to where the car should go (road), and high values are assigned to where the car should avoid (obstacles).
A 3D image is flattened to 2D

Optical Flow Vector: direction and magnitude of colour intensity changes from the movement of objects with the same brightness.
The direction of the arrow indicates which way the object is moving, and the magnitude/length of it indicates how fast it is moving.

Using optical flow to calculate time to collision TTC- example: optical flow vectors outward signify that they are expanding fast and the car is about to hit something and brakes are required, low TTC.

Balance Strategy Approach: Set of behavior roles to be applied when the average of the vector is above/below predicted thresholds.

Identify a dominant plane(then get the visual potential field and apply the balance strategy method): optical flow vectors come from the moving clouds, trees, etc. Some/most of this is unnecessary. Example: the motion of a car, the dominant plane is the ground or the road

Building an APF: artificial potential field from the optical flow vector
Target-global minimum- lowest point in the landscape- Attractive forces
Obstacles- Local maxima- peaks/hills in the landscape- Repulsive forces- push the vehicle away from them- the gradient lines from the optical flow vector are used to generate this field
Smooth and consistent- clear path- global min
Vector points outward, grows rapidly- indicate obstacle- local max

Risk field: Urisk- distance to lane boundaries and obstacles is used to compute total risk

# implementation
The simulation environment that is uploaded (simulation_setup.py) already builds the road, obstacles,
and car, so we only need the navigation script.
From the paper:
Optical flow shows how pixels move between frames.
If the car moves forward, flow vectors spread outward.
The point where they originate is the Focus of Expansion (FOE).
Obstacles disturb this pattern.
1. Capture camera images from PyBullet
2. Convert to grayscale and detect feature points
3. Use Lucas-Kanade to track motion
4. Large motion indicates obstacles
5. Steer opposite to obstacle side
Flow vectors increase near an obstacle.
If more motion on LEFT - obstacle on left -go RIGHT
If more motion on RIGHT -obstacle on right -go LEFT
Else -go straight
We use Lucas-Kanade optical flow to track feature movement between frames.
Large motion indicates nearby obstacles.
We divide the image into left and right halves.
If motion is higher on one side, we steer in the opposite direction.
This acts like a simplified potential field.
What you will see: Car moves forward, Detects obstacles, Zig-zags to avoid them, Reaches
end wall.

flow = new_pts - old_pts
each point has:(dx, dy)
dx -movement in x direction (left/right)
dy -movement in y direction (up/down)
