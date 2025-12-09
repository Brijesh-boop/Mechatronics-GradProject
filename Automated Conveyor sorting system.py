"""
Automated Conveyor Sorter 

"""

import pybullet as p
import pybullet_data
import time
import random
from dataclasses import dataclass


#GLOBAL CONSTANTS

DT = 1.0 / 240.0

# Conveyor geometry
BELT_SPEED   = 0.4         #forward speed of blocks (along +x)
BELT_X0      = -0.5
BELT_LEN     = 1.4
BELT_WIDTH   = 0.30
BELT_HEIGHT  = 0.05
BELT_Z       = 0.10        #belt center in z

#Sensor
SENSOR_X       = BELT_X0 + 0.7
SENSOR_HALF_X  = 0.03

#Bins (aligned with sensor x)
BIN_X        = SENSOR_X
BIN_SIZE_X   = 0.35
BIN_SIZE_Y   = 0.30
BIN_WALL_T   = 0.015
BIN_SIZE_Z   = BELT_HEIGHT
BIN_OFFSET_Y = BELT_WIDTH / 2 + BIN_SIZE_Y / 2 + 0.02

#Blocks
BLOCK_HALF      = [0.025, 0.025, 0.025]
SPAWN_INTERVAL  = 1.0
TOTAL_BLOCKS    = 20

#Sideways motion after stop
SIDE_SPEED    = 0.5
SIDE_DURATION = 0.6

#Colors (RGBA)
COL_BELT       = [0.12, 0.12, 0.13, 1.0]
COL_BELT_SLAT  = [0.18, 0.18, 0.20, 1.0]
COL_ROLLER     = [0.20, 0.22, 0.25, 1.0]

COL_RED_BIN    = [0.85, 0.15, 0.20, 1.0]
COL_BLUE_BIN   = [0.15, 0.25, 0.85, 1.0]
COL_SENSOR_R   = [1.0, 0.3, 0.3, 0.25]
COL_SENSOR_B   = [0.3, 0.4, 1.0, 0.25]
COL_ARROW      = [0.1, 0.8, 0.3]



#UTILITIES
def setup_world():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(DT)
    p.loadURDF("plane.urdf")


def make_box(half, pos, rgba, mass=0.0, friction=0.7):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    bid = p.createMultiBody(mass, col, vis, pos)
    p.changeDynamics(
        bid, -1,
        lateralFriction=friction,
        linearDamping=0.04,
        angularDamping=0.04
    )
    return bid


def make_cylinder(radius, half_length, pos, rgba, axis="z", mass=0.0, friction=0.7):
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=half_length * 2)
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=half_length * 2, rgbaColor=rgba)

    if axis == "z":
        orn = [0, 0, 0, 1]
    elif axis == "y":
        orn = p.getQuaternionFromEuler([1.5708, 0, 0])
    elif axis == "x":
        orn = p.getQuaternionFromEuler([0, 1.5708, 0])
    else:
        orn = [0, 0, 0, 1]

    bid = p.createMultiBody(mass, col, vis, pos, orn)
    p.changeDynamics(
        bid, -1,
        lateralFriction=friction,
        linearDamping=0.04,
        angularDamping=0.04
    )
    return bid


@dataclass
class Block:
    bid: int
    color: str           #"red" or "blue"
    state: str = "belt"  #"belt" → moving, "side" → sliding, "done"
    side_dir: int = 0    #-1 = left, +1 = right
    side_until: float = 0.0



#MAIN SORTER CLASS


class SideBinSorter:

    def __init__(self):
        self.blocks = []
        self.stats = {"red": 0, "blue": 0, "missed": 0}

        #moving belt slats
        self.slats = []      #body ids
        self.slat_x = []     #current x positions
        self.slat_z = None   #z height

    
    def build(self):
        self._build_belt_and_slats()
        self._build_bins()
        self._build_sensors()
        self._build_arrows()

        p.resetDebugVisualizerCamera(
            cameraDistance=2.4,
            cameraYaw=-45,
            cameraPitch=-30,
            cameraTargetPosition=[BELT_X0 + BELT_LEN / 2, 0, BELT_Z + 0.05]
        )
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    
    def _build_belt_and_slats(self):
        #Main belt slab (static collision)
        make_box(
            [BELT_LEN / 2, BELT_WIDTH / 2, BELT_HEIGHT / 2],
            [BELT_X0 + BELT_LEN / 2, 0, BELT_Z],
            COL_BELT,
            mass=0.0,
            friction=0.9
        )

        #Visual slats on top of the belt (no collision, animated)
        n_slats = 10
        slat_len = BELT_LEN / n_slats
        slat_half_z = 0.002
        self.slat_z = BELT_Z + BELT_HEIGHT / 2 + slat_half_z

        slat_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[slat_len / 2 * 0.9, BELT_WIDTH / 2 * 0.95, slat_half_z],
            rgbaColor=COL_BELT_SLAT
        )

        for i in range(n_slats):
            cx = BELT_X0 + (i + 0.5) * slat_len
            bid = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,  #visual only
                baseVisualShapeIndex=slat_vis,
                basePosition=[cx, 0, self.slat_z]
            )
            self.slats.append(bid)
            self.slat_x.append(cx)

        #Rollers at both ends (static, just visuals)
        roller_radius = 0.03
        half_width = BELT_WIDTH / 2
        roller_z = BELT_Z - BELT_HEIGHT / 2 + roller_radius * 0.9

        make_cylinder(
            radius=roller_radius,
            half_length=half_width,
            pos=[BELT_X0, 0, roller_z],
            rgba=COL_ROLLER,
            axis="y",
            mass=0.0,
            friction=0.8
        )
        make_cylinder(
            radius=roller_radius,
            half_length=half_width,
            pos=[BELT_X0 + BELT_LEN, 0, roller_z],
            rgba=COL_ROLLER,
            axis="y",
            mass=0.0,
            friction=0.8
        )

    
    def _animate_belt_slats(self):
        """Scroll slats visually in the SAME direction as block motion."""
        if not self.slats:
            return

        #Slats move along +x like the blocks
        vis_speed = BELT_SPEED * 1.0
        dx = vis_speed * DT

        for i, bid in enumerate(self.slats):
            x = self.slat_x[i] + dx
            #Wrap when we slabs past the end of the belt
            end_x = BELT_X0 + BELT_LEN
            if x > end_x:
                x -= BELT_LEN
            self.slat_x[i] = x

            p.resetBasePositionAndOrientation(
                bid,
                [x, 0, self.slat_z],
                [0, 0, 0, 1]
            )

   
    def _build_bins(self):
        wall_half_z = BIN_SIZE_Z / 2

        #RED BIN (left) 
        red_center_y = -BIN_OFFSET_Y

        make_box(
            [BIN_SIZE_X / 2, BIN_SIZE_Y / 2, 0.01],
            [BIN_X, red_center_y, 0.0],
            COL_RED_BIN,
            mass=0.0
        )
        make_box(
            [BIN_SIZE_X / 2, BIN_WALL_T / 2, wall_half_z],
            [BIN_X,
             red_center_y - BIN_SIZE_Y / 2 + BIN_WALL_T / 2,
             wall_half_z],
            COL_RED_BIN,
            mass=0.0
        )
        make_box(
            [BIN_WALL_T / 2, BIN_SIZE_Y / 2, wall_half_z],
            [BIN_X + BIN_SIZE_X / 2 - BIN_WALL_T / 2,
             red_center_y,
             wall_half_z],
            COL_RED_BIN,
            mass=0.0
        )
        make_box(
            [BIN_WALL_T / 2, BIN_SIZE_Y / 2, wall_half_z],
            [BIN_X - BIN_SIZE_X / 2 + BIN_WALL_T / 2,
             red_center_y,
             wall_half_z],
            COL_RED_BIN,
            mass=0.0
        )

        p.addUserDebugText(
            "RED BIN",
            [BIN_X, red_center_y - BIN_SIZE_Y / 2 - 0.12, BIN_SIZE_Z + 0.02],
            [0.9, 0.2, 0.2],
            textSize=1.2,
            lifeTime=0
        )

        #BLUE BIN (right) 
        blue_center_y = BIN_OFFSET_Y

        make_box(
            [BIN_SIZE_X / 2, BIN_SIZE_Y / 2, 0.01],
            [BIN_X, blue_center_y, 0.0],
            COL_BLUE_BIN,
            mass=0.0
        )
        make_box(
            [BIN_SIZE_X / 2, BIN_WALL_T / 2, wall_half_z],
            [BIN_X,
             blue_center_y + BIN_SIZE_Y / 2 - BIN_WALL_T / 2,
             wall_half_z],
            COL_BLUE_BIN,
            mass=0.0
        )
        make_box(
            [BIN_WALL_T / 2, BIN_SIZE_Y / 2, wall_half_z],
            [BIN_X + BIN_SIZE_X / 2 - BIN_WALL_T / 2,
             blue_center_y,
             wall_half_z],
            COL_BLUE_BIN,
            mass=0.0
        )
        make_box(
            [BIN_WALL_T / 2, BIN_SIZE_Y / 2, wall_half_z],
            [BIN_X - BIN_SIZE_X / 2 + BIN_WALL_T / 2,
             blue_center_y,
             wall_half_z],
            COL_BLUE_BIN,
            mass=0.0
        )

        p.addUserDebugText(
            "BLUE BIN",
            [BIN_X, blue_center_y + BIN_SIZE_Y / 2 + 0.12, BIN_SIZE_Z + 0.02],
            [0.2, 0.3, 0.9],
            textSize=1.2,
            lifeTime=0
        )

    
    def _build_sensors(self):
        vis_red = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[SENSOR_HALF_X, BELT_WIDTH / 2, 0.03],
            rgbaColor=COL_SENSOR_R
        )
        vis_blue = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[SENSOR_HALF_X, BELT_WIDTH / 2, 0.03],
            rgbaColor=COL_SENSOR_B
        )

        p.createMultiBody(0, -1, vis_red,
                          [SENSOR_X, 0, BELT_Z + 0.13])
        p.createMultiBody(0, -1, vis_blue,
                          [SENSOR_X, 0, BELT_Z + 0.20])

        p.addUserDebugText(
            "RED SENSOR",
            [SENSOR_X, -0.25, BELT_Z + 0.30],
            [0.9, 0.2, 0.2],
            1.0,
            0
        )
        p.addUserDebugText(
            "BLUE SENSOR",
            [SENSOR_X, 0.25, BELT_Z + 0.30],
            [0.2, 0.3, 0.9],
            1.0,
            0
        )

    
    def _build_arrows(self):
        z = BELT_Z + BELT_HEIGHT / 2 + 0.006
        n_arrows = 5
        for i in range(n_arrows):
            x0 = BELT_X0 + (i + 0.4) * (BELT_LEN / n_arrows)
            x1 = x0 + 0.09
            p.addUserDebugLine(
                [x0, 0, z],
                [x1, 0, z],
                COL_ARROW,
                lineWidth=2,
                lifeTime=0
            )
            p.addUserDebugLine(
                [x1, 0, z],
                [x1 - 0.03, 0.02, z],
                COL_ARROW,
                lineWidth=2,
                lifeTime=0
            )
            p.addUserDebugLine(
                [x1, 0, z],
                [x1 - 0.03, -0.02, z],
                COL_ARROW,
                lineWidth=2,
                lifeTime=0
            )

    def spawn_block(self):
        color = random.choice(["red", "blue"])
        rgba = COL_RED_BIN if color == "red" else COL_BLUE_BIN

        start_x = BELT_X0 + 0.05
        pos = [start_x, 0.0,
               BELT_Z + BELT_HEIGHT / 2 + BLOCK_HALF[2] + 0.003]

        bid = make_box(BLOCK_HALF, pos, rgba,
                       mass=0.08, friction=0.35)
        self.blocks.append(Block(bid, color))

    def update(self, t):
        #animate belt visuals first
        self._animate_belt_slats()

        #then handle blocks
        for blk in self.blocks:
            try:
                pos, _ = p.getBasePositionAndOrientation(blk.bid)
            except p.error:
                continue

            x, y, z = pos

            #1) along the belt
            if blk.state == "belt":
                if (BELT_Z - BELT_HEIGHT/2 - 0.02) < z < (BELT_Z + BELT_HEIGHT/2 + 0.12):
                    p.resetBaseVelocity(blk.bid, [BELT_SPEED, 0, 0])
                else:
                    p.resetBaseVelocity(blk.bid, [0, 0, 0])

                #hit sensor
                if abs(x - SENSOR_X) < SENSOR_HALF_X:
                    blk.state = "side"
                    blk.side_dir = -1 if blk.color == "red" else +1
                    blk.side_until = t + SIDE_DURATION
                    print(f"{blk.color.upper()} block -> sensor hit, sliding {'LEFT' if blk.side_dir < 0 else 'RIGHT'}")

            #2) sideways into bin
            if blk.state == "side":
                if t <= blk.side_until:
                    p.resetBaseVelocity(blk.bid, [0, blk.side_dir * SIDE_SPEED, 0])
                else:
                    p.resetBaseVelocity(blk.bid, [0, 0, 0])
                    blk.state = "done"

            #3) simple stats
            if z < BIN_SIZE_Z + 0.02 and x > BIN_X - BIN_SIZE_X / 2:
                if abs(y + BIN_OFFSET_Y) < BIN_SIZE_Y / 2:
                    if blk.color == "red":
                        self.stats["red"] += 1
                    else:
                        self.stats["missed"] += 1
                elif abs(y - BIN_OFFSET_Y) < BIN_SIZE_Y / 2:
                    if blk.color == "blue":
                        self.stats["blue"] += 1
                    else:
                        self.stats["missed"] += 1


#MAIN LOOP

def main():
    setup_world()
    sorter = SideBinSorter()
    sorter.build()

    start = time.time()
    next_spawn = 0.0
    spawned = 0

    print("\nConveyor sorter running…")
    print("Animated rugged belt + side bins.")
    print("Blocks move forward, stop at sensor, then slide into bins.\n")
    print("Press 'q' to quit.\n")

    while p.isConnected():
        now = time.time()
        sim_t = now - start

        if spawned < TOTAL_BLOCKS and sim_t >= next_spawn:
            sorter.spawn_block()
            spawned += 1
            next_spawn += SPAWN_INTERVAL

        sorter.update(sim_t)
        p.stepSimulation()
        time.sleep(DT)

        keys = p.getKeyboardEvents()
        if ord('q') in keys or ord('Q') in keys:
            break

    print("\nFinal stats:", sorter.stats)
    p.disconnect()


if __name__ == "__main__":
    main()
