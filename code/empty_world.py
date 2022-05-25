import math
from direct.task import Task
from direct.actor.Actor import Actor
from direct.showbase.ShowBase import ShowBase
from pandac.PandaModules import *
from panda3d.core import CardMaker
import numpy as np
import socket
import select


def FicTrac_output(HOST, PORT, Trackball_r):
    # UDP
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((HOST, PORT))
        sock.setblocking(0)

        # Keep receiving data until FicTrac closes
        data = ""
        timeout_in_seconds = 1
        while True:
            # Check to see whether there is data waiting
            ready = select.select([sock], [], [], timeout_in_seconds)

            # Only try to receive data if there is data waiting
            if ready[0]:
                # Receive one data frame
                new_data = sock.recv(1024)

                # Uh oh?
                if not new_data:
                    break

                # Decode received data
                data += new_data.decode('UTF-8')

                # Find the first frame of data
                endline = data.find("\n")
                line = data[:endline]  # copy first frame
                data = data[endline + 1:]  # delete first frame

                # Tokenise
                toks = line.split(", ")

                # Check that we have sensible tokens
                if ((len(toks) < 24) | (toks[0] != "FT")):
                    print('Bad read')
                    continue

                # Extract FicTrac variables
                # (see https://github.com/rjdmoore/fictrac/blob/master/doc/data_header.txt for descriptions)
                # cnt = int(toks[1])  # frame counter
                dr_cam = [float(toks[2]), float(toks[3]), float(toks[4])]  # delta rotation vector (cam)
                # err = float(toks[5])  # delta rotation error score
                # dr_lab = [float(toks[6]), float(toks[7]), float(toks[8])]  # delta rotation vector (lab)
                # r_cam = [float(toks[9]), float(toks[10]), float(toks[11])]  # absolute rotation vector (cam)
                # r_lab = [float(toks[12]), float(toks[13]), float(toks[14])]  # absolute rotation vector (lab)
                # posx = float(toks[15])  # integrated x position (lab)
                # posy = float(toks[16])  # integrated y position (lab)
                heading = float(toks[17])  # integrated animal heading (lab)
                # step_dir = float(toks[18])  # animal movement direction (lab)
                # step_mag = float(toks[19])  # animal movement speed
                # intx = float(toks[20])  # integrated forward motion
                # inty = float(toks[21])  # integrated side motion
                # ts = float(toks[22])  # timestamp
                # seq = int(toks[23])  # sequence counter
                dt = float(toks[24])  # delta timestamp

                # Return output
                return dr_cam * Trackball_r, heading, dt


class MyApp(ShowBase):

    def __init__(self):

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                       SETTINGS (real world)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # DISPLAYS
        # We have three high-speed gaming displays positioned at the front and to the sides of the track-ball. Thus,
        # span the virtual environment across these three display regions.
        self.w = 1920 * 1
        self.h = 1080
        # Calibration of the setup. The screen in the front is elevated by 2cm in order to ensure a smooth transition
        # between display regions. This is reflected by the variable z_offset_F. All screens have the same width and
        # height. The animal's head is roughly 0.5cm above the ground. The virtual floor is roughly 0.2cm below the
        # midline of the lateral monitors
        self.dist2screen_F = 25.3  # [cm]
        self.dist2screen_L = 28.5  # [cm]
        self.dist2screen_R = 28.5  # [cm]
        self.z_offset_F = 2  # [cm]
        self.screen_w = 54.336  # [cm]
        self.screen_h = 30.564  # [cm]
        self.animal_h = 0.5  # [cm]
        self.floor_h = -0.2  # [cm]
        # Calculate the vertical field of view (fov) for each display.
        self.screen_F_fov_v = math.degrees(math.atan((self.screen_w / 2) / self.dist2screen_F)) * 2  # front
        self.screen_L_fov_v = math.degrees(math.atan((self.screen_w / 2) / self.dist2screen_L)) * 2  # left
        self.screen_R_fov_v = math.degrees(math.atan((self.screen_w / 2) / self.dist2screen_R)) * 2  # right
        # Calculate the horizontal field of view (fov) for each display. Note that we subtract the offset in z from each
        # screen's height. But we come to this later when we construct the camera views.
        self.screen_F_fov_h = math.degrees(
            math.atan(((self.screen_h - self.z_offset_F) / 2) / self.dist2screen_F)) * 2  # front
        self.screen_L_fov_h = math.degrees(
            math.atan(((self.screen_h - self.z_offset_F) / 2) / self.dist2screen_L)) * 2  # left
        self.screen_R_fov_h = math.degrees(
            math.atan(((self.screen_h - self.z_offset_F) / 2) / self.dist2screen_R)) * 2  # right

        # DEVICES & SOFTWARE
        # Establish connection with FicTrac
        self.HOST = '127.0.0.1'  # The (receiving) host IP address (sock_host)
        self.PORT = 1111  # The (receiving) host port (sock_port)
        # Trackball radius
        self.Trackball_r = 4

        # PRC FILE
        loadPrcFileData("",
                        """sync-video #t
                        show-frame-rate-meter #t
                        fullscreen #f
                        win-origin 0 0
                        undecorated #t
                        cursor-hidden #t
                        win-size %d %d
                        auto-single-cpu-affinity #f
                        """ % (self.w, self.h))

        ShowBase.__init__(self)

        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #                                    SETTINGS (virtual world)
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        # TEST ANIMAL
        # Give the starting position and heading of the animal.
        self.heading = 0  # heading           [deg]
        self.xPos = 0  # left-right           [cm]
        self.yPos = 0  # forward-backward     [cm]

        # FLOOR
        # As ground, use simple gray pattern of 25% uniform noise across a 5000x5000 px/cm image. We scale the image by
        # a value of 100 and repeat it 5 times. This results in a 500x500 cm large virtual area with a resolution of
        # 50 px/cm. For powerful computers, decrease the scale and increase the number of repetitions.
        cm = CardMaker('')
        card_scale = 100
        card_reps = 5
        cm.setFrame(-card_scale, card_scale, -card_scale, card_scale)
        floor = self.render.attachNewNode(PandaNode("floor"))
        for y in range(card_reps):
            for x in range(card_reps):
                nn = floor.attachNewNode(cm.generate())
                nn.setP(-90)
                nn.setPos((x - card_reps / 2) * ((card_scale * card_reps) / (card_reps / 2)),
                          (y - card_reps / 2) * ((card_scale * card_reps) / (card_reps / 2)), 0)
        floor.setTexture(self.loader.loadTexture("images/floor.jpg"))

        # SKY
        self.setBackgroundColor(1, 1, 1)

        # CAMERAS
        # This is a crucial part of the script that ensures a realistic display of the virtual environment. Please do
        # not touch. A 'camera' is now the real animal's window into the virtual world. Thus, the FOVs depend on the
        # hardware configuration of the setup.
        # Construct the cameras' lenses.
        # --- front
        self.lens_F = PerspectiveLens()
        self.lens_F.setFov(self.screen_F_fov_v, self.screen_F_fov_h)
        self.lens_F.setNearFar(0.001, 10000)
        # --- left
        self.lens_L = PerspectiveLens()
        self.lens_L.setFov(self.screen_L_fov_v, self.screen_L_fov_h)
        self.lens_L.setNearFar(0.001, 10000)
        # --- right
        self.lens_R = PerspectiveLens()
        self.lens_R.setFov(self.screen_R_fov_v, self.screen_R_fov_h)
        self.lens_R.setNearFar(0.001, 10000)
        # Now define the correct display regions. This is a little tricky as the front screen is elevated in z-direction
        # by two centimeters. We solve this by cutting off the upper 2cm of the frontal display and the lower 2cm of the
        # lateral displays. Reparent the lateral cameras to the frontal one for each synchronisation.
        # --- front camera
        dr = self.cam.node().getDisplayRegion(0)
        dr.setDimensions(1 / 3, 2 / 3, 0, (self.screen_h - self.z_offset_F) / self.screen_h)
        self.cam.node().setLens(self.lens_F)
        self.cam.setHpr(self.heading, 0, 0)
        self.cam.setPos(self.xPos, self.yPos, self.animal_h)
        # --- left camera
        self.cam_left = self.makeCamera(self.win, scene=self.render, lens=self.lens_L,
                                        displayRegion=(0, 1 / 3, self.z_offset_F / self.screen_h, 1))
        self.cam_left.reparentTo(self.cam)
        self.cam_left.setHpr(90, 0, 0)
        # --- right camera
        self.cam_right = self.makeCamera(self.win, scene=self.render, lens=self.lens_R,
                                         displayRegion=(2 / 3, 1, self.z_offset_F / self.screen_h, 1))
        self.cam_right.reparentTo(self.cam)
        self.cam_right.setHpr(-90, 0, 0)

        # TASKS
        # In Panda3D, a task is executed at each frame. For now, we have two main tasks: the animation and displacement
        # of the virtual animals; and reconstruction of the fictive path. As FicTrac may run at a different refresh rate
        # as the virtual environment, we put it in a separate chain of tasks (cf. threading).
        # Create chain for FicTrac
        self.taskMgr.setupTaskChain('BallTracking', numThreads=1, tickClock=None,
                                    threadPriority=None, frameBudget=-1,
                                    frameSync=None, timeslicePriority=None)
        # Taskmanager
        # --- Handle FicTrac
        # --- Handle tracking
        self.taskMgr.add(self.saveTracking, "saveTracking")
        self.taskMgr.add(self.update_VirtEnvPos, "update_VirtEnvPos", taskChain='BallTracking')


    # --- Update virtual position of real animal -----------------------------------------------------------------------
    def update_VirtEnvPos(self, task):
        # Get tracking from FicTrac
        dr_lab, heading, dt = FicTrac_output(self.HOST, self.PORT, self.Trackball_r)
        # Calculate change in position, given the current heading
        dX = dr_lab[0] * math.cos(heading) - dr_lab[1] * math.sin(heading)
        dY = dr_lab[1] * math.cos(heading) - dr_lab[0] * math.sin(heading)
        # Update parameters
        self.xPos += dX
        self.yPos += dY
        self.heading = math.degrees(heading)
        # Update heading
        self.cam.setPos(self.xPos, self.yPos, self.animal_h)
        self.cam.setHpr(self.heading, 0, 0)
        return Task.cont

    # --- Save tracking in the VE for both virtual and real animals ----------------------------------------------------
    def saveTracking(self, task):

        l = 3
        currData = [0] * l
        currData[0] = self.cnt
        currData[1] = round(self.xPos, 3)
        currData[2] = round(self.yPos, 3)

        self.tracking.write(str(currData))
        self.tracking.write("\n")

        # Update frame counter
        self.cnt += 1
        return Task.cont


game = MyApp()
game.run()
