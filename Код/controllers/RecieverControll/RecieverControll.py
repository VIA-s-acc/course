# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its robot the following behavior:
According to the messages it receives, the robot change its
behavior.
"""

from controller import AnsiCodes, Robot


class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Controller(Robot):
    Mode = Enumerate('MOVE_FORWARD INV_CLOCK_ CLOCK STOP')
    timeStep = 32
    maxSpeed = 1000.0
    mode = Mode.STOP
    motors = []
    distanceSensors = []

    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))

    def __init__(self):
        super(Controller, self).__init__()
        self.mode = self.Mode.STOP
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timeStep)
        self.motors.append(self.getDevice("left wheel motor"))
        self.motors.append(self.getDevice("right wheel motor"))
        self.motors[0].setPosition(float("inf"))
        self.motors[1].setPosition(float("inf"))
        self.motors[0].setVelocity(0.0)
        self.motors[1].setVelocity(0.0)
        # print("Motors:", self.motors, "\nReceiver:", self.receiver)
        # for dsnumber in range(0, 2):
        #     self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
        #     self.distanceSensors[-1].enable(self.timeStep)

    def run(self):
        while True:
            # Read the supervisor order.
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getString()
                self.receiver.nextPacket()
                
                if message == 'F':
                    self.mode = self.Mode.MOVE_FORWARD
                elif message == 'CC':
                    self.mode = self.Mode.INV_CLOCK_
                elif message == 'C':
                    self.mode = self.Mode.CLOCK
                elif message == 'S':
                    self.mode = self.Mode.STOP

            
            speeds = [0.0, 0.0]
            # Send actuators commands according to the mode.
            if self.mode == self.Mode.MOVE_FORWARD:
                speeds[0] = 1
                speeds[1] = 1
            elif self.mode == self.Mode.INV_CLOCK_:
                speeds[0] = -0.5
                speeds[1] = 0.5
            elif self.mode == self.Mode.CLOCK:
                speeds[0] = 0.5
                speeds[1] = -0.5
            elif self.mode == self.Mode.STOP:
                speeds[0] = 0
                speeds[1] = 0
            
            self.motors[0].setVelocity(speeds[0])
            self.motors[1].setVelocity(speeds[1])

            if self.step(self.timeStep) == -1:
                break


controller = Controller()
controller.run()
