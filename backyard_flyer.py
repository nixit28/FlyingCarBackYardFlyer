import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5
class DIRECTION(Enum):
    NORTH = 0
    EAST =1
    SOUTH =2
    WEST=3

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.all_waypoints = np.array([[10.0, 0.0, 3.0, 0.0], [10.0, 10.0, 3.0, 0.0], [0.0, 10.0, 3.0, 0.0], [0.0, 0.0, 3.0, 0.0]])
        self.in_mission = True
        self.check_state = {'last_path': -1, 'direction': None}

        # initial state
        self.flight_state = States.MANUAL


        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        print('PATH: %d :: \nCurrent Progress::'% self.check_state['last_path'])
        print(self.local_position)
        if self.evaluation_for_Waypoints():
            self.waypoint_transition()
        elif self.flight_state==States.WAYPOINT and self.check_state['last_path']==len(self.all_waypoints)-1 and\
        self.check_state['direction'] == DIRECTION.WEST and self.local_position[1] < self.target_position[1]+.05:
            self.landing_transition()
        elif self.flight_state == States.LANDING:
            if abs(self.local_position[2]) < 0.01:
                self.disarming_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        #WHY IN UP AND DOWN IT WAS DONE



    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        if self.flight_state == States.ARMING:
            self.takeoff_transition()
        if self.flight_state == States.DISARMING:
            self.manual_transition()

    def evaluation_for_Waypoints(self):
        if self.flight_state == States.TAKEOFF and -1 * self.local_position[2] > self.target_position[2] * .95:
            return True
        elif self.flight_state == States.WAYPOINT and self.check_state['last_path']<len(self.all_waypoints)-1:
            if self.check_state['direction'] == DIRECTION.NORTH:
                return self.local_position[0] > self.target_position[0] * .95
            if self.check_state['direction'] == DIRECTION.EAST:
                return self.local_position[1] > self.target_position[1] * .95
            if self.check_state['direction'] == DIRECTION.SOUTH:
                return self.local_position[0] < self.target_position[0]+.05

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        path = self.check_state['last_path']+1
        temp = self.all_waypoints[path]-self.target_position
        value=0
        for i in range(2):
            if temp[i]>0 or temp[i]<0:
                value = i
                break
        if value == 0:
            if temp[value]>0:
                self.check_state['direction'] = DIRECTION.NORTH
            else:
                self.check_state['direction'] = DIRECTION.SOUTH
        if value == 1:
            if temp[value]>0:
                self.check_state['direction'] = DIRECTION.EAST
            else:
                self.check_state['direction'] = DIRECTION.WEST
        return self.all_waypoints[path], path


    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2]=3
        self.takeoff(3)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("Waypoint Transition")
        self.target_position, path = self.calculate_box()
        print(self.target_position)
        time.sleep(2)
        self.cmd_position(self.target_position[0],
                          self.target_position[1],
                          self.target_position[2],
                          self.target_position[3])
        self.check_state['last_path'] = path
        print(self.check_state)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        time.sleep(2)
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
