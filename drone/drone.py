# -*- coding: utf-8 -*-

import sys
import time
from threading import Thread
from typing import Tuple

from dronekit import *
from pymavlink import mavutil
class drone(Thread):
    # Drone heartbeat
    hearbeat = 0.0
    update_interval = 1  # Update period
    # Drone attitude
    pitch = 0.0
    yaw = 0.0
    roll = 0.0
    # Drone position
    altitude = 0.0
    lat = 0.0
    lon = 0.0
    # Missions
    current_mission = 0
    mission = []

    def __init__(self, connection="172.30.208.1:14550"):
        # Drone connection start
        super().__init__()
        try:
            self.vehicle = connect(connection,wait_ready=True)

            # If not connected
            if not (self.vehicle):
                exit("Can not connect to Vehicle\nPlease check your connection.")

            # If connected well!
            else:

                self.thread = Thread(target=self.run, daemon=True)
                self.thread.start()

                print("Connected\n-- System status --\n %s" % self.vehicle.system_status.state)
                print("Attitude [%s]" % self.vehicle.attitude)

        except Exception as e:
            exit("error occured while connecting to vehicle :\n %s" % e)

    def run(self):
        while True:
            # Update drone data.
            self.update_data()
            time.sleep(self.update_interval)

    def update_data(self):
        """
        Updates the data (e.g. attitude, position) from the Drone.
        :return: No Returns...
        """
        self.hearbeat = self.vehicle.last_heartbeat
        self.pitch = self.vehicle.attitude.pitch * 180 / math.pi
        self.yaw = self.vehicle.attitude.yaw * 180 / math.pi
        self.roll = self.vehicle.attitude.roll * 180 / math.pi
        self.altitude = self.vehicle.location.global_relative_frame.alt
        self.lat = self.vehicle.location.global_frame.lat
        self.lon = self.vehicle.location.global_frame.lon

    def get_gps(self) -> Tuple[float, float]:
        """
        Gets the GPS data.
        :return: latitude, longitude
        """
        return self.lat, self.lon

    def takeoff(self,altitude) -> bool:
        """
        Check the system and TAKE OFF if system's good.
        :param altitude:
        :return: True or False
        """
        fail_count = 0

        while not self.vehicle.is_armable:
            fail_count += 1
            print(" Waiting for vehicle to arm... count %d" % fail_count)
            time.sleep(1)

        while True:
            print(" Altitude: ", self.altitude)
            if self.altitude >= altitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)
        return True

    def arm_check(self,count = 5):
        pass

    def show_debug(self):
        while True:
            self.update_param()
            print(f"lat {self.get_gps()[0]}, lng {self.get_gps()[1]} , alt : {self.altitude}")
            time.sleep(1)

    def RTL(self):
        """
        Run Mode RTL
        :return:
        """
        print("RTL start")
        self.mode = VehicleMode("RTL")

    def mission(self):
        """
        Run the drone Mission
        :return:
        """
        if not self.mission:
            print("No mission, Return to Home(Launch)")

    def update_mission(self,*locs : LocationGlobalRelative):
        """
        Update the mission with cordinates
        """
        self.mission.extend(locs)

# from multiprocessing import Process

if __name__ == "__main__":
    # Position for test flight #. 1
    point1 = LocationGlobalRelative(-35.36284032, 149.16559905, 10)
    # Position for test flight #. 2
    point2 = LocationGlobalRelative(-35.36248006, 149.16493783, 10)

    vehicle = drone("172.30.208.1:14550")
    vehicle.update_mission(point1)
    # vehicle.start()

    vehicle.takeoff(10)
    thr = Thread(target=vehicle.show_debug)
    thr.start()
    thr.join()

    print("mission 1 start")
    vehicle.vehicle.simple_goto(point1)
    time.sleep(10)
    print("mission 2 start")
    vehicle.vehicle.simple_goto(point2)
    time.sleep(10)
    vehicle.RTL()
    # while True:
    #     try:
    #         vehicle.update_param()
    #         print(f"lat {vehicle.get_gps()[0]}, lng {vehicle.get_gps()[1]} , alt : {vehicle.altitude}")
    #     except KeyboardInterrupt:
    #         exit("interrupted")