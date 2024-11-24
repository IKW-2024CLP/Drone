import time
from threading import Thread

import sys

import dronekit
from dronekit import *
class drone(Thread):
    connection = "172.30.208.1:14550"
    hearbeat = 0.0
    mission_altitude = 10
    pitch = 0.0
    yaw = 0.0
    roll = 0.0
    lat = 0.0
    lon = 0.0

    point1 = LocationGlobalRelative(-35.36284032, 149.16559905, mission_altitude)
    point2 = LocationGlobalRelative(-35.36248006, 149.16493783, mission_altitude)

    def __init__(self, connection=connection):
        try:
            self.vehicle = connect(connection,wait_ready=True)
            print("Connected")
            print("-- System status --\n %s" % self.vehicle.system_status.state)
            print("Attitude %s" % self.vehicle.attitude)

        except Exception as e:
            exit("error occured while connecting to vehicle :\n %s" % e )

    def takeoff_mission(self):
        while not self.vehicle.is_armable:
            print("vehicle arming")
            time.sleep(1)

    def update_param(self):
        self.hearbeat = self.vehicle.last_heartbeat
        self.pitch = self.vehicle.attitude.pitch * 180 / math.pi
        self.yaw = self.vehicle.attitude.yaw * 180 / math.pi
        self.roll = self.vehicle.attitude.roll * 180 / math.pi
        self.altitude = self.vehicle.location.global_relative_frame.alt
        self.lat = self.vehicle.location.global_frame.lat
        self.lon = self.vehicle.location.global_frame.lon

    def get_gps(self):
        return self.lat, self.lon

    def takeoff(self,altitude):
        self.mission_altitude = altitude
        fail_count = 0
        print("Arming")
        while not self.vehicle.is_armable:
            if fail_count > 5:
                sys.exit("Cannot arm the drone!!")
            print("Waiting for armable")
            fail_count += 1
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            if fail_count > 5:
                sys.exit("Cannot arm the drone!!")
            print("Waiting for armable")
            fail_count += 1
            time.sleep(1)

        print("Taking off")
        self.vehicle.simple_takeoff(altitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        vehicle.mission()

    def show(self):
        while True:
            self.update_param()
            print(f"lat {self.get_gps()[0]}, lng {self.get_gps()[1]} , alt : {self.altitude}")
            time.sleep(1)

    def mission(self):
        print("mission 1 start")
        self.vehicle.simple_goto(self.point1)
        time.sleep(10)
        print("mission 2 start")
        self.vehicle.simple_goto(self.point2)
        time.sleep(10)
        print("RTL start")
        self.vehicle.mode = VehicleMode("RTL")

# from multiprocessing import Process

if __name__ == "__main__":
    vehicle = drone("172.30.208.1:14550")
    # vehicle.start()

    vehicle.takeoff(10)
    thr = Thread(target=vehicle.show)
    thr.start()
    thr.join()
    # while True:
    #     try:
    #         vehicle.update_param()
    #         print(f"lat {vehicle.get_gps()[0]}, lng {vehicle.get_gps()[1]} , alt : {vehicle.altitude}")
    #     except KeyboardInterrupt:
    #         exit("interrupted")