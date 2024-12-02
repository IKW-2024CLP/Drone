# -*- coding: utf-8 -*-
import time
from threading import Thread
from typing import Tuple

from dronekit import *


# from pymavlink import mavutil

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
    missions = []

    def __del__(self):
        if 'vehicle' in locals() or self is not None:
            vehicle.RTL()
    def __init__(self, connection="172.30.208.1:14550"):
        # Drone connection start
        super().__init__()
        try:
            self.vehicle = connect(connection, wait_ready=True)

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

    def run(self) -> None:
        """
        Run every second with Thread
        :return:
        """
        while True:
            self.update_data()
            time.sleep(self.update_interval)

    def update_data(self) -> None:
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

    def takeoff(self, altitude=10.0) -> bool:
        """
        Check the system and TAKE OFF if system's good.
        :param altitude:
        :return: True or False
        """
        fail_count = 0

        while not self.vehicle.is_armable:
            fail_count += 1
            print(" Waiting for vehicle is armable... count %d" % fail_count)
            time.sleep(1)

            if fail_count > 5:
                return False

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.arm()
        self.vehicle.simple_takeoff(altitude)

        while True:
            print(" Takeoff Altitude: ", self.altitude)
            if self.altitude >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
        # FIXME: 궁금한 미친짓
        # self.vehicle.armed = False
        return True

    def show_debug(self) -> None:
        print(f"lat {self.get_gps()[0]}, lng {self.get_gps()[1]} , alt : {self.altitude}")

    def RTL(self,reason=None) -> bool:
        """
        Run Mode RTL
        :return:
        """
        self.vehicle.mode = VehicleMode("RTL")
        print("RTL start Reason :", str(reason))

        while not self.vehicle.location.global_relative_frame.alt <= 0.01:
            # Sometime RTL mode disabled by unknown drone issue.
            if not self.vehicle.mode == VehicleMode("RTL"):
                self.vehicle.mode = VehicleMode("RTL")
            if reason is not None:
                print("RTL {%s} Altitude: %f " % (reason, self.altitude))
            else:
                print("RTL Altitude:", self.altitude)

        print("Done to Land, Disarming.")
        self.vehicle.armed = False
        if not self.vehicle.armed:
            return False
        print("Disconnect the vehicle.")
        self.vehicle.close()

    def LAND(self):
        self.vehicle.mode = VehicleMode("LAND")

    def mission(self, start_altitude=altitude) -> bool:
        """
        Run the drone Mission
        :return:
        """
        # If the mission is empty
        if not self.missions:
            print("No mission, Return to Home(Launch)")
            self.RTL(reason="No mission")
            return False
        # If drone has mission.
        else:
            print("Mission start")

            # TODO: 이륙 여부 검증 후 이륙
            # FIXME: 이륙도 못했는데 바로 임무수행 들어감...
            # First, fly the drone!
            if self.altitude < 1.0:
                print("Take off for mission.")
                if self.takeoff(start_altitude):
                    print("Take off done, Mission start")

            for point in self.missions:
                while not self.get_distance_metres(point) < 0.8:
                    print("Distance metres : ", self.get_distance_metres(point))
                    self.vehicle.simple_goto(point)
                # self.goto(point.lat, point.lon)

            self.RTL(reason="End Mission")
            return True

    def update_mission(self, *locs: LocationGlobalRelative) -> bool:
        """
        Update the mission with LocationGlobalRelative data
        """
        self.missions.extend(locs)
        return True

    def get_location_metres(self, d_north, d_east):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = d_north / earth_radius
        dLon = d_east / (earth_radius * math.cos(math.pi * self.lat / 180))

        # New position in decimal degrees
        newlat = self.lat + (dLat * 180 / math.pi)
        newlon = self.lon + (dLon * 180 / math.pi)
        if type(self.vehicle.location.global_relative_frame) is LocationGlobal:
            print("lg")
            targetlocation = LocationGlobal(newlat, newlon, self.altitude)
        elif type(self.vehicle.location.global_relative_frame) is LocationGlobalRelative:
            print("lgr")
            targetlocation = LocationGlobalRelative(newlat, newlon, self.altitude)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation

    def get_distance_metres(self, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - self.lat
        dlong = aLocation2.lon - self.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def goto(self, d_north, d_east) -> bool:
        """
        Moves the vehicle to a position d_north metres North and d_east metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """

        targetLocation = self.get_location_metres(d_north, d_east)
        targetDistance = self.get_distance_metres(targetLocation)
        self.vehicle.simple_goto(targetLocation)

        # IF not GUIDED mode...
        if not self.vehicle.mode.name == "GUIDED":
            return False
        # Must be set GUIDED
        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= targetDistance * 0.01:
                print("Reached target")
                return True


# from multiprocessing import Process

if __name__ == "__main__":
    # Position for test flight #. 1
    vehicle = drone("172.30.208.1:14550")
    try:
        point1 = LocationGlobalRelative(-35.36316902, 149.16529954, 10)
        point2 = LocationGlobalRelative(-35.36330254, 149.16513464, 10)
        point3 = LocationGlobalRelative(-35.36334282, 149.16522457, 5)

        # Update the Mission
        vehicle.update_mission(point1, point2,point3)

        print(vehicle.takeoff(10))
        if vehicle.mission():
            print("Mission Complete")
        else:
            exit("Mission failed")
    except KeyboardInterrupt:
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.RTL()
        raise ConnectionError("Please do not close the connection like this way..")
