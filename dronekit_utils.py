from dronekit import connect, VehicleMode, LocationGlobal, Command
from pymavlink import mavutil
import time
import sys
import argparse
import math

# vehicle = connect(connection_string)

# # Get some vehicle attributes (state)


# while vehicle.is_armable: continue

# print('Setting Mode')
# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True
# vehicle.simple_takeoff(10)

class PX4_Utils(object):

    def __init__(self,connection_string):
        self.vehicle = connect(connection_string)
        self.home_position_set = False
        self.cmds = self.vehicle.commands

    def initialize_vehicle(self):
        """
        Vehicle Initializer
        """
        print('Executing basic pre-arm checks')
        # Waiting for initializer to bootup
        if self.vehicle.mode.name == 'INITIALISING':
            print('Waiting for vehicle to initialise')
            time.sleep(1)

        while self.vehicle.gps_0.fix_type < 2:
            print('Waiting for GPS...: {}'.format(self.vehicle.gps_0.fix_type))
            time.sleep(1)

        self.home = self.get_home_location()

        print('Clearing previous commands')
        self.clear_commands()
        self.vehicle.flush()
        time.sleep(1)

        print('Setting mode to auto')
        self.set_mode(mavutil.mavlink.MAV_MODE_AUTO_ARMED)
        
        print('Arming vehicle')
        self.arm_vehicle()
        
        print('Waiting for vehicle to be armed')
        while not self.vehicle.armed:
            print('Waiting for arming...')
            time.sleep(1)



    def clear_commands(self):
        self.cmds.clear()
        self.vehicle.flush()
        return True

    def set_commands(self, command):
        self.cmds.add(command)

    def upload_commands(self):
        self.cmds.upload()
        self.vehicle.flush()

    def arm_vehicle(self):
        self.vehicle.armed = True
        self.vehicle.flush()

    def get_home_location(self):
        return self.vehicle.location.global_relative_frame

    def show_params(self):
        print("Get some vehicle attribute values:")
        print(" GPS: {}".format(self.vehicle.gps_0))
        print(" Battery: {}".format(self.vehicle.battery))
        print(" Last Heartbeat: {}".format(self.vehicle.last_heartbeat))
        print(" Is Armable?: {}".format(self.vehicle.is_armable))
        print(" System status: {}".format(self.vehicle.system_status.state))
        print(" Mode: {}".format(self.vehicle.mode.name))    # settable

    def set_mode(self,mode):
        self.vehicle._master.mav.command_long_send(
                                        self.vehicle._master.target_system, 
                                        self.vehicle._master.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                        0,
                                        mode,
                                        0, 0, 0, 0, 0, 0)

    def get_location_offset_meters(self, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the home. 
        The returned Location adds the entered `alt` value to the altitude of the home.
        The function is useful when you want to move the vehicle around specifying locations relative to the current 
        vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*self.home.lat/180))

        #New position in decimal degrees
        newlat = self.home.lat + (dLat * 180/math.pi)
        newlon = self.home.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,self.home.alt+alt)

    def takeoff(self):
        self.get_home_location()
        wp = self.get_location_offset_meters(
                                            0, 
                                            0, 
                                            10)
        cmd = Command(0,
                    0,
                    0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                    0, 1, 0, 0, 0, 0, 
                    wp.lat, wp.lon, wp.alt)
        self.set_commands(cmd)
        self.upload_commands()

if __name__=='__main__':
    connection_string = '127.0.0.1:14540'
    vehicle = PX4_Utils(connection_string)
    vehicle.initialize_vehicle()
    # time.sleep(10)
    vehicle.takeoff()
    while True: continue
    # vehicle.show_params()
