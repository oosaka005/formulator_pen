"""Driver for the CNC to operate it through code, instead of through candle or similar software
"""

import serial
import time
from threading import Event
import math
import csv
import os
from pathlib import Path
import pandas as pd

class CNC():
    NORMAL_SPEED = 3000
    SLOW_SPEED = 300
    SAFE_Y = 150      # Minimum safe Y position

    def __init__(self, config_path, cnc_type, positions, virtual=True, serial_port=None):
        # Load configuration with defaults
        config = self.load_config(config_path)
        cnc_config = config.get('cnc', {}).get(cnc_type, {})
        
        # Set connection parameters with defaults
        connection = cnc_config.get('connection', {})
        self.BAUD_RATE = connection.get('baud_rate', 115200)
        self.SERIAL_PORT = serial_port if serial_port is not None else connection.get('serial_port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        
        # Set movement limits with defaults
        limits = cnc_config.get('limits', {})
        self.X_LOW_BOUND = limits.get('x_min', 0)
        self.X_HIGH_BOUND = limits.get('x_max', 400)
        self.Y_LOW_BOUND = limits.get('y_min', -20)
        self.Y_HIGH_BOUND = limits.get('y_max', 20)
        self.Z_LOW_BOUND = limits.get('z_min', -40)
        self.Z_HIGH_BOUND = limits.get('z_max', 0)

        # Store position data and virtual mode
        self.LOCATIONS = self._convert_positions_to_dict(positions)
        self.VIRTUAL = virtual
        self._ser = None
        
        # Load SAFE_Y position if defined in positions
        if not positions.empty and "positionID" in positions.columns:
            safe_row = positions[positions["positionID"] == "SAFE_Y"]
            if not safe_row.empty and pd.notna(safe_row.iloc[0]["y"]):
                self.SAFE_Y = float(safe_row.iloc[0]["y"])
        print("Connected to CNC Machine!")

    def load_config(self, config_path):
        """Load CNC configuration parameters from YAML file."""
        try:
            import yaml
            with open(config_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except ImportError:
            print(f"[WARNING] PyYAML not installed, using defaults")
            return {}
        except (FileNotFoundError, Exception) as e:
            print(f"[WARNING] Could not load config from {config_path}, using defaults")
            return {}
    
    def _convert_positions_to_dict(self, positions_df):
        """Convert position DataFrame to dictionary format for internal use"""
        locations = {}
        if positions_df.empty:
            return locations
            
        for _, row in positions_df.iterrows():
            position_id = row['positionID']
            locations[position_id] = {
                'x': float(row['x']) if pd.notna(row['x']) else 0.0,
                'y': float(row['y']) if pd.notna(row['y']) else 0.0,
                'z': float(row['z']) if pd.notna(row['z']) else 0.0
            }
            # Include dispenserID if present
            if 'dispenserID' in row and pd.notna(row['dispenserID']):
                locations[position_id]['dispenserID'] = row['dispenserID']
        return locations


    def _get_parking_position_id(self, dispenser_id):
        """Get parking position ID (P#) for given dispenser ID."""
        for position_id, position_data in self.LOCATIONS.items():
            if (position_id.startswith('P') and 
                'dispenserID' in position_data and 
                position_data['dispenserID'] == dispenser_id):
                return position_id
        return None

    def get_offset_position_id(self, dispenser_id: str, prefix: str):
        """Lookup offset position ID by dispenser and prefix.
        Prefix conventions:
          - "O": generic XY/Z offset for dispensing positions (applied to all locations for that dispenser)
          - "SR": safe-run offset for dispensing positions (applied only during safe moves to avoid collisions)
          - "SS": safe-set offset for parking positions (applied when moving to parking to ensure parked tool is out of the way)
        """
        for position_id, position_data in self.LOCATIONS.items():
            if (position_id.startswith(prefix) and
                'dispenserID' in position_data and
                position_data['dispenserID'] == dispenser_id):
                return position_id
        return None

    #Return to origin (currently doesn't use limit switches)
    def home(self):
        print("Returning CNC to origin")
        # Stage 1: adjust Z to 0 while keeping current X,Y
        self.move_to_point(x=None, y=None, z=0, speed=self.NORMAL_SPEED, gtype="G0")
        # Stage 2: move X to 0 (hold Y,Z)
        self.move_to_point(x=0, y=None, z=None, speed=self.NORMAL_SPEED, gtype="G0")
        # Stage 3: move Y to 0
        self.move_to_point(x=None, y=0, z=None, speed=self.NORMAL_SPEED, gtype="G0")

    def staged_home(self):
        """Return to origin (0,0,0) with staged sequence:
        1) Raise/adjust Y to SAFE_Y keeping current X,Z.
        2) Move X to 0 and Z to 0 while holding Y at SAFE_Y.
        3) Move Y to 0.
        This avoids direct diagonal travel from an arbitrary parked/tool position.
        """
        print("[HOME] Staged home sequence start")
        # Step 1: Y -> SAFE_Y (keep X,Z unchanged)
        self.move_to_point_safe(x=None, y=self.SAFE_Y, z=0, speed=self.NORMAL_SPEED)
        # Step 2: X -> 0, Z -> 0 with Y fixed
        self.move_to_point(x=0, y=None, z=None, speed=self.NORMAL_SPEED)
        # Step 3: Y -> 0
        self.move_to_point(x=None, y=0, z=None, speed=self.NORMAL_SPEED)
        print("[HOME] Staged home sequence complete at (0,0,0)")

    #Wait for the CNC machine to complete its motion
    def wait_for_movement_completion(self,ser, cleaned_line):
        Event().wait(1)
        if cleaned_line != '$X' or '$$':  #May be a bug*********************************
            idle_count = 0
            while True:
                ser.reset_input_buffer()
                command = str.encode('?' + '\n')
                ser.write(command)
                grbl_out = ser.readline()
                grbl_response = grbl_out.strip().decode('utf-8')

                if grbl_response != 'ok':
                    if grbl_response.find('Idle') > 0:
                        idle_count += 1
                if idle_count > 0:
                    break
        return

    #Command the CNC machine to move to a point (x,y,z) with specific speed. Gtype could be G0 or G1, G0 moves at maximum possible speed
    def move_to_point(self,x=None,y=None,z=None,speed=3000,gtype="G1"):
        if self.coordinates_within_bounds(x,y,z):
            gcode = self.get_gcode_path_to_point(x,y,z,speed,gtype)
            print(f"Moved To (X{x}, Y{y}, Z{z}): ", self.follow_gcode_path(gcode))
        else:
            print(f"Cannot move to (X{x}, Y{y}, Z{z}), coordinates not within bounds")

    #Command the CNC machine to move to its max height then to an xy location then down to the target z
    def move_to_point_safe(self,x,y,z,speed=3000,gtype="G1"):
        if self.coordinates_within_bounds(x,y,z):
            gcode = self.get_gcode_path_to_point(x=None,y=None,z=self.Z_HIGH_BOUND,speed=speed,gtype=gtype) #Max height
            gcode += self.get_gcode_path_to_point(x=x,y=y,z=self.Z_HIGH_BOUND,speed=speed,gtype=gtype) #xy travel
            gcode += self.get_gcode_path_to_point(x=None,y=None,z=z,speed=speed,gtype=gtype) #Down
            print(f"Moved safely To (X{x}, Y{y}, Z{z}): ", self.follow_gcode_path(gcode))
        else:
            print(f"Cannot move to (X{x}, Y{y}, Z{z}), coordinates not within bounds")

    #Move to a location defined in the Locations file
    def move_to_location(self, location_name, safe=False, speed=3000):
        print(f"Moving to location: {location_name}")
        x, y, z = self.get_location_position(location_name)
        if safe:
            self.move_to_point_safe(x, y, z, speed=speed)
        else:
            self.move_to_point(x, y, z, speed=speed)

    #Get the location in x,y,z from name
    def get_location_position(self, location_name):
        # Current CSV uses x, y, z columns directly
        x = float(self.LOCATIONS[location_name]['x'])
        y = float(self.LOCATIONS[location_name]['y']) 
        z = float(self.LOCATIONS[location_name]['z'])
        
        return x,y,z
    
    def _get_offset_position_id(self, dispenser_id):
        """Get generic offset position ID (O#) for given dispenser ID."""
        return self.get_offset_position_id(dispenser_id, "O")
    
    #Get position with offset applied for dispensers
    def get_location_position_with_offset(self, location_name, dispenser_id):
        """Get position with dispenser-specific offset applied"""
        # Get base location position
        base_x = float(self.LOCATIONS[location_name]['x'])
        base_y = float(self.LOCATIONS[location_name]['y'])
        base_z = float(self.LOCATIONS[location_name]['z'])
        
        # Get offset position dynamically from pos_master
        offset_position_id = self._get_offset_position_id(dispenser_id)
        if offset_position_id and offset_position_id in self.LOCATIONS:
            offset_x = float(self.LOCATIONS[offset_position_id]['x'])
            offset_y = float(self.LOCATIONS[offset_position_id]['y'])
            offset_z = float(self.LOCATIONS[offset_position_id]['z'])
        else:
            # No offset for unknown dispensers
            offset_x = offset_y = offset_z = 0
        
        # Apply offset
        final_x = base_x + offset_x
        final_y = base_y + offset_y
        final_z = base_z + offset_z
        
        return final_x, final_y, final_z
    
    #Move to a location with dispenser offset applied
    def move_to_location_with_offset(self, location_name, dispenser_id, safe=False, speed=3000):
        """Move to a location with dispenser-specific offset applied"""
        print(f"Moving to location: {location_name} (with {dispenser_id} offset)")
        x, y, z = self.get_location_position_with_offset(location_name, dispenser_id)
        if safe:
            self.move_to_point_safe(x, y, z, speed=speed)
        else:
            self.move_to_point(x, y, z, speed=speed)

    def move_to_waste_bin(self):
        """Move to waste bin position for preprocessing."""
        print("[CNC] Moving to Waste Bin")
        self.move_to_location('WB', safe=True)

    def move_to_dispensing_position(self, tool_id: str, location: str):
        """Move to dispensing position for specified location with offset."""
        print(f"[CNC] Moving {tool_id} to dispensing position {location}")
        self.move_to_location_with_offset(location, tool_id, safe=True)

    #Returns the gcode to move to a point (x,y,z)
    #You can create a long gcode list to execute all at once instead of doing it line-by-line
    def get_gcode_path_to_point(self,x=None,y=None,z=None,speed=500,gtype="G1"):
        return_string = gtype
        if x != None:
            return_string += f" X{x}"
        if y != None:
            return_string += f" Y{y}"
        if z != None:
            return_string += f" Z{z}"
        if speed != None:
            return_string += f" F{speed}"
        return return_string + " \n"

    #Check if you are within the stage
    def coordinates_within_bounds(self,x,y,z):
        within_x_bounds = within_y_bounds = within_z_bounds = False
        if x is None: 
            within_x_bounds = True
        elif self.X_LOW_BOUND <= x <= self.X_HIGH_BOUND:
            within_x_bounds = True
        if y is None: 
            within_y_bounds = True
        elif self.Y_LOW_BOUND <= y <= self.Y_HIGH_BOUND:
            within_y_bounds = True
        if z is None: 
            within_z_bounds = True
        elif self.Z_LOW_BOUND <= z <= self.Z_HIGH_BOUND:
            within_z_bounds = True
        return within_x_bounds and within_y_bounds and within_z_bounds

    #Wake up the CNC machine
    def wake_up(self,ser):
        ser.write(str.encode("\r\n\r\n"))
        time.sleep(1)  # Wait for cnc to initialize
        ser.flushInput()  # Flush startup text in serial input

    def open(self):
        if self.VIRTUAL:
            return
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE)
        self.wake_up(self._ser)

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    #Follows a gcode path, will execute as many commands at a time as the buffer is set to
    #If the buffer is too high, the machine may not complete all the commands
    def follow_gcode_path(self,gcode, buffer=20):
        if not self.VIRTUAL:
            self.open()
            ser = self._ser
            out_strings = []
            commands = gcode.split('\n')
            for i in range (0, math.ceil(len(commands)/buffer)):
                try:
                    buffered_commands = commands[i*buffer:(i+1)*buffer]
                except:
                    buffered_commands = commands[i*buffer:]
                buffered_gcode = ""
                for j in range (0, len(buffered_commands)):
                    buffered_gcode += buffered_commands[j]
                    buffered_gcode += "\n"

                command = str.encode(buffered_gcode)
                ser.write(command)
                self.wait_for_movement_completion(ser, buffered_gcode)
                grbl_out = ser.readline()  # Wait for response
                out_string =grbl_out.strip().decode('utf-8')
                out_strings.append(out_string)
                print("Movement commands rendered:", len(buffered_commands)+i*buffer)
            return out_strings
