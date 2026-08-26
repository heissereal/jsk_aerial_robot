#!/usr/bin/env python3

import serial
import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus


def parse_gngga(sentence):
    """Parse GNGGA/GPGGA sentence for position and fix quality."""
    parts = sentence.split(',')

    if len(parts) < 15:
        return None

    try:
        time_str = parts[1]
        lat = parts[2]
        lat_dir = parts[3]
        lon = parts[4]
        lon_dir = parts[5]

        fix_quality = int(parts[6]) if parts[6] else 0
        num_satellites = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 0.0
        altitude = float(parts[9]) if parts[9] else 0.0

        return {
            'time': time_str,
            'lat': lat,
            'lat_dir': lat_dir,
            'lon': lon,
            'lon_dir': lon_dir,
            'fix_quality': fix_quality,
            'satellites': num_satellites,
            'hdop': hdop,
            'altitude': altitude
        }

    except (ValueError, IndexError):
        return None


def convert_to_degrees(value, direction):
    """Convert NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees."""

    if not value:
        return None

    try:
        if direction in ['N', 'S']:
            degrees = int(value[:2])
            minutes = float(value[2:])
        elif direction in ['E', 'W']:
            degrees = int(value[:3])
            minutes = float(value[3:])
        else:
            return None

        decimal = degrees + minutes / 60.0

        if direction in ['S', 'W']:
            decimal = -decimal

        return decimal

    except (ValueError, IndexError):
        return None


class GPSNode:
    def __init__(self):
        # ROS parameter
        self.port = rospy.get_param('~port', '/dev/gps')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.frame_id = rospy.get_param('~frame_id', 'gps')

        # Publisher
        self.fix_pub = rospy.Publisher(
            '/gps/fix',
            NavSatFix,
            queue_size=10
        )

        rospy.loginfo(
            "Connecting to %s at %d baud...",
            self.port,
            self.baudrate
        )

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1
            )
            # Enable NMEA messages including GGA
            command = "$PCAS03,1,1,1,1,1,1,1,1,0,0,,,0,0*02\r\n"
            self.ser.write(command.encode("ascii"))
            self.ser.flush()

            rospy.loginfo("Sent NMEA output configuration")

        except serial.SerialException as e:
            rospy.logfatal(
                "Could not open serial port %s: %s",
                self.port,
                str(e)
            )
            raise

        rospy.loginfo("Connected! Waiting for GPS data...")

        self.line_buffer = ""

    def publish_fix(self, gga_data):
        lat = convert_to_degrees(
            gga_data['lat'],
            gga_data['lat_dir']
        )

        lon = convert_to_degrees(
            gga_data['lon'],
            gga_data['lon_dir']
        )

        # Position has not been obtained yet
        if lat is None or lon is None:
            return

        msg = NavSatFix()

        # Header
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        # GPS status
        if gga_data['fix_quality'] > 0:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = gga_data['altitude']

        # Rough covariance estimate using HDOP
        #
        # HDOP itself is not directly a position variance, so this is only
        # an approximate indication of uncertainty.
        if gga_data['hdop'] > 0:
            variance = gga_data['hdop'] ** 2

            msg.position_covariance = [
                variance, 0.0,      0.0,
                0.0,      variance, 0.0,
                0.0,      0.0,      variance * 4.0
            ]

            msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            )

        else:
            msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )

        self.fix_pub.publish(msg)

        rospy.loginfo_throttle(
            1.0,
            "GPS FIX: lat=%.9f lon=%.9f alt=%.2f m "
            "sat=%d hdop=%.2f fix=%d",
            lat,
            lon,
            gga_data['altitude'],
            gga_data['satellites'],
            gga_data['hdop'],
            gga_data['fix_quality']
        )

    def run(self):
        while not rospy.is_shutdown():

            try:
                if self.ser.in_waiting > 0:

                    data = self.ser.read(
                        self.ser.in_waiting
                    ).decode(
                        'utf-8',
                        errors='ignore'
                    )

                    self.line_buffer += data

                    while '\n' in self.line_buffer:

                        line, self.line_buffer = \
                            self.line_buffer.split('\n', 1)

                        line = line.strip()

                        # L76K may output GN or GP talker IDs
                        if line.startswith(('$GNGGA', '$GPGGA')):

                            gga_data = parse_gngga(line)

                            if gga_data:
                                self.publish_fix(gga_data)

            except serial.SerialException as e:
                rospy.logerr(
                    "Serial communication error: %s",
                    str(e)
                )
                break

            except Exception as e:
                rospy.logwarn(
                    "GPS parsing error: %s",
                    str(e)
                )

            rospy.sleep(0.001)

        self.ser.close()


def main():
    rospy.init_node('gps_monitor')

    try:
        gps_node = GPSNode()
        gps_node.run()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
