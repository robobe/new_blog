from pymavlink import mavutil
# cast helpers
from typing import cast
from pymavlink.dialects.v20.common import MAVLink_message
from pymavlink.dialects.v20.ardupilotmega import MAVLink
from pymavlink.dialects.v20 import common
from pymavlink.mavutil import mavtcp

conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn = cast(mavtcp, conn)

def request_message_interval(conn: mavtcp, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """


    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        common.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )


# Make sure the connection is valid
conn.wait_heartbeat()

request_message_interval(conn, common.MAVLINK_MSG_ID_GPS_RAW_INT, 1)

# Get some information !
while True:
    try:
        msg = conn.recv_match(blocking=True)
        msg = cast(MAVLink_message, msg)
        if msg:
            print(msg.get_type(), msg.to_dict())
    except Exception as _:
        pass