ardupilot-sdk-python
====================

SDK for controlling ardupilot.

# Example:

ac = ArduPilot(SERIALPORT, BAUD)

ac.arm()

ac.takeoff()

ac.flyTo(lat, lon, alt)

ac.land()

