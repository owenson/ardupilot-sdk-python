import sys, serial,time,thread

from pymavlink import mavlinkv10 as mavlink, mavutil

class ArduPilot:
 def __init__(self, port, baud, verbose=True):
   self.verbose = verbose

   self.vprint("Connecting...")
   self.mav = mavutil.mavlink_connection(port, baud, autoreconnect=True)
   self.vprint("Waiting for first heartbeat")
   self.mav.wait_heartbeat()

   self.modes = self.mav.mode_mapping()
   self.lastSentHeartbeat = time.time()

   self.vprint("Starting message handler thread")
   thread.start_new_thread(self.mavMsgHandler, (self.mav,))


 #handle incoming messages
 def mavMsgHandler(self, m):
   while True:
     msg= m.recv_msg()

     #send heartbeats to autopilot
     if time.time() - self.lastSentHeartbeat > 1.0:
       self.mav.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)       
       self.lastSentHeartbeat = time.time()


     if msg is None or msg.get_type() == "BAD_DATA" :
       time.sleep(0.01)
       continue

     #enable data streams after start up - can't see another way of doing this.     
     if msg.get_type() == "STATUSTEXT" and "START" in msg.text:
       self.vprint( msg.text)
       self.vprint("Enabling data streams")
       self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA1)
       self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTENDED_STATUS)
       self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA2)
       self.setDataStreams(mavlink.MAV_DATA_STREAM_POSITION)
       
     if "ACK" in msg.get_type():
       print msg

 def flyTo(self, lat, lon, alt):
   #																				seqfrm cmd                          cur at p1 p2 p3 p4  x    y    z
   self.mav.mav.mission_item_send(self.mav.target_system, self.mav.target_component, 0, 0, mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 5, 0, 0, 0, lat, lon, alt)


 def takeoff(self, alt=5):
   #																				seqfrm cmd                          cur at p1 p2 p3 p4  x    y    z
   self.mav.mav.mission_item_send(self.mav.target_system, self.mav.target_component, 0, 0, mavlink.MAV_CMD_NAV_TAKEOFF, 2,  0, 0, 0, 0, 0, 0, 0, alt)              

 def land(self):
   self.mav.mav.command_long_send(
                            self.mav.target_system,  # target_system
                            mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
                            mavlink.MAV_CMD_NAV_LAND, # command
                            0, # nout
                            1, # param1 
                            0, # param2 
                            0, # param3
                            0, # param4
                            0, # param5
                            0, # param6
                            0) # param7 

 def arm(self):
   self.mav.arducopter_arm()
 
 def disarm(self):
   self.mav.arducopter_disarm()
   
 def isArmed(self):
   return self.mav.motors_armed()
      
 def setDataStreams(self, which=mavlink.MAV_DATA_STREAM_ALL, freq=4, enable=True):
   self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component, which, freq, 1 if enable else 0)

 def vprint(self, str):
   if self.verbose:
     print str

 def getMode(self):
   return self.mav.flightmode
 
 def setMode(self, mode):
   if mode not in self.modes:
     raise NameError("Unknown mode: "+mode+ " Valid modes are:" + ','.join(self.modes.keys()))

   retry = 5

   while self.mav.flightmode != mode and retry > 0:   
     self.mav.set_mode(mode)
     time.sleep(1)
     retry -= 1
   
   if self.mav.flightmode != mode:
     raise Warning("Failed to set mode")


 def getAttitude(self):
   if 'ATTITUDE' not in self.mav.messages:
     raise Error("Haven't received attitude information yet")
   att = self.mav.messages['ATTITUDE']
   return self.dictCopy(att, ['pitch','roll','yaw'])
   
 def getAltitude(self):
   alt = self.mav.field('VFR_HUD', 'alt', None)
   if alt is None:
     raise Error("Haven't received that information yet")
   
   return alt
   
 def getHeading(self):
   head = self.mav.field('VFR_HUD', 'heading', None)
   if head is None:
     raise Error("Haven't received that information yet")
   
   return head
 
 def getSpeed(self):
   spd = self.mav.field('VFR_HUD', 'airspeed', None)
   if spd is None:
     raise Error("Haven't received that information yet")
   
   return spd
 
 def getLocation(self):
   if 'GPS_RAW_INT' not in self.mav.messages:
     raise Error("Haven't received that information yet")
   
   gps = self.mav.messages['GPS_RAW_INT']
   
   nd = self.dictCopy(gps, ['fix_type', 'lat', 'lon'])
   nd['lat'] = nd['lat'] / 10e6;
   nd['lon'] = nd['lon'] / 10e6;
   return nd
   
 def dictCopy(self, src, keys):
   newDict = {}
   for k in keys:
     newDict[k] = getattr(src,k)
   return newDict

from ardupilot import *
ac = ArduPilot("/dev/ttyACM0", 115200)











   
#  def getLocation(self):
#     return (self.mav.

#time.sleep(5)

#m.wait_heartbeat()

#m.wait_heartbeat()

#m.mav.request_data_stream_send(m.target_system, m.target_component, mavlink.MAV_DATA_STREAM_ALL, 4, 1) 

#m.mav.mission_request_list_send(m.target_system, m.target_component)

#modes = m.mode_mapping()

#m.param_fetch_all()


#  type = msg.get_type()

#  if type == "PARAM_VALUE":
#    params[msg.param_id] = msg

#  if type == "HEARTBEAT":
#    print "MODE: ", m.flightmode #mode_string_v10(msg)
#  print msg
