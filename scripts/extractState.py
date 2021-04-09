#!/usr/bin/python
print("Hello")
import rosbag
from jetyak_uav_utils.msg import ObservedState
import sys
print(sys.argv)
assert len(sys.argv)==3, "python extractState.py input_bag output_csv"
bagName = sys.argv[1]
bag = rosbag.Bag(bagName, 'r')
out = open(sys.argv[2],"w")
for topic,msg,t in bag.read_messages(topics="/jetyak_uav_vision/state"):
	out.write("%f,%f,%f,%f,%f,%f,%f\n"%(t.to_sec(),msg.drone_p.x,msg.drone_p.y,msg.drone_p.z,msg.boat_p.x,msg.boat_p.y,msg.boat_p.z))
out.close()
bag.close()
