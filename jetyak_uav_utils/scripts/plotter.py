import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import sys
bag = rosbag.Bag(sys.argv[1])
x=[]
xc=[]
ts=[]
tsc=[]
for topic, msg, t in bag.read_messages(topics=['/tag_pose']):
	x.append(msg.pose.position.x-3)
	ts.append(t.to_sec())
for topic, msg, t in bag.read_messages(topics=['/dji_sdk/flight_control_setpoint_generic']):
	xc.append(msg.axes[0])
	tsc.append(t.to_sec())
bag.close()

plt.plot(ts,x)
plt.plot(tsc,xc)
plt.show()
