rosservice call /set_waypoints "waypoints:
  waypoints:
  - {lat: 0.0, lon: 0.0, alt: 2.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 1.0, lon: 0.0, alt: 2.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 1.0, lon: 1.0, alt: 2.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 1.0, lon: 1.0, alt: 3.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 0.0, lon: 1.0, alt: 3.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 0.0, lon: 0.0, alt: 3.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}
  - {lat: 0.0, lon: 0.0, alt: 2.0, heading: 0.0, radius: 0.1, loiter_time: 2.0}"

# 0,0,2
# 1,0,2
# 1,1,2
# 1,1,3
# 0,1,3
# 0,0,3
# 0,0,2
