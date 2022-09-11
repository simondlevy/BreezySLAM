from breezyslam.algorithms import RMHC_SLAM
from python.breezyslam.sensors import MyLidar

lidar = MyLidar()
mapbytes = bytearray(800*800)
slam = RMHC_SLAM(lidar, 800, 35) 

while True:

    scan = readLidar()

    slam.update(scan)

    x, y, theta = slam.getpos(scan)

    slam.getmap(mapbytes)