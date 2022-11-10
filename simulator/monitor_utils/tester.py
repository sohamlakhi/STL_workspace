from geometry_utils import Pose, Point, Orientation

p = Point(1,2)
o = Orientation(1)

pose = Pose(p, o)


print(pose.point.get_nparray())