# Algorithm for coming up with which points on the path to target
from HermiteSpline import createPoints

start_point = (7, 4, 0)
initial_velo = (3, 5, 12)
end_point = (0, 0, 45)

x, y, z = createPoints(start_point, initial_velo, end_point)
