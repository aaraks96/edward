from src.map import maps
from src.search import search
from src.robot import edward_bot
from src.visualize import display
from src.Map import Map_plot as map
# import cv2

M = map(150, 300, 5, 15, 4)

E = edward_bot(5,5,30,0, M)
resolution = 10

Path1 = search([200,120], [10,10],10, M,E,1)
path1 = Path1.A_star()

# D1 = display(Path1.start_position, Path1.goal_position, M.x_min,M.y_min,M.x_max,M.y_max)
# obstacle_list = []
# D1.display_map(path1,Path1.visited_node,obstacle_list)

