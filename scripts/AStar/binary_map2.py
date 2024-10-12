#!/usr/bin/env python3

from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from AStar.a_star import a_star
from AStar.utils import plot_path


def path_planing(start_node,goal_node):

    gmap = OccupancyGridMap.from_png("/home/abd/ros_ws/src/my_robot_controll/scripts/AStar/ssss.png", 0.1)

    path, path_px = a_star(start_node, goal_node, gmap, movement='4N',meter=0)

    gmap.plot()

    if path:

        plot_path(path_px)
    else:
        print('Goal is not reachable')

        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.show()


path_planing((0,0),(200,93))