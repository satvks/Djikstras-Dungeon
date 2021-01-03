from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    #   initial_position
    dist = {}
    prev = {}
    dist[initial_position] = 0
    prev[initial_position] = None

    queue = []
    heappush(queue, (0, initial_position)) 

    while queue:

        currentCost, current = heappop(queue)

        if current == destination:
            path = []
            if current == destination:
                while current:
                    path.append(current) 
                    current = prev[current]
                path.reverse()
            return path

        adjacent = adj(graph, current)

        for next, cost in adjacent:
            newCost = currentCost + cost
            if next not in dist or newCost < dist[next]:
                dist[next] = newCost
                prev[next] = current
                heappush(queue, (newCost, next))


    return None


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    dist = {}
    dist[initial_position] = 0

    queue = []
    heappush(queue, (0, initial_position)) 

    while len(queue) != 0:

        currentCost, current = heappop(queue)

        adjacent = adj(graph, current)

        for next, cost in adjacent:
            newCost = currentCost + cost
            if next not in dist or newCost < dist[next]:
                dist[next] = newCost
                heappush(queue, (newCost, next))

    return dist


def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.
    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    cell_x, cell_y = cell
    cell_cost = level['spaces'][cell]

    check_list = [(x,y) for x in [cell_x + 1, cell_x, cell_x - 1] 
                 for y in [cell_y + 1, cell_y, cell_y - 1] 
                 if not (x == cell_x and y == cell_y)]
    ret_list = []

    for space in check_list:
        space_x, space_y = space

        if space in level['spaces']:
            space_cost = level['spaces'][space]

            if (abs(space_x - cell_x) == 1) and (abs(space_y - cell_y) == 1):
                path_cost = (sqrt(2)*0.5) * space_cost + (sqrt(2)*0.5) * cell_cost
            else:
                path_cost = (0.5) * space_cost + (0.5) * cell_cost

            ret_list.append((space, path_cost))

    return ret_list


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!\n")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'test_maze.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_maze_costs.csv')
