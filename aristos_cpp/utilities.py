import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance
from queue import Queue

# def mark_restricted_columns(arr):
#     """
#     Marks unsafe points that are restricted in the column they belong to by obstacles (1) both above and below,
#     but marks rows as reachable if there is at least one free cell (0) between these obstacles.
#     After marking restricted columns, demarks the cells if the row they belong to is not bounded both left and right by obstacles.

#     Parameters
#     ----------
#     arr : np.ndarray
#         The 2D numpy array where unsafe points need to be marked.

#     Returns
#     -------
#     np.ndarray
#         A numpy array with unsafe points marked as 1, but demarked if the row is not bounded left and right.
#     """
#     rows, cols = arr.shape
#     result = np.zeros_like(arr)
    
#     # Step 1: Mark restricted columns based on the original criteria
#     for col in range(cols):
#         # Identify the rows that are unsafe and need to be checked
#         unsafe_rows = np.where(arr[:, col] == -1)[0]
#         if len(unsafe_rows) == 0:
#             continue
        
#         # Check if the column has obstacles (1) both above and below the unsafe points
#         for row in unsafe_rows:
#             # Check above
#             above = np.any(arr[:row, col] == 1)
#             # Check below
#             below = np.any(arr[row+1:, col] == 1)
            
#             if above and below:
#                 # Check for free cells between obstacles above and below
#                 above_obstacle_index = np.max(np.where(arr[:row, col] == 1)[0]) if np.any(arr[:row, col] == 1) else -1
#                 below_obstacle_index = np.min(np.where(arr[row+1:, col] == 1)[0]) + row + 1 if np.any(arr[row+1:, col] == 1) else rows
                
#                 # Check if there's at least one free cell (0) between the obstacles
#                 if np.any(arr[above_obstacle_index+1:below_obstacle_index, col] == 0):
#                     result[row, col] = 0  # Mark as reachable (0)
#                 else:
#                     result[row, col] = 1  # Mark as restricted (1)

#     # Step 2: Demark cells that are restricted if their row is not bounded left and right by obstacles
#     for row in range(rows):
#         for col in range(cols):
#             if result[row, col] == 1:  # Check if the cell was marked as restricted
#                 # Check left
#                 left = np.any(arr[row, :col] == 1)
#                 # Check right
#                 right = np.any(arr[row, col+1:] == 1)
                
#                 if not (left and right):
#                     result[row, col] = 0  # Demark the cell as reachable (0)
    
#     return result

def mark_restricted_columns(arr):
    """
    Marks unsafe points that are restricted in the column they belong to by obstacles (1) both above and below,
    but marks rows as reachable if there is at least one free cell (0) between these obstacles.

    Parameters
    ----------
    arr : np.ndarray
        The 2D numpy array where unsafe points need to be marked.

    Returns
    -------
    np.ndarray
        A numpy array with unsafe points marked as 1.
    """
    rows, cols = arr.shape
    result = np.zeros_like(arr)
    
    for col in range(cols):
        # Identify the rows that are unsafe and need to be checked
        unsafe_rows = np.where(arr[:, col] == -1)[0]
        if len(unsafe_rows) == 0:
            continue
        
        # Check if the column has obstacles (1) both above and below the unsafe points
        for row in unsafe_rows:
            # Check above
            above = np.any(arr[:row, col] == 1)
            # Check below
            below = np.any(arr[row+1:, col] == 1)
            
            if above and below:
                # Check for free cells between obstacles above and below
                above_obstacle_index = np.max(np.where(arr[:row, col] == 1)[0]) if np.any(arr[:row, col] == 1) else -1
                below_obstacle_index = np.min(np.where(arr[row+1:, col] == 1)[0]) + row + 1 if np.any(arr[row+1:, col] == 1) else rows
                
                # Check if there's at least one free cell (0) between the obstacles
                if np.any(arr[above_obstacle_index+1:below_obstacle_index, col] == 0):
                    result[row, col] = 0  # Mark as reachable (0)
                else:
                    result[row, col] = 1  # Mark as restricted (1)

    return result

def flood_fill(arr, x, y, marker):
    """
    Performs a flood-fill operation on a given 2D numpy array starting from the position (x, y).

    Parameters
    ----------
    arr : np.ndarray
        The 2D numpy array to perform the flood-fill operation on.
    x : int
        The x-coordinate of the starting point of the flood-fill.
    y : int
        The y-coordinate of the starting point of the flood-fill.
    marker : int
        The marker used to label the connected area during the flood-fill operation.
    """
    # Check if the current position is out of bounds of the array
    if x < 0 or x >= arr.shape[0] or y < 0 or y >= arr.shape[1]:
        return
     # Check if the current position is not a valid point to fill
    if arr[x, y] != 0 and arr[x, y] != -1:
        return
    
    arr[x, y] = marker
    
    # Recursively call flood_fill for all neighboring cells. 
    # Neighbors are cells that are connected horizontally or vertically.
    flood_fill(arr, x+1, y, marker)
    flood_fill(arr, x-1, y, marker)
    flood_fill(arr, x, y+1, marker)
    flood_fill(arr, x, y-1, marker)
    
    return

def get_edit_unreachable_points(arr, position):
    """
    Identifies and returns a NumPy array of the same dimension as the initial array, 
    filled with 1s for all the unreachable points from the current position.

    Parameters
    ----------
    arr : np.ndarray
        The 2D numpy array from which unreachable points are identified.
    position : tuple
        The starting position (x, y) from where unreachable points are identified.

    Returns
    -------
    arr_copy : np.ndarray
        A numpy array with 1s at unreachable points and 0s elsewhere.
    """

    marker = 2  # Start marker for labeling unreachable areas
    
    # Copy the original array to avoid modifying it
    x, y = position
    arr_copy = arr.copy()
    
    # Perform flood-fill from the current position
    flood_fill(arr_copy, y, x, marker)
    
    
    # Replace reachable points (0s or -1s) with 1s in the copied array
    arr_copy[(arr_copy == 0) | (arr_copy == -1)] = 1
    arr_copy[arr_copy == marker] = 0

    return arr_copy

def get_unreachable_points(arr, position):
    """
    Identifies and returns a NumPy array of the same dimension as the initial array, 
    filled with 1s for all the unreachable points from the current position.

    Parameters
    ----------
    arr : np.ndarray
        The 2D numpy array from which unreachable points are identified.
    position : tuple
        The starting position (x, y) from where unreachable points are identified.

    Returns
    -------
    arr_copy : np.ndarray
        A numpy array with 1s at unreachable points and 0s elsewhere.
    """

    marker = 2  # Start marker for labeling unreachable areas
    
    # Copy the original array to avoid modifying it
    x, y = position
    arr_copy = arr.copy()
    
    # Perform flood-fill from the current position
    flood_fill(arr_copy, y, x, marker)
    
    # Mark unreachable points based on flood-fill
    flood_fill_result = (arr_copy != marker).astype(int)
    
    # Mark unsafe points based on column restrictions
    restricted_result = mark_restricted_columns(arr)
    
    # Combine both results
    unreachable_points = np.maximum(flood_fill_result, restricted_result)

    return unreachable_points

def get_soft_points(arr):
    """
    Identifies and returns a NumPy array of the same dimension as the initial array, 
    filled with 1s for all the soft points.

    Parameters
    ----------
    arr : np.ndarray
        The 2D numpy array from which soft points are identified.

    Returns
    -------
    arr_copy : np.ndarray
        A numpy array with 1s at soft points and 0s elsewhere.
    """

    # Copy the original array to avoid modifying it
    arr_copy = arr.copy()
    
    # Replace all points with 0 except from -1s with 1s
    arr_copy[arr_copy != -1] = 0    
    arr_copy[arr_copy == -1] = 1
    
    return arr_copy

def patch_array(arr1, arr2, x0, y0):
    """
    Patches a given 2D array onto another one at the specified position.

    Parameters
    ----------
    arr1 : np.ndarray
        The 2D numpy array to be patched.
    arr2 : np.ndarray
        The 2D numpy array to be patched onto.
    x0 : int
        The x-coordinate of the starting position to patch arr1 onto arr2.
    y0 : int
        The y-coordinate of the starting position to patch arr1 onto arr2.

    Returns
    -------
    np.ndarray
        The resulting 2D numpy array after patching arr1 onto arr2.
    """

    arr2 = arr2.copy()
    n, m = arr1.shape
    i_range = slice(x0, x0 + m)
    j_range = slice(y0, y0 + n)
    try:
        arr2[j_range, i_range] = arr1
    except Exception as e:
        print(str(e))

    return arr2

def is_valid_move(x, y, grid):
    '''
    Checks if a move to the position (x, y) is valid within the given grid.
    '''
    rows, cols = grid.shape
    return (0 <= x < rows) and (0 <= y < cols) and (grid[x][y] != 1)

def bfs(grid, border_points):
    """
    Performs a breadth-first search on a grid from a set of starting points, 
    computing the distance from each starting point to every other point in the grid.

    Parameters
    ----------
    grid : np.ndarray
        The 2D numpy array representing the grid.
    border_points : list
        A list of tuples where each tuple represents a starting point for the BFS.

    Returns
    -------
    np.ndarray
        A 2D numpy array where each cell contains the minimum distance to one of the starting points.
    """

    rows, cols = grid.shape
    distance_map = np.full((rows, cols), np.inf)
    queue = Queue()
    
    for point in border_points:
        queue.put(point)
        distance_map[point] = 0

    moves = [(0, 1), (1, 0), (0, -1), (-1, 0)] 
    while not queue.empty():
        x, y = queue.get()
        for dx, dy in moves:
            new_x, new_y = x + dx, y + dy
            if is_valid_move(new_x, new_y, grid) and distance_map[x][y] + 1 < distance_map[new_x][new_y]:
                distance_map[new_x][new_y] = distance_map[x][y] + 1
                queue.put((new_x, new_y))
    return distance_map


def find_central_point(grid):
    """
    Finds the most centrally located reachable point in a given 2D grid.

    Parameters
    ----------
    grid : np.ndarray
        The 2D numpy array representing the grid.

    Returns
    -------
    tuple
        The coordinates (x, y) of the most centrally located reachable point in the grid.
    """

    rows, cols = grid.shape
    center = (rows // 2, cols // 2)
    
    # Get the border points of the grid and compute the distance map. 
    border_points = list((x, y) for x in range(rows) for y in [0, cols-1]) + list((x, y) for x in [0, rows-1] for y in range(cols))
    # Compute the distance map from the border points which will be used to find the closest point to the center.
    distance_map = bfs(grid, border_points)
    valid_points = np.argwhere(grid != 1)
    _, closest_point = min((distance.euclidean(center, point), tuple(point)) for point in valid_points if np.isfinite(distance_map[tuple(point)]))
    return closest_point
