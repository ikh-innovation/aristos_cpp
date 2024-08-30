import os
from shutil import rmtree
from typing import Union, List
import numpy as np
from aristos_cpp.map import Parser
from aristos_cpp.solver import Solver
from aristos_cpp.router import Router

class CoveragePlanner:

    def __init__(self, hard_map: str, soft_map: str, robot_config: str, **kwargs) -> None:
        ##1. MAP INITIALIZATION
        self._map = Parser(hard_map, soft_map, robot_config, **kwargs)
        self._solver = None

    @property
    def map(self):
        return self._map

    @property
    def solver(self):
        return self._solver

    def run(self, polygon: Union[np.ndarray, List], initial_position=[0, 0], direction='dnorth', unit: str = 'meter'):
        """
        Run the coverage planner algorithm on the given polygon.
        
        Parameters
        ----------
        polygon : Union[np.ndarray, List]
            The polygon to be covered.
        initial_position : List, optional
            The initial position of the robot, by default [0, 0].
        direction : str, optional
            The initial direction of the robot, by default 'dnorth'.
        unit : str, optional
            The unit of the polygon, by default 'meter'.
        
        Returns
        -------
        np.ndarray
            The absolute path of the robot. It is the path in the real-world coordinates.
        List
            The rescaled path of the robot. It is the path in the high-dimensional grid.
        List
            The path of the robot. It is the path in the low-dimensional grid.
        """
        ## 2. SELECT POLYGON AREA OF THE MAP
        # Select the polygon and run the solver
        self._map.select(polygon, unit=unit)
        
        ## 3. RUN THE SOLVER ON THE SELECTED AREA
        self._solver = Solver(self._map.grid_downscaled, position=initial_position, direction=direction)
        self._solver.run()
        
        # Print solver info which returns return {'length': len(self._path), 'covered': f"{round(len(self._visited) / total_length * 100, 1)}%", 'totalLength': total_length, 'turns': self._turns}
        info = self._solver.info()
        # print("Path Info:", info)
        print("Path length: ", info['length'])
        print("Percentage covered: ", info['covered'])
        print("Total length: ", info['totalLength'])
        print("Number of turns: ", info['turns'])   
        # print("Number of unsafe turns: ", info['unsafeTurns'])
        
        ## 4. RESCALE THE PATH
        # Rescale the path and get the absolute path
        router = Router(self._map)
        rescaled_path = router.rescale_path(self._solver._path)
        self._rescaled_path = rescaled_path

        path = [d[0] for d in rescaled_path] # Get the coordinates of the path 
        path.append(rescaled_path[-1][1])
        
        path = np.array(path)

        # Compute absolute path by adding the minimum x and y values to the path coordinates and flipping the y values to match the map coordinates
        x0, y0 = np.nanmin(self._map._polygon, axis=0) # Get the minimum x and y values
        ymax, _ = self._map._grid.shape # Get the maximum y value of the map

        abs_path = [[c[0]+x0, ymax - (c[1]+y0)] for c in path] 
        # Convert the path to meters if the unit is meter
        if unit == 'meter':
            abs_path = np.array([self._map.pixel_to_meter(c) for c in abs_path]) 
        try:
            rmtree(self.solver._working_path)  # Remove the working directory of the solver 
        except:
            pass

        return abs_path, rescaled_path, self._solver._path
