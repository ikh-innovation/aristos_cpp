import os
from dataclasses import dataclass
from typing import Union, Optional, Tuple

import cv2
import numpy as np
import pygeos as pg
import yaml
from PIL import Image, ImageOps, ImageDraw

@dataclass
class ImageDefs:
    """
    This data class is used to hold the image definitions. It includes image path, resolution,
    origin point, thresholds for considering pixels as occupied or free, and a flag to negate the image or not.
    """

    image: str
    resolution: float
    origin: list
    occupied_thresh: float
    free_thresh: float
    negate: bool
    
    @classmethod
    def from_yaml(cls, yaml_file):
        location = os.path.dirname(yaml_file)
        with open(yaml_file, 'r') as f:
            defs = yaml.safe_load(f)
        image = defs['image'] if os.path.isabs(defs['image']) else os.path.join(location, defs['image'])
        return cls(
            image=image, 
            resolution=defs['resolution'], 
            origin=defs['origin'], 
            occupied_thresh=defs['occupied_thresh'],
            free_thresh=defs['free_thresh'],
            negate=defs['negate']
        )
    
@dataclass
class RobotDefs:
    """
    This data class is used to hold the robot definitions. It includes robot and mower dimensions.
    """

    robot_dim: float
    mower_dim: float

    @classmethod
    def from_yaml(cls, yaml_file: str):
        with open(yaml_file, 'r') as f:
            defs = yaml.safe_load(f)
        return cls(
            robot_dim=defs['robot_dim'],
            mower_dim=defs['mower_dim']
        )

class Parser:
    """
    This class is used for parsing and processing maps.
    """
    
    def __init__(self, hard_map: str, soft_map: str, robot_config: str, cell_dim: Optional[int] = None, occupied_cell_threshold: float = 0.5, use_disc: bool = False) -> None:
        """
        The initializer for the Parser class.

        Parameters
        ----------
        hard_map : str
            Path to the yaml file that defines the hard obstacles on the map.
        soft_map : str
            Path to the yaml file that defines the soft obstacles on the map.
        robot_config : str
            Path to the yaml file that defines the robot configuration.
        cell_dim : Optional[int]
            The dimension of the cells that the map grid will be split into. If None, it will be calculated based on the robot dimensions and map resolution.
        occupied_cell_threshold : float
            Threshold for considering a cell as occupied based on the percentage of occupied pixels in the cell.
        use_disc : bool
            Flag to use a disc-shaped structuring element for dilation and erosion instead of a rectangular one.
        """

        # Read the image definitions and robot definitions    
        self._obs_defs = ImageDefs.from_yaml(hard_map)
        self._soft_defs = ImageDefs.from_yaml(soft_map)
        self._robot_defs = RobotDefs.from_yaml(robot_config)

        # Set the cell dimension and occupied cell threshold
        self._cell_dim = self.default_cell_dim if cell_dim is None else cell_dim
        self._occupied_cell_threshold = occupied_cell_threshold
        self._use_disc = use_disc

        # Process the hard map only
        hard = 1*self._read(self._obs_defs)

        # Perform a morphological closing operation to fill small gaps (dilation followed by erosion to close small holes inside the foreground objects)
        kernel = np.ones((50, 50), np.uint8)
        hard = cv2.morphologyEx(hard.astype('uint8'), cv2.MORPH_CLOSE, kernel)
        hard = hard.astype('int')

        soft = 1*self._read(self._soft_defs)
        hard[(soft==1) & (hard!=1)] = -1 # Soft obstacles are there where hard obstacles
        
        self._grid = hard
        self._grid_dilated = self.dilate()  # dilate the grid to account for the robot's dimension
        self._grid_eroded = self.erode() # erode the grid after dilation to account for the mower's dimension
        self._grid_downscaled = None
        self._free_cell_areas = None
        
        self._selection = False
        self._polygon = None
        self._selected_grid = None
        self._selected_grid_dilated = None
        self._selected_grid_eroded = None
        self._selected_grid_downscaled = None
        self._selected_free_cell_areas = None

    @property
    def grid(self) -> np.ndarray:
        return self._grid if not self._selection else self._selected_grid
    
    @property
    def grid_dilated(self) -> np.ndarray:
        return self._grid_dilated if not self._selection else self._selected_grid_dilated
    
    @property
    def grid_eroded(self) -> np.ndarray:
        return self._grid_eroded if not self._selection else self._selected_grid_eroded
    
    @property
    def grid_downscaled(self) -> np.ndarray:
       return self._grid_downscaled if not self._selection else self._selected_grid_downscaled
    
    @property
    def default_cell_dim(self) -> int:
        map_resolution = self._obs_defs.resolution
        mower_dim = self._robot_defs.mower_dim
        return np.ceil(mower_dim/map_resolution).astype(int)
    
    def _read(self, defs: ImageDefs) -> np.ndarray:
        """
        Reads the image file and returns a binary numpy array based on the thresholds defined in the image definitions.
        
        Parameters
        ----------
        defs : ImageDefs
            The image definitions.
        
        Returns
        -------
        np.ndarray
            The binary numpy array representing the image.
        """
        # Open the image file and convert it to a binary numpy array based on the thresholds
        # where 1 represents occupied and 0 represents free. If negate is set to True, the values are inverted.
        im = ImageOps.grayscale(Image.open(defs.image))
        arr = (255 - np.array(im)) / 255.0 if not defs.negate else np.array(im) / 255.0
        im.close()
        return arr>=defs.occupied_thresh

    def show(self, map_type: str = 'original', reverse: bool = False, draw_contour: bool = False) -> Image.Image:
        """
        Returns both the total grid and the selected grid as an Image object. Requires the coverage_planner to be run first.
        
        Parameters
        ----------
        map_type : str
            The type of the map to show. It can be 'original', 'dilated', 'eroded', or 'downscaled'.
        reverse : bool
            If set to True, it will reverse the colors of the grid.
        draw_contour : bool
            If set to True, it will draw a red contour around the selected grid region.

        Returns
        -------
        Image.Image 
            The total grid and the selected grid as an Image object.
        """
        if map_type == 'dilated':
            arr_total =  self._grid_dilated
            arr_selected = self._selected_grid_dilated
        elif map_type == 'eroded':
            arr_total = self._grid_eroded
            arr_selected = self._selected_grid_eroded
        elif map_type == 'original':
            arr_total = self._grid
            arr_selected = self._selected_grid
        elif map_type == 'downscaled':
            # arr_total = self._grid_downscaled
            arr_selected = self._selected_grid_downscaled
        else:
            raise ValueError(f'Invalid grid type: {map_type}')
        
        if not map_type == 'downscaled':
            arr_total = arr_total.astype(np.float64)
            arr_total[arr_total == -1] = 0.5
            arr_total = ((1 - arr_total) * 255).astype(np.uint8) if reverse else (arr_total * 255).astype(np.uint8)
            im_total = Image.fromarray(arr_total)
        else: 
            im_total = None 
        arr_selected = arr_selected.astype(np.float64)
        arr_selected[arr_selected == -1] = 0.5
        arr_selected = ((1 - arr_selected) * 255).astype(np.uint8) if reverse else (arr_selected * 255).astype(np.uint8)
        im_selected = Image.fromarray(arr_selected)
        
        # Draw the contour around the selected region
        if draw_contour:
            draw = ImageDraw.Draw(im_total)
            
            for i in range(len(self._polygon)):
                draw.line([tuple(self._polygon[i]), tuple(self._polygon[(i+1) % len(self._polygon)])], fill= 128, width=15)

        return im_total, im_selected

    def pixel_to_meter(self, coordinates: Tuple[int]) -> Tuple[float]:
        """
        Converts a pair of coordinates from pixel to meters.

        Parameters
        ----------
        coordinates : Tuple[int]
            The coordinates in pixels.
        
        Returns
        ------- 
        Tuple[float]
            The coordinates in meters.
        """
        resolution = self._obs_defs.resolution
        x0, y0, _ = self._obs_defs.origin
        x, y = coordinates
        return (x*resolution + x0, y*resolution + y0)

    def meter_to_pixel(self, coordinates: Tuple[float]) -> Tuple[int]:
        """
        Converts a pair of coordinates from meters to pixels.
        
        Parameters
        ----------
        coordinates : Tuple[float]
            The coordinates in meters.
        
        Returns
        -------
        Tuple[int]
            The coordinates in pixels.
        """
        resolution = self._obs_defs.resolution
        x0, y0, _ = self._obs_defs.origin
        x, y = coordinates
        return (round((x - x0) / resolution), round((y - y0) / resolution))

    def _reverse_polygon(self, polygon: Union[np.ndarray, list]) -> np.ndarray:
        """
        Reverse y coordinates of a polygon so that axis origin is upper left.

        Parameters
        ----------
        polygon : Union[np.ndarray, list]
            The vertices of the polygon that defines the region to select. Can be either a list or a numpy array.
        
        Returns
        -------
        np.ndarray
            The reversed polygon.
        """
        maxy, _ = self._grid.shape
        reversed = np.array([[c[0], max(0, maxy - c[1])] for c in polygon])
        return reversed
    
    def select(self, polygon: Union[np.ndarray, list], unit: str = 'meter') -> None:
        """
        Selects a region of the grid defined by a polygon. 

        Parameters
        ----------
        polygon : Union[np.ndarray, list]
            The vertices of the polygon that defines the region to select. Can be either a list or a numpy array.
        unit : str
            The unit of the polygon. It can be 'meter' or 'pixel'. If 'meter', the coordinates of the polygon are in meters.

        Raises
        ------
        ValueError
            If the unit is not 'meter' or 'pixel'.
            If the polygon is out of the map boundaries.
        """

        def select_grid(grid, polygon: np.ndarray):      
            xmin, ymin = np.nanmin(polygon, axis=0)
            xmax, ymax = np.nanmax(polygon, axis=0)
            polygon_geo = pg.polygons(polygon - np.array([[xmin, ymin]*polygon.shape[0]]).reshape(polygon.shape))
            selected_grid = grid[ymin:ymax,xmin:xmax]
            selected_grid = np.array([
                [
                    selected_grid[j][i] if selected_grid[j][i] == 1 or pg.intersects(pg.points([i, j]), polygon_geo) else 0
                    for i in range(0, selected_grid.shape[1])
                ] for j in range(0, selected_grid.shape[0])
            ])
            return selected_grid

        if unit not in ['meter', 'pixel']:
            raise ValueError("`unit` should be one of 'meter', 'pixel'")

        if unit == 'meter':
            polygon = np.array([self.meter_to_pixel(coordinates) for coordinates in polygon])

        polygon = self._reverse_polygon(polygon)

        self._selected_grid = select_grid(self._grid, polygon)
        self._selected_grid_dilated = select_grid(self._grid_dilated, polygon)
        self._selected_grid_eroded = select_grid(self._grid_eroded, polygon) 
        self._selection = True
        self._polygon = polygon
        
        self._selected_grid_downscaled = self.downscale_grid()
        self.show(map_type='downscaled', reverse=True)

    def dilate(self) -> np.ndarray:
        """
        Performs a dilation operation on the grid based on the robot's dimension. 

        Returns
        -------
        np.ndarray
            The dilated grid as a numpy array.
        """
        
        boolean_grid = (self._grid > 0).astype(np.uint8)
        soft = 1*self._read(self._soft_defs)
        
        map_resolution = self._obs_defs.resolution
        robot_dim = self._robot_defs.robot_dim

        dim = np.ceil(robot_dim/map_resolution).astype(int)
        if self._use_disc:
            struct_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dim, dim))
        else:
            struct_elem = cv2.getStructuringElement(cv2.MORPH_RECT, (dim, dim))
        dilated_binary = cv2.dilate(boolean_grid, struct_elem)

        non_hard = np.where((dilated_binary == 0) & (soft == 1))
        dilated_grid = dilated_binary.copy().astype(np.int64)
        dilated_grid[non_hard[0], non_hard[1]] = -1
                            
        return dilated_grid
    
    def erode(self) -> np.ndarray:
        """
        Performs an erosion operation on the grid based on the mowers's dimension.

        Returns
        -------
        np.ndarray
            The eroded grid as a numpy array.
        """
        
        boolean_grid = (self._grid_dilated > 0).astype(np.uint8)
        soft = 1*self._read(self._soft_defs)

        map_resolution = self._obs_defs.resolution
        mower_dim = self._robot_defs.mower_dim

        dim = np.ceil(mower_dim/map_resolution).astype(int)
        if self._use_disc:
            struct_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dim, dim))
        else:
            struct_elem = cv2.getStructuringElement(cv2.MORPH_RECT, (dim, dim))
        eroded_binary = cv2.erode(boolean_grid, struct_elem)

        non_hard = np.where((eroded_binary == 0) & (soft == 1))
        eroded_grid = eroded_binary.copy().astype(np.int64)
        eroded_grid[non_hard[0], non_hard[1]] = -1

        return eroded_grid

    def pad_and_expand(self, selected_grid: np.ndarray, pad_height: int, pad_width: int, pad_grid: Optional[np.ndarray] = None, polygon: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Expands a selected map area based on the given parameters. 

        Parameters
        ----------
        selected_grid : np.ndarray
            The grid to be expanded.
        pad_height : int
            The height to pad the grid.
        pad_width : int
            The width to pad the grid.
        pad_grid : Optional[np.ndarray]
            If provided, pad_grid will be used to expand the original grid.
        polygon : Optional[np.ndarray]
            If pad_grid is provided, this should also be provided. It's used to locate the selected area in the larger pad_grid which contains it.

        Returns
        -------
        np.ndarray
            The expanded grid as a numpy array.
        """

        height, width = selected_grid.shape
  
        if pad_grid is None:
            padded_image = np.pad(selected_grid, ((0, pad_height), (0, pad_width)), mode='constant', constant_values=0)
            return padded_image
        
        if polygon is None:
            raise Exception("polygon must be provided if pad_grid is provided")

        if isinstance(polygon, list):
            polygon = np.array(polygon)

        xmin, ymin = np.nanmin(polygon, axis=0)
        xmax, ymax = np.nanmax(polygon, axis=0)

        padded_image = np.zeros((height + pad_height, width + pad_width))
        padded_image[:height, :width] = selected_grid

        if pad_height > 0:
            padded_image[height:, :width] = pad_grid[ymax: ymax + pad_height, xmin: xmin + width]
        if pad_width > 0:
            padded_image[:height + pad_height, width:] = pad_grid[ymin: ymin + height + pad_height, xmax: xmax + pad_width]

        return padded_image

    def split_to_cells(self, grid: np.ndarray, grid_pad: Optional[np.ndarray] = None, polygon: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Splits a grid into cells of a certain dimension. The grid can be padded before the splitting operation.

        Parameters
        ----------
        grid : np.ndarray
            The grid to be split.
        grid_pad : Optional[np.ndarray]
            If provided, it will be used to pad the grid before splitting.
        polygon : Optional[np.ndarray]
            If grid_pad is provided, this should also be provided. It's used to select the padding area from grid_pad.

        Returns
        -------
        np.ndarray
            The split grid as a numpy array.
        """
        # Pad the grid if needed when splitting because the grid should be divisible by the cell dimension
        height, width = grid.shape
        pad_height = (self._cell_dim - height % self._cell_dim) % self._cell_dim
        pad_width = (self._cell_dim - width % self._cell_dim) % self._cell_dim
        
        if grid_pad is None:
            padded_image = self.pad_and_expand(grid, pad_height, pad_width)
        else:
            padded_image = self.pad_and_expand(grid, pad_height, pad_width, grid_pad, polygon)

        # Split the padded grid into cells 
        padded_height, _ = padded_image.shape
        cells = padded_image.reshape(padded_height//self._cell_dim, self._cell_dim, -1, self._cell_dim).swapaxes(1,2)
        
        return cells

    def downscale_grid(self, occupied_cell_threshold: Optional[float] = None) -> None:
        """
        Downscales the grid by replacing each cell with a single value that indicates whether it's occupied or not based on a threshold.

        Parameters
        ----------
        occupied_cell_threshold : Optional[float]
            Threshold for considering a cell as occupied based on the percentage of occupied pixels in the cell.
        """
        
        if occupied_cell_threshold is None:
            occupied_cell_threshold = self._occupied_cell_threshold

        # Split the dilated grid into cells 
        dilated_cells = self.split_to_cells(self.grid_dilated, self._grid_dilated, self._polygon)
        
        # Replace each cell with a single value based on the percentage of occupied pixels in the cell
        cell_obstacle_count = np.count_nonzero(dilated_cells==1, axis=(2,3))
        cell_obstacle_perc = cell_obstacle_count / self._cell_dim**2
        downscaled_grid = (cell_obstacle_perc > occupied_cell_threshold)

        # replace boolean values with 1 and 0
        downscaled_grid = downscaled_grid.astype(np.int64)
        
        # Now we have only occupied and free cells, we need to replace some cells with soft obstacles
        # if the percentage of soft obstacles in the cell is greater than the threshold then the cell is considered as a soft obstacle
        soft_cell_threshold = 0.05
        cell_soft_count = np.count_nonzero(dilated_cells==-1, axis=(2,3))
        cell_soft_perc = cell_soft_count / self._cell_dim**2
        downscaled_grid[cell_soft_perc > soft_cell_threshold] = -1
        
        # print number of -1 values
        print(f"Number of soft cells: {np.count_nonzero(downscaled_grid == -1)}")
        # print(f"Number of occupied cells: {np.count_nonzero(downscaled_grid == 1)}")
        
        return downscaled_grid
