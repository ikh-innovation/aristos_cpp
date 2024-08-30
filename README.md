<a id="readme-top"></a>
<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#project-structure">Project Structure</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
# Coverage Path Planner for Aristos

## About The Project

//TODO:
This is a Python-based library designed to provide an advanced path planning and area coverage solution for Aristos robot, primarily focusing on efficiently managing solar park maintenance tasks. It utilizes unique partitioning strategies and adaptive path planning to maximize efficiency and accuracy in complex environments. 

It is a modified version of Tango - https://github.com/LIBRA-AI-Tech/tango but taking into account for constraints the solar panels impose.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

//TODO:
* [![UP][UnifiedPlanning]][UP-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

//TODO: To get a local copy up and running follow these simple example steps.

### Prerequisites

Planner is designed as a Python package, ensuring easy installation alongside its required dependencies.

To install the package, use a package manager like pip:

```bash
pip install git+https://github.com/ikh-innovation/aristos_cpp
```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

You have two options to install the Planner.
1) If you want to use it independently of ROS then you should install Planner as a Python package, ensuring easy installation alongside its required dependencies.

    To install the package, use a package manager like pip:

    ```bash
    pip install git+https://github.com/LIBRA-AI-Tech/tango
    ```
2) If you want to use it as ROS package then:
   
   1. Clone the repo
      ```bash
      git clone https://github.com/ikh-innovation/aristos_cpp.git
      ```
   2. //TODO: 
   
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- PROJECT STRUCTURE -->
## Project Structure

//TODO:
The main components of the project are spread across several Python files:

- `domain.pddl`: This file defines a domain for a robot navigation and coverage task using a planning domain definition language. The robot operates on a grid of cells, and the domain specifies the types, predicates, and actions that govern the robot's movement, direction changes, and the states of the cells. The domain's purpose is to model a robot's task of navigating and covering a grid. The robot can move to new cells, revisit old ones, and change directions as needed to explore the grid.
  
- `problem_complete_coverage.j2`: This file defines a specific problem instance for the robot navigation and coverage domain, written in PDDL. The problem specifies the grid layout, the robot's starting conditions, and the goal to be achieved. The file makes use of template-like structures to dynamically generate the problem details based on parameters such as grid size, obstacles, initial conditions and goals. This file describes a particular scenario for the robot to navigate, covering a grid while avoiding obstacles and respecting movement constraints.
  
- `problem_partial_coverage.j2`: Same as `problem_complete_coverage.j2` but instead of having multiple goals to achieve coverage, this problem defines as goal a specific cell.
  
- `coverage_planner.py`: The CoveragePlanner class acts as a leverage for the inputs and runs the main program to compute the coverage paths for a robot. It takes in a hard map, soft map, and robot configuration as inputs (yaml files) and provides a run method to execute the planning algorithm. The run method:
  1. selects a polygon to be covered and runs the solver
  2. Prints path information, including length, percentage covered, total length, and number of turns.
  3. Rescales the path and retrieves the absolute path.

- `solver.py`: This file implements the coverage path planning algorithm for navigating a robot in a grid-based environment. Utilizing the Unified Planning Framework (UPF), the module defines planning problems in PDDL (Planning Domain Definition Language), solves them optimally, and generates a ction plans for the robot. The solver keeps track of the robot's position, visited cells, and manages movement decisions to ensure all accessible areas are covered.

- `map.py`: This module includes classes and methods for map management and conversion. It reads map and robot configuration files, dilates the map based on the robot's dimensions to designate accessible areas, and transforms the high-resolution map into a low-resolution grid, marking cells as occupied if an obstacle's coverage exceeds a certain threshold. It can also select a polygonal region of the original map for localized operation.

- `router.py`: This module projects a computed path from a simplified grid back to the original high-resolution map. It includes functions that aid in handling partial obstacle coverage, avoiding collisions, and converting diagonal lines to orthogonal paths, ensuring smooth and safe navigation for the AGV.

- `utilities.py`: This file holds various utility functions used across the project.

- `plot_utils.py`: The PlotUtils class is for visualization purposes. It provides functions to plot the environment map, the planned path, and the actual path taken, which can be useful for debugging and understanding robot's movements. The four functions that can be used are:
  1. plot_map -> plots 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

- There is a demo notebook `demo.ipynb` in examples folder where demonstrates a usage of the Planner.

- Generally, one cas use the wrapper class `CoveragePlanner`:

    ```python
    from cpp_solver import CoveragePlanner
    ```

    and initialize the class as follows:
    ```python
    planner = CoveragePlanner(main_yaml_file_path, obstacles_yaml_file_path, robot_dimensions_yaml_file_path)
    ```

    Here, `main_yaml_file_path` and `unsafe_yaml_file_path` refer to the paths of the map definitions for the main map and the mapof unsafe areas, respectively. `robot_dimensions_yaml_file_path` is the path to a YAML file containing information about the robot's dimension characteristics; it should contain the following:

    - **robot_dim**: The robot's maximum dimension.
    - **mower_dim**: The robot's (effective) mowing radius.

    Obtain the path by:
    ```python
    # - abs path is the path in the absolute map coordinate system
    # - rescaled path is the path in the high-dimensional grid coordinate system
    # - path is the path in the in the low-dimensional grid coordinate system
    abs_path, rescaled_path, path = planner.run(polygon=polygon)
    ```
    where `polygon` is a polygon contained in the map; a list of x, y coordinates in meters. Firstly. you should find xmin, xmax, ymin, ymax of the map you want to use and the select x, y that are inside those boundaries.      

- Otherwise, you can utilize the `Solver Class` to obtain paths.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

//TODO:
- [x] Add Changelog
- [x] Add back to top links
- [ ] Add Additional Templates w/ Examples
- [ ] Multi-language Support
    - [ ] Chinese
    - [ ] Spanish

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
[UnifiedPlanning]: https://avatars.githubusercontent.com/u/79856489?s=48&v=4
[UP-url]: https://unified-planning.readthedocs.io/en/latest/index.html

<!-- CONTACT -->
## Contact

Sotirios Stavrakopoulos - [stavrako](https://github.com/stavrako) - sstavrakopoulos@iknowhow.com

Project Link: [https://github.com/ikh-innovation/aristos_cpp](https://github.com/ikh-innovation/aristos_cpp)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [TANGO](https://github.com/LIBRA-AI-Tech/tango)
* [Unified-Planning](https://github.com/aiplan4eu/unified-planning)

<p align="right">(<a href="#readme-top">back to top</a>)</p>