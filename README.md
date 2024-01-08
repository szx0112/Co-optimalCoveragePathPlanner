# A Co-optimal Coverage Path Planning Method for Aerial Inspection of Complex Structures https://doi.org/10.1016/j.eswa.2020.113535
Original source code of the paper: A Co-optimal Coverage Path Planning Method for Aerial Inspection of Complex Structures published in Expert Systems with Applicaitons in 2020

![1-s2 0-S0957417420303596-gr10](https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/1638700c-252f-4b39-91a9-7060f5c9f546)

The utilization of unmanned aerial vehicles (UAVs) in survey and inspection of civil infrastructure has been growing rapidly. However, computationally efficient solvers that find optimal flight paths while ensuring high-quality data acquisition of the complete 3D structure remains a difficult problem. Existing solvers typically prioritize efficient flight paths, or coverage, or reducing computational complexity of the algorithm – but these objectives are not co-optimized holistically. In this work we introduce a co-optimal coverage path planning (CCPP) method that simultaneously co-optimizes the UAV path, the quality of the captured images, and reducing computational complexity of the solver all while adhering to safety and inspection requirements. The result is a highly parallelizable algorithm that produces more efficient paths where quality of the useful image data is improved. The path optimization algorithm utilizes a particle swarm optimization (PSO) framework which iteratively optimizes the coverage paths without needing to discretize the motion space or simplify the sensing models as is done in similar methods. The core of the method consists of a cost function that measures both the quality and efficiency of a coverage inspection path, and a greedy heuristic for the optimization enhancement by aggressively exploring the viewpoints search spaces. To assess the proposed method, a coverage path quality evaluation method is also presented in this research, which can be utilized as the benchmark for assessing other CPP methods for structural inspection purpose. The effectiveness of the proposed method is demonstrated by comparing the quality and efficiency of the proposed approach with the state-of-art through both synthetic and real-world scenes. The experiments show that our method enables significant performance improvement in coverage inspection quality while preserving the path efficiency on different test geometries.

![1-s2 0-S0957417420303596-gr2-2](https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/d7534f69-39fc-47d7-b763-0fd4cfdd4f57)

# Libraries
1. Numpy
2. LKH-TSP
3. Deap
4. VTK

Code was originally developed in Ubuntu 16.04 python 3.4, tested in Ubuntu 20.04 python 3.7.

# Models
Four models can be found /meshes folder, including three models used in SIP and the paper

# Parameters
Parameters are stored in param.py

# How to run
1. select the models in /meshes folder
2. edit or use the default parameters stored in param.py
3. exeute the main function in main.py

# TO DO
The following fucntions are not included in this version.
1. OBBTree
2. RRT
3. quality evaluation



