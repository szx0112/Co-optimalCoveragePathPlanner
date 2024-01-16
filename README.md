# Co-optimal Coverage Path Planner (CCPP)

Original source code of the paper: 
Shang, Z., Bradley, J., & Shen, Z. (2020). A co-optimal coverage path planning method for aerial scanning of complex structures. Expert Systems with Applications, 158, 113535. https://doi.org/10.1016/j.eswa.2020.113535
![1-s2 0-S0957417420303596-gr10](https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/1638700c-252f-4b39-91a9-7060f5c9f546)

Abstract: The utilization of unmanned aerial vehicles (UAVs) in survey and inspection of civil infrastructure has been growing rapidly. However, computationally efficient solvers that find optimal flight paths while ensuring high-quality data acquisition of the complete 3D structure remains a difficult problem. Existing solvers typically prioritize efficient flight paths, or coverage, or reducing computational complexity of the algorithm – but these objectives are not co-optimized holistically. In this work we introduce a co-optimal coverage path planning (CCPP) method that simultaneously co-optimizes the UAV path, the quality of the captured images, and reducing computational complexity of the solver all while adhering to safety and inspection requirements. The result is a highly parallelizable algorithm that produces more efficient paths where quality of the useful image data is improved. The path optimization algorithm utilizes a particle swarm optimization (PSO) framework which iteratively optimizes the coverage paths without needing to discretize the motion space or simplify the sensing models as is done in similar methods. The core of the method consists of a cost function that measures both the quality and efficiency of a coverage inspection path, and a greedy heuristic for the optimization enhancement by aggressively exploring the viewpoints search spaces. To assess the proposed method, a coverage path quality evaluation method is also presented in this research, which can be utilized as the benchmark for assessing other CPP methods for structural inspection purpose. The effectiveness of the proposed method is demonstrated by comparing the quality and efficiency of the proposed approach with the state-of-art through both synthetic and real-world scenes. The experiments show that our method enables significant performance improvement in coverage inspection quality while preserving the path efficiency on different test geometries.

![1-s2 0-S0957417420303596-gr2-2](https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/d7534f69-39fc-47d7-b763-0fd4cfdd4f57)

# Libraries
1. Numpy
2. LKH-TSP
3. Deap
4. VTK

Code was originally developed under Ubuntu 16.04 python 3.4, tested in Ubuntu 20.04 python 3.8.

# Models
Four models can be found /meshes folder, including three models used in SIP and the paper

# Parameters
Parameters are stored in param.py

# How to run
1. View and select the models in /meshes folder
2. Use the default parameters or select your own in param.py
3. Run main.py

# Examples
hoaHakanaia (pop=30, iter=30)
1. When ε=1.0
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/1ff8fc05-c1e7-47d9-b9c0-f3c582336846" width="400" height="300">
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/0788bc2a-e8c7-4fae-bbdf-08d8d9d97c67" width="400" height="300">

2. When ε=0.5
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/f76c8a1c-141a-4f96-9104-83b5c6c42b72" width="400" height="300">
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/88af7f1d-455d-44e1-9ff3-5d83517789be" width="400" height="300">

3. When ε=0.0
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/3286fe4e-8d31-4467-973d-b7fb1774680e" width="400" height="300">
<img src="https://github.com/szx0112/co-optimal-path-planning.github.io/assets/10392640/846a2137-b369-440a-b22a-c8e082df8c64" width="400" height="300">


# TO DO
This repository is still under development. The following fucntions are not included in the current update.
1. OBBTree
2. RRT*
3. Quality evaluation
4. etc.

# Credits
This algorithm was developed by Zhexiong Shang under the supervison and support from Prof. Zhigang Shen and Prof. Justin Bradley from the University of Nebraska-Lincoln

