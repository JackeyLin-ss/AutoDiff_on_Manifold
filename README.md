# Auto Diff on Manifold using Ceres Solver 
---
This repository contains the implementation of optimazation on Lie group(manifold) using ceres solver auto-diff feature.  
**Ceres automatic differentiation** is a technique that can compute exact derivatives, fast, while requiring about the same effort from the user as is needed to use numerical differentiation. 
More details and introduction can be found [here](http://ceres-solver.org/automatic_derivatives.html)

Here we use two methods (Analytic Derivatives / Automatic Derivatives ) to solve camera pose estimation problem


## 0. Experiment Data 
  I use real data from **Euroc** to compare different methods.
  ![euroc dataset map points ](images/Euroc_dataset.png)


## 1.Analytic Derivatives
 
## 2.Automatic Derivatives


## 3.Prerequisites
  ### Ceres Solver 
  Ceres solver (Version 1.14.x) is required.  
  
  ### Pangolin
  We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

  ### Eigen3 
  Eigen3.3.9 has been tested. 


## 4.Build and Run 
I provide a script `run.sh` in the project directory for you to build and run the code. 


## 5. Result 


  

