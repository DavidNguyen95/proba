# Probabilistic robot project
Project 2: Bearing only SLAM  

#Dataset:
Put data in datasets folder (type bearing)

# Explaination file in octave/lsSLAM folder:
lsSLAM_full.m : Set init parameters for running program  
linearize_pose_pose_constraint.m : compute Jacobian between 2 robot positions.  
linearize_pose_landmakr_bearing_constraint.m : Compute the error of a pose-landmark with bearing only constraint  
linearize_and_solve.m : performs one iteration of the Gauss-Newton algorithm  
compute_global_error.m : Computes the total error of the graph  


# Running project:
`cd octave/lsSLAM`  
`octave-cli lsSLAM_full.m`  

# Results
During loop, I save the all plots in plots folder. Here is some sample of results  
Init state  
![alt text](https://github.com/DavidNguyen95/proba/blob/main/plots/lsslam_000.png)
9th interative:  
![alt text](https://github.com/DavidNguyen95/proba/blob/main/plots/lsslam_009.png)
13th interative:  
![alt text](https://github.com/DavidNguyen95/proba/blob/main/plots/lsslam_013.png)
555th interative:  
![alt text](https://github.com/DavidNguyen95/proba/blob/main/plots/lsslam_555.png)
999th interative:  
![alt text](https://github.com/DavidNguyen95/proba/blob/main/plots/lsslam_998.png)




