# Initial structure and division of tasks 


## Structure 
Optional things in parentheses. 

- (Undistortion): Image -> Image
- Feature detection (SIFT): Image -> Descriptors 
- Matching: Descriptors -> Correspondences  
- Outlier removal (pose estimation with 1point RANSAC): Correspondences -> Correspondences (w/o outliers)  
- Pose estimation (8-point algorithm): Correspondences -> Pose  
- (Non-Linear refinement): Pose, Correspondences -> Pose  
- Triangulation: Pose, Correspondences -> 3D Points  

## Division:
- Isar: Feature detection, matching  
- Ramón: Outlier removal  
- Justin: 8-point algorithm  
- Joshua: Triangulation

