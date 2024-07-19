### main.cpp
- Extract 68 Facial Landmark 2D points from the input image.
- Load the PointCloud (pcd file) of our input.
- Get the 68 Facial Landmarks 3D points from the PointCloud.
- Apply Procusters algorithm on the 68 points of the BFM and the input cloud.
- Apply ICP optimization on the Transformed BFM and the input Point cloud.
- Save the optimized BFM as .dat

### FaceModel.h
- Define a Base Face Model(BFM) class using Face parameters(Shape, Expression, ...) loaded from CSV files holding the data of a Basel Face model.

### ICPOptimizer.h
- Define the CeresOptimier class that optimizes the Energy under our constraints.
  - PointToPlaneConstraint class: Computes a weighted distance between a point in the Source model to the target plane defined by the target point and the normal of the target face model.
  - PointToPointConstraint class: Sum of all the differences between source and target points.
  - Regularization: Sum of all shape and expression coefficients.
