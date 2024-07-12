// #pragma once

// // The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
// #define GLOG_NO_ABBREVIATED_SEVERITIES

// #include <ceres/ceres.h>
// #include <ceres/rotation.h>
// #include <flann/flann.hpp>

// #include "SimpleMesh.h"
// #include "NearestNeighbor.h"
// #include "PointCloud.h"
// #include "ProcrustesAligner.h"
// #include "FaceModel.h"

// #define TRANSFER   0
// #define SHAPE_REGULARIZATION_WEIGHT 0.000001
// #define EXPRESSION_REGULARIZATION_WEIGHT 0.0000005

// /**
//  * Helper methods for writing Ceres cost functions.
//  */
// template <typename T>
// static inline void fillVector(const Vector3f& input, T* output) {
//     output[0] = T(input[0]);
//     output[1] = T(input[1]);
//     output[2] = T(input[2]);
// }


// /**
//  * Pose increment is only an interface to the underlying array (in constructor, no copy
//  * of the input array is made).
//  * Important: Input array needs to have a size of at least 6.
//  */
// template <typename T>
// class PoseIncrement {
// public:
//     explicit PoseIncrement(T* const array) : m_array{ array } { }

//     void setZero() {
//         for (int i = 0; i < 6; ++i)
//             m_array[i] = T(0);
//     }

//     T* getData() const {
//         return m_array;
//     }

//     /**
//      * Applies the pose increment onto the input point and produces transformed output point.
//      * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
//      * beforehand).
//      */
//     void apply(T* inputPoint, T* outputPoint) const {
//         // pose[0,1,2] is angle-axis rotation.
//         // pose[3,4,5] is translation.
//         const T* rotation = m_array;
//         const T* translation = m_array + 3;

//         T temp[3];
//         ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

//         outputPoint[0] = temp[0] + translation[0];
//         outputPoint[1] = temp[1] + translation[1];
//         outputPoint[2] = temp[2] + translation[2];
//     }

//     /**
//      * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
//      * transformation 4x4 matrix.
//      */
//     static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
//         // pose[0,1,2] is angle-axis rotation.
//         // pose[3,4,5] is translation.
//         double* pose = poseIncrement.getData();
//         double* rotation = pose;
//         double* translation = pose + 3;

//         // Convert the rotation from SO3 to matrix notation (with column-major storage).
//         double rotationMatrix[9];
//         ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

//         // Create the 4x4 transformation matrix.
//         Matrix4f matrix;
//         matrix.setIdentity();
//         matrix(0, 0) = float(rotationMatrix[0]);	matrix(0, 1) = float(rotationMatrix[3]);	matrix(0, 2) = float(rotationMatrix[6]);	matrix(0, 3) = float(translation[0]);
//         matrix(1, 0) = float(rotationMatrix[1]);	matrix(1, 1) = float(rotationMatrix[4]);	matrix(1, 2) = float(rotationMatrix[7]);	matrix(1, 3) = float(translation[1]);
//         matrix(2, 0) = float(rotationMatrix[2]);	matrix(2, 1) = float(rotationMatrix[5]);	matrix(2, 2) = float(rotationMatrix[8]);	matrix(2, 3) = float(translation[2]);

//         return matrix;
//     }

// private:
//     T* m_array;
// };


// template <typename T>
// class ExpShapeCoeffIncrement {
// public:
//     explicit ExpShapeCoeffIncrement(T* const arrayExpCoef, T* const arrayShapeCoef) :
//     m_arrayExpCoef{ arrayExpCoef },
//     m_arrayShapeCoef{ arrayShapeCoef }
//     { }

//     void setZero() {
//         for (int i = 0; i < 64; ++i)
//             m_arrayExpCoef[i] = T(0);

//         for (int i = 0; i < 80; ++i)
//             m_arrayShapeCoef[i] = T(0);
//     }

//     T* getExpCoeff() const {
//         return m_arrayExpCoef;
//     }

//     T* getShapeCoeff() const {
//         return m_arrayShapeCoef;
//     }

//     void apply(const int inputIndex, T* outputPoint) const {
//         const T* expCoef = m_arrayExpCoef;
//         const T* shapeCoef = m_arrayShapeCoef;
//         T* faces = new T[3];

//         dotFace_ceres(inputIndex, expCoef, shapeCoef, faces);

//         outputPoint[0] = faces[0];
//         outputPoint[1] = faces[1];
//         outputPoint[2] = faces[2];

//         delete [] faces;
//     }
//     /*int writeExpCoeff(){
//         std::ofstream out("../sample_face/expression", ios::out | ios::binary);
//         if(!out) {
//             cout << "Cannot open file.";
//             return 1;
//         }
//         out.write((char *) m_arrayExpCoef, sizeof m_arrayExpCoef);
//         out.close();
//     } */

    
//     void writeExp(){
//         std::ofstream myfile;
//         myfile.open ("../sample_face/expression.csv");
//         for(int i=0 ; i<64 ; i++){
//             myfile << std::to_string(m_arrayExpCoef[i])+"\n";
//         }
//         myfile.close();
//     }
    

//     void readExp() {

//         std::string line;
//         std::ifstream file("../sample_face/expression.csv");
//         std::string value;

//         int ind=0;
//         while (getline(file, line)) {

//             std::stringstream lineStringStream(line);

//             while (std::getline(lineStringStream, value, ','))
//             {
//                 std::cout<<value<<std::endl;
//                 m_arrayExpCoef[ind]=stod(value);
//                 ind++;
//             }
//         }
//     }

//     /*void readExpCoeff(){
//         std::ifstream in("../sample_face/expression", ios::in | ios::binary);
//         //std::cout<<"lel"<<std::endl;
//         //std::cout<<sizeof m_arrayExpCoef<<std::endl;
//         double* new_exp = new double [64];
        
//         in.read((char *) new_exp, 64);//sizeof m_arrayExpCoef);
//         std::cout<<"-----------------------------------"<<std::endl;
//         in.close();
//         int diff=0;
//         for(int i=0 ; i<64 ; i++){
//             if(m_arrayExpCoef[i]!=new_exp[i]) diff++;
//         }
//         std::cout<<diff<<std::endl;
//         m_arrayExpCoef = new_exp ;
        
//     }*/


// private:
//     T* m_arrayExpCoef;
//     T* m_arrayShapeCoef;
// };

// /**
//  * Optimization constraints.
//  */
// class PointToPointConstraint {
// public:
//     PointToPointConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) :
//         m_sourcePoint{ sourcePoint },
//         m_targetPoint{ targetPoint },
//         m_weight{ weight }
//     { }

//     template <typename T>
//     bool operator()(const T* const pose, T* residuals) const {
//         // TODO: Implemented the point-to-point cost function.
//         // The resulting 3D residual should be stored in residuals array. To apply the pose 
//         // increment (pose parameters) to the source point, you can use the PoseIncrement
//         // class.
//         // Important: Ceres automatically squares the cost function.

//         PoseIncrement<T> poseIncrement(const_cast<T*>(pose));

//         T sourcePoint[3] = { static_cast<T>(m_sourcePoint.x()), 
//                             static_cast<T>(m_sourcePoint.y()), 
//                             static_cast<T>(m_sourcePoint.z()) };

//         T transformedSource[3];

//         poseIncrement.apply(sourcePoint, transformedSource);

//         for (int i = 0; i < 3; ++i) {
//             T targetCoord;
//             if (i == 0) targetCoord = static_cast<T>(m_targetPoint.x());
//             else if (i == 1) targetCoord = static_cast<T>(m_targetPoint.y());
//             else targetCoord = static_cast<T>(m_targetPoint.z());

//             residuals[i] = static_cast<T>(m_weight) * (transformedSource[i] - targetCoord);
//         }
//         return true;
//     }

//     static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) {
//         return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
//             new PointToPointConstraint(sourcePoint, targetPoint, weight)
//             );
//     }

// protected:
//     const Vector3f m_sourcePoint;
//     const Vector3f m_targetPoint;
//     const float m_weight;
//     const float LAMBDA = 0.1f;
// };

// class PointToPlaneConstraint {
// public:
//     PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
//         m_sourcePoint{ sourcePoint },
//         m_targetPoint{ targetPoint },
//         m_targetNormal{ targetNormal },
//         m_weight{ weight }
//     { }

//     template <typename T>
//     bool operator()(const T* const pose, T* residuals) const {
//         // TODO: Implemented the point-to-plane cost function.
//         // The resulting 1D residual should be stored in residuals array. To apply the pose 
//         // increment (pose parameters) to the source point, you can use the PoseIncrement
//         // class.
//         // Important: Ceres automatically squares the cost function.

//         PoseIncrement<T> poseIncrement(const_cast<T*>(pose));

//         // Define and initialize source point
//         T sourcePoint[3] = { static_cast<T>(m_sourcePoint.x()), 
//                             static_cast<T>(m_sourcePoint.y()), 
//                             static_cast<T>(m_sourcePoint.z()) };

//         T transformedSource[3];

//         // Apply pose increment to source point
//         poseIncrement.apply(sourcePoint, transformedSource);

//         // Calculate delta
//         T delta[3];
//         for (int i = 0; i < 3; ++i) {
//             switch (i) {
//                 case 0:
//                     delta[i] = transformedSource[i] - static_cast<T>(m_targetPoint.x());
//                     break;
//                 case 1:
//                     delta[i] = transformedSource[i] - static_cast<T>(m_targetPoint.y());
//                     break;
//                 case 2:
//                     delta[i] = transformedSource[i] - static_cast<T>(m_targetPoint.z());
//                     break;
//             }
//         }

//         // Calculate residuals
//         residuals[0] = static_cast<T>(m_weight) * (delta[0] * static_cast<T>(m_targetNormal.x()) +
//                                                 delta[1] * static_cast<T>(m_targetNormal.y()) +
//                                                 delta[2] * static_cast<T>(m_targetNormal.z()));

//         return true;
//     }

//     static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
//         return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
//             new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight)
//             );
//     }

// protected:
//     const Vector3f m_sourcePoint;
//     const Vector3f m_targetPoint;
//     const Vector3f m_targetNormal;
//     const float m_weight;
//     const float LAMBDA = 1.0f;
// };


// /**
//  * ICP optimizer - Abstract Base Class
//  */
// class ICPOptimizer {
// public:
//     ICPOptimizer() :
//         m_bUsePointToPlaneConstraints{ false },
//         m_nIterations{ 20 },
//         m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchFlann>() }
//     { }

//     void setMatchingMaxDistance(float maxDistance) {
//         m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
//     }

//     void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints) {
//         m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
//     }

//     void setNbOfIterations(unsigned nIterations) {
//         m_nIterations = nIterations;
//     }

//     virtual void estimateExpShapeCoeffs(const PointCloud& target) = 0;
//     virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) = 0;

// protected:
//     bool m_bUsePointToPlaneConstraints;
//     unsigned m_nIterations;
//     std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

//     std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4f& pose) {
//         std::vector<Vector3f> transformedPoints;
//         transformedPoints.reserve(sourcePoints.size());

//         const auto rotation = pose.block(0, 0, 3, 3);
//         const auto translation = pose.block(0, 3, 3, 1);

//         for (const auto& point : sourcePoints) {
//             transformedPoints.push_back(rotation * point + translation);
//         }

//         return transformedPoints;
//     }

//     std::vector<Vector3f> transformNormals(const std::vector<Vector3f>& sourceNormals, const Matrix4f& pose) {
//         std::vector<Vector3f> transformedNormals;
//         transformedNormals.reserve(sourceNormals.size());

//         const auto rotation = pose.block(0, 0, 3, 3);

//         for (const auto& normal : sourceNormals) {
//             transformedNormals.push_back(rotation.inverse().transpose() * normal);
//         }

//         return transformedNormals;
//     }

//     void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
//         const unsigned nPoints = sourceNormals.size();

//         for (unsigned i = 0; i < nPoints; i++) {
//             Match& match = matches[i];
//             if (match.idx >= 0) {
//                 const auto& sourceNormal = sourceNormals[i];
//                 const auto& targetNormal = targetNormals[match.idx];

//                 // TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
//                 // if(sourceNormal.dot(targetNormal) < std::cos(Eigen::degreesToRadians(60.0f))){
//                 //     match.idx = -1;
//                 // }
//                 double cosineAngle = sourceNormal.dot(targetNormal) / (sourceNormal.norm() * targetNormal.norm());
//                 double angle = std::acos(cosineAngle);

//                 const double angleThreshold = 1.04667;
//                 if (angle > angleThreshold) {
//                     match.idx = -1;
//                 }
//             }
//         }
//     }
// };


// /**
//  * ICP optimizer - using Ceres for optimization.
//  */
// class CeresICPOptimizer : public ICPOptimizer {
// public:
//     CeresICPOptimizer() {}

//     virtual void estimateExpShapeCoeffs(const PointCloud &target) override {
//         // Build the index of the FLANN tree (for fast nearest neighbor lookup).
//         m_nearestNeighborSearch->buildIndex(target.getPoints());

//         double incrementArrayExp[64];
//         double incrementArrayShape[80];

//         auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<double>(incrementArrayExp, incrementArrayShape);
//         expShapeCoeffIncrement.setZero();

//         FaceModel* faceModel = FaceModel::getInstance();
//         for (int i = 0; i < m_nIterations; ++i) {
//             // Compute the matches.
//             std::cout << "Matching points ..." << std::endl;
//             clock_t begin = clock();

            
//             SimpleMesh faceMesh;
//             if (!faceMesh.loadMesh("transformed_model_procusters.off")) {
//                 std::cout << "Mesh file wasn't read successfully at location: " << "transformed_model.off" << std::endl;
//             }

//             PointCloud faceModelPoints{faceMesh};

            
//             auto matches = m_nearestNeighborSearch->queryMatches(faceModelPoints.getPoints());
//             int matchCtr = 0;
//             for(Match match : matches) {
//                 if (match.idx >= 0)
//                    matchCtr++;
//             }
//             std::cout << "match count before pruning:" << matchCtr << std::endl;
//             pruneCorrespondences(faceModelPoints.getNormals(), target.getNormals(), matches);

//             clock_t end = clock();
//             double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
//             std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

//             matchCtr = 0;
//             for(Match match : matches) {
//                 if (match.idx >= 0)
//                    matchCtr++;
//             }
//             std::cout << "match count:" << matchCtr << std::endl;
//             // Prepare point-to-point and point-to-plane constraints.
//             ceres::Problem problem;
//             customPrepareConstraints(target.getPoints(), target.getNormals(), matches, expShapeCoeffIncrement, problem);
//             //customPrepareConstraints(target.getPoints(), matches, expShapeCoeffIncrement, problem);

//             // Configure options for the solver.
//             ceres::Solver::Options options;
//             configureSolver(options);
            
//             // Run the solver (for one iteration).
//             ceres::Solver::Summary summary;
//             ceres::Solve(options, &problem, &summary);
//             std::cout << summary.BriefReport() << std::endl;
//             //std::cout << summary.FullReport() << std::endl;

//             //update face model with these params
//             if(i == m_nIterations-1){
//                 std::cout<<"-----------LAST ITERATION--------------"<<std::endl;
//                 if(TRANSFER){
//                     expShapeCoeffIncrement.readExp();
//                     std::cout<<"-----------TRANSFER--------------"<<std::endl;
//                 }
//                 else{
//                     std::cout<<"------EXPRESSION WRITE-----------"<<std::endl;
//                     expShapeCoeffIncrement.writeExp();
//                 }
//             }
//             faceModel->expCoefAr = expShapeCoeffIncrement.getExpCoeff();        
//             faceModel->shapeCoefAr = expShapeCoeffIncrement.getShapeCoeff();

//             MatrixXd transformed_mesh;
//             transformed_mesh = faceModel->transform(faceModel->pose, faceModel->scale);
            
//             faceModel->write_off("../sample_face/transformed_model.off",transformed_mesh);

//             std::cout << "Optimization iteration done." << std::endl;
//         }
//         MatrixXd transformed_mesh;
//         transformed_mesh = faceModel->transform(faceModel->pose, faceModel->scale);

//         faceModel->write_off("../sample_face/result.off",transformed_mesh);
        
//         faceModel->write_obj("../sample_face/result.obj",transformed_mesh);
//     }

//     virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) override {
//         // Build the index of the FLANN tree (for fast nearest neighbor lookup).
//         m_nearestNeighborSearch->buildIndex(target.getPoints());

//         // The initial estimate can be given as an argument.
//         Matrix4f estimatedPose = initialPose;

//         // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
//         // the rotation angle) and 3 parameters for the translation vector. 
//         double incrementArray[6];
//         auto poseIncrement = PoseIncrement<double>(incrementArray);
//         poseIncrement.setZero();

//         for (int i = 0; i < m_nIterations; ++i) {
//             // Compute the matches.
//             std::cout << "Matching points ..." << std::endl;
//             clock_t begin = clock();

//             auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
//             auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

//             auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
//             pruneCorrespondences(transformedNormals, target.getNormals(), matches);

//             clock_t end = clock();
//             double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
//             std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

//             // Prepare point-to-point and point-to-plane constraints.
//             ceres::Problem problem;
//             prepareConstraints(transformedPoints, target.getPoints(), target.getNormals(), matches, poseIncrement, problem);

//             // Configure options for the solver.
//             ceres::Solver::Options options;
//             configureSolver(options);

//             // Run the solver (for one iteration).
//             ceres::Solver::Summary summary;
//             ceres::Solve(options, &problem, &summary);
//             std::cout << summary.BriefReport() << std::endl;
//             //std::cout << summary.FullReport() << std::endl;

//             // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
//             Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
//             estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
//             poseIncrement.setZero();

//             std::cout << "Optimization iteration done." << std::endl;
//         }

//         // Store result
//         initialPose = estimatedPose;
//     }


// private:
//     void configureSolver(ceres::Solver::Options& options) {
//         // Ceres options.
//         options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//         options.use_nonmonotonic_steps = false;
//         options.linear_solver_type = ceres::DENSE_QR;
//         options.minimizer_progress_to_stdout = 1;
//         options.max_num_iterations = 1;
//         options.num_threads = 8;
//     }

//     void customPrepareConstraints(const std::vector<Vector3f> &targetPoints,
//                                   const std::vector<Vector3f> &targetNormals,
//                                   const std::vector<Match> matches,
//                                   const ExpShapeCoeffIncrement<double> &expShapeCoeffIncrement,
//                                   ceres::Problem &problem) const {
                                    
//         const unsigned nPoints = targetPoints.size();
//         for (unsigned i = 0; i < 35709; ++i) {
//             const auto match = matches[i];
//             if (match.idx >= 0) {
//                 const auto &targetPoint = targetPoints[match.idx];

//                 const int sourcePointIndex = i;
//                 problem.AddResidualBlock(
//                         MyCustomConstraint::create(sourcePointIndex, targetPoint, match.weight),
//                         nullptr,
//                         expShapeCoeffIncrement.getExpCoeff(),
//                         expShapeCoeffIncrement.getShapeCoeff()
//                 );

//                 const auto& targetNormal = targetNormals[match.idx];
//                 if (!targetNormal.allFinite())
//                     continue;

//                 problem.AddResidualBlock(
//                     PointToPlaneConstr::create(sourcePointIndex, targetPoint, targetNormal, match.weight),
//                     nullptr, 
//                     expShapeCoeffIncrement.getExpCoeff(),
//                     expShapeCoeffIncrement.getShapeCoeff()
//                 );
//             }
//         }

//         ceres::CostFunction* shape_cost = new ceres::AutoDiffCostFunction<ShapeCostFunction, 80, 80>(
// 				new ShapeCostFunction(SHAPE_REGULARIZATION_WEIGHT)
// 				);
// 		problem.AddResidualBlock(shape_cost, nullptr, expShapeCoeffIncrement.getShapeCoeff());

    
//         ceres::CostFunction* expression_cost = new ceres::AutoDiffCostFunction<ExpressionCostFunction, 64, 64>(
// 				new ExpressionCostFunction(EXPRESSION_REGULARIZATION_WEIGHT)
// 				);
// 		problem.AddResidualBlock(expression_cost, nullptr, expShapeCoeffIncrement.getExpCoeff());

//     }

//     void customPrepareConstraints(const std::vector<Vector3f> &targetPoints,
//                                   const std::vector<Match> matches,
//                                   const ExpShapeCoeffIncrement<double> &expShapeCoeffIncrement,
//                                   ceres::Problem &problem) const {
//         for (unsigned i = 0; i < 35709; ++i) {
//             const auto match = matches[i];
//             if (match.idx >= 0) {
//                 const auto &targetPoint = targetPoints[match.idx];

//                 const int sourcePointIndex = i;
//                 problem.AddResidualBlock(
//                         MyCustomConstraint::create(sourcePointIndex, targetPoint, match.weight),
//                         nullptr,
//                         expShapeCoeffIncrement.getExpCoeff(),
//                         expShapeCoeffIncrement.getShapeCoeff()
//                 );

//             }
//         }
//     }

//     void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const std::vector<Match> matches, const PoseIncrement<double>& poseIncrement, ceres::Problem& problem) const {
//         const unsigned nPoints = sourcePoints.size();

//         for (unsigned i = 0; i < nPoints; ++i) {
//             const auto match = matches[i];
//             if (match.idx >= 0) {
//                 const auto& sourcePoint = sourcePoints[i];
//                 const auto& targetPoint = targetPoints[match.idx];

//                 if (!sourcePoint.allFinite() || !targetPoint.allFinite())
//                     continue;

//                 // TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block) 
//                 // to the Ceres problem.
//                 ceres::CostFunction* pointToPointCostFunction = PointToPointConstraint::create(sourcePoint, targetPoint, 1.0f);
//                 problem.AddResidualBlock(pointToPointCostFunction, nullptr, poseIncrement.getData());

//                 if (m_bUsePointToPlaneConstraints) {
//                     const auto& targetNormal = targetNormals[match.idx];

//                     if (!targetNormal.allFinite())
//                         continue;

//                     // TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block) 
//                     // to the Ceres problem.
//                     ceres::CostFunction* pointToPlaneCostFunction = PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, 1.0f);
//                     problem.AddResidualBlock(pointToPlaneCostFunction, nullptr, poseIncrement.getData());

//                 }
//             }
//         }
//     }
// };


// /**
//  * ICP optimizer - using linear least-squares for optimization.
//  */
// class LinearICPOptimizer : public ICPOptimizer {
// public:
//     LinearICPOptimizer() {}

//         virtual void estimateExpShapeCoeffs(const PointCloud &target) override {}


//     virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) override {
//         // Build the index of the FLANN tree (for fast nearest neighbor lookup).
//         m_nearestNeighborSearch->buildIndex(target.getPoints());

//         // The initial estimate can be given as an argument.
//         Matrix4f estimatedPose = initialPose;

//         for (int i = 0; i < m_nIterations; ++i) {
//             // Compute the matches.
//             std::cout << "Matching points ..." << std::endl;
//             clock_t begin = clock();

//             auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
//             auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

//             auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
//             pruneCorrespondences(transformedNormals, target.getNormals(), matches);

//             clock_t end = clock();
//             double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
//             std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

//             std::vector<Vector3f> sourcePoints;
//             std::vector<Vector3f> targetPoints;

//             // Add all matches to the sourcePoints and targetPoints vector,
//             // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
//             // the matches vector holds the index of the matching target point.
//             for (int j = 0; j < transformedPoints.size(); j++) {
//                 const auto& match = matches[j];
//                 if (match.idx >= 0) {
//                     sourcePoints.push_back(transformedPoints[j]);
//                     targetPoints.push_back(target.getPoints()[match.idx]);
//                 }
//             }

//             // Estimate the new pose
//             if (m_bUsePointToPlaneConstraints) {
//                 estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals()) * estimatedPose;
//             }
//             else {
//                 estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
//             }

//             std::cout << "Optimization iteration done." << std::endl;
//         }

//         // Store result
//         initialPose = estimatedPose;
//     }

// private:
//     Matrix4f estimatePosePointToPoint(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
//         ProcrustesAligner procrustAligner;
//         Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

//         return estimatedPose;
//     }

//     Matrix4f estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals) {
//         const unsigned nPoints = sourcePoints.size();

//         // Build the system
//         MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
//         VectorXf b = VectorXf::Zero(4 * nPoints);

//         for (unsigned i = 0; i < nPoints; i++) {
//             const auto& s = sourcePoints[i];
//             const auto& d = targetPoints[i];
//             const auto& n = targetNormals[i];

//             // TODO: Add the point-to-plane constraints to the system
//             Eigen::RowVector3f crossProductTransposed = n.cross(s).cast<float>().transpose();
//             Eigen::RowVector3f normalTransposed = n.cast<float>().transpose();
//             float dotProduct = n.cast<float>().dot(d - s);
//             A.block<1, 3>(4 * i, 0) = crossProductTransposed;
//             A.block<1, 3>(4 * i, 3) = normalTransposed;
//             b(4 * i) = dotProduct;
//             // TODO: Add the point-to-point constraints to the system
//             A.block<3, 3>(4 * i + 1, 0).setZero();
//             A.block<3, 3>(4 * i + 1, 3).setIdentity();
//             b.segment<3>(4 * i + 1) = d - s;

//             //TODO: Optionally, apply a higher weight to point-to-plane correspondences
//             const float scalingFactor = std::sqrt(10.0f);
//             b(4 * i) *= scalingFactor;

//         }

//         // TODO: Solve the system
//         Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

//         float alpha = x(0), beta = x(1), gamma = x(2);

//         // Build the pose matrix
//         Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
//             AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
//             AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

//         Vector3f translation = x.tail(3);

//         // TODO: Build the pose matrix using the rotation and translation matrices
//         Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
//         estimatedPose.topLeftCorner<3, 3>() = rotation;
//         estimatedPose.topRightCorner<3, 1>() = translation;


//         return estimatedPose;
//     }
// };


#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
using namespace std;

#include "SimpleMesh.h"
#include "NearestNeighbor.h"
#include "PointCloud.h"
#include "ProcrustesAligner.h"
#include "FaceModel.h"

#define SHAPE_REGULARIZATION_WEIGHT 0.000001
#define EXPRESSION_REGULARIZATION_WEIGHT 0.0000005

#define TRANSFER   0
struct ShapeCostFunction
{
	ShapeCostFunction(double weight_)
		: weight{ weight_ }
	{}

	template<typename T>
	bool operator()(T const* shape_weights, T* residuals) const
	{

		for (int i = 0; i < 80; i++) {
			residuals[i] = shape_weights[i] * T(sqrt(weight));
		}
		return true;
	}

private:
	const double weight;
};
struct ExpressionCostFunction
{
	ExpressionCostFunction(double weight_)
		: weight{weight_}
	{}

	template<typename T>
	bool operator()(T const* exp_weights, T* residuals) const
	{
		for (int j = 0; j < 64; j++) {
			residuals[j] = exp_weights[j] * T(sqrt(weight));
		}
		return true;
	}

private:
	const double weight;
};

/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement {
public:
    explicit PoseIncrement(T* const array) : m_array{ array } { }

    void setZero() {
        for (int i = 0; i < 6; ++i)
            m_array[i] = T(0);
    }

    T* getData() const {
        return m_array;
    }

    /**
     * Applies the pose increment onto the input point and produces transformed output point.
     * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
     * beforehand).
     */
    void apply(T* inputPoint, T* outputPoint) const {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        const T* rotation = m_array;
        const T* translation = m_array + 3;

        T temp[3];
        ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

        outputPoint[0] = temp[0] * m_scale+ translation[0];
        outputPoint[1] = temp[1] * m_scale+ translation[1];
        outputPoint[2] = temp[2] * m_scale+ translation[2];
    }

    /**
     * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
     * transformation 4x4 matrix.
     */
    static Matrix4d convertToMatrix(const PoseIncrement<double>& poseIncrement) {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        double* pose = poseIncrement.getData();
        double* rotation = pose;
        double* translation = pose + 3;

        // Convert the rotation from SO3 to matrix notation (with column-major storage).
        double rotationMatrix[9];
        ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

        // Create the 4x4 transformation matrix.
        Matrix4d matrix;
        matrix.setIdentity();
        matrix(0, 0) = double(rotationMatrix[0]);	matrix(0, 1) = double(rotationMatrix[3]);	matrix(0, 2) = double(rotationMatrix[6]);	matrix(0, 3) = double(translation[0]);
        matrix(1, 0) = double(rotationMatrix[1]);	matrix(1, 1) = double(rotationMatrix[4]);	matrix(1, 2) = double(rotationMatrix[7]);	matrix(1, 3) = double(translation[1]);
        matrix(2, 0) = double(rotationMatrix[2]);	matrix(2, 1) = double(rotationMatrix[5]);	matrix(2, 2) = double(rotationMatrix[8]);	matrix(2, 3) = double(translation[2]);

        return matrix;
    }

    void init(Matrix4d pose, double scale) {

        double rotationMatrix[9];
        double* rotation;

        rotationMatrix[0] = pose(0,0); rotationMatrix[1] = pose(0,1); rotationMatrix[2] = pose(0,2);
        rotationMatrix[3] = pose(1,0); rotationMatrix[4] = pose(1,1); rotationMatrix[5] = pose(1,2);
        rotationMatrix[6] = pose(2,0); rotationMatrix[7] = pose(2,1); rotationMatrix[8] = pose(2,2);
        ceres::RotationMatrixToAngleAxis(rotationMatrix, rotation);
        cout << "iç\t" << rotation[0] <<  rotation[1]<< rotation[2] << endl;
        m_array[0] = T(rotation[0]); m_array[1] = T(rotation[1]); m_array[2] = T(rotation[2]);
        cout << "iç2" << endl;
        m_scale = T(scale);
        cout << "iç3" << endl;

    }

private:
    T* m_array;
    T m_scale;
};

template <typename T>
static inline void dotFace_ceres(int pointIndex, const T* expCoef, const T* shapeCoef, T* face) {
    FaceModel* faceModel = FaceModel::getInstance();

    // T* expression =  new T[3];
    // T* shape = new T[3];

    // T* pointVertex = new T[3];

    T expression[3];
    T shape[3];
    T pointVertex[3];

    for (int i = 0; i < 3; i++) {
        T sum = T(0.0);
        for (int j = 0; j < 64; j++) {
            sum += faceModel->expBaseAr[pointIndex*3 + i][j] * expCoef[j];
        }
        expression[i] = T(sum);
    }

    for (int i = 0; i < 3; i++) {
        T sum = T(0.0);
        for (int j = 0; j < 80; j++) {
            sum += faceModel->idBaseAr[pointIndex*3 + i][j] * shapeCoef[j];
        }
        shape[i] = T(sum);
    }

    for (int i = 0; i < 3; i++) {
        pointVertex[i] = expression[i] + shape[i] + faceModel->meanshapeAr[pointIndex*3 + i];
        //face[i] = expression[i] + shape[i] + faceModel->meanshapeAr[pointIndex*3 + i];
    }

    //apply scale
    for (int i = 0; i < 3; i++) {
        pointVertex[i] = pointVertex[i] * T(faceModel->scale);
    }

    //apply rotation
    for (int i = 0; i < 3; i++) {
        T sum = T(0.0);
        for (int j = 0; j < 3; j++) {
            sum += pointVertex[j] * faceModel->rotation(i,j);
        }
        face[i] = T(sum);
    }
 
    //apply translation
    for (int i = 0; i < 3; i++) {
        face[i] += T(faceModel->translation(i));
    }

    // delete [] expression;
    // delete [] shape;
    // delete [] pointVertex;

}


template <typename T>
class ExpShapeCoeffIncrement {
public:
    explicit ExpShapeCoeffIncrement(T* const arrayExpCoef, T* const arrayShapeCoef) :
    m_arrayExpCoef{ arrayExpCoef },
    m_arrayShapeCoef{ arrayShapeCoef }
    { }

    void setZero() {
        for (int i = 0; i < 64; ++i)
            m_arrayExpCoef[i] = T(0);

        for (int i = 0; i < 80; ++i)
            m_arrayShapeCoef[i] = T(0);
    }

    T* getExpCoeff() const {
        return m_arrayExpCoef;
    }

    T* getShapeCoeff() const {
        return m_arrayShapeCoef;
    }

    void apply(const int inputIndex, T* outputPoint) const {
        const T* expCoef = m_arrayExpCoef;
        const T* shapeCoef = m_arrayShapeCoef;
        T* faces = new T[3];

        dotFace_ceres(inputIndex, expCoef, shapeCoef, faces);

        outputPoint[0] = faces[0];
        outputPoint[1] = faces[1];
        outputPoint[2] = faces[2];

        delete [] faces;
    }
    /*int writeExpCoeff(){
        std::ofstream out("../sample_face/expression", ios::out | ios::binary);
        if(!out) {
            cout << "Cannot open file.";
            return 1;
        }
        out.write((char *) m_arrayExpCoef, sizeof m_arrayExpCoef);
        out.close();
    } */

    
    void writeExp(){
        std::ofstream myfile;
        myfile.open ("../sample_face/expression.csv");
        for(int i=0 ; i<64 ; i++){
            myfile << std::to_string(m_arrayExpCoef[i])+"\n";
        }
        myfile.close();
    }
    

    void readExp() {

        std::string line;
        std::ifstream file("../sample_face/expression.csv");
        std::string value;

        int ind=0;
        while (getline(file, line)) {

            std::stringstream lineStringStream(line);

            while (std::getline(lineStringStream, value, ','))
            {
                std::cout<<value<<std::endl;
                m_arrayExpCoef[ind]=stod(value);
                ind++;
            }
        }
    }

    /*void readExpCoeff(){
        std::ifstream in("../sample_face/expression", ios::in | ios::binary);
        //std::cout<<"lel"<<std::endl;
        //std::cout<<sizeof m_arrayExpCoef<<std::endl;
        double* new_exp = new double [64];
        
        in.read((char *) new_exp, 64);//sizeof m_arrayExpCoef);
        std::cout<<"-----------------------------------"<<std::endl;
        in.close();
        int diff=0;
        for(int i=0 ; i<64 ; i++){
            if(m_arrayExpCoef[i]!=new_exp[i]) diff++;
        }
        std::cout<<diff<<std::endl;
        m_arrayExpCoef = new_exp ;
        
    }*/


private:
    T* m_arrayExpCoef;
    T* m_arrayShapeCoef;
};


class MyCustomConstraint {
public:
    MyCustomConstraint(const int sourcePointIndex, const Vector3f& targetPoint, const float weight) :
            m_sourcePointIndex{ sourcePointIndex },
            m_targetPoint{ targetPoint },
            m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const expCoeff, const T* const shapeCoeff, T* residuals) const {
        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<T>(const_cast<T*>(expCoeff), const_cast<T*>(shapeCoeff));
        T p_s_tilda[3];
        expShapeCoeffIncrement.apply(m_sourcePointIndex, p_s_tilda);
        residuals[0] = T(LAMBDA) * T(m_weight) * (p_s_tilda[0] - T(m_targetPoint[0]));
        residuals[1] = T(LAMBDA) * T(m_weight) * (p_s_tilda[1] - T(m_targetPoint[1]));
        residuals[2] = T(LAMBDA) * T(m_weight) * (p_s_tilda[2] - T(m_targetPoint[2]));

        return true;
    }

    static ceres::CostFunction* create(const int sourcePointIndex, const Vector3f& targetPoint, const float weight) {
        return new ceres::AutoDiffCostFunction<MyCustomConstraint, 3, 64, 80>(
                new MyCustomConstraint(sourcePointIndex, targetPoint, weight)
        );
    }

protected:
    const int m_sourcePointIndex;
    const Vector3f m_targetPoint;
    const float m_weight;
    const float LAMBDA = 0.1f;
};

class PointToPlaneConstr {
public:
    PointToPlaneConstr(const int sourcePointIndex, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
            m_sourcePointIndex{ sourcePointIndex },
            m_targetPoint{ targetPoint },
            m_targetNormal{ targetNormal },
            m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const expCoeff, const T* const shapeCoeff, T* residuals) const {
        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<T>(const_cast<T*>(expCoeff), const_cast<T*>(shapeCoeff));
        T p_s_tilda[3];
        expShapeCoeffIncrement.apply(m_sourcePointIndex, p_s_tilda);

        residuals[0] = T(LAMBDA) * T(m_weight) * T(m_targetNormal[0]) * (p_s_tilda[0] - T(m_targetPoint[0]));
        residuals[0] += T(LAMBDA) * T(m_weight) * T(m_targetNormal[1]) * (p_s_tilda[1] - T(m_targetPoint[1]));
        residuals[0] += T(LAMBDA) * T(m_weight) * T(m_targetNormal[2]) * (p_s_tilda[2] - T(m_targetPoint[2]));

        return true;
    }

    static ceres::CostFunction* create(const int sourcePointIndex, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
        return new ceres::AutoDiffCostFunction<PointToPlaneConstr, 1, 64, 80>(
                new PointToPlaneConstr(sourcePointIndex, targetPoint, targetNormal, weight)
        );
    }

protected:
    const int m_sourcePointIndex;
    const Vector3f m_targetPoint;
    const Vector3f m_targetNormal;
    const float m_weight;
    const float LAMBDA = 0.1f;
};

/**
 * ICP optimizer - Abstract Base Class
 */
class ICPOptimizer {
public:
    // ICPOptimizer() :
    //         m_nIterations{ 20 },
    //         m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchFlann>() }
    // { }
    ICPOptimizer() :
            m_nIterations{ 20 },
            m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchBruteForce>() }
    { }

    void setMatchingMaxDistance(float maxDistance) {
        m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
    }


    void setNbOfIterations(unsigned nIterations) {
        m_nIterations = nIterations;
    }

    virtual void estimateExpShapeCoeffs(const PointCloud& target) = 0;
    virtual void estimateExpShapeCoeffs(const std::vector<Eigen::Vector3f> &target) = 0;


protected:
    unsigned m_nIterations;
    std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;
    std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4d& pose, double scale) {
        std::vector<Vector3f> transformedPoints;
        transformedPoints.reserve(sourcePoints.size());

        const auto rotation = FaceModel::getInstance()->rotation;
        const auto translation = FaceModel::getInstance()->translation;

        for (const auto& point : sourcePoints) {
            Vector3d pointD = point.cast<double>();
            pointD *= scale;
            Vector3d tmp = rotation * pointD + translation;
            Vector3f tmpf = tmp.cast<float>();
            transformedPoints.push_back(tmpf);
        }

        return transformedPoints;
    }

    std::vector<Vector3f> transformNormals(const std::vector<Vector3f>& sourceNormals, const Matrix4d& pose, double scale) {
        std::vector<Vector3f> transformedNormals;
        transformedNormals.reserve(sourceNormals.size());

        const auto rotation = FaceModel::getInstance()->rotation;

        for (const auto& normal : sourceNormals) {
            Vector3d normalD = normal.cast<double>();
            normalD *= scale;
            Vector3d tmp = rotation.inverse().transpose() * normalD;
            Vector3f tmpf = tmp.cast<float>();
            transformedNormals.push_back(tmpf);
        }

        return transformedNormals;
    }

    void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
        const unsigned nPoints = sourceNormals.size();
        int sum = 0;
        for (unsigned i = 0; i < nPoints; i++) {
            Match& match = matches[i];
            if (match.idx >= 0) {
                sum++;
                const auto& sourceNormal = sourceNormals[i];
                const auto& targetNormal = targetNormals[match.idx];

                // TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
                if(acos(sourceNormal.dot(targetNormal) / (sourceNormal.norm() * targetNormal.norm())) * 180.0 / M_PI > 60.0) {
                    match.idx = -1;
                }
            }
        }
        std::cout << "sum: " << sum << std::endl;
    }
};


/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer {
public:
    CeresICPOptimizer() {}

    virtual void estimateExpShapeCoeffs(const PointCloud &target) override {
        // Build the index of the FLANN tree (for fast nearest neighbor lookup).
        m_nearestNeighborSearch->buildIndex(target.getPoints());

        double incrementArrayExp[64];
        double incrementArrayShape[80];

        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<double>(incrementArrayExp, incrementArrayShape);
        expShapeCoeffIncrement.setZero();

        FaceModel* faceModel = FaceModel::getInstance();
        for (int i = 0; i < m_nIterations; ++i) {
            // Compute the matches.
            std::cout << "Matching points ..." << std::endl;
            clock_t begin = clock();

            
            SimpleMesh faceMesh;
            if (!faceMesh.loadMesh("transformed_model_procusters.off")) {
                std::cout << "Mesh file wasn't read successfully at location: " << "transformed_model_procusters.off" << std::endl;
            }

            PointCloud faceModelPoints{faceMesh};
            // for (size_t i = 0; i < faceModelPoints.getPoints().size(); ++i)
            //         std::cout << "    " << faceModelPoints.getPoints()[i].x()
            //       << " " << faceModelPoints.getPoints()[i].y()
            //       << " " << faceModelPoints.getPoints()[i].z() << std::endl;

            
            auto matches = m_nearestNeighborSearch->queryMatches(faceModelPoints.getPoints());
            int matchCtr = 0;
            for(Match match : matches) {
                if (match.idx >= 0)
                   matchCtr++;
            }
            std::cout << "match count before pruning:" << matchCtr << std::endl;
            pruneCorrespondences(faceModelPoints.getNormals(), target.getNormals(), matches);

            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

            matchCtr = 0;
            for(Match match : matches) {
                if (match.idx >= 0)
                   matchCtr++;
            }
            std::cout << "match count:" << matchCtr << std::endl;
            // Prepare point-to-point and point-to-plane constraints.
            ceres::Problem problem;
            customPrepareConstraints(target.getPoints(), target.getNormals(), matches, expShapeCoeffIncrement, problem);
            //customPrepareConstraints(target.getPoints(), matches, expShapeCoeffIncrement, problem);

            // Configure options for the solver.
            ceres::Solver::Options options;
            configureSolver(options);
            
            // Run the solver (for one iteration).
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;
            //std::cout << summary.FullReport() << std::endl;

            //update face model with these params
            if(i == m_nIterations-1){
                std::cout<<"-----------LAST ITERATION--------------"<<std::endl;
                if(TRANSFER){
                    expShapeCoeffIncrement.readExp();
                    std::cout<<"-----------TRANSFER--------------"<<std::endl;
                }
                else{
                    std::cout<<"------EXPRESSION WRITE-----------"<<std::endl;
                    expShapeCoeffIncrement.writeExp();
                }
            }
            faceModel->expCoefAr = expShapeCoeffIncrement.getExpCoeff();        
            faceModel->shapeCoefAr = expShapeCoeffIncrement.getShapeCoeff();

            MatrixXd transformed_mesh;
            transformed_mesh = faceModel->transform(faceModel->pose, faceModel->scale);
            
            faceModel->write_off("transformed_model_icp.off",transformed_mesh);
            
            std::cout << "Optimization iteration done." << std::endl;
        }
        MatrixXd transformed_mesh;
        transformed_mesh = faceModel->transform(faceModel->pose, faceModel->scale);

        faceModel->write_off("result.off",transformed_mesh);
        
        faceModel->write_obj("result.obj",transformed_mesh);
    }

    virtual void estimateExpShapeCoeffs(const std::vector<Eigen::Vector3f> &target) override {
        // Build the index of the FLANN tree (for fast nearest neighbor lookup).
        
        m_nearestNeighborSearch->buildIndex(target);

        double incrementArrayExp[64];
        double incrementArrayShape[80];

        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<double>(incrementArrayExp, incrementArrayShape);
        expShapeCoeffIncrement.setZero();

        FaceModel* faceModel = FaceModel::getInstance();
        for (int i = 0; i < m_nIterations; ++i) {
            // Compute the matches.
            std::cout << "Matching points ..." << std::endl;
            clock_t begin = clock();

            
            SimpleMesh faceMesh;
            if (!faceMesh.loadMesh("transformed_model.off")) {
                std::cout << "Mesh file wasn't read successfully at location: " << "transformed_model.off" << std::endl;
            }

            PointCloud faceModelPoints{faceMesh};
            std::cout << " inside mesh " << faceModelPoints.getPoints()[faceModel->key_points[31]] << std::endl;
            
            
            //auto matches = m_nearestNeighborSearch->queryMatches(faceModelPoints.getPoints());
            auto matches = m_nearestNeighborSearch->queryMatches(faceModelPoints.getPoints());


            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;


            int matchCtr = 0;
            for(Match match : matches) {
                if (match.idx >= 0)
                   matchCtr++;
            }
            std::cout << "match count:" << matchCtr << std::endl;
            // Prepare point-to-point and point-to-plane constraints.
            ceres::Problem problem;
            customPrepareConstraints(target, matches, expShapeCoeffIncrement, problem);

            // Configure options for the solver.
            ceres::Solver::Options options;
            configureSolver(options);

            // Run the solver (for one iteration).
            ceres::Solver::Summary summary;
            exit(0);
            ceres::Solve(options, &problem, &summary);
        
            std::cout << summary.BriefReport() << std::endl;

            //update face model with these params
            faceModel->expCoefAr = expShapeCoeffIncrement.getExpCoeff();
            faceModel->shapeCoefAr = expShapeCoeffIncrement.getShapeCoeff();

            MatrixXd transformed_mesh;
            transformed_mesh = faceModel->transform(faceModel->pose, faceModel->scale);
            faceModel->write_off("../sample_face/transformed_model.off",transformed_mesh);

            std::cout << "Optimization iteration done." << std::endl;
        }
        faceModel->write_off("../sample_face/result.off");
    }


private:
    void configureSolver(ceres::Solver::Options &options) {
        // Ceres options.
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.use_nonmonotonic_steps = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = 1;
        options.max_num_iterations = 1;
        options.num_threads = 8;
    }

    void customPrepareConstraints(const std::vector<Vector3f> &targetPoints,
                                  const std::vector<Vector3f> &targetNormals,
                                  const std::vector<Match> matches,
                                  const ExpShapeCoeffIncrement<double> &expShapeCoeffIncrement,
                                  ceres::Problem &problem) const {
                                    
        const unsigned nPoints = targetPoints.size();
        for (unsigned i = 0; i < 35709; ++i) {
            const auto match = matches[i];
            if (match.idx >= 0) {
                const auto &targetPoint = targetPoints[match.idx];

                const int sourcePointIndex = i;
                problem.AddResidualBlock(
                        MyCustomConstraint::create(sourcePointIndex, targetPoint, match.weight),
                        nullptr,
                        expShapeCoeffIncrement.getExpCoeff(),
                        expShapeCoeffIncrement.getShapeCoeff()
                );

                const auto& targetNormal = targetNormals[match.idx];
                if (!targetNormal.allFinite())
                    continue;

                problem.AddResidualBlock(
                    PointToPlaneConstr::create(sourcePointIndex, targetPoint, targetNormal, match.weight),
                    nullptr, 
                    expShapeCoeffIncrement.getExpCoeff(),
                    expShapeCoeffIncrement.getShapeCoeff()
                );
            }
        }

        ceres::CostFunction* shape_cost = new ceres::AutoDiffCostFunction<ShapeCostFunction, 80, 80>(
				new ShapeCostFunction(SHAPE_REGULARIZATION_WEIGHT)
				);
		problem.AddResidualBlock(shape_cost, nullptr, expShapeCoeffIncrement.getShapeCoeff());

    
        ceres::CostFunction* expression_cost = new ceres::AutoDiffCostFunction<ExpressionCostFunction, 64, 64>(
				new ExpressionCostFunction(EXPRESSION_REGULARIZATION_WEIGHT)
				);
		problem.AddResidualBlock(expression_cost, nullptr, expShapeCoeffIncrement.getExpCoeff());

    }

    void customPrepareConstraints(const std::vector<Vector3f> &targetPoints,
                                  const std::vector<Match> matches,
                                  const ExpShapeCoeffIncrement<double> &expShapeCoeffIncrement,
                                  ceres::Problem &problem) const {
        for (unsigned i = 0; i < 35709; ++i) {
            const auto match = matches[i];
            if (match.idx >= 0) {
                const auto &targetPoint = targetPoints[match.idx];

                const int sourcePointIndex = i;
                problem.AddResidualBlock(
                        MyCustomConstraint::create(sourcePointIndex, targetPoint, match.weight),
                        nullptr,
                        expShapeCoeffIncrement.getExpCoeff(),
                        expShapeCoeffIncrement.getShapeCoeff()
                );

            }
        }
    }
};