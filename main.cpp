#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "FaceModel.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>


#define SHOW_BUNNY_CORRESPONDENCES 0

#define USE_POINT_TO_PLANE	0
#define USE_LINEAR_ICP		0

#define RUN_SHAPE_ICP		0
#define RUN_SEQUENCE_ICP	1

// void debugCorrespondenceMatching() {
// 	// Load the source and target mesh.
// 	const std::string filenameSource = std::string("../../../Data/bunny_part2_trans.off");
// 	const std::string filenameTarget = std::string("../../../Data/bunny_part1.off");

// 	SimpleMesh sourceMesh;
// 	if (!sourceMesh.loadMesh(filenameSource)) {
// 		std::cout << "Mesh file wasn't read successfully." << std::endl;
// 		return;
// 	}

// 	SimpleMesh targetMesh;
// 	if (!targetMesh.loadMesh(filenameTarget)) {
// 		std::cout << "Mesh file wasn't read successfully." << std::endl;
// 		return;
// 	}

// 	PointCloud source{ sourceMesh };
// 	PointCloud target{ targetMesh };
	
// 	// Search for matches using FLANN.
// 	std::unique_ptr<NearestNeighborSearch> nearestNeighborSearch = std::make_unique<NearestNeighborSearchFlann>();
// 	nearestNeighborSearch->setMatchingMaxDistance(0.0001f);
// 	nearestNeighborSearch->buildIndex(target.getPoints());
// 	auto matches = nearestNeighborSearch->queryMatches(source.getPoints());

// 	// Visualize the correspondences with lines.
// 	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, Matrix4f::Identity());
// 	auto sourcePoints = source.getPoints();
// 	auto targetPoints = target.getPoints();

// 	for (unsigned i = 0; i < 100; ++i) { // sourcePoints.size()
// 		const auto match = matches[i];
// 		if (match.idx >= 0) {
// 			const auto& sourcePoint = sourcePoints[i];
// 			const auto& targetPoint = targetPoints[match.idx];
// 			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::cylinder(sourcePoint, targetPoint, 0.002f, 2, 15), resultingMesh, Matrix4f::Identity());
// 		}
// 	}

// 	resultingMesh.writeMesh(std::string("correspondences.off"));
// }

// int alignBunnyWithICP() {
// 	// Load the source and target mesh.
// 	const std::string filenameSource = std::string("../../Data/bunny_part2_trans.off");
// 	const std::string filenameTarget = std::string("../../Data/bunny_part1.off");

// 	SimpleMesh sourceMesh;
// 	if (!sourceMesh.loadMesh(filenameSource)) {
// 		std::cout << "Mesh file wasn't read successfully at location: " << filenameSource << std::endl;
// 		return -1;
// 	}

// 	SimpleMesh targetMesh;
// 	if (!targetMesh.loadMesh(filenameTarget)) {
// 		std::cout << "Mesh file wasn't read successfully at location: " << filenameTarget << std::endl;
// 		return -1;
// 	}

// 	// Estimate the pose from source to target mesh with ICP optimization.
// 	ICPOptimizer* optimizer = nullptr;
// 	if (USE_LINEAR_ICP) {
// 		optimizer = new LinearICPOptimizer();
// 	}
// 	else {
// 		optimizer = new CeresICPOptimizer();
// 	}
	
// 	optimizer->setMatchingMaxDistance(0.0003f);
// 	if (USE_POINT_TO_PLANE) {
// 		optimizer->usePointToPlaneConstraints(true);
// 		optimizer->setNbOfIterations(10);
// 	}
// 	else {
// 		optimizer->usePointToPlaneConstraints(false);
// 		optimizer->setNbOfIterations(20);
// 	}

// 	PointCloud source{ sourceMesh };
// 	PointCloud target{ targetMesh };

//     Matrix4f estimatedPose = Matrix4f::Identity();
// 	optimizer->estimatePose(source, target, estimatedPose);
	
// 	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
// 	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
// 	if (SHOW_BUNNY_CORRESPONDENCES) {
// 		for (const auto& sourcePoint : source.getPoints()) {
// 			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(sourcePoint, 0.001f), resultingMesh, estimatedPose);
// 		}
// 		for (const auto& targetPoint : target.getPoints()) {
// 			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(targetPoint, 0.001f, Vector4uc(255, 255, 255, 255)), resultingMesh, Matrix4f::Identity());
// 		}
// 	}
// 	resultingMesh.writeMesh(std::string("bunny_icp.off"));
// 	std::cout << "Resulting mesh written." << std::endl;

// 	delete optimizer;

// 	return 0;
// }

// int reconstructRoom() {
// 	std::string filenameIn = std::string("../../Data/rgbd_dataset_freiburg1_xyz/");
// 	std::string filenameBaseOut = std::string("mesh_");

// 	// Load video
// 	std::cout << "Initialize virtual sensor..." << std::endl;
// 	VirtualSensor sensor;
// 	if (!sensor.init(filenameIn)) {
// 		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
// 		return -1;
// 	}

// 	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
// 	sensor.processNextFrame();
// 	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
	
// 	// Setup the optimizer.
// 	ICPOptimizer* optimizer = nullptr;
// 	if (USE_LINEAR_ICP) {
// 		optimizer = new LinearICPOptimizer();
// 	}
// 	else {
// 		optimizer = new CeresICPOptimizer();
// 	}

// 	optimizer->setMatchingMaxDistance(0.1f);
// 	if (USE_POINT_TO_PLANE) {
// 		optimizer->usePointToPlaneConstraints(true);
// 		optimizer->setNbOfIterations(10);
// 	}
// 	else {
// 		optimizer->usePointToPlaneConstraints(false);
// 		optimizer->setNbOfIterations(20);
// 	}

// 	// We store the estimated camera poses.
// 	std::vector<Matrix4f> estimatedPoses;
// 	Matrix4f currentCameraToWorld = Matrix4f::Identity();
// 	estimatedPoses.push_back(currentCameraToWorld.inverse());

// 	int i = 0;
// 	const int iMax = 22;
// 	while (sensor.processNextFrame() && i <= iMax) {
// 		float* depthMap = sensor.getDepth();
// 		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
// 		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

// 		// Estimate the current camera pose from source to target mesh with ICP optimization.
// 		// We downsample the source image to speed up the correspondence matching.
// 		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8 };
// 		optimizer->estimatePose(source, target, currentCameraToWorld);
		
// 		// Invert the transformation matrix to get the current camera pose.
// 		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
// 		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
// 		estimatedPoses.push_back(currentCameraPose);

// 		if (i % 5 == 0) {
// 			// We write out the mesh to file for debugging.
// 			SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.1f };
// 			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
// 			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

// 			std::stringstream ss;
// 			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
// 			std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
// 			if (!resultingMesh.writeMesh(ss.str())) {
// 				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
// 				return -1;
// 			}
// 		}
		
// 		i++;
// 	}

// 	delete optimizer;

// 	return 0;
// }

int main() {


	FaceModel* model = FaceModel::getInstance();

    std::vector<Eigen::Vector3f> targetPoints = {
        {339.0f, 231.0f, -51.26377f},
        {341.0f, 247.0f, -54.394043f},
        {346.0f, 260.0f, -57.10093f},
        {348.0f, 274.0f, -57.813145f},
        {352.0f, 289.0f, -53.41708f},
        {356.0f, 303.0f, -42.512188f},
        {362.0f, 314.0f, -27.945274f},
        {370.0f, 324.0f, -13.493111f},
        {387.0f, 331.0f, -5.274869f},
        {404.0f, 326.0f, -7.851542f},
        {418.0f, 316.0f, -18.056065f},
        {429.0f, 308.0f, -29.213854f},
        {437.0f, 293.0f, -36.972614f},
        {443.0f, 279.0f, -38.976994f},
        {447.0f, 264.0f, -36.700962f},
        {449.0f, 249.0f, -32.729908f},
        {452.0f, 235.0f, -28.608301f},
        {341.0f, 224.0f, 6.196493f},
        {346.0f, 220.0f, 17.03346f},
        {354.0f, 220.0f, 24.801369f},
        {360.0f, 220.0f, 29.541372f},
        {368.0f, 222.0f, 31.73381f},
        {398.0f, 222.0f, 38.350487f},
        {404.0f, 222.0f, 39.09272f},
        {414.0f, 220.0f, 37.66686f},
        {422.0f, 222.0f, 33.61022f},
        {433.0f, 227.0f, 25.210922f},
        {381.0f, 243.0f, 29.819656f},
        {379.0f, 258.0f, 32.66819f},
        {377.0f, 268.0f, 36.990982f},
        {377.0f, 279.0f, 35.818264f},
        {373.0f, 281.0f, 18.79889f},
        {377.0f, 283.0f, 21.497822f},
        {381.0f, 285.0f, 23.289898f},
        {387.0f, 283.0f, 23.320747f},
        {391.0f, 281.0f, 21.96367f},
        {352.0f, 237.0f, 10.821422f},
        {356.0f, 235.0f, 18.450817f},
        {362.0f, 237.0f, 20.377682f},
        {370.0f, 239.0f, 18.565527f},
        {364.0f, 241.0f, 18.750368f},
        {356.0f, 241.0f, 15.707586f},
        {400.0f, 241.0f, 25.596058f},
        {406.0f, 237.0f, 30.149288f},
        {412.0f, 239.0f, 31.466301f},
        {420.0f, 241.0f, 25.682638f},
        {414.0f, 243.0f, 28.237394f},
        {406.0f, 243.0f, 28.176151f},
        {364.0f, 297.0f, 5.497734f},
        {370.0f, 297.0f, 14.995447f},
        {377.0f, 295.0f, 21.20818f},
        {381.0f, 297.0f, 22.265253f},
        {387.0f, 295.0f, 22.640322f},
        {395.0f, 297.0f, 18.384308f},
        {406.0f, 297.0f, 10.33053f},
        {395.0f, 301.0f, 15.638853f},
        {389.0f, 306.0f, 17.632006f},
        {383.0f, 306.0f, 17.351776f},
        {377.0f, 306.0f, 15.570858f},
        {370.0f, 301.0f, 11.963271f},
        {366.0f, 297.0f, 6.0162396f},
        {377.0f, 299.0f, 16.507935f},
        {383.0f, 299.0f, 18.626564f},
        {389.0f, 299.0f, 18.051886f},
        {404.0f, 297.0f, 10.109316f},
        {389.0f, 297.0f, 18.08264f},
        {383.0f, 299.0f, 17.961336f},
        {377.0f, 297.0f, 16.285294f}
    };
    std::vector<std::pair<int, int>> keypoints_2d = {
    {339, 239}, {339, 256}, {341, 268}, {343, 281}, {348, 295}, {356, 308}, {364, 316}, {370, 324}, 
    {387, 331}, {406, 326}, {420, 316}, {431, 308}, {439, 293}, {445, 276}, {447, 264}, {449, 249}, 
    {452, 233}, {341, 224}, {346, 220}, {354, 220}, {360, 220}, {366, 222}, {395, 224}, {404, 222}, 
    {412, 220}, {422, 222}, {431, 227}, {381, 245}, {379, 258}, {377, 270}, {377, 279}, {373, 281}, 
    {377, 285}, {381, 285}, {387, 285}, {391, 283}, {352, 239}, {356, 237}, {362, 237}, {370, 241}, 
    {364, 243}, {358, 241}, {400, 241}, {406, 239}, {412, 239}, {418, 241}, {412, 245}, {406, 245}, 
    {366, 297}, {370, 297}, {379, 297}, {383, 297}, {387, 297}, {398, 297}, {406, 297}, {398, 301}, 
    {389, 306}, {383, 306}, {377, 306}, {373, 301}, {366, 297}, {379, 299}, {383, 301}, {389, 299}, 
    {404, 297}, {389, 299}, {383, 299}, {379, 297}
};

	pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/cloud_1.pcd", cloud) == -1) // Load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    std::cout << "Read cloud file 1" << std::endl;

    std::vector<pcl::PointXYZ> keypoints;
    std::vector<Eigen::Vector3f> keypoints_vectors;
    std::vector<int> x_val;
    std::vector<int> y_val;

    // for (const auto& point : keypoints_2d) {
    //     int x = point.first;
    //     int y = point.second;
    //     pcl::PointXYZ keypoint = cloud.at(x / 2, y / 2);

    //     x_val.push_back(x);
    //     y_val.push_back(y);
    //     keypoints.push_back(keypoint);
    //     Eigen::Vector3f vec(keypoint.x, keypoint.y, keypoint.z);
    //     keypoints_vectors.push_back(vec);
    // }
    for (const auto& point : keypoints_2d) {
        int x = point.first;
        int y = point.second;

        // Ensure the indices are within bounds
        if (x / 2 >= cloud.width || y / 2 >= cloud.height) {
            // std::cerr << "Point (" << x << ", " << y << ") is out of bounds in the point cloud" << std::endl;
            continue;
        }

        pcl::PointXYZ keypoint = cloud.at(x / 2, y / 2);

        // Check for NaN values in the keypoint
        if (pcl::isFinite(keypoint)) {
            x_val.push_back(x);
            y_val.push_back(y);
            keypoints.push_back(keypoint);
            Eigen::Vector3f vec(keypoint.x, keypoint.y, keypoint.z);
            keypoints_vectors.push_back(vec);
        } 
    }

    std::cout << "Calculated Keypoints vector" << std::endl;

    int min_x = *std::min_element(x_val.begin(), x_val.end());
    int min_y = *std::min_element(y_val.begin(), y_val.end());
    int max_x = *std::max_element(x_val.begin(), x_val.end());
    int max_y = *std::max_element(y_val.begin(), y_val.end());

    float* cropped_depth_map = new float[540 * 960];
    int ind = 0;
    for (int y = 0; y < 540; y++) {
        for (int x = 0; x < 960; x++) {
            // std::cout << "shakek" << std::endl;
            // std::cout << cloud.at(x, y) << std::endl;
            cropped_depth_map[ind] = cloud.at(x, y).z;
            // std::cout << "5alas 7asal 5eer" << std::endl;
            ind++;
        }
    }
    std::cout << "Calculated cropped depth map" << std::endl;

    Eigen::Matrix3f depthIntrinsics;
    Eigen::Matrix4f depthExtrinsics;
    depthExtrinsics.setIdentity();
    depthIntrinsics << 1052.667867276341 / 2, 0, 962.4130834944134 / 2, 0, 1052.020917785721 / 2, 536.2206151001486 / 2, 0, 0, 1;



    std::vector<Eigen::Vector3f> pclPoints;

    for (int y = 0; y < 540; y++) {
        for (int x = 0; x < 960; x++) {
            pcl::PointXYZ keypoint = cloud.at(x, y);

            Eigen::Vector3f point(keypoint.x, keypoint.y, keypoint.z);
            pclPoints.push_back(point);
        }
    }

    std::cout << "pre" << std::endl;

    PointCloud cropped_cloud = PointCloud(cropped_depth_map, pclPoints, 960, 540);


    std::vector<int> indicess;
	pcl::removeNaNFromPointCloud(cloud, cloud, indicess);
    std::cout << "Removed Nan points from cloud" << std::endl;

    
    targetPoints = keypoints_vectors;
    // for(int i = 0; i < targetPoints.size() ; ++i){
    //     std::cout << "-> " << targetPoints[i] << std::endl;
    // }

	std::vector<Vector3f> sourcePoints = model->key_vectors;

	Eigen::Vector3f target_diff;
    target_diff << targetPoints[1](0) - targetPoints[16](0),	targetPoints[1](1)-targetPoints[16](1), targetPoints[1](2)-targetPoints[16](2);
	Eigen::Vector3f source_diff;
    source_diff << sourcePoints[1](0) - sourcePoints[16](0), 	sourcePoints[1](1)-sourcePoints[16](1), sourcePoints[1](2)-sourcePoints[16](2);
	
	double target_scale = target_diff.norm();
    double source_scale = source_diff.norm();
	
	double scale = target_scale / source_scale;

    for(int ind= 0 ; ind<sourcePoints.size() ; ind ++) {
        sourcePoints[ind] = scale * sourcePoints[ind] ;
    }

	// std::vector<Vector3f> sp_deneme ;
    // sp_deneme.push_back(sourcePoints[0]);sp_deneme.push_back(sourcePoints[16]);sp_deneme.push_back(sourcePoints[27]);sp_deneme.push_back(sourcePoints[8]) ;

    // std::vector<Vector3f> tp_deneme ;
    // tp_deneme.push_back(targetPoints[0]);tp_deneme.push_back(targetPoints[16]);tp_deneme.push_back(targetPoints[27]);tp_deneme.push_back(targetPoints[8]); ;

    ProcrustesAligner aligner;
	Matrix4f estimatedPose = aligner.estimatePose(sourcePoints, targetPoints);
	
	Matrix4d estimatedPoseD = estimatedPose.cast<double>();
    std::cout << estimatedPoseD << std::endl;

	model->pose = estimatedPoseD;
    model->scale = scale;
    model->rotation = estimatedPoseD.block<3,3>(0,0);
    model->translation = estimatedPoseD.block<3,1>(0,3);

	MatrixXd transformed_mesh;
	transformed_mesh = model->transform(model->pose, model->scale);
	model->write_obj("transformed_model_procusters.obj",transformed_mesh);
	model->write_off("transformed_model_procusters.off",transformed_mesh);

	SimpleMesh faceMesh;
	if (!faceMesh.loadMesh("transformed_model.off")) {
		std::cout << "Mesh file wasn't read successfully at location: " << "transformed_model.off" << std::endl;
	}

	PointCloud faceModelPoints{faceMesh}; // source


	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/cloud_1.pcd", pcl_cloud) == -1) // Load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, indices);
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, estimatedPose);


	PointCloud target_cloud(pcl_cloud);
    

    std::cout << target_cloud.getPoints()[10] << std::endl; 
    std::cout << faceModelPoints.getPoints()[10] << std::endl; 
	

	// Estimate the pose from source to target mesh with ICP optimization.
	// CeresICPOptimizer* optimizer = nullptr;
	// optimizer = new CeresICPOptimizer();
	
    // optimizer->setMatchingMaxDistance(0.000005f);
    // optimizer->setNbOfIterations(10);
	
	// optimizer->estimateExpShapeCoeffs(target_cloud);
    CeresICPOptimizer * optimizer = nullptr;
    optimizer = new CeresICPOptimizer();
    
    optimizer->setMatchingMaxDistance(0.000005f);
    optimizer->setNbOfIterations(10);
    optimizer->estimateExpShapeCoeffs(cropped_cloud);

	
	// optimizer->estimatePose(faceModelPoints, target_cloud, estimatedPose);
	
	// SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	// resultingMesh.writeMesh(std::string("bunny_icp.off"));

	
	// std::cout << estimatedPose << std::endl;

	return 0;
}
