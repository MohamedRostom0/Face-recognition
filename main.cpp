#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "FaceModel.h"

#define SHOW_BUNNY_CORRESPONDENCES 0

#define USE_POINT_TO_PLANE	0
#define USE_LINEAR_ICP		0

#define RUN_SHAPE_ICP		0
#define RUN_SEQUENCE_ICP	1

void debugCorrespondenceMatching() {
	// Load the source and target mesh.
	const std::string filenameSource = std::string("../../../Data/bunny_part2_trans.off");
	const std::string filenameTarget = std::string("../../../Data/bunny_part1.off");

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return;
	}

	PointCloud source{ sourceMesh };
	PointCloud target{ targetMesh };
	
	// Search for matches using FLANN.
	std::unique_ptr<NearestNeighborSearch> nearestNeighborSearch = std::make_unique<NearestNeighborSearchFlann>();
	nearestNeighborSearch->setMatchingMaxDistance(0.0001f);
	nearestNeighborSearch->buildIndex(target.getPoints());
	auto matches = nearestNeighborSearch->queryMatches(source.getPoints());

	// Visualize the correspondences with lines.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, Matrix4f::Identity());
	auto sourcePoints = source.getPoints();
	auto targetPoints = target.getPoints();

	for (unsigned i = 0; i < 100; ++i) { // sourcePoints.size()
		const auto match = matches[i];
		if (match.idx >= 0) {
			const auto& sourcePoint = sourcePoints[i];
			const auto& targetPoint = targetPoints[match.idx];
			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::cylinder(sourcePoint, targetPoint, 0.002f, 2, 15), resultingMesh, Matrix4f::Identity());
		}
	}

	resultingMesh.writeMesh(std::string("correspondences.off"));
}

int alignBunnyWithICP() {
	// Load the source and target mesh.
	const std::string filenameSource = std::string("../../Data/bunny_part2_trans.off");
	const std::string filenameTarget = std::string("../../Data/bunny_part1.off");

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully at location: " << filenameSource << std::endl;
		return -1;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully at location: " << filenameTarget << std::endl;
		return -1;
	}

	// Estimate the pose from source to target mesh with ICP optimization.
	ICPOptimizer* optimizer = nullptr;
	if (USE_LINEAR_ICP) {
		optimizer = new LinearICPOptimizer();
	}
	else {
		optimizer = new CeresICPOptimizer();
	}
	
	optimizer->setMatchingMaxDistance(0.0003f);
	if (USE_POINT_TO_PLANE) {
		optimizer->usePointToPlaneConstraints(true);
		optimizer->setNbOfIterations(10);
	}
	else {
		optimizer->usePointToPlaneConstraints(false);
		optimizer->setNbOfIterations(20);
	}

	PointCloud source{ sourceMesh };
	PointCloud target{ targetMesh };

    Matrix4f estimatedPose = Matrix4f::Identity();
	optimizer->estimatePose(source, target, estimatedPose);
	
	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	if (SHOW_BUNNY_CORRESPONDENCES) {
		for (const auto& sourcePoint : source.getPoints()) {
			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(sourcePoint, 0.001f), resultingMesh, estimatedPose);
		}
		for (const auto& targetPoint : target.getPoints()) {
			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(targetPoint, 0.001f, Vector4uc(255, 255, 255, 255)), resultingMesh, Matrix4f::Identity());
		}
	}
	resultingMesh.writeMesh(std::string("bunny_icp.off"));
	std::cout << "Resulting mesh written." << std::endl;

	delete optimizer;

	return 0;
}

int reconstructRoom() {
	std::string filenameIn = std::string("../../Data/rgbd_dataset_freiburg1_xyz/");
	std::string filenameBaseOut = std::string("mesh_");

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(filenameIn)) {
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
	
	// Setup the optimizer.
	ICPOptimizer* optimizer = nullptr;
	if (USE_LINEAR_ICP) {
		optimizer = new LinearICPOptimizer();
	}
	else {
		optimizer = new CeresICPOptimizer();
	}

	optimizer->setMatchingMaxDistance(0.1f);
	if (USE_POINT_TO_PLANE) {
		optimizer->usePointToPlaneConstraints(true);
		optimizer->setNbOfIterations(10);
	}
	else {
		optimizer->usePointToPlaneConstraints(false);
		optimizer->setNbOfIterations(20);
	}

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());

	int i = 0;
	const int iMax = 22;
	while (sensor.processNextFrame() && i <= iMax) {
		float* depthMap = sensor.getDepth();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8 };
		optimizer->estimatePose(source, target, currentCameraToWorld);
		
		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		if (i % 5 == 0) {
			// We write out the mesh to file for debugging.
			SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.1f };
			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

			std::stringstream ss;
			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
			std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
			if (!resultingMesh.writeMesh(ss.str())) {
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			}
		}
		
		i++;
	}

	delete optimizer;

	return 0;
}

int main() {


	FaceModel* model = FaceModel::getInstance();

	// std::vector<Vector3f> targetPoints = data->key_vectors;
	 std::vector<Eigen::Vector3f> targetPoints = {
        {137., 240., -85.92155},
        {140., 264., -81.15781},
        {143., 288., -76.27054},
        {146., 306., -69.03136},
        {152., 327., -53.7936},
        {161., 342., -30.054216},
        {170., 348., -2.8260052},
        {185., 354., 23.479996},
        {212., 360., 38.621555},
        {239., 357., 31.712843},
        {263., 354., 12.164688},
        {284., 348., -10.076692},
        {302., 333., -29.442987},
        {314., 315., -41.688686},
        {320., 297., -46.937668},
        {326., 276., -50.34531},
        {335., 252., -53.961445},
        {152., 207., -7.665689},
        {164., 201., 6.13177},
        {176., 198., 16.93147},
        {188., 198., 24.63061},
        {200., 201., 29.189041},
        {245., 204., 37.821613},
        {257., 201., 37.362846},
        {269., 201., 34.105515},
        {284., 204., 28.425138},
        {299., 216., 18.271336},
        {221., 225., 37.87167},
        {218., 237., 48.276306},
        {215., 249., 60.441814},
        {215., 261., 63.294724},
        {203., 273., 40.13742},
        {209., 276., 45.007755},
        {218., 276., 48.519512},
        {227., 276., 47.70018},
        {233., 276., 44.971146},
        {170., 228., 7.1205516},
        {179., 222., 17.116602},
        {188., 222., 19.72416},
        {200., 228., 19.013172},
        {191., 231., 20.587048},
        {179., 231., 16.077826},
        {248., 231., 28.51834},
        {257., 225., 32.973515},
        {269., 225., 34.33488},
        {278., 231., 26.970598},
        {269., 234., 32.82133},
        {257., 234., 33.29259},
        {185., 306., 29.884432},
        {194., 297., 42.56733},
        {209., 291., 50.5199},
        {215., 291., 52.790028},
        {221., 291., 52.883045},
        {236., 300., 48.298386},
        {248., 309., 38.222115},
        {236., 312., 48.35514},
        {224., 315., 52.60904},
        {212., 315., 52.29711},
        {203., 315., 49.51622},
        {194., 309., 42.607273},
        {188., 303., 30.705593},
        {206., 300., 46.46957},
        {215., 300., 49.569782},
        {224., 300., 49.022953},
        {248., 309., 38.071213},
        {224., 303., 49.785515},
        {215., 303., 49.560917},
        {206., 303., 47.098385}
    };
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

	std::vector<Vector3f> sp_deneme ;
    sp_deneme.push_back(sourcePoints[0]);sp_deneme.push_back(sourcePoints[16]);sp_deneme.push_back(sourcePoints[27]);sp_deneme.push_back(sourcePoints[8]) ;

    std::vector<Vector3f> tp_deneme ;
    tp_deneme.push_back(targetPoints[0]);tp_deneme.push_back(targetPoints[16]);tp_deneme.push_back(targetPoints[27]);tp_deneme.push_back(targetPoints[8]); ;

    ProcrustesAligner aligner;
	Matrix4f estimatedPose = aligner.estimatePose(sp_deneme, tp_deneme);

	std::cout << "balabizo" << std::endl;
	std::cout << estimatedPose << std::endl;

	return 0;
}
