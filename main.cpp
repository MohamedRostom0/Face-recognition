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


#define TRANSFER_EXPRESSIONS	0


void transferExpression(const FaceModel& face1, FaceModel& face2) {
    Eigen::VectorXd expCoef = face1.getExpressionCoefficients();
    face2.setExpressionCoefficients(expCoef);
    face2.write_off("output.off", face2.getAsEigenMatrix(face2.get_mesh()));
}

int main() {

    if(TRANSFER_EXPRESSIONS){
        FaceModel* face1 = FaceModel::getInstance();
        face1->load("face_1.dat");

        FaceModel* face2 = FaceModel::getInstance();
        face2->load("face_2.dat");

        Eigen::VectorXd expCoefFace1 = face1->getExpressionCoefficients();
        face2->setExpressionCoefficients(expCoefFace1);

        // Generate the updated mesh for face2
        double* updatedMeshFace2 = face2->get_mesh();
        Eigen::MatrixXd updatedMesh = face2->getAsEigenMatrix(updatedMeshFace2);

        // Save the updated mesh to an off file
        face2->write_off("face2_updated.off", updatedMesh);

        return 0; 
    }

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
// std::vector<std::pair<int, int>> keypoints_2d = {
//         {544, 344}, {544, 357}, {545, 366}, {547, 375}, {550, 388},
//         {555, 395}, {561, 400}, {567, 405}, {581, 409}, {595, 408},
//         {606, 403}, {614, 398}, {621, 391}, {626, 380}, {628, 371},
//         {631, 360}, {634, 349}, {550, 333}, {555, 332}, {561, 330},
//         {565, 330}, {572, 332}, {593, 333}, {600, 332}, {606, 332},
//         {612, 333}, {618, 336}, {581, 346}, {581, 353}, {579, 360},
//         {579, 366}, {573, 369}, {576, 371}, {581, 372}, {586, 371},
//         {589, 369}, {558, 343}, {561, 341}, {567, 341}, {572, 344},
//         {567, 346}, {562, 346}, {593, 346}, {598, 343}, {604, 344},
//         {609, 346}, {604, 347}, {598, 347}, {564, 380}, {569, 378},
//         {576, 377}, {581, 378}, {586, 378}, {595, 378}, {603, 381},
//         {593, 388}, {587, 391}, {579, 392}, {575, 391}, {570, 388},
//         {565, 380}, {575, 380}, {581, 381}, {587, 381}, {601, 381},
//         {587, 386}, {579, 386}, {575, 386}
//     };
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
            cropped_depth_map[ind] = cloud.at(x, y).z;
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
	
	// optimizer->estimateExpShapeCoeffs(target_cloud);
    CeresICPOptimizer * optimizer = nullptr;
    optimizer = new CeresICPOptimizer();
    
    optimizer->setMatchingMaxDistance(0.000005f);
    optimizer->setNbOfIterations(5);
    optimizer->estimateExpShapeCoeffs(cropped_cloud);

	return 0;
}
