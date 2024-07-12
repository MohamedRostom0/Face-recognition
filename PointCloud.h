#pragma once
#include "SimpleMesh.h"
#include "Eigen.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

class PointCloud {
public:
	PointCloud() {}

	PointCloud(const SimpleMesh& mesh) {
		const auto& vertices = mesh.getVertices();
		const auto& triangles = mesh.getTriangles();
		const unsigned nVertices = vertices.size();
		const unsigned nTriangles = triangles.size();

		// Copy vertices.
		m_points.reserve(nVertices);
		for (const auto& vertex : vertices) {
			m_points.push_back(Vector3f{ vertex.position.x(), vertex.position.y(), vertex.position.z() });
		}

		// Compute normals (as an average of triangle normals).
		m_normals = std::vector<Vector3f>(nVertices, Vector3f::Zero());
		for (size_t i = 0; i < nTriangles; i++) {
			const auto& triangle = triangles[i];
			Vector3f faceNormal = (m_points[triangle.idx1] - m_points[triangle.idx0]).cross(m_points[triangle.idx2] - m_points[triangle.idx0]);

			m_normals[triangle.idx0] += faceNormal;
			m_normals[triangle.idx1] += faceNormal;
			m_normals[triangle.idx2] += faceNormal;
		}
		for (size_t i = 0; i < nVertices; i++) {
			m_normals[i].normalize();
		}
	}

	// PointCloud(float* depthMap, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const unsigned width, const unsigned height, unsigned downsampleFactor = 1, float maxDistance = 0.1f) {
	// 	// Get depth intrinsics.
	// 	float fovX = depthIntrinsics(0, 0);
	// 	float fovY = depthIntrinsics(1, 1);
	// 	float cX = depthIntrinsics(0, 2);
	// 	float cY = depthIntrinsics(1, 2);
	// 	const float maxDistanceHalved = maxDistance / 2.f;

	// 	// Compute inverse depth extrinsics.
	// 	Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
	// 	Matrix3f rotationInv = depthExtrinsicsInv.block(0, 0, 3, 3);
	// 	Vector3f translationInv = depthExtrinsicsInv.block(0, 3, 3, 1);

	// 	// Back-project the pixel depths into the camera space.
	// 	std::vector<Vector3f> pointsTmp(width * height);

	// 	// For every pixel row.
	// 	#pragma omp parallel for
	// 	for (int v = 0; v < height; ++v) {
	// 		// For every pixel in a row.
	// 		for (int u = 0; u < width; ++u) {
	// 			unsigned int idx = v*width + u; // linearized index
	// 			float depth = depthMap[idx];
	// 			if (depth == MINF) {
	// 				pointsTmp[idx] = Vector3f(MINF, MINF, MINF);
	// 			}
	// 			else {
	// 				// Back-projection to camera space.
	// 				pointsTmp[idx] = rotationInv * Vector3f((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth) + translationInv;
	// 			}
	// 		}
	// 	}

	// 	// We need to compute derivatives and then the normalized normal vector (for valid pixels).
	// 	std::vector<Vector3f> normalsTmp(width * height);

	// 	#pragma omp parallel for
	// 	for (int v = 1; v < height - 1; ++v) {
	// 		for (int u = 1; u < width - 1; ++u) {
	// 			unsigned int idx = v*width + u; // linearized index

	// 			const float du = 0.5f * (depthMap[idx + 1] - depthMap[idx - 1]);
	// 			const float dv = 0.5f * (depthMap[idx + width] - depthMap[idx - width]);
	// 			if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved) {
	// 				normalsTmp[idx] = Vector3f(MINF, MINF, MINF);
	// 				continue;
	// 			}

	// 			// TODO: Compute the normals using central differences. 
	// 			// https://stackoverflow.com/questions/34644101/calculate-surface-normals-from-depth-image-using-neighboring-pixels-cross-produc
	// 			// nice explanation! (-dz/dx, -dz/dy, 1)
	// 			// TODO have to take in consideration also the field of view!
	// 			Eigen::Vector3f normalVector(du, -dv, 1.0f);
	// 			normalVector.normalize();
	// 			normalsTmp[idx] = normalVector;
	// 		}
	// 	}

	// 	// We set invalid normals for border regions.
	// 	for (int u = 0; u < width; ++u) {
	// 		normalsTmp[u] = Vector3f(MINF, MINF, MINF);
	// 		normalsTmp[u + (height - 1) * width] = Vector3f(MINF, MINF, MINF);
	// 	}
	// 	for (int v = 0; v < height; ++v) {
	// 		normalsTmp[v * width] = Vector3f(MINF, MINF, MINF);
	// 		normalsTmp[(width - 1) + v * width] = Vector3f(MINF, MINF, MINF);
	// 	}

	// 	// We filter out measurements where either point or normal is invalid.
	// 	const unsigned nPoints = pointsTmp.size();
	// 	m_points.reserve(std::floor(float(nPoints) / downsampleFactor));
	// 	m_normals.reserve(std::floor(float(nPoints) / downsampleFactor));

	// 	for (int i = 0; i < nPoints; i = i + downsampleFactor) {
	// 		const auto& point = pointsTmp[i];
	// 		const auto& normal = normalsTmp[i];

	// 		if (point.allFinite() && normal.allFinite()) {
	// 			m_points.push_back(point);
	// 			m_normals.push_back(normal);
	// 		}
	// 	}
	// }

	// New constructor to accept pcl::PointCloud
    // PointCloud(const pcl::PointCloud<pcl::PointXYZ>& pclCloud) {
    //     m_points.reserve(pclCloud.size());
    //     for (const auto& point : pclCloud) {
    //         m_points.push_back(Eigen::Vector3f(point.x, point.y, point.z));
    //     }

    //     // Since normals are not part of pcl::PointXYZ, we might set them to zero or calculate them if necessary.
    //     m_normals = std::vector<Vector3f>(pclCloud.size(), Eigen::Vector3f::Zero());
    //     // Compute normals if needed here
    // }

	PointCloud(const pcl::PointCloud<pcl::PointXYZ>& pclCloud) {
    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(pclCloud.makeShared());

    // Create a NormalEstimation object and set the input cloud and search method
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pclCloud.makeShared());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*normals);

    // Combine points and normals
    m_points.reserve(pclCloud.size());
    m_normals.reserve(pclCloud.size());
    for (size_t i = 0; i < pclCloud.size(); ++i) {
        const auto& point = pclCloud.points[i];
        const auto& normal = normals->points[i];

        m_points.push_back(Eigen::Vector3f(point.x, point.y, point.z));
        m_normals.push_back(Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z));
    }
}

	// New constructor to accept pcl::PointCloud with normals
	PointCloud(const pcl::PointCloud<pcl::PointNormal>& pclCloud) {
		m_points.reserve(pclCloud.size());
		m_normals.reserve(pclCloud.size());
		for (const auto& point : pclCloud) {
			m_points.push_back(Eigen::Vector3f(point.x, point.y, point.z));
			m_normals.push_back(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z));
		}
	}

	PointCloud(float* depthMap, std::vector<Vector3f> pointsTmp,const unsigned width, const unsigned height, unsigned downsampleFactor = 1, float maxDistance = 0.1f) {
        const float maxDistanceHalved = maxDistance / 2.f;
        std::vector<Vector3f> normalsTmp(width * height);

#pragma omp parallel for
        for (int v = 1; v < height - 1; ++v) {
            for (int u = 1; u < width - 1; ++u) {
                unsigned int idx = v * width + u; // linearized index

                const float du = 0.5f * (depthMap[idx + 1] - depthMap[idx - 1]);
                const float dv = 0.5f * (depthMap[idx + width] - depthMap[idx - width]);
                if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved) {
                    normalsTmp[idx] = Vector3f(MINF, MINF, MINF);
                    continue;
                }

                // TODO: Compute the normals using central differences. 
                normalsTmp[idx] = -(pointsTmp[idx+1] - pointsTmp[idx-1]).cross(pointsTmp[idx + width] - pointsTmp[idx - width]);
                normalsTmp[idx].normalize();
            }
        }

        // We set invalid normals for border regions.
        for (int u = 0; u < width; ++u) {
            normalsTmp[u] = Vector3f(MINF, MINF, MINF);
            normalsTmp[u + (height - 1) * width] = Vector3f(MINF, MINF, MINF);
        }
        for (int v = 0; v < height; ++v) {
            normalsTmp[v * width] = Vector3f(MINF, MINF, MINF);
            normalsTmp[(width - 1) + v * width] = Vector3f(MINF, MINF, MINF);
        }

        // We filter out measurements where either point or normal is invalid.
        const unsigned nPoints = pointsTmp.size();
        m_points.reserve(std::floor(float(nPoints) / downsampleFactor));
        m_normals.reserve(std::floor(float(nPoints) / downsampleFactor));
        // rgb.reserve(std::floor(float(nPoints) / downsampleFactor));

        for (int i = 0; i < nPoints; i = i + downsampleFactor) {
            const auto& point = pointsTmp[i];
            const auto& normal = normalsTmp[i];

            if (point.allFinite() && normal.allFinite()) {
                // rgb.push_back(rgb_[i]);
                m_points.push_back(point);
                m_normals.push_back(normal);
            }
        }
    }

	bool readFromFile(const std::string& filename) {
		std::ifstream is(filename, std::ios::in | std::ios::binary);
		if (!is.is_open()) {
			std::cout << "ERROR: unable to read input file!" << std::endl;
			return false;
		}
		std::cout << "here 1" << std::endl;
		char nBytes;
		is.read(&nBytes, sizeof(char));

		unsigned int n;
		is.read((char*)&n, sizeof(unsigned int));

		std::cout << "here 2" << std::endl;
		if (nBytes == sizeof(float)) {
			float* ps = new float[3 * n];

			is.read((char*)ps, 3 * sizeof(float) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(float) * n);
			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}
		else {
			double* ps = new double[3 * n];

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_points.push_back(p);
			}

			is.read((char*)ps, 3 * sizeof(double) * n);

			for (unsigned int i = 0; i < n; i++) {
				Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
				m_normals.push_back(p);
			}

			delete ps;
		}


		//std::ofstream file("pointcloud.off");
		//file << "OFF" << std::endl;
		//file << m_points.size() << " 0 0" << std::endl;
		//for(unsigned int i=0; i<m_points.size(); ++i)
		//	file << m_points[i].x() << " " << m_points[i].y() << " " << m_points[i].z() << std::endl;
		//file.close();

		return true;
	}

	std::vector<Vector3f>& getPoints() {
		return m_points;
	}

	const std::vector<Vector3f>& getPoints() const {
		return m_points;
	}

	std::vector<Vector3f>& getNormals() {
		return m_normals;
	}

	const std::vector<Vector3f>& getNormals() const {
		return m_normals;
	}

	unsigned int getClosestPoint(Vector3f& p) {
		unsigned int idx = 0;

		float min_dist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < m_points.size(); ++i) {
			float dist = (p - m_points[i]).norm();
			if (min_dist > dist) {
				idx = i;
				min_dist = dist;
			}
		}

		return idx;
	}

public:
    std::vector<Eigen::Vector3f> rgb;
	
private:
	std::vector<Vector3f> m_points;
	std::vector<Vector3f> m_normals;

};