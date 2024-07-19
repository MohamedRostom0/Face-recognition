
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

        FaceModel* faceModel = FaceModel::getInstance();

        T expression[3];
        T shape[3];
        T pointVertex[3];

        for (int i = 0; i < 3; i++) {
            T sum = T(0.0);
            for (int j = 0; j < 64; j++) {
                sum += faceModel->expBaseAr[inputIndex*3 + i][j] * expCoef[j];
            }
            expression[i] = T(sum); // dot product expression
        }

        for (int i = 0; i < 3; i++) {
            T sum = T(0.0);
            for (int j = 0; j < 80; j++) {
                sum += faceModel->idBaseAr[inputIndex*3 + i][j] * shapeCoef[j];
            }
            shape[i] = T(sum); // dot product shape
        }

        for (int i = 0; i < 3; i++) {
            pointVertex[i] = expression[i] + shape[i] + faceModel->meanshapeAr[inputIndex*3 + i]; // change face params by (exp + shape)
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
            faces[i] = T(sum);
        }
    
        //apply translation
        for (int i = 0; i < 3; i++) {
            faces[i] += T(faceModel->translation(i));
        }

        outputPoint[0] = faces[0];
        outputPoint[1] = faces[1];
        outputPoint[2] = faces[2];

        delete [] faces;
    }

private:
    T* m_arrayExpCoef;
    T* m_arrayShapeCoef;
};

struct ShapeCostFunction
{
	ShapeCostFunction(double weight_)
		: weight{ weight_ }
	{}

	template<typename T>
	bool operator()(T const* shape_weights, T* residuals) const
	{

		for (int i = 0; i < 80; i++) {
			residuals[i] = shape_weights[i] * T(weight);
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
			residuals[j] = exp_weights[j] * T(weight);
		}
		return true;
	}

private:
	const double weight;
};

class PointToPointConstraint {
public:
    PointToPointConstraint(const int sourcePointIndex, const Vector3f& targetPoint, const float weight) :
            m_sourcePointIndex{ sourcePointIndex },
            m_targetPoint{ targetPoint },
            m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const expCoeff, const T* const shapeCoeff, T* residuals) const {
        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<T>(const_cast<T*>(expCoeff), const_cast<T*>(shapeCoeff));
        
        T transformedPoint[3];
        expShapeCoeffIncrement.apply(m_sourcePointIndex, transformedPoint);

        T delta[3];
        delta[0] = transformedPoint[0] - T(m_targetPoint[0]);
        delta[1] = transformedPoint[1] - T(m_targetPoint[1]);
        delta[2] = transformedPoint[2] - T(m_targetPoint[2]);
        
        residuals[0] = T(LAMBDA) * T(m_weight) * delta[0];
        residuals[1] = T(LAMBDA) * T(m_weight) * delta[1];
        residuals[2] = T(LAMBDA) * T(m_weight) * delta[2];

        return true;
    }

    static ceres::CostFunction* create(const int sourcePointIndex, const Vector3f& targetPoint, const float weight) {
        return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 64, 80>(
                new PointToPointConstraint(sourcePointIndex, targetPoint, weight)
        );
    }

protected:
    const int m_sourcePointIndex;
    const Vector3f m_targetPoint;
    const float m_weight;
    const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint {
public:
    PointToPlaneConstraint(const int sourcePointIndex, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
            m_sourcePointIndex{ sourcePointIndex },
            m_targetPoint{ targetPoint },
            m_targetNormal{ targetNormal },
            m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const expCoeff, const T* const shapeCoeff, T* residuals) const {
        auto expShapeCoeffIncrement = ExpShapeCoeffIncrement<T>(const_cast<T*>(expCoeff), const_cast<T*>(shapeCoeff));
        
        T transformedSource[3];
        expShapeCoeffIncrement.apply(m_sourcePointIndex, transformedSource);

        T delta[3];
        delta[0] = transformedSource[0] - T(m_targetPoint[0]);
        delta[1] = transformedSource[1] - T(m_targetPoint[1]);
        delta[2] = transformedSource[2] - T(m_targetPoint[2]);
        
        residuals[0] = (T(LAMBDA) * T(m_weight) * T(m_targetNormal[0]) * delta[0]);
        residuals[0] += (T(LAMBDA) * T(m_weight) * T(m_targetNormal[1]) * delta[1]);
        residuals[0] += (T(LAMBDA) * T(m_weight) * T(m_targetNormal[2]) * delta[2]);

        return true;
    }

    static ceres::CostFunction* create(const int sourcePointIndex, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
        return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 64, 80>(
                new PointToPlaneConstraint(sourcePointIndex, targetPoint, targetNormal, weight)
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

    void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
        const unsigned nPoints = sourceNormals.size();

        for (unsigned i = 0; i < nPoints; i++) {
            Match& match = matches[i];
            if (match.idx >= 0) {
                const auto& sourceNormal = sourceNormals[i];
                const auto& targetNormal = targetNormals[match.idx];

                // TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
                if(acos(sourceNormal.dot(targetNormal) / (sourceNormal.norm() * targetNormal.norm())) * 180.0 / M_PI > 60.0) {
                    match.idx = -1;
                }
            }
        }
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

        faceModel->save("face_1.dat");
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
                        PointToPointConstraint::create(sourcePointIndex, targetPoint, match.weight),
                        nullptr,
                        expShapeCoeffIncrement.getExpCoeff(),
                        expShapeCoeffIncrement.getShapeCoeff()
                );

                const auto& targetNormal = targetNormals[match.idx];
                if (!targetNormal.allFinite())
                    continue;

                problem.AddResidualBlock(
                    PointToPlaneConstraint::create(sourcePointIndex, targetPoint, targetNormal, match.weight),
                    nullptr, 
                    expShapeCoeffIncrement.getExpCoeff(),
                    expShapeCoeffIncrement.getShapeCoeff()
                );
            }
        }

        ceres::CostFunction* shape_cost = new ceres::AutoDiffCostFunction<ShapeCostFunction, 80, 80>(
				new ShapeCostFunction(0.00001)
				);
		problem.AddResidualBlock(shape_cost, nullptr, expShapeCoeffIncrement.getShapeCoeff());

    
        ceres::CostFunction* expression_cost = new ceres::AutoDiffCostFunction<ExpressionCostFunction, 64, 64>(
				new ExpressionCostFunction(0.00005)
				);
		problem.AddResidualBlock(expression_cost, nullptr, expShapeCoeffIncrement.getExpCoeff());

    }
};