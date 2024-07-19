#include <iostream>
#include <fstream>
#include <vector>
#include "SimpleMesh.h"

#pragma once

#include "Eigen.h"
#include <mutex>

template <typename T>
static inline void dotProduct(double** input1, double* input2, const int dim, T* output) {
    for (int i = 0; i < 107127; i++) {
        double sum = 0;
        for (int j = 0; j < dim; j++) {
            sum += input1[i][j] * input2[j];
        }
        output[i] = T(sum);
    }
}


template <typename T>
static inline void sum_params(double* input1, double* input2, double* input3, T* output) {

    for (int i = 0; i < 107127; i++) {
        output[i] = T(input1[i]) + T(input2[i]) + T(input3[i]);
    }
}


static std::vector<unsigned int> readKeypoints(std::string fileToOpen) {

    std::vector<unsigned int> keypoints;

    std::string line;
    std::ifstream file(fileToOpen);
    std::string value;

    while (getline(file, line)) {

        std::stringstream lineStringStream(line);

        while (std::getline(lineStringStream, value, ','))
        {
            keypoints.push_back(stoi(value));
        }
    }
    return keypoints;
}


static std::vector<Triangle> readTriangle(std::string fileToOpen) {

    std::vector<unsigned int> triangle;
    std::vector<Triangle> m_triangles;

    std::string line;
    std::ifstream file(fileToOpen);
    std::string value;

    while (getline(file, line)) {

        Triangle t;
        std::stringstream lineStringStream(line);

        while (std::getline(lineStringStream, value, ','))
        {
            triangle.push_back(stoi(value));
        }

        t.idx0 = triangle[0];
        t.idx1 = triangle[1];
        t.idx2 = triangle[2];

        m_triangles.push_back(t);

        triangle.clear();
    }

    return m_triangles;
}


// Taken from https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files/blob/master/source_file.cpp
static Eigen::MatrixXd readMatrixCsv(std::string fileToOpen)
{
    std::vector<double> matrixEntries;

    // in this object we store the data from the matrix
    std::ifstream matrixDataFile(fileToOpen);

    // this variable is used to store the row of the matrix that contains commas
    std::string matrixRowString;

    // this variable is used to store the matrix entry;
    std::string matrixEntry;

    // this variable is used to track the number of rows
    int matrixRowNumber = 0;


    while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

        while (std::getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
        }
        matrixRowNumber++; //update the column numbers
    }

    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}


class FaceModel {
protected:
    FaceModel() {

        std::cout << "Data reading..." << std::endl;
        std::string data_path = "../data";
        idBase = readMatrixCsv(data_path + "/idBase.csv");
        expBase = readMatrixCsv(data_path + "/expBase.csv");
        meanshape = readMatrixCsv(data_path + "/meanshape.csv");


        key_points = readKeypoints(data_path + "/kp_inds.csv");
        m_triangles = readTriangle(data_path + "/tri.csv");

        std::cout << "Data reading completed..." << std::endl;

        meanshapeAr = new double[107127];
        expBaseAr = new double*[107127];
        idBaseAr = new double*[107127];

        for( int i = 0; i < 107127; i++) {
            meanshapeAr[i] = meanshape(i);

            expBaseAr[i] = new double[64];
            idBaseAr[i] = new double[80];

            for( int j = 0; j < 64; j++) {
                expBaseAr[i][j] = expBase(i, j);
            }

            for( int j = 0; j < 80; j++) {
                idBaseAr[i][j] = idBase(i, j);
            }
        }

        expCoefAr = new double[64];
        shapeCoefAr = new double[80];

        for( int j = 0; j < 64; j++) {
            expCoefAr[j] = 0.0;
        }

        for( int j = 0; j < 80; j++) {
            shapeCoefAr[j] = 0.0;
        }

        //Parameters for inner steps
        expression = new double[107127];
        shape = new double[107127];
        face = new double[107127];
        face_t = new double[107127];

        shapeCoef = VectorXd::Zero(80);
        expCoef = VectorXd::Zero(64);

        transformation = Matrix4f ::Identity();
        rotation = Matrix3d::Identity(3, 3);
        translation = Vector3d::Zero(3);

        pose = Matrix4d::Identity();
        scale = 0.0;

        createKeyVector();
    }

    ~FaceModel() {
        for( int i = 0; i < 64; i++) {
            delete[] expBaseAr[i];
        }

        for( int i = 0; i < 80; i++) {
            delete[] idBaseAr[i];
        }

        delete[] expBaseAr;
        delete[] idBaseAr;
        delete[] meanshapeAr;

        delete[] expression;
        delete[] shape;
        delete[] face;
        delete[] face_t;

        delete[] shapeCoefAr;
        delete[] expCoefAr;
    }

public:
    FaceModel(FaceModel &other) = delete;

    void operator=(const FaceModel &) = delete;

    static FaceModel *getInstance();

    static double** getIdBaseAr();

    static double** getExpBaseAr();


    void clear() {
        shapeCoef = VectorXd::Zero(80);
        expCoef = VectorXd::Zero(64);

        rotation = MatrixXd::Identity(3, 3);
        translation = VectorXd::Zero(3);
    }

    double* get_mesh() {
        dotProduct(expBaseAr, expCoefAr, 64, expression);
        dotProduct(idBaseAr, shapeCoefAr, 80, shape);
        sum_params(expression, shape, meanshapeAr, face);
        return face;
    }


    Eigen::MatrixXd transform(Eigen::Matrix4d pose, double scale) {
        Eigen::MatrixXd mesh = getAsEigenMatrix(get_mesh());
        mesh *= scale;
        // return mesh * pose.block<3,4>(0,0);
        auto transposed_pose = pose.transpose();
        auto rotation = transposed_pose.block<3,3>(0,0);
        auto translation = transposed_pose.block<1,3>(3,0);

        return (mesh*rotation).rowwise() + translation;
    }

    Eigen::MatrixXd getAsEigenMatrix(double* face) {
        Eigen::MatrixXd result(35709, 3);
        for(int i = 0; i < 35709; i++) {
            for(int j = 0; j < 3; j++) {
                result(i,j) = face[i * 3 + j];
            }
        }
        return result;
    }

    void setVerticesFromSimpleMesh(const SimpleMesh& mesh) {
        std::cout << "inside setVerticies" << std::endl;
        const auto& vertices = mesh.getVertices();
        for (int i = 0; i < vertices.size(); ++i) {
            meanshape(i, 0) = vertices[i].position.x();
            meanshape(i, 1) = vertices[i].position.y();
            meanshape(i, 2) = vertices[i].position.z();
        }
    }

    Eigen::VectorXd getExpressionCoefficients() const {
        return Eigen::VectorXd::Map(expCoefAr, 64);
    }

    void setExpressionCoefficients(const Eigen::VectorXd& newExpCoef) {
        for (int i = 0; i < 64; i++) {
            expCoefAr[i] = newExpCoef[i];
        }
    }

    void write_off(std::string filename) {
        std::cout << "Writing mesh...\n";
        std::ofstream file;

        Eigen::MatrixXd mesh = getAsEigenMatrix(get_mesh());

        file.open(filename.c_str());
        file << "OFF\n";
        file << "35709 70789 0\n";
        for (int i = 0; i < mesh.rows(); i++) {
            file << mesh(i, 0) << " " << mesh(i, 1) << " " << mesh(i, 2) << "\n";
        }
        for ( auto t : m_triangles) {
            file << "3 " << t.idx0 << " " << t.idx1 << " " << t.idx2 << "\n";
        }
    }

    void write_off(std::string filename, Eigen::MatrixXd mesh) {
        std::cout << "Writing mesh...\n";
        std::ofstream file;

        std::cout << mesh.rows() << " " << mesh.cols() << std::endl;

        file.open(filename.c_str());
        file << "OFF\n";
        file << "35709 70789 0\n";
        for (int i = 0; i < mesh.rows(); i++) {
            file << mesh(i, 0) << " " << mesh(i, 1) << " " << mesh(i, 2) << "\n";
        }
        for ( auto t : m_triangles) {
            file << "3 " << t.idx0 << " " << t.idx1 << " " << t.idx2 << "\n";
        }
    }

    void createKeyVector(){
        Eigen::MatrixXd mesh = getAsEigenMatrix(get_mesh()).reshaped<RowMajor>(35709, 3);
        for(int i = 0 ; i < this->key_points.size() ; i++){
            Vector3f a(mesh(this->key_points[i], 0),mesh(this->key_points[i], 1),mesh(this->key_points[i], 2));
            this->key_vectors.push_back(a);
        }
    }

    void write_obj(std::string filename, Eigen::MatrixXd mesh) {
    std::cout << "Writing mesh...\n";
    std::ofstream file;

    std::cout << mesh.rows() << " " << mesh.cols() << std::endl;

    file.open(filename.c_str());
    for (int i = 0; i < mesh.rows(); i++) {
        file << "v "<<mesh(i, 0) << " " << mesh(i, 1) << " " << mesh(i, 2) << "\n";
    }
    for ( auto t : m_triangles) {
        file << "f " << t.idx0 +1<<"//"<<t.idx0 +1 << " " << t.idx1+1 <<"//"<<t.idx1+1 << " " << t.idx2+1<<"//" <<t.idx2+1<< "\n";
    }
}

    void save(const std::string &filename) {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file for saving.");
        }

        // Save idBase
        int rows = idBase.rows();
        int cols = idBase.cols();
        file.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
        file.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
        file.write(reinterpret_cast<const char *>(idBase.data()), rows * cols * sizeof(double));

        // Save expBase
        rows = expBase.rows();
        cols = expBase.cols();
        file.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
        file.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
        file.write(reinterpret_cast<const char *>(expBase.data()), rows * cols * sizeof(double));

        // Save meanshape
        rows = meanshape.rows();
        cols = meanshape.cols();
        file.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
        file.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
        file.write(reinterpret_cast<const char *>(meanshape.data()), rows * cols * sizeof(double));

        // Save key_points
        size_t size = key_points.size();
        file.write(reinterpret_cast<const char *>(&size), sizeof(size));
        file.write(reinterpret_cast<const char *>(key_points.data()), size * sizeof(unsigned int));

        // Save m_triangles
        size = m_triangles.size();
        file.write(reinterpret_cast<const char *>(&size), sizeof(size));
        file.write(reinterpret_cast<const char *>(m_triangles.data()), size * sizeof(Triangle));

        // Save coefficients
        file.write(reinterpret_cast<const char *>(shapeCoefAr), 80 * sizeof(double));
        file.write(reinterpret_cast<const char *>(expCoefAr), 64 * sizeof(double));

        // Save transformation data
        file.write(reinterpret_cast<const char *>(rotation.data()), 9 * sizeof(double));
        file.write(reinterpret_cast<const char *>(translation.data()), 3 * sizeof(double));
        file.write(reinterpret_cast<const char *>(pose.data()), 16 * sizeof(double));
        file.write(reinterpret_cast<const char *>(&scale), sizeof(double));

        file.close();
    }

    void load(const std::string &filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file for loading.");
        }

        // Load idBase
        int rows, cols;
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        idBase.resize(rows, cols);
        file.read(reinterpret_cast<char *>(idBase.data()), rows * cols * sizeof(double));

        // Load expBase
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        expBase.resize(rows, cols);
        file.read(reinterpret_cast<char *>(expBase.data()), rows * cols * sizeof(double));

        // Load meanshape
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        meanshape.resize(rows, cols);
        file.read(reinterpret_cast<char *>(meanshape.data()), rows * cols * sizeof(double));

        // Load key_points
        size_t size;
        file.read(reinterpret_cast<char *>(&size), sizeof(size));
        key_points.resize(size);
        file.read(reinterpret_cast<char *>(key_points.data()), size * sizeof(unsigned int));

        // Load m_triangles
        file.read(reinterpret_cast<char *>(&size), sizeof(size));
        m_triangles.resize(size);
        file.read(reinterpret_cast<char *>(m_triangles.data()), size * sizeof(Triangle));

        // Load coefficients
        file.read(reinterpret_cast<char *>(shapeCoefAr), 80 * sizeof(double));
        file.read(reinterpret_cast<char *>(expCoefAr), 64 * sizeof(double));

        // Load transformation data
        file.read(reinterpret_cast<char *>(rotation.data()), 9 * sizeof(double));
        file.read(reinterpret_cast<char *>(translation.data()), 3 * sizeof(double));
        file.read(reinterpret_cast<char *>(pose.data()), 16 * sizeof(double));
        file.read(reinterpret_cast<char *>(&scale), sizeof(double));

        file.close();
    }


public:
    Eigen::MatrixXd idBase;
    Eigen::MatrixXd expBase;
    std::vector<Triangle> m_triangles;
    Eigen::MatrixXd meanshape;
    std::vector<unsigned int> key_points;
    std::vector<Vector3f> key_vectors;
    std::vector<Eigen::Vector3d> vertices;
    Eigen::VectorXd faces;

    Eigen::VectorXd shapeCoef;
    Eigen::VectorXd expCoef;

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    Eigen::Matrix4f transformation;

    //Store in array for ceres usage
    double** idBaseAr;
    double** expBaseAr;
    double* meanshapeAr;
    double* shapeCoefAr;
    double* expCoefAr;

    //Parameters for inner steps
    double* expression;
    double* shape;
    double* face;
    double* face_t;
    double scale_factor;

    Matrix4d pose;
    double scale;

private:
    static FaceModel* m_pInstance;
    static std::mutex m_mutex;
};


FaceModel* FaceModel::m_pInstance{nullptr};
std::mutex FaceModel::m_mutex;


FaceModel *FaceModel::getInstance()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_pInstance == nullptr)
    {
        m_pInstance = new FaceModel();
    }
    return m_pInstance;
}

double** FaceModel::getIdBaseAr()
{
    return FaceModel::getInstance()->idBaseAr;
}

double** FaceModel::getExpBaseAr()
{
    return FaceModel::getInstance()->expBaseAr;
}