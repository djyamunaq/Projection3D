#include <eigen3/Eigen/Dense>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <vector>
#include "Projection.h"

#define WHITE cv::Scalar(255, 255, 255)
#define BLACK cv::Scalar(0, 0, 0)
#define GREEN cv::Scalar(255, 255, 0)
#define BLUE cv::Scalar(255, 0, 0)
#define RED cv::Scalar(0, 0, 255)

int main() {
    /* Parameters */
        /* focal distance */
    double f = 100;
        /* cube size */
    double l = 50;
        /* Cube distance from origin */
    double z = 50;

    Projection::Proj3D projector(f);

    /* Cube base coordinates */
    Eigen::Matrix<double, 8, 3> cubeCoorners;
    cubeCoorners <<     -l/2, l/2, -l/2,
                        l/2, l/2, -l/2,
                        -l/2, -l/2, -l/2,
                        l/2, -l/2, -l/2,
                        -l/2, -l/2, l/2,
                        l/2, -l/2, l/2,
                        -l/2, l/2, l/2,
                        l/2, l/2, l/2;

    /* Angles in degrees */
    double thetaZ = 0;
    double thetaY = 0;
    double thetaX = 0;
    /* Quaternion to represent rotation (Angles in radians) */
    Eigen::AngleAxisd angleAxisX = Eigen::AngleAxisd(thetaX*(M_PI/180), Eigen::Vector3d::UnitX());     
    Eigen::AngleAxisd angleAxisY = Eigen::AngleAxisd(thetaY*(M_PI/180), Eigen::Vector3d::UnitY());     
    Eigen::AngleAxisd angleAxisZ = Eigen::AngleAxisd(thetaZ*(M_PI/180), Eigen::Vector3d::UnitZ());     
    Eigen::Quaterniond Q = angleAxisX*angleAxisY*angleAxisZ;
    /* Transformation Matrix */
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    T.block<3, 3>(0, 0) = Q.toRotationMatrix();
    T(2,3) = z + l/2;
    T(3,3) = 1;
    /* Calculate transformed coordinates for cube */
        /* Transpose cube coordinates */
    Eigen::Matrix<double, 4, 8> cubeCoornersTransf = Eigen::Matrix<double, 4, 8>::Ones();
    cubeCoornersTransf.block<3, 8>(0, 0) = cubeCoorners.transpose();
        /* Multiply cube coordinates by transformation matrix */
    cubeCoornersTransf = T*cubeCoornersTransf;

    double w = 600;
    double h = 400;
    cv::Mat img;

    while(true) {
        char key = (char) cv::waitKey(50);

        if(key == 'w') {
            thetaX++;
        } else if(key == 's') {
            thetaX--;
        } else if(key == 'd') {
            thetaY++;
        } else if(key == 'a') {
            thetaY--;
        } else if(key == 'k') {
            thetaZ++;
        } else if(key == 'j') {
            thetaZ--;
        } else if(key == '+') {
            double curr_f = projector.getFocalDistance(); 
            projector.setFocalDistance(++curr_f);
        } else if(key == '-') {
            double curr_f = projector.getFocalDistance(); 
            projector.setFocalDistance(--curr_f);
        }

        /* Quaternion to represent rotation (Angles in radians) */
        angleAxisX = Eigen::AngleAxisd(thetaX*(M_PI/180), Eigen::Vector3d::UnitX());     
        angleAxisY = Eigen::AngleAxisd(thetaY*(M_PI/180), Eigen::Vector3d::UnitY());     
        angleAxisZ = Eigen::AngleAxisd(thetaZ*(M_PI/180), Eigen::Vector3d::UnitZ());     
        Q = angleAxisX*angleAxisY*angleAxisZ;
        /* Transformation Matrix */
        T = Eigen::Matrix4d::Zero();
        T.block<3, 3>(0, 0) = Q.toRotationMatrix();
        T(2,3) = z + l/2;
        T(3,3) = 1;

        /* Calculate transformed coordinates for cube */
            /* Transpose cube coordinates */
        cubeCoornersTransf = Eigen::Matrix<double, 4, 8>::Ones();
        cubeCoornersTransf.block<3, 8>(0, 0) = cubeCoorners.transpose();
            /* Multiply cube coordinates by transformation matrix */
        cubeCoornersTransf = T*cubeCoornersTransf;

        img = cv::Mat::zeros(h, w, CV_8UC3);

        /* Projection of cube corners in plane */
        std::vector<Eigen::Vector2d> vs;
        for(int i=0; i<8; i++) {
            /* Cube corner vector from origin */
            Eigen::Vector3d X = cubeCoornersTransf.transpose().block<1,3>(i, 0);
            
            double Ox = w/2;
            double Oy = h/2;

            Eigen::Vector2d translation2D = {Ox, Oy};
            Eigen::Vector2d projection = projector.calculateProjection(X);
            projection = projection + translation2D;

            vs.push_back(projection);
        }

        /* Projection of cube edges in plane */
            /* Edge 01 */
        cv::line(img, cv::Point2d(vs[0](0), vs[0](1)), cv::Point2d(vs[1](0), vs[1](1)), RED, 1);
            /* Edge 02 */
        cv::line(img, cv::Point2d(vs[0](0), vs[0](1)), cv::Point2d(vs[2](0), vs[2](1)), RED, 1);
            /* Edge 06 */
        cv::line(img, cv::Point2d(vs[0](0), vs[0](1)), cv::Point2d(vs[6](0), vs[6](1)), GREEN, 1);
            /* Edge 13 */        
        cv::line(img, cv::Point2d(vs[1](0), vs[1](1)), cv::Point2d(vs[3](0), vs[3](1)), RED, 1);
            /* Edge 17 */
        cv::line(img, cv::Point2d(vs[1](0), vs[1](1)), cv::Point2d(vs[7](0), vs[7](1)), RED, 1);
            /* Edge 23 */
        cv::line(img, cv::Point2d(vs[2](0), vs[2](1)), cv::Point2d(vs[3](0), vs[3](1)), RED, 1);
            /* Edge 24 */
        cv::line(img, cv::Point2d(vs[2](0), vs[2](1)), cv::Point2d(vs[4](0), vs[4](1)), RED, 1);
            /* Edge 35 */
        cv::line(img, cv::Point2d(vs[3](0), vs[3](1)), cv::Point2d(vs[5](0), vs[5](1)), RED, 1);
            /* Edge 45 */
        cv::line(img, cv::Point2d(vs[4](0), vs[4](1)), cv::Point2d(vs[5](0), vs[5](1)), RED, 1);
            /* Edge 46 */
        cv::line(img, cv::Point2d(vs[4](0), vs[4](1)), cv::Point2d(vs[6](0), vs[6](1)), GREEN, 1);
            /* Edge 67 */
        cv::line(img, cv::Point2d(vs[5](0), vs[5](1)), cv::Point2d(vs[7](0), vs[7](1)), RED, 1);
            /* Edge 67 */
        cv::line(img, cv::Point2d(vs[6](0), vs[6](1)), cv::Point2d(vs[7](0), vs[7](1)), GREEN, 1);

        cv::imshow("3D Projection into 2D", img);
    }
    
    return 0;
}