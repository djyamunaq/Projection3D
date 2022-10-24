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

#define WHITE cv::Scalar(255, 255, 255)
#define BLACK cv::Scalar(0, 0, 0)
#define GREEN cv::Scalar(255, 255, 0)
#define BLUE cv::Scalar(255, 0, 0)
#define RED cv::Scalar(0, 0, 255)

namespace Projection {
    /*
     * Projection from 3D space in 2D space
     */
    class Proj3D {
        public:
            Proj3D (double f);
            Proj3D ();
            ~Proj3D ();
            /* Set intrinsical parameters */
            void setFocalDistance(double f);
            double getFocalDistance();
            Eigen::Vector2d calculateProjection(Eigen::Vector3d coordinates);
            Eigen::Matrix<double, Eigen::Dynamic, 2> calculateProjection(Eigen::Matrix<double, Eigen::Dynamic, 3> coordinates);
        private:
            /* focal distance */
            double f{0};
    };
}
