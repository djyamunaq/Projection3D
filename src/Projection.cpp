#include "Projection.h"

Projection::Proj3D ::Proj3D (double f) {
    this->f = f;
};

Projection::Proj3D ::Proj3D () {};

Projection::Proj3D::~Proj3D() {};

void Projection::Proj3D::setFocalDistance(double f) {
    this->f = f;
}

double Projection::Proj3D::getFocalDistance() {
    return this->f;
}

Eigen::Vector2d Projection::Proj3D::calculateProjection(Eigen::Vector3d coordinates) {
    const double z = coordinates[2];
    const double k0 = this->f/z;

    Eigen::Vector2d p = {k0*coordinates[0], k0*coordinates[1]};

    return p;
}

Eigen::Matrix<double, Eigen::Dynamic, 2> Projection::Proj3D::calculateProjection(Eigen::Matrix<double, Eigen::Dynamic, 3> coordinates) {
    /* Number of coordinates */
    int nrows = coordinates.rows();

    /* Matrix of projected coordinates */
    Eigen::Matrix<double, 1, 2> pCoordinates;
    pCoordinates.resize(nrows, 2);

    for(int i=0; i<nrows; i++) {
        Eigen::Vector3d c = coordinates.row(i);
        Eigen::Vector2d v = this->calculateProjection(c);

        pCoordinates.row(i) = v;
    }

    return pCoordinates;
}