/***************************************************************************
 * CREME - Credible Radar Ego-Motion Estimation
 *
 * Copyright (C) 2022 Chair of Automation Technology / TU Chemnitz
 * For more information see https://mytuc.org/creme
 *
 * CREME is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CREME is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file pcl_mex_conversions.cpp
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief point cloud to mxArray interfaces.
 */

#include "mex.h"
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_mex_conversions.h"

void mxArrayToPointCloud(const mxArray *mxIn,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    mwSize nrows = mxGetM(mxIn);
    mwSize ncols = mxGetN(mxIn);

    double *inMatrix;
    inMatrix = mxGetPr(mxIn);

    Eigen::MatrixXd inMatrixEigen(Eigen::Map<Eigen::MatrixXd> (
            inMatrix, nrows, ncols));

    if (ncols != 3) {
        mexErrMsgIdAndTxt("PCLMex:mxArrayToPointCloud:not3Columns", "Pointcloud must be Mx3 matrix.");
    }

    for (size_t i = 0; i < inMatrixEigen.rows(); i++) {
        pcl::PointXYZ temp;
        temp.x = inMatrixEigen(i, 0);
        temp.y = inMatrixEigen(i, 1);
        temp.z = inMatrixEigen(i, 2);
        cloud->push_back(temp);
    }
}
mxArray* pointCloudToMxArray(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    Eigen::MatrixXf outMatrixXf(cloud->getMatrixXfMap());

    Eigen::MatrixXd outMatrixXd = outMatrixXf.cast<double>();

    Eigen::MatrixXd outMatrixXdCleaned = outMatrixXd.block(0, 0,
        outMatrixXd.rows() - 1, outMatrixXd.cols());

    outMatrixXdCleaned.transposeInPlace();

    /* create the output matrix */
    mxArray* mxOut = mxCreateDoubleMatrix((mwSize) outMatrixXdCleaned.rows(),
        (mwSize) outMatrixXdCleaned.cols(), mxREAL);

    /* get a pointer to the real data in the output matrix */
    double *outMatrix = mxGetPr(mxOut);

    Eigen::Map<Eigen::MatrixXd>( outMatrix,
        outMatrixXdCleaned.rows(), outMatrixXdCleaned.cols() ) =
      outMatrixXdCleaned;
    return mxOut;
}
