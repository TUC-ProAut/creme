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
 * @file cov_func_point_to_point_2d.h
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief ICP covariance calculation.
 */

#include <cmath>
#include <math.h>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

/**
 * @brief This function optimized the covariance matrix,
 *         according to the optimized value, output from reg_2d().
 * @param[in] data_pi  The corresponding points in the data_cloud or source cloud or Pi.
 * @param[in] model_qi The correspondences in the model or target or Qi
 * @param[in] transform The transformation matrix as returned by ICP
 * @param[in] covz_qi covariance for source cloud.
 * @param[in] covz_pi covariance for target cloud.
 * @return 6x6 matrix which is the COVARIANCE of ICP in [x,y,z,a,b,c], [a b c] are Yaw, Pitch and Roll respectively.
*/

void calculate_ICP_COV_2d(pcl::PointCloud<pcl::PointXYZ>& data_pi, pcl::PointCloud<pcl::PointXYZ>& model_qi, Eigen::Matrix4d& transform, Eigen::MatrixXd& ICP_COV, std::vector<Eigen::MatrixXd>& covz_qi, std::vector<Eigen::MatrixXd>& covz_pi);