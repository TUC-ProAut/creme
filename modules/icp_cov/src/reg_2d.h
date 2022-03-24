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
 * @file reg_2d.h
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief 2D ICP registration.
 */

#include <iostream>
#include <sstream>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// for 2d registration 
#include <pcl/registration/transformation_estimation_2D.h>

// for 2d covariance 
#include "cov_func_point_to_point_2d.h"

// for Correspondences 
#include <pcl/registration/correspondence_estimation.h>


/**
 * @brief This function calculates the transformation matrix.
 * @param[in] cloud_in 
 * @param[in] cloud_out 
 * @param[in] final_transformation 
 * @param[in] all_correspondences_out 
 * @param[out] nr_iterations 
 * @param[out] ReciprocalCorrespondences 
 */
void calculate_reg_2d(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out, Eigen::Matrix4d& final_transformation, pcl::Correspondences &all_correspondences_out, int&  nr_iterations, bool ReciprocalCorrespondences);