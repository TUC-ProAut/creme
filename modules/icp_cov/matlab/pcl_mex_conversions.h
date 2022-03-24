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
 * @file pcl_mex_conversions.h
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief TODO.
 */

#pragma once

/**
 *  @brief This function takes input point and return it in point cloud 
 *  the complete code you can find it in : 
 *  https://github.com/jandob/pcl_mex
 *
 */

/**
 * @brief This function takes input mxArray, then returns point cloud. 
 * 
 * @param[in] mxIn mArray points
 * @return cloud point cloud 
 */
void mxArrayToPointCloud(const mxArray *mxIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/**
 * @brief This function takes point cloud, then returns mArray.
 * 
 * @param[in] cloud, point cloud 
 * @return mxArray* 
 */

mxArray* pointCloudToMxArray(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
