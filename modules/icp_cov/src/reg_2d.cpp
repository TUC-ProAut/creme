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
 * @file reg_2d.cpp
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief 2D ICP registration.
 */

#include <iostream>
#include <sstream>
#include "reg_2d.h"
#include <fstream>

using namespace std;

void calculate_reg_2d(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out, Eigen::Matrix4d& final_transformation, pcl::Correspondences &all_correspondences_out, int&  nr_iterations, bool ReciprocalCorrespondences = false) 

{   
    //********  apply initial guess to one pointcould, cld1 ********

     pcl::transformPointCloud(cloud_in, cloud_in, final_transformation);

    //******** ICP, Transformation Estimation ******** 

    pcl::PointCloud<pcl::PointXYZ> cld1, cld2;      // define two new point cloud variables 
    pcl::copyPointCloud(cloud_in, cld1);            // copy point cloud to new varriable  
    pcl::copyPointCloud(cloud_out,cld2);            // copy point cloud to new varriable 
    
    Eigen::Matrix4d transform_eigen;                // this like a temp for the transformation matrix

    Eigen::Matrix4d convergence_criteria;           // 
    
    // counting how many iterations to estimate the Tr
    nr_iterations = 0; 

    do
    {
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cld1_Ptr (new pcl::PointCloud<pcl::PointXYZ> (cld1));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cld2_Ptr (new pcl::PointCloud<pcl::PointXYZ> (cld2));

        est.setInputSource (cld1_Ptr);
        est.setInputTarget (cld2_Ptr);

        if (ReciprocalCorrespondences){
            est.determineReciprocalCorrespondences(all_correspondences_out);    // determine Reciprocal Correspondences
        }
        else{
            est.determineCorrespondences(all_correspondences_out);           // determine normal Correspondences
        }
        
        

        pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, double> est2D;
        // est2D.estimateRigidTransformation (cld1, cld2 , all_correspondences_out,final_transformation);

        est2D.estimateRigidTransformation (cld1, cld2 , all_correspondences_out,transform_eigen);

        // rotate/transform data based on this estimated transformation
        pcl::transformPointCloud(cld1, cld1, transform_eigen);

        // check condition 
        // check if the estimated transformation is almost identity 
        convergence_criteria = Eigen::Matrix4d :: Identity () - transform_eigen;
        nr_iterations = nr_iterations + 1;
        // std::cout << " nr_iterations: \n " << nr_iterations << std::endl;

        // update the transformation 
        final_transformation = transform_eigen * final_transformation;
        
    }while( (convergence_criteria.norm() > 0.0000005) && (nr_iterations < 50) ); 
    
}