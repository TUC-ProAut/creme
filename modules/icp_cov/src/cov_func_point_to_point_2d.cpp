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

#include <iostream>
#include <sstream>
#include <fstream>
#include "cov_func_point_to_point_2d.h"

void calculate_ICP_COV_2d(pcl::PointCloud<pcl::PointXYZ>& data_pi, pcl::PointCloud<pcl::PointXYZ>& model_qi, Eigen::Matrix4d& transform, Eigen::MatrixXd& ICP_COV, std::vector<Eigen::MatrixXd>& covz_pi, std::vector<Eigen::MatrixXd>& covz_qi)
{

    double Tx = transform(0,3);
    double Ty = transform(1,3);
    double yaw   = atan2f(transform(1,0), transform(0,0));
    double x, y, a;
    x = Tx; y = Ty;
    a = yaw;
    // Matrix initialization
    Eigen::MatrixXd d2J_dX2(3,3);
    d2J_dX2 = Eigen::MatrixXd::Zero(3,3);

    /****  Calculating d2J_dX2  ****/
    for (size_t s = 0; s < data_pi.points.size(); ++s ) // this assumes that both point set are equal 
    {
        double pix = data_pi.points[s].x;
        double piy = data_pi.points[s].y;

        double qix = model_qi.points[s].x;
        double qiy = model_qi.points[s].y;

        /************************************************************

        d2J_dX2 -- X is the [R|T] in the form of [x, y, a]
        x, y      is the translation part 
        a         is the rotation part in Euler format
        [x, y, a] is acquired from the Transformation Matrix returned by ICP.

        Now d2J_dX2 is a 3x3 matrix of the form

        d2J_dx2     d2J_dydx	d2J_dadx
        d2J_dxdy    d2J_dy2	    d2J_dady
        d2J_dxda    d2J_dyda    d2J_da2

        *************************************************************/

        double  d2J_dx2,     d2J_dydx,	  d2J_dadx,
                d2J_dxdy,    d2J_dy2,	  d2J_dady,
                d2J_dxda,    d2J_dyda,    d2J_da2;

        // These terms are generated from the provided Matlab scipts. We just have to copy
        // the expressions from the matlab output with two very simple changes.
        // The first one being the the sqaure of a number 'a' is shown as a^2 in matlab,
        // which is converted to pow(a,2) in the below expressions.
        // The second change is to add ';' at the end of each expression :)
        // In this way, matlab can be used to generate these terms for various objective functions of ICP
        // and they can simply be copied to the C++ files and with appropriate changes to ICP estimation,
        // its covariance can be easily estimated.

        //  first col
        d2J_dx2 = 2;

        d2J_dxdy = 0;

        d2J_dxda = - 2*piy*cos(a) - 2*pix*sin(a);

        // second col
        d2J_dydx = 0;
        
        d2J_dy2 = 2;

        d2J_dyda = 2*pix*cos(a) - 2*piy*sin(a);

        d2J_dadx = - 2*piy*cos(a) - 2*pix*sin(a);

        d2J_dady = 2*pix*cos(a) - 2*piy*sin(a) ;

        d2J_da2 = (piy*cos(a) + pix*sin(a))*(2*piy*cos(a) + 2*pix*sin(a)) + (pix*cos(a) - piy*sin(a))*(2*pix*cos(a) - 2*piy*sin(a)) + (2*pix*cos(a) - 2*piy*sin(a))*(qix - x - pix*cos(a) + piy*sin(a)) - (2*piy*cos(a) + 2*pix*sin(a))*(y - qiy + piy*cos(a) + pix*sin(a));

        // 2D 
        Eigen::MatrixXd d2J_dX2_temp(3,3);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dadx,
                        d2J_dxdy,    d2J_dy2,	  d2J_dady,
                        d2J_dxda,    d2J_dyda,    d2J_da2;


        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }   // End of the FOR loop!!!

    // n is the number of correspondences
    int n = data_pi.points.size();
    
    if (n > 200) n = 200; 

    Eigen::MatrixXd d2J_dZdX(3,4*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        double pix = data_pi.points[k].x;
        double piy = data_pi.points[k].y;

        double qix = model_qi.points[k].x;
        double qiy = model_qi.points[k].y;

       Eigen::MatrixXd d2J_dZdX_temp(3,4);

       double d2J_dpix_dx, d2J_dpiy_dx, d2J_dqix_dx, d2J_dqiy_dx,
              d2J_dpix_dy, d2J_dpiy_dy, d2J_dqix_dy, d2J_dqiy_dy,
              d2J_dpix_da, d2J_dpiy_da, d2J_dqix_da, d2J_dqiy_da;

        d2J_dpix_dx = 2*cos(a);

        d2J_dpix_dy = 2*sin(a);

        d2J_dpix_da =2*cos(a)*(y - qiy + piy*cos(a) + pix*sin(a)) + 2*sin(a)*(qix - x - pix*cos(a) + piy*sin(a)) - cos(a)*(2*piy*cos(a) + 2*pix*sin(a)) + sin(a)*(2*pix*cos(a) - 2*piy*sin(a));

        d2J_dpiy_dx = -2*sin(a);

        d2J_dpiy_dy = 2*cos(a);


        d2J_dpiy_da = 2*cos(a)*(qix - x - pix*cos(a) + piy*sin(a)) - 2*sin(a)*(y - qiy + piy*cos(a) + pix*sin(a)) + cos(a)*(2*pix*cos(a) - 2*piy*sin(a)) + sin(a)*(2*piy*cos(a) + 2*pix*sin(a));

        d2J_dqix_dx = -2;

        d2J_dqix_dy = 0;

        d2J_dqix_da = 2*piy*cos(a) + 2*pix*sin(a) ;

        d2J_dqiy_dx = 0;

        d2J_dqiy_dy = -2;

        d2J_dqiy_da = 2*piy*sin(a) - 2*pix*cos(a);


        d2J_dZdX_temp << d2J_dpix_dx, d2J_dpiy_dx, d2J_dqix_dx, d2J_dqiy_dx,
                         d2J_dpix_dy, d2J_dpiy_dy, d2J_dqix_dy, d2J_dqiy_dy,
                         d2J_dpix_da, d2J_dpiy_da, d2J_dqix_da, d2J_dqiy_da;


        d2J_dZdX.block<3,4>(0,4*k) = d2J_dZdX_temp;

    }

    /************************************************************
     *
     * Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper:
     * Prakhya, S.M. et al. (2015) ‘A closed-form estimate of 3D ICP covariance’, 
     * in Proc. of Intl. Conf. on Machine Vision Applications (MVA), 
     * pp. 526–529. doi:10.1109/MVA.2015.7153246.
     *
     ************************************************************/

    Eigen::MatrixXd cov_z(4*n,4*n); 
    cov_z =  Eigen::MatrixXd::Identity(4*n,4*n); //  Initialization is quit important 

    for (int k = 0; k < data_pi.points.size(); k++) 
    {

        cov_z.block<2,2>(k*4,k*4) = covz_pi[k];
        cov_z.block<2,2>(k*4+2,k*4+2) = covz_qi[k];

    }

    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();

    // std::cout << "ICP_COV = \n" << ICP_COV << std::endl;
}
