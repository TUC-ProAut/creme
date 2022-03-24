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
 * @file icp2dCov.cpp
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief Mex wrapper for calculate_ICP_COV_2d().
 */

#include <math.h>
#include <matrix.h>
#include <mex.h>

// kahag 
#include "reg_2d.h"
#include "cov_func_point_to_point_2d.h"
#include "pcl_mex_conversions.h"

//#define USEPCLIO 1

#ifdef USEPCLIO
  #include <pcl/io/io.h> 
  #include <pcl/io/pcd_io.h>
#endif


/**
 * @brief This function is a matlab c++ gateway, 
 * where is optimized the covariance matrix in matlab enviroment according to 
 * calculate_ICP_COV_2d() implementation.
 * @param[in] nrhs	Number of input (right-side) arguments, or the size of the prhs array - 6
 * @param[in] prhs	Array of input arguments
 *            - cld1 point cloud 1
 *            - cld2 point cloud 2
 *            - transformationMatrix [4 x 4]
 *            - cov_cld1 covariance
 *            - cov_cld2 covariance
 *            - point_cloud or points flage
 * @return nlhs	Number of output (left-side) arguments, or the size of the plhs array - 1
 * @return plhs	mxArray output pointer, 
 *            - 6x6 matrix which is the COVARIANCE of ICP in [x,y,z,a,b,c], [a b c] are Yaw, Pitch and Roll respectively.
*/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


  // Check number of arguments
  if (nrhs!=6)
    mexErrMsgTxt("5 input parameters required (cld1, cld2, transformation, cov_cld1 , cov_cld2 ,input_data = point_cloud,or, points).");
      
  if (nlhs!=1)
    mexErrMsgTxt("1 output parameter required : est_cov.");

  // read input flag 
  char method[128];
  mxGetString(prhs[5],method,128);
    
  // check method
  if (strcmp(method,"point_cloud") && strcmp(method,"points"))
    mexErrMsgTxt("Input data must be either 'point_cloud' or 'points'.");
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ> ());
    
  // Input point_cloud
  #ifdef USEPCLIO
  if (!strcmp(method,"point_cloud")){
      
     pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(mxArrayToString(prhs[0])), *cld1);
     pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(mxArrayToString(prhs[1])), *cld2);
  }
  #endif
  
  
  // Inputs points (x, y, z) 
  if (!strcmp(method,"points")){
      
    mxArrayToPointCloud(prhs[0], cld1);
    mxArrayToPointCloud(prhs[1], cld2);
  }


    //  read transformation matrix 
    mwSize nTr_cols = mxGetN(prhs[2]);   //  number of component in the transformation matrix
    mwSize nTr_rows = mxGetM(prhs[2]);   //
    
    double *Tr_in_mex =   (double*)mxGetPr(prhs[2]);
    Eigen::Matrix4d Tr_in(nTr_rows, nTr_cols);
      for (int i = 0; i< nTr_rows; i++ ){                   // col 
          for(int j =0; j< nTr_cols; j++){                  // row
            Tr_in(i, j) = Tr_in_mex[i+j*nTr_rows];
         }
     }
    
    //------ read covarriance for the first and second point cloud ------
    
    mxArray *cellElement;
    
    //------ read covarriance for first pointcloud  ------
    
    mwSize cov_z1_size =  mxGetNumberOfElements(prhs[3]); // cell size 
    std::vector<Eigen::MatrixXd>  z1; 
    for(int t=0; t<cov_z1_size; t++) // access each entry in input cell 
    {
                
        cellElement = mxGetCell(prhs[3], t);
        mwSize cellElement_size = mxGetNumberOfElements(cellElement);
        mwSize cellElement_cols = mxGetN(cellElement);
        mwSize cellElement_rows = mxGetM(cellElement);
        
        
        double *p =   (double*)mxGetPr(cellElement);
        // std::cout<<" p "<< *p <<std::endl;
        Eigen::MatrixXd cov_z1_in(cellElement_rows, cellElement_cols);     // 2x2
          for (int i = 0; i< cellElement_rows; i++ ){                      // col 
              for(int j =0; j< cellElement_cols; j++){                     // row
                cov_z1_in(i, j) = p[i+j*cellElement_rows];
             }
         }
    
        // creat a std::vector for the first point cloud 
        z1.push_back(cov_z1_in);
    }
    
    
    //------ read covarriance for second pointcloud  ------
    
    mwSize cov_z2_size =  mxGetNumberOfElements(prhs[4]); // cell size 
    std::vector<Eigen::MatrixXd>  z2; 
    for(int t=0; t<cov_z2_size; t++) // access each entry in input cell 
    {    
        cellElement = mxGetCell(prhs[4], t);
        mwSize cellElement_size = mxGetNumberOfElements(cellElement);
        mwSize cellElement_cols = mxGetN(cellElement);
        mwSize cellElement_rows = mxGetM(cellElement);
        
        double *p =   (double*)mxGetPr(cellElement);
        // std::cout<<" p "<< *p <<std::endl;
        Eigen::MatrixXd cov_z2_in(cellElement_rows, cellElement_cols);     // 2x2
          for (int i = 0; i< cellElement_rows; i++ ) {                     // col
              for(int j =0; j< cellElement_cols; j++) {                    // row
                cov_z2_in(i, j) = p[i+j*cellElement_rows];
             }
         }
        
        // creat a std::vector for the second point cloud 
        z2.push_back(cov_z2_in);
    }
    
    //------ Call Covarriance estimation function  ------
    Eigen::MatrixXd icp_cov(3,3);
    icp_cov = Eigen::MatrixXd::Zero(3,3);
    
    // call estimation function 
    calculate_ICP_COV_2d(*cld1, *cld2 , Tr_in, icp_cov, z1, z2);
    //std::cout << "icp_cov \n" << icp_cov << std::endl;
    
    //------ Mapping output : covarriance matrix  ------    
    mwSize rows_cov = icp_cov.rows();
    mwSize cols_cov = icp_cov.cols();
    mwSize dims_cov[2];
    dims_cov[1] = rows_cov;
    dims_cov[0] = cols_cov;
    
    plhs[0] = mxCreateNumericArray(2, dims_cov, mxDOUBLE_CLASS, mxREAL);   // Create MATLAB array of same size                  
    Eigen::Map<Eigen::MatrixXd> map(mxGetPr(plhs[0]), rows_cov, cols_cov); // Map the array
    map = icp_cov;                                                         // Copy
    
}