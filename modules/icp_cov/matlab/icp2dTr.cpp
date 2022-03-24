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
 * @file icp2dTr.cpp
 * @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief Mex wrapper for calculate_reg_2d().
 */

#include <math.h>
#include <matrix.h>
#include <mex.h>

// kahag 
#include "reg_2d.h"
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
 * @param[in] nrhs	Number of input (right-side) arguments, or the size of the prhs array - 4
 * @param[in] prhs	Array of input arguments
 *            - cld1 point cloud 1
 *            - cld2 point cloud 2
 *            - point_cloud or points flage
 *            - ReciprocalCorrespondences (optional , True)
 * @param[out] nlhs	Number of output (left-side) arguments, or the size of the plhs array - 2
 * @param[out] plhs	mxArray output pointer, 
 *            - 4x4 relative transformation matrix.
 *            - all_correspondences 
*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])

{   
   // check number of arguments
  if (nrhs!=5 && nrhs!=4) 
    mexErrMsgTxt("4 input parameters required (cld1, cld2, initial guess, input_data = point_cloud,or, points, ReciprocalCorrespondences(optional)).");
      
  if (nlhs!=3)
    mexErrMsgTxt("3 output parameter required (Tr, correspondences, iterations).");

  // read input flag 
  char method[128];
  mxGetString(prhs[3],method,128);
    
  // check method
  if (strcmp(method,"point_cloud") && strcmp(method,"points"))
    mexErrMsgTxt("Input data must be either 'point_cloud' or 'points'.");
  
  
  // creat a pointer to the two point clouds 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ> ());
    
  // Input is point_cloud
  #ifdef USEPCLIO
  if (!strcmp(method,"point_cloud")){
      
     pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(mxArrayToString(prhs[0])), *cld1);
     pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(mxArrayToString(prhs[1])), *cld2);
  }
  #endif
  
  
  // Inputs is points (x, y, z) 
  if (!strcmp(method,"points")){
      
    mxArrayToPointCloud(prhs[0], cld1);
    mxArrayToPointCloud(prhs[1], cld2);
  }
  

    // read intial guess
    mwSize nTr_cols = mxGetN(prhs[2]);
    mwSize nTr_rows = mxGetM(prhs[2]); 
    
    double *Tr_in_mex =   (double*)mxGetPr(prhs[2]);
    Eigen::Matrix4d Tr_in(nTr_rows, nTr_cols); 
    //Eigen::MatrixXd Tr_in(nTr_rows, nTr_cols);            // 4x4
      for (int i = 0; i< nTr_rows; i++ ){                   // col 
          for(int j =0; j< nTr_cols; j++){                  // row
            Tr_in(i, j) = Tr_in_mex[i+j*nTr_rows];
         }
     }
        
    // mapping input intial guess 
    Eigen::Matrix4d final_transformation;
    final_transformation = Tr_in;
    
    // output Correspondences
    pcl::Correspondences all_correspondences;
    
    // number of Iteration
     int nr_iterations = 0; 
    
     
     // read Correspondences
    bool useReciprocalCorrespondences = false;
    
    if(nrhs == 5){
        
        bool *ReciprocalCorrespondences = (bool*)mxGetPr(prhs[4]);
        useReciprocalCorrespondences = ReciprocalCorrespondences[0];
    }
    
    // std::cout << " ReciprocalCorrespondences " << ReciprocalCorrespondences[0] <<std::endl;
     
     
    calculate_reg_2d(*cld1, *cld2, final_transformation, all_correspondences, nr_iterations, useReciprocalCorrespondences);
      
    // print out all correspondences
    Eigen::MatrixXd m(all_correspondences.size(),2);
    
    pcl::Correspondence temp;
    for (int i = 0; i < all_correspondences.size(); i++)
    {   
        temp = all_correspondences[i];
        m(i,0) = temp.index_query;
        m(i,1) = temp.index_match;
        
    }

    // Mapping output : transformation matrix 
    mwSize rows_tr = final_transformation.rows();
    mwSize cols_tr = final_transformation.cols();
    mwSize dims_tr[2];
    dims_tr[1] = rows_tr;
    dims_tr[0] = cols_tr;
    plhs[0] = mxCreateNumericArray(2, dims_tr, mxDOUBLE_CLASS, mxREAL);    // Create MATLAB array of same size
    Eigen::Map<Eigen::MatrixXd> map(mxGetPr(plhs[0]), rows_tr, cols_tr);   // Map the array
    map = final_transformation;                                            // Copy
    
    // Mapping output : Correspondences
    mwSize rows_m = m.rows();
    mwSize cols_m = m.cols();
    
    // Define matrix dim  
    mwSize dims_m[2];
    dims_m[0] = rows_m ;
    dims_m[1] = cols_m ;
    plhs[1] = mxCreateNumericArray(2, dims_m, mxDOUBLE_CLASS, mxREAL);
    Eigen::Map<Eigen::MatrixXd> map_m(mxGetPr(plhs[1]), rows_m, cols_m); 
    map_m = m;
    
    // Mapping out the number of iterations
    plhs[2] = mxCreateDoubleScalar(nr_iterations);
    
}