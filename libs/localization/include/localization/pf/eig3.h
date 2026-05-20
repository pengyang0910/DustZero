/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-21 11:42:18
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2020-09-05 14:43:48
 */

/* Eigen-decomposition for symmetric 3x3 real matrices.
   Public domain, copied from the public domain Java library JAMA. */

#ifndef _eig_h
#define _eig_h

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */
void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);

#endif
