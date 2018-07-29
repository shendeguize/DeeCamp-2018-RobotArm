#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <algorithm>
#include <Eigen/Eigen>

bool ransac_estimation_line(std::vector<double>&x_sample, std::vector<double>&y_sample, double threshold, int least_number, int max_iter, int determin, double& best_beta0, double& best_beta1);
bool ransac_estimation_line(std::vector<double>&x_sample, std::vector<double>&y_sample, double threshold, int least_number, int max_iter, int determin, float &theta);
//
bool ransac_estimation_plane(std::vector<float>&x_sample, std::vector<float>&y_sample, std::vector<float>&z_sample, float g[3], float fMinHeightLowerThanCamera, double threshold, int least_number, int max_iter, float &A, float &B, float &C);
bool ransac_estimation_plane(float *pointcloud, int n, float g[3], float fMinHeightLowerThanCamera, double threshold, int least_number, int max_iter, int min_best_total, float fHeightTolerence, float fAngleTolerence, float &A, float &B, float &C, int &error_code);
float loss_estimation_plane(float x, float y, float z, float A, float B, float C);
float distance_to_plane_signed(float x, float y, float z, float A, float B, float C);
float distance_to_plane(float x, float y, float z, float A, float B, float C);
float angleLineLine(float l1[3], float l2[3]);
