using namespace std; 
#include "ransac_geometry.h"
#include <opencv2/opencv.hpp>
#include "ParamTypes.h"
#include <chrono>

#include <unistd.h>

inline float dis3D(float p1[3], float p2[3])
{
	float d = (p1[0] - p2[0])*(p1[0] - p2[0]);
	d += (p1[1] - p2[1])*(p1[1] - p2[1]);
	d += (p1[2] - p2[2])*(p1[2] - p2[2]);
	return (d == 0.0f)?0.0f:sqrt(d);
}

bool line_estimation(vector<double>&x_sample, vector<double>&y_sample, double& beta0, double& beta1)
{
	double x_bar = 0;
	double y_bar = 0;
	double xy_bar = 0; 
	double x2_bar = 0;

	for (int i = 0; i < x_sample.size(); i++)
	{
		x_bar += x_sample[i];
		y_bar += y_sample[i];
		xy_bar += x_sample[i] * y_sample[i];
		x2_bar += x_sample[i] * x_sample[i];
	}
	x_bar = x_bar / double(x_sample.size());
	y_bar = y_bar / double(y_sample.size());
	x2_bar = x2_bar / double(x_sample.size());
	xy_bar = xy_bar / double(x_sample.size());

	beta1 = (xy_bar - x_bar*y_bar) / (x2_bar - x_bar*x_bar);
	beta0 = y_bar - beta1*x_bar;

	return true;
}

double loss_estimation_line(vector<double>&x_sample, vector<double>&y_sample, double beta0, double beta1)
{
	double loss = 0;

	for (int i = 0; i < x_sample.size(); i++)
	{
		loss += (x_sample[i] * beta1 + beta0 - y_sample[i])*(x_sample[i] * beta1 + beta0 - y_sample[i]);
	}
	loss = loss / double(x_sample.size());

	return loss;
}

double loss_estimation_line(double xi, double yi, double beta0, double beta1)
{
	return (xi*beta1 + beta0 - yi)*(xi*beta1 + beta0 - yi);
}

bool ransac_estimation_line(vector<double>&x_sample, vector<double>&y_sample, double threshold, int least_number, int max_iter, int determin, double& best_beta0, double& best_beta1)
{
	if (x_sample.size() <= least_number)
	{
		return false; 
	}
	vector<int> index_table;
	for (int i = 0; i < x_sample.size(); i++)
	{
		index_table.push_back(i);
	}
	vector<double> inliers_x;
	vector<double> inliers_y;
	//double best_error = 9999999999;
	int best_total = 0;
	//double best_beta0;
	//double best_beta1;
	for (int k = 0; k < max_iter; k++)
	{
		srand(time(NULL));
		random_shuffle(index_table.begin(), index_table.end());
		for (int i = 0; i < least_number; i++)
		{
 			inliers_x.push_back(x_sample[index_table[i]]);
			inliers_y.push_back(y_sample[index_table[i]]);
		}
		double beta0, beta1;
		line_estimation(inliers_x, inliers_y, beta0, beta1);
		int sum_total = least_number;
		for (int i = least_number; i < x_sample.size(); i++)
		{
			if (loss_estimation_line(x_sample[index_table[i]], y_sample[index_table[i]], beta0, beta1) < threshold)
			{
				inliers_x.push_back(x_sample[index_table[i]]);
				inliers_y.push_back(y_sample[index_table[i]]);
				sum_total++;
			}
		}
		line_estimation(inliers_x, inliers_y, beta0, beta1);
		if (sum_total > best_total)
		{
			best_beta0 = beta0;
			best_beta1 = beta1;
			best_total = sum_total;
		}
		inliers_x.clear();
		inliers_y.clear();

	}
	if (best_total < 35)
	{
		return false;
	}
	return true;
}

bool ransac_estimation_line(std::vector<double>&x_sample, std::vector<double>&y_sample, double threshold, int least_number, int max_iter, int determin, float &theta)
{
	double a, b;
	bool bSuccess = ransac_estimation_line(x_sample, y_sample, 2.0f, 4, 100, 5, a, b);
	if (!bSuccess)
	{
		theta = 0.0f;
		return false;
	}
	else
	{
		theta = atan2(b, 1) - CA_PI / 2.0f;
		if (theta > CA_PI)
		{
			theta -= 2 * CA_PI;
		}
		else if (theta <= -CA_PI)
		{
			theta += 2 * CA_PI;
		}
		if (theta < -CA_PI / 2.0f)
		{
			theta += CA_PI;
		}
		else if (theta > CA_PI / 2.0f)
		{
			theta -= CA_PI;
		}
		return true;
	}
}

bool plane_estimation(float *pointcloud, int N, float &A, float &B, float &C)
{
	Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
	Eigen::Vector3f V = Eigen::Vector3f::Zero();
	for (int i = 0; i < N; i++)
	{
		M(0, 0) += pointcloud[3*i] * pointcloud[3*i];
		M(0, 1) += pointcloud[3*i] * pointcloud[3*i+1];
		M(0, 2) += pointcloud[3*i];

		M(1, 0) += pointcloud[3*i] * pointcloud[3*i+1];
		M(1, 1) += pointcloud[3*i+1] * pointcloud[3*i+1];
		M(1, 2) += pointcloud[3*i+1];

		M(2, 0) += pointcloud[3*i];
		M(2, 1) += pointcloud[3*i+1];
		M(2, 2) += 1;

		V(0) += pointcloud[3*i] * pointcloud[3*i+2];
		V(1) += pointcloud[3*i+1] * pointcloud[3*i+2];
		V(2) += pointcloud[3*i+2];
	}

	Eigen::Matrix3f M_1 = M.inverse();
	Eigen::Vector3f Vout = M_1*V;
	A = Vout(0);
	B = Vout(1);
	C = Vout(2);

	return true;
}

bool plane_estimation(vector<float>&x_sample, vector<float>&y_sample, vector<float>&z_sample, float &A, float &B, float &C)
{
	Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
	Eigen::Vector3f V = Eigen::Vector3f::Zero();
	for (int i = 0; i < x_sample.size(); i++)
	{
		M(0, 0) += x_sample[i] * x_sample[i];
		M(0, 1) += x_sample[i] * y_sample[i];
		M(0, 2) += x_sample[i];

		M(1, 0) += x_sample[i] * y_sample[i];
		M(1, 1) += y_sample[i] * y_sample[i];
		M(1, 2) += y_sample[i];

		M(2, 0) += x_sample[i];
		M(2, 1) += y_sample[i];
		M(2, 2) += 1;

		V(0) += x_sample[i] * z_sample[i];
		V(1) += y_sample[i] * z_sample[i];
		V(2) += z_sample[i];
	}

	Eigen::Matrix3f M_1 = M.inverse();

	Eigen::Vector3f Vout = M_1*V;
	A = Vout(0);
	B = Vout(1);
	C = Vout(2);

	return true;
}

inline float loss_estimation_plane(float x, float y, float z, float A, float B, float C)
{
	return (A*x + B*y + C - z)*(A*x + B*y + C - z);
}

float distance_to_plane_signed(float x, float y, float z, float A, float B, float C)
{
	float d = A*A + B*B + C*C;
	if (d == 0.0f)
	{
		return 0.0f;
	}
	return (A*x + B*y + C - z) / sqrt(d);
}


float distance_to_plane(float x, float y, float z, float A, float B, float C)
{
	float d = A*A + B*B + C*C;
	if (d == 0.0f)
	{
		return 0.0f;
	}
	return fabs(A*x + B*y + C - z) / sqrt(d);
}

inline void SPTransformFromZCameraToZImage(const stCameraIntrinsics& zIntrinsics, Eigen::Vector3f& zCamera, Eigen::Vector3f& zImage)
{
	zImage.z() = zCamera.z();
	zImage.y() = (zCamera.y() / zImage.z())*zIntrinsics.fFocalLengthVertical + zIntrinsics.fPrincipalPointCoordV;
	zImage.x() = (zCamera.x() / zImage.z())*zIntrinsics.fFocalLengthHorizontal + zIntrinsics.fPrincipalPointCoordU;
}

float angleLineLine(float l1[3], float l2[3])
{
	float m1 = l1[0] * l1[0] + l1[1] * l1[1] + l1[2] * l1[2];
	m1 = (m1 == 0.0f) ? 0.0f : sqrt(m1);

    float m2 = l2[0] * l2[0] + l2[1] * l2[1] + l2[2] * l2[2];
	m2 = (m2 == 0.0f) ? 0.0f : sqrt(m2);

	float dot = l1[0] * l2[0] + l1[1] * l2[1] + l1[2] * l2[2];
	return acos(dot/(m1*m2));
}

bool ransac_estimation_plane(float *pointcloud, int n, float g[3], float fMinHeightLowerThanCamera, double threshold, int least_number, int max_iter, int min_best_total, float fHeightTolerence, float fAngleTolerence, float &A, float &B, float &C, int &error_code)
{
	error_code = 0;
	auto t1 = std::chrono::high_resolution_clock::now();
	if (pointcloud == NULL)
	{
		error_code = 10;
		return false;
	}
	if (n < 2 * least_number)
	{
		error_code = 10;
		return false;
	}

	float* inlier = new float[3 * n];
	float best_dis = 0.0f;
	int best_total = min_best_total;
	float best_theta = 0.0f;
	float best_fMeanHeight = 0.0f;
	bool b_HavePlanePassed = false;

	int* index_table = new int[n];
	int* inlier_table = new int[n];
	int Ninlier;
	for (int k = 0; k < max_iter; k++)
	{
		for (int i = 0; i < n; ++i)
		{
			index_table[i] = i;
		}
		for (int i = 0; i < least_number; ++i)
		{
			int j = rand()%(n - i);
			if (j != 0)
			{
				std::swap(index_table[i], index_table[i + j]);
			}
		}

		for (int i = 0; i < least_number; i++)
		{
			int _i = 3 * index_table[i];
			for (int k = 0; k < 3; k++)
			{
				inlier[3 * i + k] = pointcloud[_i + k];
			}
		}

		float A_ = 0.0f; float B_ = 0.0f; float C_ = 0.0f;
		plane_estimation(inlier, least_number, A_, B_, C_);
		int sum_total = least_number;
		Ninlier = 0;
		for (int i = least_number; i < n; i++)
		{
			int _i = 3*index_table[i];
			if (loss_estimation_plane(pointcloud[_i], pointcloud[_i+1], pointcloud[_i+2], A_, B_, C_) < threshold)
			{
				inlier_table[Ninlier] = index_table[i];
				Ninlier++;
				sum_total++;
			}
		}

		//weight based on angle between gravity and plane
		float normal[3];
		float m1 = A_*A_ + B_*B_ + 1;
		m1 = sqrt(m1);
		normal[0] = A_ / m1; normal[1] = B_ / m1; normal[2] = -1 / m1;

		float theta = angleLineLine(g, normal);
		float fMeanHeight = 0.0f;
		if (sum_total > best_total)
		{
			for (int i = 0; i < Ninlier; i++)
			{
				int _i = 3 * inlier_table[i];
				for (int k = 0; k < 3; k++)
				{
					inlier[3 * i + k] = pointcloud[_i + k];
				}
				fMeanHeight += pointcloud[_i + 1];
			}
			plane_estimation(inlier, Ninlier, A_, B_, C_);
			A = A_;
			B = B_;
			C = C_;
			best_total = sum_total;
			best_theta = theta;
            best_dis = distance_to_plane(0.0, 0.0, 0.0, A, B, C);
			best_fMeanHeight = fMeanHeight / Ninlier;
			b_HavePlanePassed = true;
		}
	}

	if (best_theta > CV_PI / 2.0f)
	{
		best_theta = CV_PI - best_theta;
	}

	auto t2 = std::chrono::high_resolution_clock::now();
	auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

	delete index_table; index_table = NULL;
	delete inlier; inlier = NULL;
	delete inlier_table; inlier_table = NULL;

	/*if (best_fMeanHeight < -0.3f)
	{
		error_code = 1;
		return false;
	}
	if (!b_HavePlanePassed)
	{
		error_code = 2;
		return false;
	}
	if (best_theta > fAngleTolerence)
	{
		error_code = 3;
		return false;
	}
	if (fabs(best_dis - fMinHeightLowerThanCamera) > fHeightTolerence)
	{
		error_code = 4;
		return false;
	}*/
	return true;
}

bool ransac_estimation_plane(std::vector<float>&x_sample, std::vector<float>&y_sample, std::vector<float>&z_sample, float g[3], float fMinHeightLowerThanCamera, double threshold, int least_number, int max_iter, float &A, float &B, float &C)
{
	auto t1 = std::chrono::high_resolution_clock::now();

	threshold = threshold*threshold;
	if (x_sample.size() <= least_number)
	{
		return false;
	}
	vector<float> inliers_x;
	vector<float> inliers_y;
	vector<float> inliers_z;
	float best_score = 0.0f;
	int best_total = 0;
	float best_theta;

	int n = x_sample.size();
	int* index_table = new int[n];
	int* inlier_table = new int[n];
	int Ninlier;
	for (int k = 0; k < max_iter; k++)
	{
		for (int i = 0; i < n; ++i)
		{
			index_table[i] = i;
		}
		int _count = 1;
		int j = rand() % n;
		if (j != 0)
		{
			std::swap(index_table[0], index_table[j]);
		}
		float p1[3];
		p1[0] = x_sample[index_table[0]];
		p1[1] = y_sample[index_table[0]];
		p1[2] = z_sample[index_table[0]];

		while(_count < least_number)
		{
			j = rand() % (n - _count);
			if (j != 0)
			{
				float p2[3];
				p2[0] = x_sample[index_table[j]];
				p2[1] = y_sample[index_table[j]];
				p2[2] = z_sample[index_table[j]];
				if (dis3D(p1, p2) < 0.1)
				{
					std::swap(index_table[_count], index_table[_count + j]);
					_count++;
				}
			}
		}
		/*for (int i = 0; i < least_number; ++i) 
		{
			int j = rand()%(n - i);
			if (j != 0) 
			{
				std::swap(index_table[i], index_table[i + j]);
			}
		}*/

		for (int i = 0; i < least_number; i++)
		{
			inliers_x.push_back(x_sample[index_table[i]]);
			inliers_y.push_back(y_sample[index_table[i]]);
			inliers_z.push_back(z_sample[index_table[i]]);
		}

		float A_, B_, C_;
		plane_estimation(inliers_x, inliers_y, inliers_z, A_, B_, C_);
		int sum_total = least_number;
		Ninlier = 0;
		for (int i = least_number; i < x_sample.size(); i++)
		{
			if (loss_estimation_plane(x_sample[index_table[i]], y_sample[index_table[i]], z_sample[index_table[i]], A_, B_, C_) < threshold)
			{
				inlier_table[Ninlier] = index_table[i];
				Ninlier++;
				sum_total++;
			}
		}

		//weight based on angle between gravity and plane
		float normal[3];
		normal[0] = A_; normal[1] = B_; normal[2] = C_;
		float theta = angleLineLine(g, normal);
		if (sum_total > best_total)
		//if (_f*sum_total > best_score)
		{
			for (int i = 0; i < Ninlier; i++)
			{
				inliers_x.push_back(x_sample[inlier_table[i]]);
				inliers_y.push_back(y_sample[inlier_table[i]]);
				inliers_z.push_back(z_sample[inlier_table[i]]);
			}
			plane_estimation(inliers_x, inliers_y, inliers_z, A_, B_, C_);
			A = A_;
			B = B_;
			C = C_;
			best_total = sum_total;
			best_theta = theta;
		}

		inliers_x.clear();
		inliers_y.clear();
		inliers_z.clear();
	}

	printf("best_theta = %f %d\n", best_theta, best_total);

	auto t2 = std::chrono::high_resolution_clock::now();
	auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

	delete index_table; index_table = NULL;
	//printf("dt = %f\n", 1000*diff.count() * 0.000001f);
	if (best_total < 10)
	{
		return false;
	}
	return true;
}
