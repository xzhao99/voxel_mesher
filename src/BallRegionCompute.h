﻿#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath> 
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

struct Point_Index 
{
	int parentIndex;
	int index;

	Point_Index(int d, int b) {
		parentIndex = d;
		index = b;
	}
};

class BallRegion 
{
public:
	std::vector<std::vector<double>> pointCloudData;
	std::vector<int> pointCloudData_boxIndex;//the box index of point cloud
	std::vector<std::vector<double>> pointNormal;//normal vector	
	std::vector<int> pointBorder;//minx,y,x; maxx,y,z
	std::vector<std::vector<int>> squareBoxes; //the point index in different boxes， if no，then to 0。	
	std::vector<std::vector<double>> squareBoxesCReal;
	std::vector<int> squareBoxesCenter;//store the point index is near to the real center
	std::vector<int> XYZNumber;//achieve the box number in different axes
	std::vector<double> minXYZ;//achieve the min cordinate of XYZ
	double unitSize;//the box scale
	double radius; //Ball region radius	
	int pointNumEsti;//points define for radius estimation,8 points should be find at least
	int boxNum;//Voxel biggest number(Totally)
	std::vector<std::vector<int>> pointNeibor;//neibor points in Ball for every points
	pcl::KdTreeFLANN<pcl::PointXYZ> pcl_kdtree;//kdtree

public:
	void BallRegion_init(const std::vector<std::vector<double>>& pointCloudData_input,
											 const std::vector<std::vector<double>>& pointNormal_input, 
											 const std::vector<int>& pointBorder_input);
											 
	//init for simplification point cloud
	void BallRegion_init_simpli(const std::vector<std::vector<double>>& pointCloudData_input,
															const std::vector<std::vector<double>>& pointNormal_input, 
															const BallRegion& br_input, 
															int pointNumEsti_input);

	
		

	std::vector<std::vector<double>> BallRegion_ReturnBox(int pointIndex) {
		std::vector<double> point_V = pointCloudData[pointIndex];
		//int x_num2 = (point_V[0] - minXYZ[0]) / unitSize;
		//int y_num2 = (point_V[1] - minXYZ[1]) / unitSize;
		//int z_num2 = (point_V[2] - minXYZ[2]) / unitSize;
		int boxIndex = pointCloudData_boxIndex[pointIndex];
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];
		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		double x1 = minXYZ[0] + (x_num - 1) * unitSize;
		double y1 = minXYZ[1] + y_num * unitSize;
		double z1 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v1;
		v1.push_back(x1);
		v1.push_back(y1);
		v1.push_back(z1);
		double x2 = minXYZ[0] + x_num * unitSize;
		double y2 = minXYZ[1] + y_num * unitSize;
		double z2 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v2;
		v2.push_back(x2);
		v2.push_back(y2);
		v2.push_back(z2);
		double x3 = minXYZ[0] + (x_num - 1) * unitSize;
		double y3 = minXYZ[1] + (y_num - 1) * unitSize;
		double z3 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v3;
		v3.push_back(x3);
		v3.push_back(y3);
		v3.push_back(z3);
		double x4 = minXYZ[0] + x_num * unitSize;
		double y4 = minXYZ[1] + (y_num - 1) * unitSize;
		double z4 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v4;
		v4.push_back(x4);
		v4.push_back(y4);
		v4.push_back(z4);

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++

		double x5 = minXYZ[0] + (x_num - 1) * unitSize;
		double y5 = minXYZ[1] + y_num * unitSize;
		double z5 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v5;
		v5.push_back(x5);
		v5.push_back(y5);
		v5.push_back(z5);
		double x6 = minXYZ[0] + x_num * unitSize;
		double y6 = minXYZ[1] + y_num * unitSize;
		double z6 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v6;
		v6.push_back(x6);
		v6.push_back(y6);
		v6.push_back(z6);
		double x7 = minXYZ[0] + (x_num - 1) * unitSize;
		double y7 = minXYZ[1] + (y_num - 1) * unitSize;
		double z7 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v7;
		v7.push_back(x7);
		v7.push_back(y7);
		v7.push_back(z7);
		double x8 = minXYZ[0] + x_num * unitSize;
		double y8 = minXYZ[1] + (y_num - 1) * unitSize;
		double z8 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v8;
		v8.push_back(x8);
		v8.push_back(y8);
		v8.push_back(z8);

		std::vector<std::vector<double>> result;
		result.push_back(v1);
		result.push_back(v2);
		result.push_back(v3);
		result.push_back(v4);
		result.push_back(v5);
		result.push_back(v6);
		result.push_back(v7);
		result.push_back(v8);

		return result;
	}

	std::vector<std::vector<double>> BallRegion_ReturnBox_Index(int boxIndex) {
		//reconstruct number
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		double x1 = minXYZ[0] + (x_num - 1) * unitSize;
		double y1 = minXYZ[1] + y_num * unitSize;
		double z1 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v1;
		v1.push_back(x1);
		v1.push_back(y1);
		v1.push_back(z1);
		double x2 = minXYZ[0] + x_num * unitSize;
		double y2 = minXYZ[1] + y_num * unitSize;
		double z2 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v2;
		v2.push_back(x2);
		v2.push_back(y2);
		v2.push_back(z2);
		double x3 = minXYZ[0] + (x_num - 1) * unitSize;
		double y3 = minXYZ[1] + (y_num - 1) * unitSize;
		double z3 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v3;
		v3.push_back(x3);
		v3.push_back(y3);
		v3.push_back(z3);
		double x4 = minXYZ[0] + x_num * unitSize;
		double y4 = minXYZ[1] + (y_num - 1) * unitSize;
		double z4 = minXYZ[2] + (z_num - 1) * unitSize;
		std::vector<double> v4;
		v4.push_back(x4);
		v4.push_back(y4);
		v4.push_back(z4);

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++

		double x5 = minXYZ[0] + (x_num - 1) * unitSize;
		double y5 = minXYZ[1] + y_num * unitSize;
		double z5 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v5;
		v5.push_back(x5);
		v5.push_back(y5);
		v5.push_back(z5);
		double x6 = minXYZ[0] + x_num * unitSize;
		double y6 = minXYZ[1] + y_num * unitSize;
		double z6 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v6;
		v6.push_back(x6);
		v6.push_back(y6);
		v6.push_back(z6);
		double x7 = minXYZ[0] + (x_num - 1) * unitSize;
		double y7 = minXYZ[1] + (y_num - 1) * unitSize;
		double z7 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v7;
		v7.push_back(x7);
		v7.push_back(y7);
		v7.push_back(z7);
		double x8 = minXYZ[0] + x_num * unitSize;
		double y8 = minXYZ[1] + (y_num - 1) * unitSize;
		double z8 = minXYZ[2] + z_num * unitSize;
		std::vector<double> v8;
		v8.push_back(x8);
		v8.push_back(y8);
		v8.push_back(z8);

		std::vector<std::vector<double>> result;
		result.push_back(v1);
		result.push_back(v2);
		result.push_back(v3);
		result.push_back(v4);
		result.push_back(v5);
		result.push_back(v6);
		result.push_back(v7);
		result.push_back(v8);

		return result;
	}

	std::vector<int> BallRegion_ReturnNeibor_kd(std::vector<double> p) {

		int K = pointNumEsti + 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		std::vector<int> result;
		double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
		pcl::PointXYZ searchPoint;
		searchPoint.x = p[0];
		searchPoint.y = p[1];
		searchPoint.z = p[2];
		pcl_kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

		if (pointNKNSquaredDistance[0] == 0) {
			result.insert(result.end(), pointIdxNKNSearch.begin() + 1, pointIdxNKNSearch.end());
		}
		else {
			result.insert(result.end(), pointIdxNKNSearch.begin(), pointIdxNKNSearch.end() - 1);
		}
		return result;
	}

	int BallRegion_ReturnBoxIndex(std::vector<double> p) {

		double xNum = (p[0] - minXYZ[0]) / unitSize;
		double yNum = (p[1] - minXYZ[1]) / unitSize;
		double zNum = (p[2] - minXYZ[2]) / unitSize;

		int index_Compute = xNum + (yNum - 1) * XYZNumber[0] +
			(zNum - 1) * XYZNumber[0] * XYZNumber[1];

		return index_Compute;	
	}

private:

	void BallRegion_EstimateRadius() {
		//
		std::vector<std::vector<int>> pointNeiborEstimate;
		std::vector<double> pointNeiborRadiusEst;
		double maxGlobal = -9999;

		for (int i = 0; i < pointCloudData.size(); i++) {

			std::vector<int> empty_i;
			pointNeiborEstimate.push_back(empty_i);
			pointNeiborRadiusEst.push_back(-9999);
		}

		for (int i = 0; i < pointNeiborEstimate.size(); i++) {
			if (i % 1000 == 0) {
				std::cout << (pointNeiborEstimate.size() - i) / 1000 << ",";
			}
			double max = -9999;
			std::vector<double> p_i = pointCloudData[i];
			int boxIndex = pointCloudData_boxIndex[i];
			std::vector<int> pointCloudData_candi =
				BallRegion_ReturnNeiborBox(boxIndex);
			for (int j = 0; j < pointCloudData_candi.size(); j++) {
				int index_j = pointCloudData_candi[j];
				if (i == index_j) {
					continue;
				}
				std::vector<double> p_ij = pointCloudData[index_j];
				double dis_ij = disPointPair(p_i, p_ij);
				if (dis_ij == 0) {
					continue;
				}
				if (pointNeiborEstimate[i].size() < (pointNumEsti - 1)) {
					pointNeiborEstimate[i].push_back(index_j);
					if (dis_ij > max) {
						max = dis_ij;
					}
				}
				else if (pointNeiborEstimate[i].size() == (pointNumEsti - 1)) {
					pointNeiborEstimate[i].push_back(index_j);
					if (dis_ij > max) {
						max = dis_ij;
					}
					std::vector<int> remangagePNE = BallRegion_MaxPointChange(i, pointNeiborEstimate[i]);
					for (int k = 0; k < remangagePNE.size(); k++) {
						pointNeiborEstimate[i][k] = remangagePNE[k];
					}
				}
				else {
					if (dis_ij < max) {
						pointNeiborEstimate[i][(pointNumEsti - 1)] = index_j;
						std::vector<int> remangagePNE = BallRegion_MaxPointChange(i, pointNeiborEstimate[i]);
						for (int k = 0; k < remangagePNE.size(); k++) {
							pointNeiborEstimate[i][k] = remangagePNE[k];
						}
						int maxIndex = pointNeiborEstimate[i][(pointNumEsti - 1)];
						std::vector<double> p_max_now = pointCloudData[maxIndex];
						max = disPointPair(p_i, p_max_now);
					}
				}
			}
			pointNeiborRadiusEst[i] = max;
			if (max > maxGlobal) {
				maxGlobal = max;
			}
		}
		std::cout << std::endl;

		radius = maxGlobal;
		if (pointNeibor.size() > 0) {
			pointNeibor.clear();
		}

		for (int i = 0; i < pointNeiborEstimate.size(); i++) {
			pointNeibor.push_back(pointNeiborEstimate[i]);
		}
	}

	void BallRegion_EstimateRadius_KDTree() {
		//pointNeibor each points neibor
		//radius		
		std::vector<std::vector<int>> pointNeiborEstimate(pointCloudData.size());
		//create a “searchPoint” which is assigned random coordinates
		int K = pointNumEsti + 1;
		double rMax = -1;
		clock_t t1 = clock();

		for (int i = 0; i < pointCloudData.size(); i++) {
			std::vector<int> empty;
			pointNeibor.push_back(empty);
		}

#pragma omp parallel for
		for (int i = 0; i < pointCloudData.size(); i++) {
			if (i % 10000 == 0) {
				std::cout << (pointNeiborEstimate.size() - i) / 10000 << ",";
			}
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointCloudData[i][0];
			searchPoint.y = pointCloudData[i][1];
			searchPoint.z = pointCloudData[i][2];
			pcl_kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			double x1 = pointCloudData[pointIdxNKNSearch[1]][0];
			double y1 = pointCloudData[pointIdxNKNSearch[1]][1];
			double z1 = pointCloudData[pointIdxNKNSearch[1]][2];
			std::vector<double> ppp;
			ppp.push_back(x1);
			ppp.push_back(y1);
			ppp.push_back(z1);
			double dis = disPointPair(ppp, pointCloudData[i]);
			double disSqure = dis * dis;

			if (pointNKNSquaredDistance[0] == 0) {
				pointNeibor_i.insert(pointNeibor_i.end(), pointIdxNKNSearch.begin() + 1, pointIdxNKNSearch.end());
			}
			else {
				pointNeibor_i.insert(pointNeibor_i.end(), pointIdxNKNSearch.begin(), pointIdxNKNSearch.end() - 1);
			}
			double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
			if (r_i > rMax) {
				rMax = r_i;
			}
			pointNeibor[i].insert(pointNeibor[i].end(), pointNeibor_i.begin(), pointNeibor_i.end());
		}
		clock_t t2 = clock();
		std::cout << std::endl;
		std::cout << "running time:" << t2 - t1 << "ms" << std::endl;
		radius = sqrt(rMax);//store radius
	}

	void BallRegion_EstimateRadius_fast(int regionSize) {
		//
		std::vector<std::vector<int>> pointNeiborEstimate;
		std::vector<double> pointNeiborRadiusEst;
		double maxGlobal = -9999;

		for (int i = 0; i < pointCloudData.size(); i++) {

			std::vector<int> empty_i;
			pointNeiborEstimate.push_back(empty_i);
			pointNeiborRadiusEst.push_back(-9999);
		}

		for (int i = 0; i < pointNeiborEstimate.size(); i++) {
			if (i % 1000 == 0) {
				std::cout << (pointNeiborEstimate.size() - i) / 1000 << ",";
			}
			//std::cout << i << ",";
			double max = -9999;
			std::vector<double> p_i = pointCloudData[i];
			int boxIndex = pointCloudData_boxIndex[i];
			std::vector<int> pointCloudData_candi =
				BallRegion_ReturnNeiborBox2(boxIndex, regionSize);
			std::vector<double> dis_v;
			std::vector<double> dis_v_store;

			for (int j = 0; j < pointCloudData_candi.size(); j++) {
				int index_j = pointCloudData_candi[j];
				if (i == index_j) {
					dis_v.push_back(99999);
					dis_v_store.push_back(99999);
					continue;
				}
				std::vector<double> p_ij = pointCloudData[index_j];
				double dis_ij = disPointPair(p_i, p_ij);
				if (dis_ij > max) {
					max = dis_ij;
				}
				dis_v.push_back(dis_ij);
				dis_v_store.push_back(dis_ij);
			}
			std::sort(dis_v.begin(), dis_v.end());
			//std::sort(dis_v.begin(), dis_v.end());
			for (int j = 0; j < dis_v.size(); j++) {
				if (j == pointNumEsti) {
					break;
				}
				double dis_j = dis_v[j];
				int searchIndex = indexReturn(dis_j, dis_v_store);
				if (searchIndex == -1) {
					std::cout << "Hello!" << std::endl;

				}
				pointNeiborEstimate[i].push_back(pointCloudData_candi[searchIndex]);
				max = dis_v[j];
			}

			pointNeiborRadiusEst[i] = max;
			if (max > maxGlobal) {
				maxGlobal = max;
			}
		}
		std::cout << std::endl;

		radius = maxGlobal;
		if (pointNeibor.size() > 0) {
			pointNeibor.clear();
		}

		for (int i = 0; i < pointNeiborEstimate.size(); i++) {
			pointNeibor.push_back(pointNeiborEstimate[i]);
		}
	}

	std::vector<int> BallRegion_MaxPointChange(int centerIndex, std::vector<int> vdata) {

		if (vdata.size() < pointNumEsti) {
			std::cout << "BallRegion_MaxPointChange: Error, point num is less than theorid!";
			return vdata;
		}
		std::vector<double> point_ct = pointCloudData[centerIndex];
		std::vector<double> point_vMax = pointCloudData[vdata[vdata.size() - 1]];
		double dMax = disPointPair(point_ct, point_vMax);
		for (int i = 0; i < vdata.size() - 1; i++) {

			std::vector<double> point_vi = pointCloudData[vdata[i]];
			double dis_i = disPointPair(point_ct, point_vi);

			if (dis_i > dMax) {
				dMax = dis_i;
				int tem;
				tem = vdata[i];
				vdata[i] = vdata[vdata.size() - 1];
				vdata[vdata.size() - 1] = tem;
			}

		}
		return vdata;
	}

	//Input points into the boxes
	void BallRegion_BoxInput();

	//build box index regions
	void BallRegion_AchieveXYZ();

	void BallRegion_AchieveXYZ2(BallRegion br_input);


	void BallRegion_RegularNormal(int start) {

		std::vector<std::vector<Point_Index>> normal_store;
		std::vector<bool> normalJudge(pointCloudData.size());
		for (int i = 0; i < normalJudge.size(); i++) {
			normalJudge[i] = false;
		}
		normalJudge[start] = true;
		std::vector<Point_Index> normal_start;
		//normal_start.push_back(pointNormal[start][0]);
		//normal_start.push_back(pointNormal[start][1]);
		//normal_start.push_back(pointNormal[start][2]);
		Point_Index pi(start, start);
		normal_start.push_back(pi);//parent point index		
		normal_store.push_back(normal_start);
		while (1) {
			// 1. achieve source points
			std::vector<Point_Index> norProcessNow = normal_store[normal_store.size() - 1];
			std::vector<Point_Index> norProcessNow_Neibor;

			// 2. achieve neibor of source points
			for (int i = 0; i < norProcessNow.size(); i++) {
				std::vector<int> neibor_i = pointNeibor[norProcessNow[i].index];
				for (int j = 0; j < neibor_i.size(); j++) {
					if (normalJudge[neibor_i[j]] == false) {
						Point_Index pij(norProcessNow[i].index, neibor_i[j]);
						//remove repeat
						bool repeatJudge = false;
						for (int k = 0; k < norProcessNow_Neibor.size(); k++) {
							if (norProcessNow_Neibor[k].index == neibor_i[j]) {
								repeatJudge = true;
							}
						}
						if (!repeatJudge) {
							norProcessNow_Neibor.push_back(pij);
						}
					}
				}
			}

			// 3. update normal
			if (norProcessNow_Neibor.size() == 0) {
				break;
			}
			else {
				normal_store.push_back(norProcessNow_Neibor);
			}
			for (int i = 0; i < norProcessNow_Neibor.size(); i++) {
				std::vector<double> n_parent = pointNormal[norProcessNow_Neibor[i].parentIndex];
				std::vector<double> n_itself = pointNormal[norProcessNow_Neibor[i].index];
				double a1 = n_parent[0] * n_itself[0] + n_parent[1] * n_itself[1] + n_parent[2] * n_itself[2];
				double a2 = -n_parent[0] * n_itself[0] - n_parent[1] * n_itself[1] - n_parent[2] * n_itself[2];
				if (a1 > 1) {
					a1 = 1;
				}
				if (a1 < -1) {
					a1 = -1;
				}
				if (a2 > 1) {
					a2 = 1;
				}
				if (a2 < -1) {
					a2 = -1;
				}
				if (acos(a1) > acos(a2)) {
					pointNormal[norProcessNow_Neibor[i].index][0] = -pointNormal[norProcessNow_Neibor[i].index][0];
					pointNormal[norProcessNow_Neibor[i].index][1] = -pointNormal[norProcessNow_Neibor[i].index][1];
					pointNormal[norProcessNow_Neibor[i].index][2] = -pointNormal[norProcessNow_Neibor[i].index][2];
				}
				normalJudge[norProcessNow_Neibor[i].index] = true;
			}
			normal_store.push_back(norProcessNow_Neibor);
		}
		//std::cout << std::endl;	
	}
	
public:

	std::vector<int> BallRegion_ReturnNeiborBox(int boxIndex) {

		//reconstruct number
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		std::vector<int> x_num_index;
		if (x_num > 1) {
			x_num_index.push_back(x_num - 1);
		}
		x_num_index.push_back(x_num);
		if (x_num < XYZNumber[0]) {
			x_num_index.push_back(x_num + 1);
		}

		std::vector<int> y_num_index;
		if (y_num > 1) {
			y_num_index.push_back(y_num - 1);
		}
		y_num_index.push_back(y_num);
		if (y_num < XYZNumber[1]) {
			y_num_index.push_back(y_num + 1);
		}

		std::vector<int> z_num_index;
		if (z_num > 1) {
			z_num_index.push_back(z_num - 1);
		}
		z_num_index.push_back(z_num);
		if (z_num < XYZNumber[2]) {
			z_num_index.push_back(z_num + 1);
		}

		std::vector<int> boxIndexSum;

		for (int i = 0; i < x_num_index.size(); i++) {
			int x_num_index_i = x_num_index[i];
			for (int j = 0; j < y_num_index.size(); j++) {
				int y_num_index_j = y_num_index[j];
				for (int k = 0; k < z_num_index.size(); k++) {
					int z_num_index_k = z_num_index[k];
					int index_Compute = x_num_index_i + (y_num_index_j - 1) * XYZNumber[0] +
						(z_num_index_k - 1) * XYZNumber[0] * XYZNumber[1];
					boxIndexSum.insert(boxIndexSum.end(), squareBoxes[index_Compute].begin(), squareBoxes[index_Compute].end());
				}
			}
		}

		//std::vector<std::vector<double>> pointCandidate;
		//for (int i = 0; i < boxIndexSum.size(); i++) {
			//int boxIndex_i = boxIndexSum[i];
			//std::vector<double> p_i = pointCloudData[boxIndex_i];
			//pointCandidate.push_back(p_i);					   		
		//}
		return boxIndexSum;//return point index
	}

	std::vector<int> BallRegion_ReturnNeiborBox_JustNeibor(int boxIndex) {

		//reconstruct number
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		std::vector<int> x_num_index;
		if (x_num > 1) {
			x_num_index.push_back(x_num - 1);
		}
		x_num_index.push_back(x_num);
		if (x_num < XYZNumber[0]) {
			x_num_index.push_back(x_num + 1);
		}

		std::vector<int> y_num_index;
		if (y_num > 1) {
			y_num_index.push_back(y_num - 1);
		}
		y_num_index.push_back(y_num);
		if (y_num < XYZNumber[1]) {
			y_num_index.push_back(y_num + 1);
		}

		std::vector<int> z_num_index;
		if (z_num > 1) {
			z_num_index.push_back(z_num - 1);
		}
		z_num_index.push_back(z_num);
		if (z_num < XYZNumber[2]) {
			z_num_index.push_back(z_num + 1);
		}

		std::vector<int> boxIndexSum;

		for (int i = 0; i < x_num_index.size(); i++) {
			int x_num_index_i = x_num_index[i];
			for (int j = 0; j < y_num_index.size(); j++) {
				int y_num_index_j = y_num_index[j];
				for (int k = 0; k < z_num_index.size(); k++) {
					int z_num_index_k = z_num_index[k];
					if (x_num_index_i == x_num && y_num_index_j == y_num && z_num_index_k == z_num) {
						continue;
					}
					else {
						int index_Compute = x_num_index_i + (y_num_index_j - 1) * XYZNumber[0] +
							(z_num_index_k - 1) * XYZNumber[0] * XYZNumber[1];
						boxIndexSum.insert(boxIndexSum.end(), squareBoxes[index_Compute].begin(), squareBoxes[index_Compute].end());
					}
				}
			}
		}

		//std::vector<std::vector<double>> pointCandidate;
		//for (int i = 0; i < boxIndexSum.size(); i++) {
			//int boxIndex_i = boxIndexSum[i];
			//std::vector<double> p_i = pointCloudData[boxIndex_i];
			//pointCandidate.push_back(p_i);					   		
		//}
		return boxIndexSum;//return point index
	}

	std::vector<int> BallRegion_ReturnNeiborBox_Box(int boxIndex) {
		

		//reconstruct number
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		std::vector<int> x_num_index;
		if (x_num > 1) {
			x_num_index.push_back(x_num - 1);
		}
		x_num_index.push_back(x_num);
		if (x_num < XYZNumber[0]) {
			x_num_index.push_back(x_num + 1);
		}

		std::vector<int> y_num_index;
		if (y_num > 1) {
			y_num_index.push_back(y_num - 1);
		}
		y_num_index.push_back(y_num);
		if (y_num < XYZNumber[1]) {
			y_num_index.push_back(y_num + 1);
		}

		std::vector<int> z_num_index;
		if (z_num > 1) {
			z_num_index.push_back(z_num - 1);
		}
		z_num_index.push_back(z_num);
		if (z_num < XYZNumber[2]) {
			z_num_index.push_back(z_num + 1);
		}

		std::vector<int> boxIndexR;

		for (int i = 0; i < x_num_index.size(); i++) {
			int x_num_index_i = x_num_index[i];			
			for (int j = 0; j < y_num_index.size(); j++) {
				int y_num_index_j = y_num_index[j];
				for (int k = 0; k < z_num_index.size(); k++) {
					int z_num_index_k = z_num_index[k];
					if (x_num_index_i == x_num && y_num_index_j == y_num && z_num_index_k == z_num) {
						continue;
					}
					else {
						int index_Compute = x_num_index_i + (y_num_index_j - 1) * XYZNumber[0] +
							(z_num_index_k - 1) * XYZNumber[0] * XYZNumber[1];
						if (index_Compute < squareBoxes.size()) {
							boxIndexR.push_back(index_Compute);						
						}						
					}
				}
			}
		}

		//std::vector<std::vector<double>> pointCandidate;
		//for (int i = 0; i < boxIndexSum.size(); i++) {
			//int boxIndex_i = boxIndexSum[i];
			//std::vector<double> p_i = pointCloudData[boxIndex_i];
			//pointCandidate.push_back(p_i);					   		
		//}
		return boxIndexR;//return point index
	}

	std::vector<int> BallRegion_ReturnNeiborBox2(int boxIndex, int regionSize) {//regionSize is the number of boxes in one axis

		//reconstruct number
		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];
		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		int regionSizeHalf = regionSize / 2;
		int min_x_num = x_num - regionSizeHalf;
		int min_y_num = y_num - regionSizeHalf;
		int min_z_num = z_num - regionSizeHalf;

		std::vector<int> x_num_index;
		for (int i = min_x_num; i <= x_num + regionSizeHalf; i++) {
			if (i >= 1 && i <= XYZNumber[0]) {
				x_num_index.push_back(i);
			}
		}

		std::vector<int> y_num_index;
		for (int i = min_y_num; i <= y_num + regionSizeHalf; i++) {
			if (i >= 1 && i <= XYZNumber[1]) {
				y_num_index.push_back(i);
			}
		}

		std::vector<int> z_num_index;
		for (int i = min_z_num; i <= z_num + regionSizeHalf; i++) {
			if (i >= 1 && i <= XYZNumber[2]) {
				z_num_index.push_back(i);
			}
		}

		std::vector<int> boxIndexSum;

		for (int i = 0; i < x_num_index.size(); i++) {
			int x_num_index_i = x_num_index[i];
			for (int j = 0; j < y_num_index.size(); j++) {
				int y_num_index_j = y_num_index[j];
				for (int k = 0; k < z_num_index.size(); k++) {
					int z_num_index_k = z_num_index[k];
					int index_Compute = x_num_index_i + (y_num_index_j - 1) * XYZNumber[0] +
						(z_num_index_k - 1) * XYZNumber[0] * XYZNumber[1];
					boxIndexSum.insert(boxIndexSum.end(), squareBoxes[index_Compute].begin(), squareBoxes[index_Compute].end());
				}
			}
		}

		//std::vector<std::vector<double>> pointCandidate;
		//for (int i = 0; i < boxIndexSum.size(); i++) {
			//int boxIndex_i = boxIndexSum[i];
			//std::vector<double> p_i = pointCloudData[boxIndex_i];
			//pointCandidate.push_back(p_i);					   		
		//}
		return boxIndexSum;//return point index
	}

	std::vector<double> BallRegion_ReturnBoxCenter_Radius(int boxIndex) {

		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		double xcenter = (minXYZ[0] + (x_num - 1) * unitSize + minXYZ[0] + x_num * unitSize) / 2;
		double ycenter = (minXYZ[1] + (y_num - 1) * unitSize + minXYZ[1] + y_num * unitSize) / 2;
		double zcenter = (minXYZ[2] + (z_num - 1) * unitSize + minXYZ[2] + z_num * unitSize) / 2;

		double x1 = minXYZ[0] + (x_num - 1) * unitSize;
		double y1 = minXYZ[1] + y_num * unitSize;
		double z1 = minXYZ[2] + (z_num - 1) * unitSize;

		double radius = sqrt((xcenter - x1) * (xcenter - x1) + (ycenter - y1) * (ycenter - y1) + (zcenter - z1) * (zcenter - z1));
		std::vector<double> result;

		double xU = minXYZ[0] + x_num * unitSize;
		double xD = minXYZ[0] + (x_num - 1) * unitSize;
		double yU = minXYZ[1] + y_num * unitSize;
		double yD = minXYZ[1] + (y_num - 1) * unitSize;
		double zU = minXYZ[2] + z_num * unitSize;
		double zD = minXYZ[2] + (z_num - 1) * unitSize;

		result.push_back(xU);
		result.push_back(xD);
		result.push_back(yU);
		result.push_back(yD);
		result.push_back(zU);
		result.push_back(zD);

		result.push_back(xcenter);
		result.push_back(ycenter);
		result.push_back(zcenter);
		result.push_back(radius);

		return result;

	}

	std::vector<double> BallRegion_ReturnBoxCenter_Center(int boxIndex) {

		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		double xcenter = (minXYZ[0] + (x_num - 1) * unitSize + minXYZ[0] + x_num * unitSize) / 2;
		double ycenter = (minXYZ[1] + (y_num - 1) * unitSize + minXYZ[1] + y_num * unitSize) / 2;
		double zcenter = (minXYZ[2] + (z_num - 1) * unitSize + minXYZ[2] + z_num * unitSize) / 2;

		std::vector<double> result;
		result.push_back(xcenter);
		result.push_back(ycenter);
		result.push_back(zcenter);
		return result;

	}

	double disPointPair(std::vector<double> p1, std::vector<double> p2) {

		double dis = (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
		dis = sqrt(dis);
		return dis;

	}

	int indexReturn(double data, std::vector<double> v) {

		for (int i = 0; i < v.size(); i++) {
			if (data == v[i]) {
				return i;
			}
		}
		return -1;
	}

private:

	int BallRegion_EstimateBoxScale(int pointcloudNum) {

		if (pointcloudNum < 10000) {
			return 10;
		}
		else if (pointcloudNum < 50000) {
			return 20;
		}
		else if (pointcloudNum < 100000) {
			return 30;
		}
		else if (pointcloudNum < 500000) {
			return 40;
		}
		else if (pointcloudNum < 1000000) {
			return 50;
		}
		else {
			return (int)pow((double)pointcloudNum / 8.0, 1.0 / 3.0);
		}
	}

};


