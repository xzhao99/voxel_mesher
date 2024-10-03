#include "BallRegionCompute.h"


void BallRegion::BallRegion_init(const std::vector<std::vector<double>>& pointCloudData_input,
                                 const std::vector<std::vector<double>>& pointNormal_input, 
                                 const std::vector<int>& pointBorder_input) 
{
		boxNum = BallRegion_EstimateBoxScale(pointCloudData_input.size());
		pointNumEsti = 12;
		pointCloudData = pointCloudData_input;
		pointNormal = pointNormal_input;
		pointBorder = pointBorder_input;

		std::cout << "Init BallRegion" << std::endl;
		BallRegion_AchieveXYZ();//build boxes region
		//std::cout << "BallRegion_BoxInput()" << std::endl;
		BallRegion_BoxInput();//put point cloud into boxRegions

		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudData.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		
    // fills a PointCloud with random data
		for (int i = 0; i < pointCloudData.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudData[i][0];
			cloud->points[i].y = pointCloudData[i][1];
			cloud->points[i].z = pointCloudData[i][2];

		}
		pcl_kdtree.setInputCloud(cloud);
		std::cout << "Estimate Radius" << std::endl;
		BallRegion_EstimateRadius_KDTree();//radius estimation
		//std::cout << "Regular Normal" << std::endl;
		//int start = 0;
		//BallRegion_RegularNormal(start);
	}


void BallRegion::BallRegion_init_simpli(const std::vector<std::vector<double>>& pointCloudData_input,
                                        const std::vector<std::vector<double>>& pointNormal_input, 
                                        const BallRegion& br_input, 
                                        int pointNumEsti_input) 
{
		boxNum = br_input.boxNum;
		pointNumEsti = pointNumEsti_input;
		pointCloudData = pointCloudData_input;
		pointNormal = pointNormal_input;

		std::cout << "BallRegion_AchieveXYZ2()" << std::endl;
		BallRegion_AchieveXYZ2(br_input);//build boxes region//
		std::cout << "BallRegion_BoxInput()" << std::endl;
		BallRegion_BoxInput();//put point cloud into boxRegions		

		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudData.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudData.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudData[i][0];
			cloud->points[i].y = pointCloudData[i][1];
			cloud->points[i].z = pointCloudData[i][2];

		}
		pcl_kdtree.setInputCloud(cloud);
		std::cout << "BallRegion_EstimateRadius()" << std::endl;
		BallRegion_EstimateRadius_KDTree();
	}


  void BallRegion::BallRegion_BoxInput() 
  {
		//achieve the center of the box
		std::vector<std::vector<double>> squareBoxesCenterReal(squareBoxes.size());
		for (int i = 0; i < squareBoxesCenterReal.size(); i++) {
			squareBoxesCenterReal[i] = BallRegion_ReturnBoxCenter_Center(i);		
		}
		squareBoxesCReal = squareBoxesCenterReal;
		//achieve the minmum distance from points to center in box
		std::vector<double> boxCenterMin(squareBoxes.size(), 9999);
		std::vector<int> boxCenterMinIndex(squareBoxes.size(), -1);

		//achieve the x, y, z
		for (int i = 0; i < pointCloudData.size(); i++) {
			double xNum = (pointCloudData[i][0] - minXYZ[0]) / unitSize;
			double yNum = (pointCloudData[i][1] - minXYZ[1]) / unitSize;
			double zNum = (pointCloudData[i][2] - minXYZ[2]) / unitSize;
			int xNumIndex = xNum;
			int yNumIndex = yNum;
			int zNumIndex = zNum;
			if (xNumIndex < xNum || xNumIndex == 0) {
				xNumIndex++;
			}
			if (yNumIndex < yNum || yNumIndex == 0) {
				yNumIndex++;
			}
			if (zNumIndex < zNum || zNumIndex == 0) {
				zNumIndex++;
			}

			int index_Num = xNumIndex + XYZNumber[0] * (yNumIndex - 1) + (XYZNumber[0] * XYZNumber[1] * (zNumIndex - 1));

      double dx = squareBoxesCenterReal[index_Num][0] - pointCloudData[i][0];
      double dy = squareBoxesCenterReal[index_Num][1] - pointCloudData[i][1];
      double dz = squareBoxesCenterReal[index_Num][2] - pointCloudData[i][2];
			double disMin = sqrt(dx * dx + dy * dy + dz * dz);

			squareBoxes[index_Num].push_back(i);
			if (boxCenterMin[index_Num] > disMin) {
				boxCenterMin[index_Num] = disMin;
				//boxCenterMinIndex[index_Num] = squareBoxes[index_Num][squareBoxes[index_Num].size() - 1];
				boxCenterMinIndex[index_Num] = squareBoxes[index_Num].size() - 1;
			}
			pointCloudData_boxIndex.push_back(index_Num);
		}
		squareBoxesCenter = boxCenterMinIndex;
		
	}


  void BallRegion::BallRegion_AchieveXYZ() 
  {
		int minX = pointBorder[0];
		int minY = pointBorder[1];
		int minZ = pointBorder[2];
		int maxX = pointBorder[3];
		int maxY = pointBorder[4];
		int maxZ = pointBorder[5];
		double min_X = pointCloudData[minX][0];
		double min_Y = pointCloudData[minY][1];
		double min_Z = pointCloudData[minZ][2];
		double max_X = pointCloudData[maxX][0];
		double max_Y = pointCloudData[maxY][1];
		double max_Z = pointCloudData[maxZ][2];

		if (minXYZ.size() > 0) {
			minXYZ.clear();
		}

		minXYZ.push_back(min_X);
		minXYZ.push_back(min_Y);
		minXYZ.push_back(min_Z);

		double x_dis = abs(max_X - min_X);
		double y_dis = abs(max_Y - min_Y);
		double z_dis = abs(max_Z - min_Z);
		double large_dis = x_dis;
		if (large_dis < y_dis) {
			large_dis = y_dis;
		}
		if (large_dis < z_dis) {
			large_dis = z_dis;
		}
		unitSize = large_dis / double(boxNum);
		double numX = x_dis / unitSize;
		double numY = y_dis / unitSize;
		double numZ = z_dis / unitSize;
		int numXI = numX;
		int numYI = numY;
		int numZI = numZ;
		if (numX > (double)numXI) {
			numXI++;
		}
		if (numY > (double)numYI) {
			numYI++;
		}
		if (numZ > (double)numZI) {
			numZI++;
		}

		if (XYZNumber.size() > 0) {
			XYZNumber.clear();
		}

		XYZNumber.push_back(numXI);
		XYZNumber.push_back(numYI);
		XYZNumber.push_back(numZI);

		//init squareBoxes
		if (squareBoxes.size() == 0) {
			squareBoxes.clear();
		}

		int squareBoxesSize = numXI * numYI * numZI;
		for (int i = 0; i <= squareBoxesSize; i++) {
			std::vector<int> squareBoxesSize_i;
			squareBoxes.push_back(squareBoxesSize_i);
		}
	}

	void BallRegion::BallRegion_AchieveXYZ2(BallRegion br_input) 
  {
		minXYZ = br_input.minXYZ;
		unitSize = br_input.unitSize;
		XYZNumber = br_input.XYZNumber;

		int squareBoxesSize = XYZNumber[0] * XYZNumber[1] * XYZNumber[2];
		for (int i = 0; i <= squareBoxesSize; i++) {
			std::vector<int> squareBoxesSize_i;
			squareBoxes.push_back(squareBoxesSize_i);
		}

	}