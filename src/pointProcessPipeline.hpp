#pragma once
#include "LoadPointCloud.hpp"
#include "normalCompute.hpp"
#include "BallRegionCompute.h"

class pointProcessPipeline {

public:
	LoadPointCloud lpc;//load different kinds of files to achieve point cloud	
	NormalEstimation ne;//normal estimation for point cloud
	BallRegion br;//neibor structure construct
	BallRegion br_fix;//add points to fix neibor structure construct(unused)
	std::vector<int> fixBox;//
	int theorld;

public:

	void pointProcessPipeline_init(const std::string& filepath, bool brj) {
		//ne.test();
		std::cout << "pointProcessPipeline_init run!"<< std::endl;
		lpc.PointCloud_Load(filepath);//support obj, off, ply, xyz, txt
		//lpc.PointCloud_Load_Fast(filepath);
		
		ne.estimateNormal_init(lpc.FileNormal);

		//if (filepath.find(".obj") >= 0) {
			//char* pfile = new char[strlen(filepath.c_str()) + 1];
			//strcpy(pfile, filepath.c_str());
			//GLMmodel* pModel = glmReadOBJ(pfile);		
		//}		
		if (ne.normalLoad()) {
			std::cout << "pointProcessPipeline_init: normal file exist and the data are loaded." << std::endl;
		}
		/*
		else if (filepath.find(".obj") >= 0) {
			std::cout << "obj normal start." << std::endl;
			char* pfile = new char[strlen(filepath.c_str()) + 1];
			strcpy(pfile, filepath.c_str());
			GLMmodel* pModel = glmReadOBJ(pfile);	
			ne.estimateNormal(pModel);		
		}*/
		else {
			std::cout << "pcl normal start." << std::endl;
			//ne.estimateNormal(lpc.pointSet_uniform, 'j');	
			ne.estimateNormal_PCL_MP(lpc.pointSet_uniform);
		}			

		if (brj) {
			std::vector<std::vector<double>> pdata = lpc.pointSet_uniform;
			std::vector<std::vector<double>> ndata = ne.normalVector;
			std::vector<int> borderdata = lpc.indexBorder;
			br.BallRegion_init(pdata, ndata, borderdata);		
		}	

	}
		 
};
