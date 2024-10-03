//#include <stdlib.h>
//#include <GL/freeglut.h>   // OpenGL GLUT Library Header
//#include "trackball.h"
#include <iostream>
#include <string>
#include <filesystem>

#include "pointProcessPipeline.hpp"
#include "GenerateRemesh.hpp"


int main()
{
	std::cout << "VoxelMesher starts ......" << std::endl;

	//input point cloud, obj, xyz, ply, txt, off
	//const std::filesystem::path pathFile{"./data/Bunny.obj"};
	const std::filesystem::path pathFile{"./data/test_point_cluster_mesh_0.ply"};
		
	//**Init point cloud**
	pointProcessPipeline ppp;	
	ppp.pointProcessPipeline_init(pathFile.string(), true);
	
	//**Remesh Process**
	const int simplificationNum = 10000;
	const std::string pathObj{"./test_output/bunny_out.ply"};
	GRemeshTool gt;
	gt.Remesh(ppp.br, simplificationNum, pathObj);
	
	std::cout << "VoxelMesher ended!" << std::endl;

	return 0;
}
