#pragma once
#include "Method_AIVS_Remesh.hpp"
#include "BallRegionCompute.h"

#include <string>
#include <vector>
#include <time.h>

class GRemeshTool {

public:
	// 1:CGAL_PoissonRemesh 
	// 2:CVT
	void Remesh(BallRegion br, int simplificationNum, const std::string& fileName) {
		
		std::vector<std::vector<double>> point_sim;
		clock_t t1;
		clock_t t2;
		std::cout << "AIVS meshing start." << std::endl;
		AIVS_Reconstruct ar;
		ar.AIVS_Reconstruct_init(br);		
		
		t1 = clock();
		ar.AIVS_Reconstruct_Remesh(simplificationNum, fileName);
		t2 = clock();
		std::cout << "Running time:" << (t2 - t1) / 1000.0 << "s" << std::endl;
		std::cout << "AIVS Remesh end." << std::endl;	

	}

};
