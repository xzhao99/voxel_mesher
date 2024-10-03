#pragma once
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <iostream>


struct SModelData
{
	
	std::vector <float> vecFaceTriangles; // = face * 9
	std::vector <float> vecFaceTriangleColors; // = face * 9
	std::vector <float> vecNormals; // = face * 9
	int iTotalConnectedTriangles;
};

class CPLYLoader
{
public:
	
	std::vector<std::vector<double>> points;
	std::vector<std::vector<double>> normals;
	std::vector<std::vector<double>> colors;
	std::vector<std::vector<int>> vecFaceIndex;	
	CPLYLoader();
	int LoadModel(char* filename);
	void Draw();	

private:

	SModelData m_ModelData;
	float* mp_vertexXYZ;
	float* mp_vertexNorm;
	float* mp_vertexRGB;
	int m_totalConnectedQuads;
	int m_totalConnectedPoints;
	int m_totalFaces;	
};
