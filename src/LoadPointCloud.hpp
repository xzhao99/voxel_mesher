#pragma once
//#include "View.h"
//#include <stdio.h>

// GLUT header
#include <stdlib.h>
#include <GL/freeglut.h>   // OpenGL GLUT Library Header
// Open file dialog
//#include "FileProcess/LoadFileDlg.h"
// The GLM code for loading and displying OBJ mesh file
#include "GLM/glm.h"
#include "GLM/glmVector.h"
// The trackball code for rotating the model
#include "trackball.h"
#include "PlyLoad.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>


class LoadPointCloud
{

public:
	
	std::vector<std::vector<double>> pointSet;
	std::vector<std::vector<double>> pointSet_uniform;
	std::vector<int> indexBorder;
	std::string FileNormal;

public:	

	void PointCloud_Load(std::string FileName) {

		int index = FileName.find_last_of(".");
		std::string objFile = FileName.substr(0, index) + ".obj";
		FileNormal = FileName.substr(0, index) + ".normal";
		if (pointSet.size() > 0) {
			pointSet.clear();
		}

		int txt_num = FileName.find(".txt");
		int xyz_num = FileName.find(".xyz");
		int ply_num = FileName.find(".ply");
		int obj_num = FileName.find(".obj");
		int off_num = FileName.find(".off");

		if (txt_num > 0|| xyz_num > 0) {			
			PointCloud_txt2obj(FileName);
		}

		if (ply_num > 0) {	
			PointCloud_ply2obj(FileName, objFile);		
		}

		if (obj_num > 0) {
			char* pfile = new char[strlen(FileName.c_str()) + 1];
			strcpy(pfile, FileName.c_str());
			GLMmodel* pModel = glmReadOBJ(pfile);
			for (int i = 1; i <= pModel->numvertices; i++) {
				double x_i = pModel->vertices[3 * i];
				double y_i = pModel->vertices[3 * i + 1];
				double z_i = pModel->vertices[3 * i + 2];
				std::vector<double> listV;
				listV.push_back(x_i);
				listV.push_back(y_i);
				listV.push_back(z_i);
				pointSet.push_back(listV);
			}
		}

		if (off_num > 0) {
			PointCloud_off2obj(FileName, objFile);		
		}

		PointCloud_Uniform();
	}

	void PointCloud_Load(GLMmodel* pModel) {

		if (pointSet.size() > 0) {
			pointSet.clear();		
		}

		for (int i = 1; i <= pModel->numvertices; i++) {

			double x_i = pModel->vertices[3 * i];
			double y_i = pModel->vertices[3 * i + 1];
			double z_i = pModel->vertices[3 * i + 2];	
			std::vector<double> listV;
			listV.push_back(x_i);
			listV.push_back(y_i);
			listV.push_back(z_i);
			pointSet.push_back(listV);		
		}

		PointCloud_Uniform();
	}

private:

	void PointCloud_txt2obj(std::string path) {

		std::fstream out2;
		out2.open(path, std::ios::in);
		int lineNum = 0;
		while (1) {
			char line[255];
			out2.getline(line, sizeof(line));	
			lineNum++;
			if (out2.eof())
			{
				break;
			}		
		}

		out2.clear();
		out2.seekg(0);

		for (int i = 0; i < lineNum; i++) {
			double x_i;
			double y_i;
			double z_i;
			out2 >> x_i;
			out2 >> y_i;
			out2 >> z_i;
			std::vector<double> vec_list;
			vec_list.push_back(x_i);
			vec_list.push_back(y_i);
			vec_list.push_back(z_i);
			pointSet.push_back(vec_list);
			std::string st;
			std::getline(out2,st);		
		}

		out2.close();
	
	}

	void PointCloud_off2obj(std::string path, std::string path2) {		

		std::vector<std::vector<double>> points;
		std::vector<std::vector<int>> faces;

		std::fstream out2;
		out2.open(path, std::ios::in);
		std::string offName;
		out2 >> offName;
		int n_off, f_off, b_off;
		out2 >> n_off >> f_off >> b_off;

		for (int i = 0; i < n_off; i++) {
			std::vector<double> points_i;
			double x_i;
			double y_i;
			double z_i;
			out2 >> x_i >> y_i >> z_i;
			points_i.push_back(x_i);
			points_i.push_back(y_i);
			points_i.push_back(z_i);
			points.push_back(points_i);
		}

		for (int i = 0; i < f_off; i++) {
			//cout << "," << i;
			std::vector<int> face_i;
			int b_num;
			int b1;
			int b2;
			int b3;
			out2 >> b_num >> b1 >> b2 >> b3;
			face_i.push_back(b1 + 1);
			face_i.push_back(b2 + 1);
			face_i.push_back(b3 + 1);
			faces.push_back(face_i);
		}

		pointSet.insert(pointSet.end(), points.begin(), points.end());

		std::ifstream fin(path2);
		if (fin)
		{
			fin.close();
			return;
		}
		else {
			fin.close();
		}

		out2.close();
		std::ofstream f1(path2, std::ios::app);

		for (int i = 0; i < n_off; i++) {
			f1 << "v" << " " << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << std::endl;
		}
		for (int i = 0; i < f_off; i++) {
			f1 << "f" << " " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << " " << std::endl;
		}
		f1 << std::endl;
		f1.close();
	}

	void PointCloud_obj2off(std::string path, std::string path2) {
		
		std::vector<std::vector<double>> points;
		std::vector<std::vector<int>> faces;
		
		int n_off, f_off, b_off;
		char* path_i_char = new char[strlen(path.c_str()) + 1];
		strcpy(path_i_char, path.c_str());
		GLMmodel* pModel = glmReadOBJ(path_i_char);
		n_off = pModel->numvertices;
		f_off = pModel->numtriangles;
		b_off = 0;

		pointSet.insert(pointSet.end(), points.begin(), points.end());

		//
		std::ifstream fin(path2);
		if (fin)
		{
			fin.close();
			return;
		}
		else {
			fin.close();
		}

		std::ofstream f1(path2, std::ios::app);
		f1 << "OFF" << std::endl;
		f1 << n_off << " " << f_off << " " << 0 << std::endl;
		for (int i = 1; i <= n_off; i++) {
			f1 << pModel->vertices[3 * i] << " " << pModel->vertices[3 * i + 1] << " " << pModel->vertices[3 * i + 2] << " " << std::endl;
		}
		for (int i = 0; i < f_off; i++) {
			f1 << 3 << " " << pModel->triangles[i].vindices[0] - 1 << " " << pModel->triangles[i].vindices[1] - 1 << " "
				<< pModel->triangles[i].vindices[2] - 1 << " " << std::endl;
		}
		f1 << std::endl;
		f1.close();
	}

	void PointCloud_off2off(std::string path, std::string path2) {

		std::ifstream fin(path2);
		if (fin)
		{
			fin.close();
			return;
		}
		else {
			fin.close();
		}

		std::vector<std::vector<double>> points;
		std::vector<std::vector<int>> faces;

		std::fstream out2;
		out2.open(path, std::ios::in);
		std::string offName;
		out2 >> offName;
		int n_off, f_off, b_off;
		out2 >> n_off >> f_off >> b_off;

		for (int i = 0; i < n_off; i++) {
			std::vector<double> points_i;
			double x_i;
			double y_i;
			double z_i;
			out2 >> x_i >> y_i >> z_i;
			points_i.push_back(x_i);
			points_i.push_back(y_i);
			points_i.push_back(z_i);
			points.push_back(points_i);
		}

		for (int i = 0; i < f_off; i++) {

			std::vector<int> face_i;
			//int b_num;
			int b1;
			int b2;
			int b3;
			out2 >> b1 >> b2 >> b3;
			face_i.push_back(b1 - 1);
			face_i.push_back(b2 - 1);
			face_i.push_back(b3 - 1);
			faces.push_back(face_i);
		}

		out2.close();

		std::ofstream f1(path2, std::ios::app);
		f1 << "OFF" << std::endl;
		f1 << n_off << " " << f_off << " " << 0 << std::endl;
		for (int i = 0; i < n_off; i++) {
			f1 << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << std::endl;
		}
		for (int i = 0; i < f_off; i++) {
			f1 << 3 << " " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << " " << std::endl;
		}
		f1 << std::endl;
		f1.close();


	}

	void PointCloud_ply2obj(std::string path, std::string path2) {		
		
		CPLYLoader plyLoader;
		char* p = new char[strlen(path.c_str()) + 1];
		strcpy(p, path.c_str());
		plyLoader.LoadModel(p); 		

		std::vector<std::vector<double>> points = plyLoader.points;
		std::vector<std::vector<int>> faces = plyLoader.vecFaceIndex;

		pointSet.insert(pointSet.end(), points.begin(), points.end());
		
		std::ifstream fin(path2);
		if (fin)
		{
			fin.close();
			return;
		}
		else {
			fin.close();
		}

		std::ofstream f1(path2, std::ios::app);

		for (int i = 0; i < points.size(); i++) {
			f1 << "v" << " " << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << std::endl;
		}
		for (int i = 0; i < faces.size(); i++) {
			f1 << "f" << " " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << " " << std::endl;
		}
		f1 << std::endl;
		f1.close();		
	}
		
	void PointCloud_Uniform() {

		double maxx, maxy, maxz, minx, miny, minz;
		maxx = minx = pointSet[0][0];
		maxy = miny = pointSet[0][1];
		maxz = minz = pointSet[0][2];

		int indexMaxX = -1;
		int indexMaxY = -1;
		int indexMaxZ = -1;
		int indexMinX = -1;
		int indexMinY = -1;
		int indexMinZ = -1;

		for (int i = 0; i < pointSet.size(); i++) {
			double xi = pointSet[i][0];
			double yi = pointSet[i][1];
			double zi = pointSet[i][2];
			if (xi < minx) {
				minx = xi;
				indexMinX = i;
			}
			if (xi > maxx) {
				maxx = xi;
				indexMaxX = i;
			}
			if (yi < miny) {
				miny = yi;
				indexMinY = i;
			}
			if (yi > maxy) {
				maxy = yi;
				indexMaxY = i;
			}
			if (zi < minz) {
				minz = zi;
				indexMinZ = i;
			}
			if (zi > maxz) {
				maxz = zi;
				indexMaxZ = i;
			}
		}
		// domain size and center
		double w = maxx - minx;
		double h = maxy - miny;
		double d = maxz - minz;
		double cx = (maxx + minx) / 2.0;
		double cy = (maxy + miny) / 2.0;
		double cz = (maxz + minz) / 2.0;

		double maxwhd = w;
		if (w < h) {
			maxwhd = h;
		}
		if (maxwhd < d) {
			maxwhd = d;
		}

		const double scale = 2.0 / maxwhd;
		std::cout << "===>Test in " << __func__ << ": scale factor = " << scale << std::endl;
		
		/* translate around center then scale */
		for (int i = 0; i < pointSet.size(); i++) {
			pointSet_uniform.push_back(pointSet[i]);
			pointSet_uniform[i][0] -= cx;
			pointSet_uniform[i][1] -= cy;
			pointSet_uniform[i][2] -= cz;
			pointSet_uniform[i][0] *= scale;
			pointSet_uniform[i][1] *= scale;
			pointSet_uniform[i][2] *= scale;
		}

		indexBorder.push_back(indexMinX);
		indexBorder.push_back(indexMinY);
		indexBorder.push_back(indexMinZ);
		indexBorder.push_back(indexMaxX);
		indexBorder.push_back(indexMaxY);
		indexBorder.push_back(indexMaxZ);

	}
};
