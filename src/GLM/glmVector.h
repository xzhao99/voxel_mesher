#pragma once
#include <GL/freeglut.h>
#include <vector>


std::vector<int> glmBox();
int glmNosePoint(GLMmodel* model);
std::vector<std::vector <int>> glmPointNeibor(GLMmodel* model);//���������ÿһ������ڵ�

int glmIndexPoint(GLMmodel* model,double x,double y,double z);

void glmTurnNorm2Trangular(GLMmodel* model);