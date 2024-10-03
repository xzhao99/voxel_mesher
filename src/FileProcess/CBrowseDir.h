
#include "stdlib.h"
#include "direct.h"
#include "string.h"
#include "io.h"
#include "stdio.h" 
#include "iostream"
#include <vector>
#include <string>
#pragma once



class CBrowseDir
{
protected:
	//��ų�ʼĿ¼�ľ���·������'\'��β
	char m_szInitDir[_MAX_PATH];
	

public:
	//ȱʡ������
	std::vector<std::string> lifp;
	CBrowseDir();
	

	//���ó�ʼĿ¼Ϊdir���������false����ʾĿ¼������
	bool SetInitDir(const char *dir);

	//��ʼ������ʼĿ¼������Ŀ¼����filespecָ�����͵��ļ�
	//filespec����ʹ��ͨ��� * ?�����ܰ���·����
	//�������false����ʾ�������̱��û���ֹ
	bool BeginBrowse(const char *filespec);

protected:
	//����Ŀ¼dir����filespecָ�����ļ�
	//������Ŀ¼,���õ����ķ���
	//�������false,��ʾ��ֹ�����ļ�
	bool BrowseDir(const char *dir, const char *filespec);

	//����BrowseDirÿ�ҵ�һ���ļ�,�͵���ProcessFile
	//�����ļ�����Ϊ�������ݹ�ȥ
	//�������false,��ʾ��ֹ�����ļ�
	//�û����Ը�д�ú���,�����Լ��Ĵ�������
	virtual bool ProcessFile(const char *filename);

	//����BrowseDirÿ����һ��Ŀ¼,�͵���ProcessDir
	//�������ڴ�����Ŀ¼������һ��Ŀ¼����Ϊ�������ݹ�ȥ
	//������ڴ������ǳ�ʼĿ¼,��parentdir=NULL
	//�û����Ը�д�ú���,�����Լ��Ĵ�������
	//�����û�����������ͳ����Ŀ¼�ĸ���
	virtual void ProcessDir(const char *currentdir, const char *parentdir);
};
