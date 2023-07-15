#include "WriteSysDataToFile.h"

const char* file = "file.dat"; //保存数据的文件名称
/*! 系统配置结构体*/


void writeSysDeploymentToFile(const systemDeployData sys){
	ofstream fout;
	fout.open(file, ios_base::out | ios_base::app | ios_base::binary);
	if (fout.is_open()){
		fout.write((char*) &sys, sizeof systemDeployData);
	}

	fout.close();
}
/*
写坐标的顺序是
225WIFI坐标
27小基站坐标
90用户坐标
*/
void writeDoubleDataToFile(const double data){
	ofstream fout;
	fout.open(file, ios_base::out | ios_base::app | ios_base::binary);
	if (fout.is_open()){
		fout.write((char*)&data, sizeof(double));
	}

	fout.close();
}