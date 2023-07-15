#include "WriteSysDataToFile.h"

const char* file = "file.dat"; //�������ݵ��ļ�����
/*! ϵͳ���ýṹ��*/


void writeSysDeploymentToFile(const systemDeployData sys){
	ofstream fout;
	fout.open(file, ios_base::out | ios_base::app | ios_base::binary);
	if (fout.is_open()){
		fout.write((char*) &sys, sizeof systemDeployData);
	}

	fout.close();
}
/*
д�����˳����
225WIFI����
27С��վ����
90�û�����
*/
void writeDoubleDataToFile(const double data){
	ofstream fout;
	fout.open(file, ios_base::out | ios_base::app | ios_base::binary);
	if (fout.is_open()){
		fout.write((char*)&data, sizeof(double));
	}

	fout.close();
}