#ifndef CODEBOOK_H
#define CODEBOOK_H

#include  <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef  complex<double> cd;


class CodeBook
{
    public:
	virtual ~CodeBook();

	static CodeBook*
		Init()
	{
			if (ptr == NULL)
			{
				ptr = new CodeBook;     //��ʼ��������ptrΪ�գ��򿪱���ռ�
			}
			return ptr;
		}

	MatrixXcd W1[16];                          //Matrix<cd, 4, 1>
	MatrixXcd W2[16];                           //Matrix<cd, 4, 2>

    private:
	CodeBook();
	static CodeBook *ptr;       //�����ָ��

};

#endif // !CODEBOOK_H

