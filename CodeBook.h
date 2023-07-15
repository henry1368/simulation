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
				ptr = new CodeBook;     //初始化，如若ptr为空，则开辟类空间
			}
			return ptr;
		}

	MatrixXcd W1[16];                          //Matrix<cd, 4, 1>
	MatrixXcd W2[16];                           //Matrix<cd, 4, 2>

    private:
	CodeBook();
	static CodeBook *ptr;       //本类的指针

};

#endif // !CODEBOOK_H

