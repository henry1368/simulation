#include "CodeBook.h"

CodeBook* CodeBook::ptr = NULL;

CodeBook::CodeBook()
{
	//�����뱾
	for (int i = 0; i < 16; i++)
	{
	   W1[i].resize(4, 1);
	   W2[i].resize(4, 2);
	}

	W1[0] << cd(0.5, 0), cd(0.5, 0), cd(0.5, 0), cd(0.5, 0);

	W1[1] << cd(0.5, 0), cd(0, 0.5), cd(-0.5, 0), cd(0, -0.5);

	W1[2] << cd(0.5, 0), cd(-0.5, 0), cd(0.5, 0), cd(-0.5, 0);

	W1[3] << cd(0.5, 0), cd(0, -0.5), cd(-0.5, 0), cd(0, 0.5);

	W1[4] << cd(0.5, 0), cd(0.3536, 0.3536), cd(0, 0.5), cd(-0.3536, 0.3536);

	W1[5] << cd(0.5, 0), cd(-0.3536, 0.3536), cd(0, -0.5), cd(0.3536, 0.3536);

	W1[6] << cd(0.5, 0), cd(-0.3536, -0.3536), cd(0, 0.5), cd(0.3536, -0.3536);

	W1[7] << cd(0.5, 0), cd(0.3536, -0.3536), cd(0, -0.5), cd(-0.3536, -0.3536);

	W1[8] << cd(0.5, 0), cd(0.5, 0), cd(-0.5, 0), cd(-0.5, 0);

	W1[9] << cd(0.5, 0), cd(0, 0.5), cd(0.5, 0), cd(0, 0.5);

	W1[10] << cd(0.5, 0), cd(-0.5, 0), cd(-0.5, 0), cd(0.5, 0);

	W1[11] << cd(0.5, 0), cd(0, -0.5), cd(0.5, 0), cd(0, -0.5);

	W1[12] << cd(0.5, 0), cd(0.5, 0), cd(0.5, 0), cd(-0.5, 0);

	W1[13] << cd(0.5, 0), cd(0.5, 0), cd(-0.5, 0), cd(0.5, 0);

	W1[14] << cd(0.5, 0), cd(-0.5, 0), cd(0.5, 0), cd(0.5, 0);

	W1[15] << cd(0.5, 0), cd(-0.5, 0), cd(-0.5, 0), cd(-0.5, 0);



	//˫���뱾
	W2[0] << cd(0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(-0.5, 0),
		cd(0.5, 0), cd(-0.5, 0),
		cd(0.5, 0), cd(0.5, 0);

	W2[1] << cd(0.5, 0), cd(0, -0.5),
		cd(0, 0.5), cd(0.5, 0),
		cd(-0.5, 0), cd(0, -0.5),
		cd(0, -0.5), cd(0.5, 0);


	W2[2] << cd(0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(-0.5, 0);

	W2[3] << cd(0.5, 0), cd(0, 0.5),
		cd(0, -0.5), cd(0.5, 0),
		cd(-0.5, 0), cd(0, 0.5),
		cd(0, 0.5), cd(0.5, 0);

	W2[4] << cd(0.5, 0), cd(-0.3536, -0.3536),
		cd(0.3536, 0.3536), cd(0, 0.5),
		cd(0, 0.5), cd(-0.3536, 0.3536),
		cd(-0.3536, 0.3536), cd(0.5, 0);

	W2[5] << cd(0.5, 0), cd(0.3536, -0.3536),
		cd(-0.3536, 0.3536), cd(0, -0.5),
		cd(0, -0.5), cd(0.3536, 0.3536),
		cd(0.3536, 0.3536), cd(0.5, 0);

	W2[6] << cd(0.5, 0), cd(0, -0.5),
		cd(-0.3536, -0.3536), cd(0.3536, -0.3536),
		cd(0, 0.5), cd(0.5, 0),
		cd(0.3536, -0.3536), cd(0.3536, 0.3536);

	W2[7] << cd(0.5, 0), cd(0, 0.5),
		cd(0.3536, -0.3536), cd(-0.3536, -0.3536),
		cd(0, -0.5), cd(0.5, 0),
		cd(-0.3536, -0.3536), cd(-0.3536, 0.3536);

	W2[8] << cd(0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(0.5, 0);

	W2[9] << cd(0.5, 0), cd(0, -0.5),
		cd(0, 0.5), cd(-0.5, 0),
		cd(0.5, 0), cd(0, 0.5),
		cd(0, 0.5), cd(0.5, 0);

	W2[10] << cd(0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0);

	W2[11] << cd(0.5, 0), cd(0.5, 0),
		cd(0, -0.5), cd(0, 0.5),
		cd(0.5, 0), cd(0.5, 0),
		cd(0, -0.5), cd(0, 0.5);

	W2[12] << cd(0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(0.5, 0);

	W2[13] << cd(0.5, 0), cd(-0.5, 0),
		cd(0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0);

	W2[14] << cd(0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(0.5, 0),
		cd(0.5, 0), cd(-0.5, 0);

	W2[15] << cd(0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(0.5, 0),
		cd(-0.5, 0), cd(-0.5, 0),
		cd(-0.5, 0), cd(-0.5, 0);

	for (size_t i = 0; i < 16; i++)
		W2[i] = W2[i] / sqrt(2.0);
}


CodeBook::~CodeBook()
{
}

