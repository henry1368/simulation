
#include "NetworkManager.h"
#include "AP.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "UserEquipment.h"
#include  "Building.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void Filter(double**X, int row, int col, double h[], int len);
double Gaussrand();

NetworkManager* NetworkManager::ptr=NULL;

NetworkManager::NetworkManager()
{
  m_BuildingContainer = new std::vector<Building*>;
  m_APContainer = new std::vector<AP*>;
  m_ENodeBContainer = new std::vector<ENodeB*>;
  m_UserEquipmentContainer = new std::vector<UserEquipment*>;
  m_HeNodeBContainer = new std::vector<HeNodeB*>;
}

NetworkManager::~NetworkManager()
{

   std::vector<AP*>::iterator iter1;
   for (iter1= m_APContainer->begin();
      iter1!= m_APContainer->end(); iter1++)
      {
        delete *iter1;
       }
   delete m_APContainer;



  std::vector<ENodeB*>::iterator iter2;
  for (iter2 = m_ENodeBContainer->begin();
	  iter2 != m_ENodeBContainer->end(); iter2++)
  {
	  delete *iter2;
  }
  delete m_ENodeBContainer;

  std::vector<HeNodeB*>::iterator iter3;
  for (iter3 = m_HeNodeBContainer->begin();
	  iter3 != m_HeNodeBContainer->end(); iter3++)
  {
	  delete *iter3;
  }
  delete m_HeNodeBContainer;

  std::vector<UserEquipment*>::iterator iter4;
  for (iter4 = m_UserEquipmentContainer->begin();
	  iter4 != m_UserEquipmentContainer->end(); iter4++)
  {
	  delete *iter4;
  }
  delete m_UserEquipmentContainer;
}



std::vector<AP*>*
NetworkManager::GetAPContainer ()
{
  return m_APContainer;
}


std::vector<ENodeB*>*
NetworkManager::GetENodeBContainer()
{
	return m_ENodeBContainer;
}


std::vector<UserEquipment*>*
NetworkManager::GetUserEquipmentContainer()
{
	return  m_UserEquipmentContainer;
}

std::vector<HeNodeB*>*
NetworkManager::GetHeNodeBContainer()
{
	return  m_HeNodeBContainer;
}


std::vector<Building*>*
NetworkManager::GetBuildingContainer()
{
	return  m_BuildingContainer;
}


//�ռ��˲�

/*

void 
NetworkManager::updateLinkInfo()
{


	for (int i = 0; i < nb_totalUe; i++)
	{
		UserEquipment* ue = m_UserEquipmentContainer->at(i);
		ue->lte_link_update();                      //�û�����λ��
	    if (ue->GetTargetEnbNode() != NULL)
		   cout << "�û�"<<ue->GetIDNetworkNode()<<",target_EnbNode:" << ue->GetTargetEnbNode()->GetIDNetworkNode() << endl;
	    else if (ue->GetTargetHenbNode() != NULL)
		   cout << "�û�" << ue->GetIDNetworkNode()<<",target_HenbNode:" << ue->GetTargetHenbNode()->GetIDNetworkNode() << endl;
	}


//����λ�ã�������λ�����¶��û���LSP��ز����ռ��˲��õ�����ski,  update_Ksi(m_scene);   //�����ŵ�����Ski��������Ӱ˥�����

}
  

void  NetworkManager::update_Ksi(int m_scene)  //����1.UMi 2.UMa 3.RMa 4.SMa
 {

	double Correlation_distance[8][5]= { 7,8,8,10,15,10,10,9,13,0,30,18,15,37,12,10,50,50,50,0,50,25,35,37,40,36,30,40,120,0,6,15,20,40,10,40,30,30,50,0};
	double delta[2][5];                  //��ͬ�����¶�Ӧ����ؾ������


    double sum_h_all[8][5]={
	                    7.5119, 8.5104, 8.5104, 10.5079, 15.4871,
		                10.5079, 10.5079, 9.5091, 13.5007, 100000,          //h(d)=exp(-(d)/delta) ��һ��(matlab���)
	                    29.4503,18.4370,15.4871,35.0557,12.5042,
	                    10.5079, 43.8023, 43.8023, 43.8023, 100000,
		                43.8023, 25.0545, 33.5208, 35.0557, 37.2596,
		                34.2949, 29.4503, 37.2596, 68.5659, 100000, 
		                6.5139, 15.4871, 20.3727, 37.2596, 10.5079,
		                37.2596, 29.4503, 29.4503, 43.8023, 100000
                       };
	double sum_h[2][5];                 //��ͬ�����¶�Ӧ��h��һ



//���վ
	for (int N = 0; N < nb_cell; N++)         //refer to D1.1.2 V1.2 WINNER II Channel Models��ITU-R M.2135-1
	{
		int LoS_UENums = 0;                                             //ע��Ϊ0����Ϊ1������
		int NLoS_UENums = 0;
		double Min_x = -100000;
		double Max_x = 100000;
		double Min_y = -100000;
		double Max_y = 100000;

		std::vector<int> LoS_UEs;               //���ڸ������Ŀ����û���
		std::vector<int> NLoS_UEs;
		std::vector<Position*> LoS_Pos;         //�����û�λ��
		std::vector<Position*> NLoS_Pos;
		int Gridn_Size_row;
		int Gridn_Size_col;

		double h[101][5] = { 0.0 };


		for (int i = 0; i < 5; i++)
		{
			delta[0][i] = Correlation_distance[2 * (m_scene - 1)][i];     //LOS
			delta[1][i] = Correlation_distance[2 * (m_scene - 1) + 1][i];   //NLOS       
		}

		for (int i = 0; i < 5; i++)
		{
			sum_h[0][i] = sum_h_all[2 * (m_scene - 1)][i];       //LOS
			sum_h[1][i] = sum_h_all[2 * (m_scene - 1) + 1][i];   //NLOS       
		}



		ENodeB* enb = m_ENodeBContainer->at(N);

		//�û�վ��Ӧ���Ӿ�ͷ��Ӿ���·��Ӧ���û���ź���λ������
		for (int j = 0; j < nb_totalUe; j++)
		{
			bool flag = m_UserEquipmentContainer->at(j)->GetLteChannel()->eNodeB_isLOS[j];
			if (flag)
			{
				LoS_UEs.push_back(j);
				LoS_Pos.push_back(m_UserEquipmentContainer->at(j)->Getposition());
				LoS_UENums++;
			}
			else
			{
				NLoS_UEs.push_back(j);
				NLoS_Pos.push_back(m_UserEquipmentContainer->at(j)->Getposition());
				NLoS_UENums++;
			}
		}

		//�Կ�����·����

		if (LoS_UENums > 1)
		{

			//step1:�õ�LOS�ռ��˲�ϵ�� 101��5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS�µ�h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[0][k])) / sum_h[0][k];
			}

			//step2:���ɾ�������
			for (int k = 0; k < LoS_UENums; k++)
			{
				double X = LoS_Pos.at(k)->GetPositionX();
				double Y = LoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //�ҵ��Ӿ��û��������С��X��Y����
					Max_x = X;
				if (Min_y < Y)
					Min_y = Y;
				if (Max_y > Y)
					Max_y = Y;
			}

			Gridn_Size_row = int(Max_y - Min_y + 2 * 100 + 1);
			Gridn_Size_col = int(Max_x - Min_x + 2 * 100 + 1);

			double ** Gridn = new double*[Gridn_Size_row];
			for (int i = 0; i < Gridn_Size_row; i++)
				Gridn[i] = new double[Gridn_Size_col];               //����ռ�


			//�������ڵ���������пռ��˲���ÿ���������˲�ϵ����h��ÿһ��
			for (int i = 0; i < 5; i++)
			{

				//���������
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //ÿ�������ϲ��������

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //ÿ���������˲�ϵ��

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //ǰ4��������Ҫ���пռ��ά�˲�
				}

				for (int k = 0; k < LoS_UENums; k++)
				{
					int  location_x = int(LoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //��k���û��������е�λ��
					int  location_y = int(LoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = LoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //����ռ�//�ͷſռ�
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (LoS_UENums==1)            //ֻ��һ��������·
		{
			int  id = LoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gaussrand();

		}



		//�Բ�������·����
		if (NLoS_UENums > 1)
		{

			//step1:�õ�LOS�ռ��˲�ϵ�� 101��5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS�µ�h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[1][k])) / sum_h[1][k];
			}

			//step2:���ɾ�������
			for (int k = 0; k < NLoS_UENums; k++)
			{
				double X = NLoS_Pos.at(k)->GetPositionX();
				double Y = NLoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //�ҵ����Ӿ��û��������С��X��Y����
					Max_x = X;
				if (Min_y < Y)
					Min_y = Y;
				if (Max_y > Y)
					Max_y = Y;
			}

			Gridn_Size_row = int(Max_y - Min_y + 2 * 100 + 1);
			Gridn_Size_col = int(Max_x - Min_x + 2 * 100 + 1);

			double ** Gridn = new double*[Gridn_Size_row];
			for (int i = 0; i < Gridn_Size_row; i++)
				Gridn[i] = new double[Gridn_Size_col];               //����ռ�


			//�������ڵ���������пռ��˲���ÿ���������˲�ϵ����h��ÿһ��
			for (int i = 0; i < 5; i++)
			{

				//���������
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //ÿ�������ϲ��������

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //ÿ���������˲�ϵ��

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //ǰ4��������Ҫ���пռ��ά�˲�
				}

				for (int k = 0; k < NLoS_UENums; k++)
				{
					int  location_x = int(NLoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //��k���û��������е�λ��
					int  location_y = int(NLoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = NLoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //����ռ�//�ͷſռ�
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (NLoS_UENums==1)        //ֻ��һ����������·
		{
			int  id = NLoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gaussrand();

		}
	}


//΢��վ��


	for (int N = 0; N < nb_totalHenb; N++)         //refer to D1.1.2 V1.2 WINNER II Channel Models��ITU-R M.2135-1
	{
		int LoS_UENums = 0;                                 //ע��Ϊ0����Ϊ1������
		int NLoS_UENums = 0;
		double Min_x = -100000;
		double Max_x = 100000;
		double Min_y = -100000;
		double Max_y = 100000;

		std::vector<int> LoS_UEs;               //���ڸ������Ŀ����û���
		std::vector<int> NLoS_UEs;
		std::vector<Position*> LoS_Pos;         //�����û�λ��
		std::vector<Position*> NLoS_Pos;
		int Gridn_Size_row;
		int Gridn_Size_col;

		double h[101][5] = { 0.0 };

		for (int i = 0; i < 5; i++)
		{
			delta[0][i] = Correlation_distance[0][i];     //LOS                //��ӦUMi
			delta[1][i] = Correlation_distance[1][i];     //NLOS       
		}

		for (int i = 0; i < 5; i++)
		{
			sum_h[0][i] = sum_h_all[0][i];       //LOS
			sum_h[1][i] = sum_h_all[1][i];       //NLOS       
		}




		HeNodeB* henb = m_HeNodeBContainer->at(N);

		//�û�վ��Ӧ���Ӿ�ͷ��Ӿ���·��Ӧ���û���ź���λ������
		for (int j = 0; j < nb_totalUe; j++)
		{
			bool flag = m_UserEquipmentContainer->at(j)->GetLteChannel()->HeNodeB_isLOS[j];
			if (flag)
			{
				LoS_UEs.push_back(j);
				LoS_Pos.push_back(m_UserEquipmentContainer->at(j)->Getposition());
				LoS_UENums++;
			}
			else
			{
				NLoS_UEs.push_back(j);
				NLoS_Pos.push_back(m_UserEquipmentContainer->at(j)->Getposition());
				NLoS_UENums++;
			}
		}

		//�Կ�����·����

		if (LoS_UENums > 1)
		{

			//step1:�õ�LOS�ռ��˲�ϵ�� 101��5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS�µ�h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[0][k])) / sum_h[0][k];
			}

			//step2:���ɾ�������
			for (int k = 0; k < LoS_UENums; k++)
			{
				double X = LoS_Pos.at(k)->GetPositionX();
				double Y = LoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //�ҵ��Ӿ��û��������С��X��Y����
					Max_x = X;
				if (Min_y < Y)
					Min_y = Y;
				if (Max_y > Y)
					Max_y = Y;
			}

			Gridn_Size_row = int(Max_y - Min_y + 2 * 100 + 1);
			Gridn_Size_col = int(Max_x - Min_x + 2 * 100 + 1);

			double ** Gridn = new double*[Gridn_Size_row];
			for (int i = 0; i < Gridn_Size_row; i++)
				Gridn[i] = new double[Gridn_Size_col];               //����ռ�


			//�������ڵ���������пռ��˲���ÿ���������˲�ϵ����h��ÿһ��
			for (int i = 0; i < 5; i++)
			{

				//���������
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //ÿ�������ϲ��������

				if (i <4 )
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //ÿ���������˲�ϵ��

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //ǰ4��������Ҫ���пռ��ά�˲�
				}

				for (int k = 0; k < LoS_UENums; k++)
				{
					int  location_x = int(LoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //��k���û��������е�λ��
					int  location_y = int(LoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = LoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //����ռ�//�ͷſռ�
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (LoS_UENums==1)             //ֻ��һ��������·
		{
			int  id = LoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gaussrand();

		}



		//�Բ�������·����
		if (NLoS_UENums > 1)
		{

			//step1:�õ�LOS�ռ��˲�ϵ�� 101��5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS�µ�h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[1][k])) / sum_h[1][k];
			}

			//step2:���ɾ�������
			for (int k = 0; k < NLoS_UENums; k++)
			{
				double X = NLoS_Pos.at(k)->GetPositionX();
				double Y = NLoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //�ҵ����Ӿ��û��������С��X��Y����
					Max_x = X;
				if (Min_y < Y)
					Min_y = Y;
				if (Max_y > Y)
					Max_y = Y;
			}

			Gridn_Size_row = int(Max_y - Min_y + 2 * 100 + 1);
			Gridn_Size_col = int(Max_x - Min_x + 2 * 100 + 1);

			double ** Gridn = new double*[Gridn_Size_row];
			for (int i = 0; i < Gridn_Size_row; i++)
				Gridn[i] = new double[Gridn_Size_col];               //����ռ�


			//�������ڵ���������пռ��˲���ÿ���������˲�ϵ����h��ÿһ��
			for (int i = 0; i < 5; i++)
			{

				//���������
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //ÿ�������ϲ��������

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //ÿ���������˲�ϵ��

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //ǰ4��������Ҫ���пռ��ά�˲�
				}

				for (int k = 0; k < NLoS_UENums; k++)
				{
					int  location_x = int(NLoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //��k���û��������е�λ��
					int  location_y = int(NLoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = NLoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //����ռ�//�ͷſռ�
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (NLoS_UENums==1)           //ֻ��һ����������·
		{
			int  id = NLoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gaussrand();

		}
	}


}


double Gaussrand()                    //������˹�����
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}


void Filter(double**X, int row, int col, double h[], int len )	 //�ú���ʵ����h�Զ�ά����X���к����������˲�   (����matlab��֤)
{
	double ** temp = new double*[row];    //�����˲����������temp��ά�����У������˲�����洢��X��
   
	for (int i = 0; i < row; i++)
	{
		temp[i] = new double[col];               //����ռ�
	}

//�����˲���
	for (int i = 0; i < row-1; i++)               //ÿһ���˲�
	{	
		for (int j = 0; j < col-1; j++)
		{
	        double sum = 0;
			int M = j; int N = 0;
			while (M >= 0 && N <= len - 1)
			{
				sum += h[N] * X[i][M];
				M--;
				N++;
			}
			temp[i][j] = sum;
		}
	}

//�����˲���
	for (int i = 0; i < col - 1; i++)           //ÿһ���˲�
	{
		for (int j = 0; j < row - 1; j++)
		{
			double sum = 0;
			int M = j; int N = 0;
			while (M >= 0 && N <= len - 1)
			{
				sum += h[N] * temp[M][i];
				M--;
				N++;
			}
			X[j][i] = sum;
		}
	}


	for (int i = 0; i < row; i++)
	{
		delete[] temp[i];              //�ͷſռ�
		temp[i] = NULL;
	}
	delete[] temp;
	temp = NULL;

}


*/



