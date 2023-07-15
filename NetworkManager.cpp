
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


//空间滤波

/*

void 
NetworkManager::updateLinkInfo()
{


	for (int i = 0; i < nb_totalUe; i++)
	{
		UserEquipment* ue = m_UserEquipmentContainer->at(i);
		ue->lte_link_update();                      //用户更新位置
	    if (ue->GetTargetEnbNode() != NULL)
		   cout << "用户"<<ue->GetIDNetworkNode()<<",target_EnbNode:" << ue->GetTargetEnbNode()->GetIDNetworkNode() << endl;
	    else if (ue->GetTargetHenbNode() != NULL)
		   cout << "用户" << ue->GetIDNetworkNode()<<",target_HenbNode:" << ue->GetTargetHenbNode()->GetIDNetworkNode() << endl;
	}


//更新位置，并根据位置重新对用户的LSP相关参数空间滤波得到参数ski,  update_Ksi(m_scene);   //更新信道参数Ski，用于阴影衰落计算

}
  

void  NetworkManager::update_Ksi(int m_scene)  //场景1.UMi 2.UMa 3.RMa 4.SMa
 {

	double Correlation_distance[8][5]= { 7,8,8,10,15,10,10,9,13,0,30,18,15,37,12,10,50,50,50,0,50,25,35,37,40,36,30,40,120,0,6,15,20,40,10,40,30,30,50,0};
	double delta[2][5];                  //不同场景下对应的相关距离参数


    double sum_h_all[8][5]={
	                    7.5119, 8.5104, 8.5104, 10.5079, 15.4871,
		                10.5079, 10.5079, 9.5091, 13.5007, 100000,          //h(d)=exp(-(d)/delta) 归一化(matlab求得)
	                    29.4503,18.4370,15.4871,35.0557,12.5042,
	                    10.5079, 43.8023, 43.8023, 43.8023, 100000,
		                43.8023, 25.0545, 33.5208, 35.0557, 37.2596,
		                34.2949, 29.4503, 37.2596, 68.5659, 100000, 
		                6.5139, 15.4871, 20.3727, 37.2596, 10.5079,
		                37.2596, 29.4503, 29.4503, 43.8023, 100000
                       };
	double sum_h[2][5];                 //不同场景下对应的h归一



//宏基站
	for (int N = 0; N < nb_cell; N++)         //refer to D1.1.2 V1.2 WINNER II Channel Models及ITU-R M.2135-1
	{
		int LoS_UENums = 0;                                             //注意为0或者为1的情形
		int NLoS_UENums = 0;
		double Min_x = -100000;
		double Max_x = 100000;
		double Min_y = -100000;
		double Max_y = 100000;

		std::vector<int> LoS_UEs;               //对于该扇区的可视用户号
		std::vector<int> NLoS_UEs;
		std::vector<Position*> LoS_Pos;         //可视用户位置
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

		//该基站对应的视距和非视距链路对应的用户编号和其位置坐标
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

		//对可视链路处理

		if (LoS_UENums > 1)
		{

			//step1:得到LOS空间滤波系数 101×5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS下的h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[0][k])) / sum_h[0][k];
			}

			//step2:生成均匀网格
			for (int k = 0; k < LoS_UENums; k++)
			{
				double X = LoS_Pos.at(k)->GetPositionX();
				double Y = LoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //找到视距用户中最大、最小的X、Y坐标
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
				Gridn[i] = new double[Gridn_Size_col];               //申请空间


			//对网格内的随机数进行空间滤波，每个参数的滤波系数是h的每一列
			for (int i = 0; i < 5; i++)
			{

				//产生随机数
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //每个网格上产生随机数

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //每个参数的滤波系数

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //前4个参数需要进行空间二维滤波
				}

				for (int k = 0; k < LoS_UENums; k++)
				{
					int  location_x = int(LoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //第k个用户在网格中的位置
					int  location_y = int(LoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = LoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //申请空间//释放空间
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (LoS_UENums==1)            //只有一条可视链路
		{
			int  id = LoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gaussrand();

		}



		//对不可视链路处理
		if (NLoS_UENums > 1)
		{

			//step1:得到LOS空间滤波系数 101×5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS下的h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[1][k])) / sum_h[1][k];
			}

			//step2:生成均匀网格
			for (int k = 0; k < NLoS_UENums; k++)
			{
				double X = NLoS_Pos.at(k)->GetPositionX();
				double Y = NLoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //找到非视距用户中最大、最小的X、Y坐标
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
				Gridn[i] = new double[Gridn_Size_col];               //申请空间


			//对网格内的随机数进行空间滤波，每个参数的滤波系数是h的每一列
			for (int i = 0; i < 5; i++)
			{

				//产生随机数
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //每个网格上产生随机数

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //每个参数的滤波系数

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //前4个参数需要进行空间二维滤波
				}

				for (int k = 0; k < NLoS_UENums; k++)
				{
					int  location_x = int(NLoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //第k个用户在网格中的位置
					int  location_y = int(NLoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = NLoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //申请空间//释放空间
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (NLoS_UENums==1)        //只有一条不可视链路
		{
			int  id = NLoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->eNodeB_Ksi[N][i] = Gaussrand();

		}
	}


//微基站：


	for (int N = 0; N < nb_totalHenb; N++)         //refer to D1.1.2 V1.2 WINNER II Channel Models及ITU-R M.2135-1
	{
		int LoS_UENums = 0;                                 //注意为0或者为1的情形
		int NLoS_UENums = 0;
		double Min_x = -100000;
		double Max_x = 100000;
		double Min_y = -100000;
		double Max_y = 100000;

		std::vector<int> LoS_UEs;               //对于该扇区的可视用户号
		std::vector<int> NLoS_UEs;
		std::vector<Position*> LoS_Pos;         //可视用户位置
		std::vector<Position*> NLoS_Pos;
		int Gridn_Size_row;
		int Gridn_Size_col;

		double h[101][5] = { 0.0 };

		for (int i = 0; i < 5; i++)
		{
			delta[0][i] = Correlation_distance[0][i];     //LOS                //对应UMi
			delta[1][i] = Correlation_distance[1][i];     //NLOS       
		}

		for (int i = 0; i < 5; i++)
		{
			sum_h[0][i] = sum_h_all[0][i];       //LOS
			sum_h[1][i] = sum_h_all[1][i];       //NLOS       
		}




		HeNodeB* henb = m_HeNodeBContainer->at(N);

		//该基站对应的视距和非视距链路对应的用户编号和其位置坐标
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

		//对可视链路处理

		if (LoS_UENums > 1)
		{

			//step1:得到LOS空间滤波系数 101×5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS下的h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[0][k])) / sum_h[0][k];
			}

			//step2:生成均匀网格
			for (int k = 0; k < LoS_UENums; k++)
			{
				double X = LoS_Pos.at(k)->GetPositionX();
				double Y = LoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //找到视距用户中最大、最小的X、Y坐标
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
				Gridn[i] = new double[Gridn_Size_col];               //申请空间


			//对网格内的随机数进行空间滤波，每个参数的滤波系数是h的每一列
			for (int i = 0; i < 5; i++)
			{

				//产生随机数
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //每个网格上产生随机数

				if (i <4 )
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //每个参数的滤波系数

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //前4个参数需要进行空间二维滤波
				}

				for (int k = 0; k < LoS_UENums; k++)
				{
					int  location_x = int(LoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //第k个用户在网格中的位置
					int  location_y = int(LoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = LoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //申请空间//释放空间
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (LoS_UENums==1)             //只有一条可视链路
		{
			int  id = LoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gaussrand();

		}



		//对不可视链路处理
		if (NLoS_UENums > 1)
		{

			//step1:得到LOS空间滤波系数 101×5
			for (int d = 0; d < 101; d++)
			{
				for (int k = 0; k < 5; k++)                                     //LOS下的h(d)=exp(-(d)/delta)
					h[d][k] = (exp(-d / delta[1][k])) / sum_h[1][k];
			}

			//step2:生成均匀网格
			for (int k = 0; k < NLoS_UENums; k++)
			{
				double X = NLoS_Pos.at(k)->GetPositionX();
				double Y = NLoS_Pos.at(k)->GetPositionY();
				if (Min_x < X)
					Min_x = X;
				if (Max_x > X)          //找到非视距用户中最大、最小的X、Y坐标
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
				Gridn[i] = new double[Gridn_Size_col];               //申请空间


			//对网格内的随机数进行空间滤波，每个参数的滤波系数是h的每一列
			for (int i = 0; i < 5; i++)
			{

				//产生随机数
				for (int m = 0; m < Gridn_Size_row; m++)
				for (int n = 0; n < Gridn_Size_col; n++)
					Gridn[m][n] = Gaussrand();                          //每个网格上产生随机数

				if (i <4)
				{
					double h_temp[101];
					for (int j = 0; j < 101; j++)
						h_temp[j] = h[j][i];                //每个参数的滤波系数

					Filter(Gridn, Gridn_Size_row, Gridn_Size_col, h_temp, 101);      //前4个参数需要进行空间二维滤波
				}

				for (int k = 0; k < NLoS_UENums; k++)
				{
					int  location_x = int(NLoS_Pos.at(k)->GetPositionX() - Min_x + 100 + 1);  //第k个用户在网格中的位置
					int  location_y = int(NLoS_Pos.at(k)->GetPositionY() - Min_y + 100 + 1);
					int  id = NLoS_UEs.at(k);
					m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gridn[location_x][location_y];
				}
			}

			for (int i = 0; i < Gridn_Size_row; i++)
			{
				delete[] Gridn[i];               //申请空间//释放空间
				Gridn[i] = NULL;
			}
			delete[] Gridn;
			Gridn = NULL;
		}

		else if (NLoS_UENums==1)           //只有一条不可视链路
		{
			int  id = NLoS_UEs.at(0);
			for (int i = 0; i < 5; i++)
				m_UserEquipmentContainer->at(id)->GetLteChannel()->HeNodeB_Ksi[N][i] = Gaussrand();

		}
	}


}


double Gaussrand()                    //产生高斯随机数
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


void Filter(double**X, int row, int col, double h[], int len )	 //该函数实现以h对二维数组X进行横向和纵向的滤波   (已用matlab验证)
{
	double ** temp = new double*[row];    //横向滤波结果储存在temp二维数组中，纵向滤波结果存储在X内
   
	for (int i = 0; i < row; i++)
	{
		temp[i] = new double[col];               //申请空间
	}

//横向滤波：
	for (int i = 0; i < row-1; i++)               //每一行滤波
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

//纵向滤波：
	for (int i = 0; i < col - 1; i++)           //每一列滤波
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
		delete[] temp[i];              //释放空间
		temp[i] = NULL;
	}
	delete[] temp;
	temp = NULL;

}


*/



