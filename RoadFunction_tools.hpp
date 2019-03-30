#ifndef _ROADFUNCTION_TOOLS_HPP_
#define _ROADFUNCTION_TOOLS_HPP_

#include <fstream>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <math.h>

#define min_compare(a, b)	(((a) < (b)) ? (a) : (b))
#define max_compare(a, b)	(((a) > (b)) ? (a) : (b))

//#define SHOW_IMFOR

//#define FALLOUT_CARNUM
//#define FAST_CARFIRST

//#define NORMAL_STARTLIMIT
//#define STRICT_STARTLIMIT
#define ONEBYONE_STARTLIMIT

#define LIMIT_NUM 16//2  3-17(map2)
#define ALPHA 20	//20  25(deadlock)   15(long)   17(best,lock)          2(worse)- 10(1843) -(worse)15   10-LIMIT_NUM:2-deadlock  5-LIMIT_NUM:2-deadlock(flyaway)
#define BETA  1	//1

using namespace std;

typedef float* Matrix_f2v2;
typedef int* Matrix_i2v2;

extern const float INF;
extern int global_carPath_row;
extern int global_roadPath_row;
extern int global_crossPath_row;
extern std::vector<int> lockRoad;

int Car_nextCrossCheck(int carID, int car_place, std::vector<Matrix_i2v2> roadMatrix_group, int (*roadPath_data)[7], int (*crossPath_data)[5], int car_startplace)
{
	if(car_place == -1)
	{
		for(int search = 0; search < global_crossPath_row; search++)
		{
			if(crossPath_data[search][0] == car_startplace)
				return search;
		}
	}
	else
	{
		Matrix_i2v2 road_position = roadMatrix_group[car_place];
		if(roadPath_data[car_place][6] == 1)//Two way choose one
		{
			for(int i = 0; i < (roadPath_data[car_place][3]*(roadPath_data[car_place][6]+1)); i++)
			{
				for(int j = 0; j < (roadPath_data[car_place][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[car_place][1]) + j) == carID)
					{
						if(i >= roadPath_data[car_place][3])//ahead
						{
							for(int search = 0; search < global_crossPath_row; search++)
							{
								if(crossPath_data[search][0] == roadPath_data[car_place][5])
									return search;
							}
						}
						else if(i < roadPath_data[car_place][3])//back
						{
							for(int search = 0; search < global_crossPath_row; search++)
							{
								if(crossPath_data[search][0] == roadPath_data[car_place][4])
									return search;
							}
						}
					}
				}
			}
		}
		else if(roadPath_data[car_place][6] == 0)//One way
		{
			for(int search = 0; search < global_crossPath_row; search++)
			{
				if(crossPath_data[search][0] == roadPath_data[car_place][5])
					return search;
			}
		}
	}
	return -1;
}

int RoadWeight_Changeable(int (*Q)[2], std::vector<Matrix_i2v2> roadMatrix_group, Matrix_f2v2 map, int map_row,int map_col, int (*roadPath_data)[7], int *carPath_data_single, int (*crossPath_data)[5])
{
	float carnum_inroad_forward[global_roadPath_row] = {0};//Wait for improve
	float carnum_inroad_backward[global_roadPath_row] = {0};//Wait for improve
	
	for(int i = 0;i < global_roadPath_row;i++)
	{
		carnum_inroad_forward[i] = 0;//Wait for improve
		carnum_inroad_backward[i] = 0;//Wait for improve
		Matrix_i2v2 road_position = roadMatrix_group[i];
		for(int j = 0;j < (roadPath_data[i][3]*(roadPath_data[i][6]+1));j++)
		{
			for(int k = 0;k < roadPath_data[i][1];k++)
			{
				if(*(road_position + j*(roadPath_data[i][1]) + k) != 0)
				{
					if(roadPath_data[i][6] == 1)
					{
						if(j < roadPath_data[i][3])//0-n-1(first):back n(first)-(2n-1):ahead
							carnum_inroad_backward[i]++;
						else
							carnum_inroad_forward[i]++;
					}
					else if(roadPath_data[i][6] == 0)
					{
						if(j < roadPath_data[i][3])//0(first)-n-1:ahead
							carnum_inroad_forward[i]++;
					}
					
				}
			}
		}
		//std::cout << "here1" << std::endl;
		/*if(carPath_data_single[3] >= roadPath_data[i][2])
			*(map + (roadPath_data[i][4]-1)*map_col + (roadPath_data[i][5]-1)) = (carnum_inroad_forward[i]+1)*((float)(roadPath_data[i][1]))/((float)(roadPath_data[i][2]));
		else
			*(map + (roadPath_data[i][4]-1)*map_col + (roadPath_data[i][5]-1)) = (carnum_inroad_forward[i]+1)*((float)(roadPath_data[i][1]))/((float)(carPath_data_single[3]));
		if(roadPath_data[i][6])
		{
			if(carPath_data_single[3] >= roadPath_data[i][2])
				*(map + (roadPath_data[i][5]-1)*map_col + (roadPath_data[i][4]-1)) = (carnum_inroad_backward[i]+1)*((float)(roadPath_data[i][1]))/((float)(roadPath_data[i][2]));
			else
				*(map + (roadPath_data[i][5]-1)*map_col + (roadPath_data[i][4]-1)) = (carnum_inroad_backward[i]+1)*((float)(roadPath_data[i][1]))/((float)(carPath_data_single[3]));
		}*/
		int map_start = 0;
		int map_end = 0;
		for(int search = 0; search < global_crossPath_row; search++)
		{
			if(crossPath_data[search][0] == roadPath_data[i][4])
			{
				map_start = search;
				break;
			}
		}
		for(int search = 0; search < global_crossPath_row; search++)
		{
			if(crossPath_data[search][0] == roadPath_data[i][5])
			{
				map_end = search;
				break;
			}
		}
		if(carPath_data_single[3] >= roadPath_data[i][2])//(carnum_inroad_forward[i]+1)*
			*(map + map_start*map_col + map_end) = ((float)(roadPath_data[i][1]))/((float)(roadPath_data[i][2]))*(1 + ALPHA*(float)pow(max_compare(Q[i][0], carnum_inroad_forward[i])/(((float)(roadPath_data[i][3]))*((float)(roadPath_data[i][2]))), BETA));
		else
			*(map + map_start*map_col + map_end) = ((float)(roadPath_data[i][1]))/((float)(carPath_data_single[3]))*(1 + ALPHA*(float)pow(max_compare(Q[i][0], carnum_inroad_forward[i])/(((float)(roadPath_data[i][3]))*((float)(roadPath_data[i][2]))), BETA));
		if(roadPath_data[i][6])
		{
			if(carPath_data_single[3] >= roadPath_data[i][2])//(carnum_inroad_backward[i]+1)*
				*(map + map_end*map_col + map_start) = ((float)(roadPath_data[i][1]))/((float)(roadPath_data[i][2]))*(1 + ALPHA*(float)pow(max_compare(Q[i][1], carnum_inroad_backward[i])/(((float)(roadPath_data[i][3]))*((float)(roadPath_data[i][2]))), BETA));
			else
				*(map + map_end*map_col + map_start) = ((float)(roadPath_data[i][1]))/((float)(carPath_data_single[3]))*(1 + ALPHA*(float)pow(max_compare(Q[i][1], carnum_inroad_backward[i])/(((float)(roadPath_data[i][3]))*((float)(roadPath_data[i][2]))), BETA));
		}
	}
	//std::cout << "here2" << std::endl;
	/*for(int i = 0;i < map_row;i++)
		for(int j = 0;j < map_col;j++)
			if(i == j)
				*(map + i*map_col + j) = 0;*/
				
	//fallout of carnum_inroad
/*#ifdef FALLOUT_CARNUM
	for(int i = 0;i < global_roadPath_row;i++)
	{
		int map_start = 0;
		int map_end = 0;
		for(int search = 0; search < global_crossPath_row; search++)
		{
			if(crossPath_data[search][0] == roadPath_data[i][4])
			{
				map_start = search;
				break;
			}
		}
		for(int search = 0; search < global_crossPath_row; search++)
		{
			if(crossPath_data[search][0] == roadPath_data[i][5])
			{
				map_end = search;
				break;
			}
		}
		
		int connected_num = 0;
		for(int j = 1; j < 5; j++)
		{
			if(crossPath_data[map_end][j] != -1)
				connected_num++;
		}
		//std::cout << "connected_num: " << connected_num << std::endl;
		for(int j = 1; j < 5; j++)
		{
			if(crossPath_data[map_end][j] != -1)
			{
				for(int k=0; k < global_roadPath_row;k++)
				{
					if(roadPath_data[k][0] == crossPath_data[map_end][j])
					{
						int k_start = 0;
						int k_end = 0;
						for(int search_k = 0; search_k < global_crossPath_row; search_k++)
						{
							if(crossPath_data[search_k][0] == roadPath_data[k][4])
							{
								k_start = search_k;
								break;
							}
						}
						for(int search_k = 0; search_k < global_crossPath_row; search_k++)
						{
							if(crossPath_data[search_k][0] == roadPath_data[k][5])
							{
								k_end = search_k;
								break;
							}
						}
						if(roadPath_data[k][4] == roadPath_data[i][5])//back to ahead
						{
							*(map + k_start*map_col + k_end) += carnum_inroad_forward[i]/(connected_num-1);
							break;
						}
						else if(roadPath_data[k][5] == roadPath_data[i][5])//back to back
						{
							if(roadPath_data[k][6] == 1)
								*(map + k_end*map_col + k_start) += carnum_inroad_backward[i]/(connected_num-1);
							break;
						}
					}
				}
			}
		}
		
		if(roadPath_data[i][6])
		{
			int connected_num = 0;
			for(int j = 1; j < 5; j++)
			{
				if(crossPath_data[roadPath_data[i][4]-1][j] != -1)
					connected_num++;
			}
			for(int j = 1; j < 5; j++)
			{
				if(crossPath_data[roadPath_data[i][4]-1][j] != -1)
				{
					for(int k=0; k < global_roadPath_row;k++)
					{
						if(roadPath_data[k][0] == crossPath_data[roadPath_data[i][5]-1][j])
						{
							*(map + (roadPath_data[k][5]-1)*map_col + (roadPath_data[k][4]-1)) += carnum_inroad_backward[i]/(connected_num-1);
						}
					}
				}
			}
		}
	}
#endif
	*/
	return 1;
}

//Maybe block by other or wait on cross(change to road needed waiting) and not entering finally
int isEnteringCross(int carOrder, int *carPath_data, int car_place, std::vector<Matrix_i2v2> roadMatrix_group, int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	if(car_place == -1 || car_place == -2)
	{
		//std::cout << "car not start OR arrived." << std::endl;
	}
	else
	{
		Matrix_i2v2 road_position = roadMatrix_group[car_place];
		if(roadPath_data[car_place][6] == 1)//Two way
		{
			for(int i = 0; i < (roadPath_data[car_place][3]*(roadPath_data[car_place][6]+1)); i++)
			{
				for(int j = 0; j < (roadPath_data[car_place][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[car_place][1]) + j) == carOrder+1)
					{
						if(i < roadPath_data[car_place][3])//back
						{
							if(j < min_compare(carPath_data[3], roadPath_data[car_place][2]))
								return 1;//Entering, but new position is unsure(may stay in old road)
							else
								return 0;//Not Entering
						}
						else//ahead
						{
							if(j+1 > roadPath_data[car_place][1] - min_compare(carPath_data[3], roadPath_data[car_place][2]))
								return 1;//Entering, but new position is unsure(may stay in old road)
							else
								return 0;//Not Entering
						}
					}
				}
			}
		}
		else if(roadPath_data[car_place][6] == 0)//One way
		{
			for(int i = 0; i < roadPath_data[car_place][3]; i++)
			{
				for(int j = 0; j < (roadPath_data[car_place][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[car_place][1]) + j) == carOrder+1)
					{
						if(j+1 > roadPath_data[car_place][1] - min_compare(carPath_data[3], roadPath_data[car_place][2]))
							return 1;//Entering, but new position is unsure(may stay in old road)
						else
							return 0;//Not Entering
					}
				}
			}
		}
	}
	//std::cout << "isEnteringCross() Find Nothing." << std::endl;
	return 0;
}

int CheckCarNum(Matrix_i2v2 road_position, int next_move, int (*roadPath_data)[7], int lane_position)
{
	int car_num = 0;
	if(roadPath_data[next_move][6] == 1)//Two way choose one
	{
		if(lane_position == 0)
		{
			for(int i = roadPath_data[next_move][3]; i < (roadPath_data[next_move][3]*(roadPath_data[next_move][6]+1)); i++)
			{
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
						car_num++;
				}
			}
		}
		else if(lane_position == 1)
		{
			for(int i = 0; i < roadPath_data[next_move][3]; i++)
			{
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
						car_num++;
				}
			}
		}
	}
	else if(roadPath_data[next_move][6] == 0)//One way
	{
		if(lane_position == 0)
		{
			for(int i = 0; i < roadPath_data[next_move][3]; i++)
			{
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
						car_num++;
				}
			}
		}
		//else
			//std::cout << "Next_move is one way road in CheckCarNum()." << std::endl;
	}
	return car_num;
}

int CheckCarNum_InUseLandAverage(Matrix_i2v2 road_position, int next_move, int (*roadPath_data)[7], int lane_position)
{
	int car_num = 0;
	int land_num = 0;
	int land_inUse = 0;
	if(roadPath_data[next_move][6] == 1)//Two way choose one
	{
		if(lane_position == 0)
		{
			for(int i = roadPath_data[next_move][3]; i < (roadPath_data[next_move][3]*(roadPath_data[next_move][6]+1)); i++)
			{
				land_inUse = 0;
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
					{
						car_num++;
						land_inUse = 1;
					}
				}
				if(land_inUse)
					land_num++;
			}
		}
		else if(lane_position == 1)
		{
			for(int i = 0; i < roadPath_data[next_move][3]; i++)
			{
				land_inUse = 0;
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
					{
						car_num++;
						land_inUse = 1;
					}
				}
				if(land_inUse)
					land_num++;
			}
		}
	}
	else if(roadPath_data[next_move][6] == 0)//One way
	{
		if(lane_position == 0)
		{
			for(int i = 0; i < roadPath_data[next_move][3]; i++)
			{
				land_inUse = 0;
				for(int j = 0; j < (roadPath_data[next_move][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[next_move][1]) + j) != 0)
					{
						car_num++;
						land_inUse = 1;
					}
				}
				if(land_inUse)
					land_num++;
			}
		}
		//else
			//std::cout << "Next_move is one way road in CheckCarNum()." << std::endl;
	}
	
	if(land_num == 0)
		return 0;
	else
		return car_num/land_num;
}

//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
void MarkFunction(int *marker, std::vector<int*> roadMatrix_group, int *next_move, std::vector<int> car_speedChart, int *car_place, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	for(int road_check = 0;road_check < global_roadPath_row;road_check++)
	{
		Matrix_i2v2 road_position = roadMatrix_group[road_check];
		if (roadPath_data[road_check][6] == 1)//Two way find one
		{
			for(int i_run = roadPath_data[road_check][3]; i_run < (roadPath_data[road_check][3]*(roadPath_data[road_check][6]+1)); i_run++)
			{
				int former_state = -3;//first car must be wait car or stop car
				for(int j_run = roadPath_data[road_check][1]-1; j_run >= 0; j_run--)
				{
					//Run from former to latter
					if(*(road_position + i_run*(roadPath_data[road_check][1]) + j_run) != 0)
					{
						int k = *(road_position + i_run*(roadPath_data[road_check][1]) + j_run)-1;
						
						if(former_state == -3)//Get first car mark
						{
							if(isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//GO!!
							{
								marker[k] = 1;
								former_state = 1;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run + min_compare(roadPath_data[car_place[k]][2], carPath_data[k][3])) = k+1;
							}
						}
						else if(former_state == 0)//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[road_check][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h) != 0)
								{
									isblock = 1;//Something block
									break;
								}
							}
							if (isblock)
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						else if(former_state == 1)
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[road_check][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h) != 0)
								{
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h - 1) = k+1;
									isblock = 1;//Something block
									marker[k] = 1;
									former_state = 1;
									break;
								}
							}
							if (!isblock)//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						//else
							//std::cout << "Error former_state: " << former_state << std::endl;

					}
				}
			}
			
			for(int i_run = 0; i_run < roadPath_data[road_check][3]; i_run++)
			{
				int former_state = -3;//first car must be wait car or stop car
				for(int j_run = 0; j_run < roadPath_data[road_check][1]; j_run++)
				{
					//Run from former to latter
					if(*(road_position + i_run*(roadPath_data[road_check][1]) + j_run) != 0)
					{
						int k = *(road_position + i_run*(roadPath_data[road_check][1]) + j_run)-1;
						
						if(former_state == -3)//Get first car mark
						{
							if(isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//Go!!
							{
								marker[k] = 1;
								former_state = 1;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run - min_compare(roadPath_data[car_place[k]][2], carPath_data[k][3])) = k+1;
							}
						}
						else if(former_state == 0)//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[car_place[k]][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run - h) != 0)
								{
									isblock = 1;//Something block
									break;
								}
							}
							if (isblock)
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run - min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						else if(former_state == 1)
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[road_check][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[road_check][1]) + j_run - h) != 0)
								{
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run - h + 1) = k+1;
									isblock = 1;//Something block
									marker[k] = 1;
									former_state = 1;
									break;
								}
							}
							if (!isblock)//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run - min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						//else
							//std::cout << "Error former_state: " << former_state << std::endl;

					}
				}
			}
		}
		else if (roadPath_data[road_check][6] == 0)//One way
		{
			for(int i_run = 0; i_run < roadPath_data[road_check][3]; i_run++)
			{
				int former_state = -3;//first car must be wait car or stop car
				for(int j_run = roadPath_data[road_check][1]-1; j_run >= 0; j_run--)
				{
					//Run from former to latter
					if(*(road_position + i_run*(roadPath_data[road_check][1]) + j_run) != 0)
					{
						int k = *(road_position + i_run*(roadPath_data[road_check][1]) + j_run)-1;
						
						if(former_state == -3)//Get first car mark
						{
							if(isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//GO!!
							{
								marker[k] = 1;
								former_state = 1;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[car_place[k]][1]) + j_run + min_compare(roadPath_data[car_place[k]][2], carPath_data[k][3])) = k+1;
							}
						}
						else if(former_state == 0)//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[road_check][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h) != 0)
								{
									isblock = 1;//Something block
									break;
								}
							}
							if (isblock)
							{
								marker[k] = 0;
								former_state = 0;
							}
							else//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						else if(former_state == 1)
						{
							int isblock = 0;
							for (int h = 1; h <= min_compare(roadPath_data[road_check][2], carPath_data[k][3]); h++)
							{
								if (*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h) != 0)
								{
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
									*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + h - 1) = k+1;
									isblock = 1;//Something block
									marker[k] = 1;
									former_state = 1;
									break;
								}
							}
							if (!isblock)//GO!!
							{
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run) = 0;
								*(road_position + i_run * (roadPath_data[road_check][1]) + j_run + min_compare(roadPath_data[road_check][2], carPath_data[k][3])) = k+1;
								marker[k] = 1;
								former_state = 1;
							}
						}
						//else
							//std::cout << "Error former_state: " << former_state << std::endl;

					}
				}
			}
		}

	}
}

//Don't need to find out who blocks you
int isBlockinRoad(int carOrder, std::vector<int*> roadMatrix_group, int car_place, int *carPath_data, int (*roadPath_data)[7])
{
	if(car_place == -1)//For cars which didn't start
	{
		std::cout << "Why deal not start car in isBlockinRoad()" << std::endl;
	}
	else//For cars on road
	{
		Matrix_i2v2 road_position = roadMatrix_group[car_place];
		if(roadPath_data[car_place][6] == 1)//Two way find one
		{
			for(int i = 0; i < (roadPath_data[car_place][3]*(roadPath_data[car_place][6]+1)); i++)
			{
				for(int j = 0; j < (roadPath_data[car_place][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[car_place][1]) + j) == carOrder+1)
					{
						if(i >= roadPath_data[car_place][3])//n(first)-(2n-1):ahead
						{
							int isblock = 0;
							for(int k = 1; k <= min_compare(roadPath_data[car_place][2], carPath_data[3]); k++)
							{
								if(j + k >= roadPath_data[car_place][1])
								{
									//std::cout << "Entering cross1." << std::endl;
									return isblock;
								}
								if(*(road_position + i*(roadPath_data[car_place][1]) + j + k) != 0)
									isblock = 1;//Something block
							}
							if(isblock)
								return 1;//Something block
							else
								return 0;//No block
						}
						else//0-n-1(first):back
						{
							int isblock = 0;
							for(int k = 1; k <= min_compare(roadPath_data[car_place][2], carPath_data[3]); k++)
							{
								if(j - k < 0)
								{
									//std::cout << "Entering cross2." << std::endl;
									return isblock;
								}
								if(*(road_position + i*(roadPath_data[car_place][1]) + j - k) != 0)
									isblock = 1;//Something block
							}
							if(isblock)
								return 1;//Something block
							else
								return 0;//No block
						}
					}
				}
			}
		}
		else//One way
		{
			for(int i = 0; i < (roadPath_data[car_place][3]); i++)
			{
				for(int j = 0; j < (roadPath_data[car_place][1]); j++)
				{
					if(*(road_position + i*(roadPath_data[car_place][1]) + j) == carOrder+1)
					{
						int isblock = 0;
						for(int k = 1; k <= min_compare(roadPath_data[car_place][2], carPath_data[3]); k++)
						{
							if(j + k >= roadPath_data[car_place][1])
							{
								//std::cout << "Entering cross3." << std::endl;
								return isblock;
							}
							if(*(road_position + i*(roadPath_data[car_place][1]) + j + k) != 0)
								isblock = 1;//Something block
						}
						if(isblock)
							return 1;//Something block
						else
							return 0;//No block
					}
				}
			}
		}
	}
	return 0;//No block
}

int CrossDrive(int *cross_roundCheckEnsure, int *cross_roundCheck, int Q_direction, int cross_check, std::vector<int> road_save, int road_check, int i, int j, int k, Matrix_i2v2 crossTo_position, int (*Q)[2], int *marker, Matrix_i2v2 road_position, int *next_move, std::vector<int> car_speedChart, int *car_place, int *car_arrive, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	int get_target = 0;
	int cannot_acrossLimit = 1;
	int last_roadLengthLeft = 0;
	if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
	{
		if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
		{
			cannot_acrossLimit = min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - (roadPath_data[road_save[road_check]][1]-1-j);
			last_roadLengthLeft = roadPath_data[road_save[road_check]][1]-1-j;
		}
		else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-(n-1)(first):back
		{
			cannot_acrossLimit = min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - j;
			last_roadLengthLeft = j;
		}
	}
	else if (roadPath_data[road_save[road_check]][6] == 0)//One way
	{
		if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
		{
			cannot_acrossLimit = min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - (roadPath_data[road_save[road_check]][1]-1-j);
			last_roadLengthLeft = roadPath_data[road_save[road_check]][1]-1-j;
		}
		/*else if(cross_check == roadPath_data[road_save[road_check]][4] - 1)//Can't go
		{
			std::cout << "Why enter the one way road in wrong direction in CrossDrive()." << std::endl;
			return 1;
		}*/
	}
	//std::cout << "last_roadLengthLeft: " << last_roadLengthLeft << std::endl;
	
	if (roadPath_data[next_move[k]][6] == 1)//Two way find one
	{
		if(crossPath_data[cross_check][0] == roadPath_data[next_move[k]][4])//n(first)-(2n-1):ahead
		{
			for(int i_run = roadPath_data[next_move[k]][3]; i_run < (roadPath_data[next_move[k]][3]*(roadPath_data[next_move[k]][6]+1)); i_run++)
			{
				if(0 >= cannot_acrossLimit)
				{
					if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-(n-1)(first):back
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1])) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
					}
					else if (roadPath_data[road_save[road_check]][6] == 0)//One way
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						/*else if(cross_check == roadPath_data[road_save[road_check]][4] - 1)//Can't go
						{
							std::cout << "Why enter the one way road in wrong direction in CrossDrive()." << std::endl;
							return 1;
						}*/
					}
				}
				else
				{
					int not_blockCount = 0;
					for(int j_run = 0; j_run < min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft; j_run++)//(roadPath_data[next_move[k]][1])
					{
						if(*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run) != 0)
						{
							//std::cout << "ID: " << *(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run) << std::endl;
							//std::cout << "min_compare: " << min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft << std::endl;
							int mark_check = *(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run)-1;

							//std::cout << "marker: "<< marker[mark_check] << " " << mark_check << std::endl;
							if(marker[mark_check] == 0)
							{
								cross_roundCheckEnsure[0] = 1;
								lockRoad.emplace_back(next_move[k]);
								return 1;//ALL BELOW REMAIN WAIT
							}

							if(j_run != 0)
							{
								//std::cout << "j_run: " << j_run << " " << *(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run - 1) << std::endl;
								*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run - 1) = k+1;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								car_place[k] = next_move[k];
								marker[k] = 1;
								Q[car_place[k]][0]++;//Q[road_save[road_check]][Q_direction]++
								get_target = 1;
								cross_roundCheck[0] = 1;//Wait car move
								break;
							}
							else
							{
								break;//This lane is blocked
							}
						}
						else
							not_blockCount++;
					}
					if(not_blockCount == min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft)
					{
						*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + not_blockCount-1) = k+1;
						*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
						car_place[k] = next_move[k];
						marker[k] = 1;
						Q[car_place[k]][0]++;
						get_target = 1;
						cross_roundCheck[0] = 1;//Wait car move
					}
					if(get_target)
						break;
				}
			}
			if(get_target == 0)//New road is totally blocked, but will not block lower priority car in this road
			{
				marker[k] = 1;
				return 2;//ALL BELOW STOP
			}
		}
		else if(crossPath_data[cross_check][0] == roadPath_data[next_move[k]][5])//0-(n-1)(first):back
		{
			for(int i_run = roadPath_data[next_move[k]][3]-1; i_run >= 0; i_run--)
			{
				if(0 >= cannot_acrossLimit)
				{
					if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-(n-1)(first):back
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1])) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
					}
					else if (roadPath_data[road_save[road_check]][6] == 0)//One way
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						/*else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//Can't go
						{
							std::cout << "Why enter the one way road in wrong direction in CrossDrive()." << std::endl;
							return 1;
						}*/
					}
				}
				else
				{
					int not_blockCount = 0;
					for(int j_run = roadPath_data[next_move[k]][1] - 1; j_run >= roadPath_data[next_move[k]][1] - (min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft); j_run--)
					{
						if(*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run) != 0)
						{
							int mark_check = *(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run)-1;

							if(marker[mark_check] == 0)
							{
								cross_roundCheckEnsure[0] = 1;
								lockRoad.emplace_back(next_move[k]);
								return 1;//ALL BELOW REMAIN WAIT
							}

							if(j_run != roadPath_data[next_move[k]][1] - 1)
							{
								*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run + 1) = k+1;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								car_place[k] = next_move[k];
								marker[k] = 1;
								Q[car_place[k]][1]++;
								get_target = 1;
								cross_roundCheck[0] = 1;//Wait car move
								break;
							}
							else
							{
								break;//This lane is blocked
							}
						}
						else
							not_blockCount++;
					}
					if(not_blockCount == min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft)//(roadPath_data[road_save[road_check]][1]-1-j)
					{
						//std::cout << "next_move[k]2: " << next_move[k] << " " << i_run << std::endl;
						
						*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + (roadPath_data[next_move[k]][1] - not_blockCount)) = k+1;
						*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
						car_place[k] = next_move[k];
						marker[k] = 1;
						Q[car_place[k]][1]++;
						get_target = 1;
						cross_roundCheck[0] = 1;//Wait car move
					}
					if(get_target)
						break;
				}
			}
			if(get_target == 0)//New road is totally blocked
			{
				marker[k] = 1;
				return 2;//ALL BELOW STOP
			}
		}
	}
	else if (roadPath_data[next_move[k]][6] == 0)//One way
	{
		if(crossPath_data[cross_check][0] == roadPath_data[next_move[k]][4])//0(first)-(n-1):ahead
		{
			for(int i_run = 0; i_run < roadPath_data[next_move[k]][3]; i_run++)
			{
				if(0 >= cannot_acrossLimit)
				{
					if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-(n-1)(first):back
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1])) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
					}
					else if (roadPath_data[road_save[road_check]][6] == 0)//One way
					{
						if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
						{
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
							*(road_position + i*(roadPath_data[road_save[road_check]][1]) + roadPath_data[road_save[road_check]][1]-1) = k+1;
							marker[k] = 1;
							get_target = 1;
							cross_roundCheck[0] = 1;//Wait car move
							return 0;
						}
						/*else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//Can't go
						{
							std::cout << "Why enter the one way road in wrong direction in CrossDrive()." << std::endl;
							return 1;
						}*/
					}
				}
				else
				{
					int not_blockCount = 0;
					for(int j_run = 0; j_run < min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft; j_run++)//(roadPath_data[next_move[k]][1])
					{
						if(*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run) != 0)
						{
							int mark_check = *(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run)-1;

							if(marker[mark_check] == 0)
							{
								cross_roundCheckEnsure[0] = 1;
								lockRoad.emplace_back(next_move[k]);
								return 1;//ALL BELOW REMAIN WAIT
							}

							if(j_run != 0)
							{
								*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + j_run - 1) = k+1;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								car_place[k] = next_move[k];
								marker[k] = 1;
								Q[car_place[k]][0]++;
								get_target = 1;
								cross_roundCheck[0] = 1;//Wait car move
								break;
							}
							else
							{
								break;//This lane is blocked
							}
						}
						else
							not_blockCount++;
					}
					if(not_blockCount == min_compare(roadPath_data[next_move[k]][2], carPath_data[k][3]) - last_roadLengthLeft)
					{
						*(crossTo_position + i_run*(roadPath_data[next_move[k]][1]) + not_blockCount-1) = k+1;
						*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
						car_place[k] = next_move[k];
						marker[k] = 1;
						Q[car_place[k]][0]++;
						get_target = 1;
						cross_roundCheck[0] = 1;//Wait car move
					}
					if(get_target)
						break;
				}
			}
			if(get_target == 0)//New road is totally blocked
			{
				marker[k] = 1;
				return 2;//ALL BELOW STOP
			}
		}
		//else if(crossPath_data[cross_check][0] == roadPath_data[next_move[k]][5])
			//std::cout << "Error enter oneway road in wrong direction" << std::endl;
	}
	return 0;
}

int CrossWayCheck(int cross_priorityAffect, int *cross_roundCheckEnsure, int *cross_roundCheck, int Q_direction, int cross_check, std::vector<int>road_save, std::vector<int>road_save_direction, int road_check, int direction, Matrix_i2v2 road_position, int i, int j, int (*Q)[2], int *marker, std::vector<int*> roadMatrix_group, int *next_move, std::vector<int> car_speedChart, int *car_place, int *car_arrive, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	int check_nextRoadBreak = 0;
	if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) != 0)
	{
		//std::cout << "CrossWayCheck(): " << *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) << std::endl;
		int k = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j)-1;

		if(marker[k] == 0 && //marker: 0-wait 1-stop (-1)-arrive (-3)-inital
				isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data)
					&& !isBlockinRoad(k, roadMatrix_group, car_place[k], carPath_data[k], roadPath_data))
		{
			if(cross_priorityAffect == 1)//0:nothing 1:wait 2:all-stop
			{
				marker[k] = 0;
				return 1;
			}
			else if(cross_priorityAffect == 2)
			{
				marker[k] = 1;
				if(roadPath_data[car_place[k]][6] == 1)
				{
					if(i >= roadPath_data[car_place[k]][3])//ahead
					{
						*(road_position + i*(roadPath_data[car_place[k]][1]) + j) = 0;
						*(road_position + i*(roadPath_data[car_place[k]][1]) + roadPath_data[car_place[k]][1]-1) = k+1;
					}
					else if(i < roadPath_data[car_place[k]][3])//back
					{
						*(road_position + i*(roadPath_data[car_place[k]][1]) + j) = 0;
						*(road_position + i*(roadPath_data[car_place[k]][1])) = k+1;
					}
				}
				else if(roadPath_data[car_place[k]][6] == 0)
				{
					if(i < roadPath_data[car_place[k]][3])//ahead
					{
						*(road_position + i*(roadPath_data[car_place[k]][1]) + j) = 0;
						*(road_position + i*(roadPath_data[car_place[k]][1]) + roadPath_data[car_place[k]][1]-1) = k+1;
					}
					//else if(i >= roadPath_data[car_place[k]][3])//Wrong direction
						//std::cout << "Wrong direction here." << std::endl;
				}
				cross_roundCheck[0] = 1;//Wait car move
				return 2;
			}
			
			//Arrive car
			if(carPath_data[k][2] == crossPath_data[cross_check][0])
			{
				*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
				car_place[k] = -2;
				car_arrive[k] = 1;
				marker[k] = -1;//0-wait 1-stop (-1)-arrive (-3)-inital
				return 0;//0:own-stop 1:wait 2:all-stop
				//Q[road_save[road_check]][Q_direction]++;
				//If count AVERAGE Speed, add here
			}
			
			//Check turn Conflict
			for(int direct = 0; direct < 4; direct++)
			{
				if(next_move[k] == road_save_direction[direct])//0:north 1:east 2:south 3:west
				{
					//std::cout << "next_move[k]: " << next_move[k] << std::endl;
					if(std::abs(direction - direct) == 2)//Go straight and straight road is availble
					{
						Matrix_i2v2 crossTo_position = roadMatrix_group[next_move[k]];//New road
						check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
						return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
					}
					else if((direct - direction == 1 && (direction == 0 || direction == 1 || direction == 2)) 
							|| (direction - direct == 3 && (direction == 3)))//Turn left
					{
						Matrix_i2v2 crossTo_position = roadMatrix_group[next_move[k]];//New road
						if(road_save_direction[(direct-2>=0)?(direct-2):(direct+2)] != -1)//Has straight road to new road is availble
						{
							int straight_roadOrder = road_save_direction[(direct-2>=0)?(direct-2):(direct+2)];
							Matrix_i2v2 straight_position = roadMatrix_group[straight_roadOrder];//The road may block
							//int get_target = 0;
							if (roadPath_data[straight_roadOrder][6] == 1)//Two way find one
							{
								if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][5])//n(first)-(2n-1):ahead
								{
									for(int j_run = (roadPath_data[straight_roadOrder][1]) - 1; j_run >= (roadPath_data[straight_roadOrder][1] - roadPath_data[straight_roadOrder][2]); j_run--)
									{
										for(int i_run = roadPath_data[straight_roadOrder][3]; i_run < (roadPath_data[straight_roadOrder][3]*(roadPath_data[straight_roadOrder][6]+1)); i_run++)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													check_nextRoadBreak = 1;//Has straight car block
													return 2;//0:own-stop 1:wait 2:all-stop
												}

											}
										}
									}
									if(check_nextRoadBreak == 0)//No straight car block
									{
										check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
										return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
									}
								}
								else if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][4])//0-(n-1)(first):back
								{
									for(int j_run = 0; j_run < roadPath_data[straight_roadOrder][1]; j_run++)
									{
										for(int i_run = roadPath_data[straight_roadOrder][3] - 1; i_run >= 0; i_run--)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;
												
												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													check_nextRoadBreak = 1;//Has straight car block
													return 2;//0:own-stop 1:wait 2:all-stop
												}

											}
										}
									}
									if(check_nextRoadBreak == 0)//No straight car block
									{
										check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
										return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
									}
								}
							}
							else if(roadPath_data[straight_roadOrder][6] == 0)//One way
							{
								if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][5])//0(first)-(n-1):ahead
								{
									for(int j_run = (roadPath_data[straight_roadOrder][1]) - 1; j_run >= (roadPath_data[straight_roadOrder][1] - roadPath_data[straight_roadOrder][2]); j_run--)
									{
										for(int i_run = 0; i_run < roadPath_data[straight_roadOrder][3]; i_run++)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													check_nextRoadBreak = 1;//Has straight car block
													return 2;//0:own-stop 1:wait 2:all-stop
												}

											}
										}
									}
									if(check_nextRoadBreak == 0)//No straight car block
									{
										check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
										return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
									}
								}
								else if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][4])
								{
									check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
									return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
								}
							}
						}
						else//No straight road to new road is availble
						{
							check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
							return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
						}
					}
					else if((direction - direct == 1 && (direction == 1 || direction == 2 || direction == 3)) 
							|| (direct - direction == 3 && (direction == 0)))//Turn right
					{
						//std::cout << "Left?: " << road_save_direction[(direct-1>=0)?(direct-1):(direct+3)] << std::endl;
						//std::cout << "Straight?: " << road_save_direction[(direct-2>=0)?(direct-2):(direct+2)] << std::endl;
						
						Matrix_i2v2 crossTo_position = roadMatrix_group[next_move[k]];//New road
						int block_fromOther = 0;
						if(road_save_direction[(direct-2>=0)?(direct-2):(direct+2)] != -1)//Straight road to new road is availble
						{
							int straight_roadOrder = road_save_direction[(direct-2>=0)?(direct-2):(direct+2)];
							Matrix_i2v2 straight_position = roadMatrix_group[straight_roadOrder];//The road may block
							//int get_target = 0;
							if (roadPath_data[straight_roadOrder][6] == 1)//Two way find one
							{
								if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][5])//n(first)-(2n-1):ahead
								{
									for(int j_run = (roadPath_data[straight_roadOrder][1]) - 1; j_run >= (roadPath_data[straight_roadOrder][1] - roadPath_data[straight_roadOrder][2]); j_run--)
									{
										for(int i_run = roadPath_data[straight_roadOrder][3]; i_run < (roadPath_data[straight_roadOrder][3]*(roadPath_data[straight_roadOrder][6]+1)); i_run++)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has straight car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
								else if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][4])//0-(n-1)(first):back
								{
									for(int j_run = 0; j_run < roadPath_data[straight_roadOrder][1]; j_run++)
									{
										for(int i_run = roadPath_data[straight_roadOrder][3] - 1; i_run >= 0; i_run--)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has straight car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
							}
							else if(roadPath_data[straight_roadOrder][6] == 0)//One way
							{
								if(crossPath_data[cross_check][0] == roadPath_data[straight_roadOrder][5])//0(first)-(n-1):ahead
								{
									for(int j_run = (roadPath_data[straight_roadOrder][1]) - 1; j_run >= (roadPath_data[straight_roadOrder][1] - roadPath_data[straight_roadOrder][2]); j_run--)
									{
										for(int i_run = 0; i_run < roadPath_data[straight_roadOrder][3]; i_run++)
										{
											if(*(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(straight_position + i_run*(roadPath_data[straight_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has straight car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
								//else if(cross_check == roadPath_data[straight_roadOrder][4] - 1)
								//{
									//std::cout << "Error leaving oneway road in wrong direction2" << std::endl;
									/*std::cout << straight_roadOrder << " " << cross_check << " " << car_check << std::endl;
									for (int i_test = 0; i_test < roadPath_data[straight_roadOrder][3]; i_test++)
									{
										for (int j_test = 0; j_test < roadPath_data[straight_roadOrder][1]; j_test++)
										{
											std::cout << *(road_position + i_test * (roadPath_data[straight_position][1]) + j_test) << " ";
										}
										std::cout << std::endl;
									}
									exit(0);*/
								//}
							}
						}
						if(road_save_direction[(direct-1>=0)?(direct-1):(direct+3)] != -1)//Left turn road to new road is availble
						{
							int left_roadOrder = road_save_direction[(direct-1>=0)?(direct-1):(direct+3)];
							Matrix_i2v2 left_position = roadMatrix_group[left_roadOrder];//The road may block
							//int get_target = 0;
							if (roadPath_data[left_roadOrder][6] == 1)//Two way find one
							{
								if(crossPath_data[cross_check][0] == roadPath_data[left_roadOrder][5])//n(first)-(2n-1):ahead
								{
									for(int j_run = (roadPath_data[left_roadOrder][1]) - 1; j_run >= (roadPath_data[left_roadOrder][1] - roadPath_data[left_roadOrder][2]); j_run--)
									{
										for(int i_run = roadPath_data[left_roadOrder][3]; i_run < (roadPath_data[left_roadOrder][3]*(roadPath_data[left_roadOrder][6]+1)); i_run++)
										{
											if(*(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has left car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
								else if(crossPath_data[cross_check][0] == roadPath_data[left_roadOrder][4])//0-(n-1)(first):back
								{
									for(int j_run = 0; j_run < roadPath_data[left_roadOrder][1]; j_run++)
									{
										for(int i_run = roadPath_data[left_roadOrder][3] - 1; i_run >= 0; i_run--)
										{
											if(*(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has left car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
							}
							else if(roadPath_data[left_roadOrder][6] == 0)//One way
							{
								if(crossPath_data[cross_check][0] == roadPath_data[left_roadOrder][5])//0(first)-(n-1):ahead
								{
									for(int j_run = (roadPath_data[left_roadOrder][1]) - 1; j_run >= (roadPath_data[left_roadOrder][1] - roadPath_data[left_roadOrder][2]); j_run--)
									{
										for(int i_run = 0; i_run < roadPath_data[left_roadOrder][3]; i_run++)
										{
											if(*(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run) != 0)
											{
												int k_find = *(left_position + i_run*(roadPath_data[left_roadOrder][1]) + j_run)-1;

												if(marker[k_find] == 0 && 
														isEnteringCross(k_find, carPath_data[k_find], car_place[k_find], roadMatrix_group, roadPath_data, crossPath_data) 
															&& !isBlockinRoad(k_find, roadMatrix_group, car_place[k_find], carPath_data[k_find], roadPath_data)
																&& next_move[k_find] == next_move[k])
												{
													block_fromOther = 1;//Has left car block
												}

												if(block_fromOther)
													break;
											}
										}
										if(block_fromOther)
											break;
									}
								}
								//else if((cross_check == roadPath_data[left_roadOrder][4]))
									//std::cout << "Error leaving oneway road in wrong direction3" << std::endl;
							}
						}
						if(!block_fromOther)//No straight or left car block
						{
							check_nextRoadBreak = CrossDrive(cross_roundCheckEnsure, cross_roundCheck, Q_direction, cross_check, road_save, road_check, i, j, k, crossTo_position, Q, marker, road_position, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
							return check_nextRoadBreak;//0:own-stop 1:wait 2:all-stop
						}
					}
				}
			}
		}
		else if(marker[k] == 0 && //marker: 0-wait 1-stop (-1)-arrive (-3)-inital
				!isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data)
					&& !isBlockinRoad(k, roadMatrix_group, car_place[k], carPath_data[k], roadPath_data))
		{	//If the waiting car don't enter cross?
			if(roadPath_data[road_save[road_check]][6] == 1)
			{
				if(i >= roadPath_data[road_save[road_check]][3])//ahead
				{
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j + min_compare(roadPath_data[road_save[road_check]][2], carPath_data[k][3])) = k+1;
					marker[k] = 1;
					cross_roundCheck[0] = 1;//Wait car move
				}
				else if(i < roadPath_data[road_save[road_check]][3])//back
				{
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j - min_compare(roadPath_data[road_save[road_check]][2], carPath_data[k][3])) = k+1;
					marker[k] = 1;
					cross_roundCheck[0] = 1;//Wait car move
				}
			}
			else if(roadPath_data[road_save[road_check]][6] == 0)
			{
				if(i < roadPath_data[road_save[road_check]][3])//ahead
				{
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
					*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j + min_compare(roadPath_data[road_save[road_check]][2], carPath_data[k][3])) = k+1;
					marker[k] = 1;
					cross_roundCheck[0] = 1;//Wait car move
				}
				//else if(i >= roadPath_data[road_save[road_check]][3])//Wrong direction
					//std::cout << "Wrong direction here." << std::endl;
			}
		}
		else if(marker[k] == 0 && //marker: 0-wait 1-stop (-1)-arrive (-3)-inital
				isBlockinRoad(k, roadMatrix_group, car_place[k], carPath_data[k], roadPath_data))
		{	//The former car turn to stop car? or still waiting car?
			int get_targetinBlockCheck = 0;
			if(roadPath_data[road_save[road_check]][6] == 1)
			{
				if(i >= roadPath_data[road_save[road_check]][3])
				{
					for(int j_check = j+1; j_check < roadPath_data[road_save[road_check]][1]; j_check++)
					{
						if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check) != 0)
						{
							int k_check = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check)-1;

							marker[k] = marker[k_check];
							if(marker[k] == 1)
							{
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check - 1) = k+1;
								cross_roundCheck[0] = 1;//Wait car move
							}
							get_targetinBlockCheck = 1;

							if(get_targetinBlockCheck)
								break;
						}
					}
					//if(get_targetinBlockCheck == 0)
					//{
						//std::cout << "Ghost Block car in Turn check1." << std::endl;
						/*std::cout << isBlockinRoad(roadMatrix_group, car_place[k], carPath_data[k], roadPath_data) << std::endl;
						int watchRID = car_place[k];
						std::cout << marker[6] << " " << k << std::endl;
						std::cout << "next_move[2155]:" << next_move[k] << " " << car_place[k] << std::endl;
						Matrix_i2v2 road_position_test = roadMatrix_group[watchRID];
						for(int i_test = 0; i_test < (roadPath_data[watchRID][3]*(roadPath_data[watchRID][6]+1)); i_test++)
						{
							for(int j_test = 0; j_test < (roadPath_data[watchRID][1]); j_test++)
							{
								std::cout << *(road_position_test + i_test * (roadPath_data[watchRID][1]) + j_test) << " ";
							}
							std::cout << std::endl;
						}
						exit(0);*/
					//}
				}
				else if(i < roadPath_data[road_save[road_check]][3])
				{
					for(int j_check = j-1; j_check >= 0; j_check--)
					{
						if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check) != 0)
						{
							int k_check = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check)-1;

							marker[k] = marker[k_check];
							if(marker[k] == 1)
							{
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check + 1) = k+1;
								cross_roundCheck[0] = 1;//Wait car move
							}
							get_targetinBlockCheck = 1;

							if(get_targetinBlockCheck)
								break;
						}
					}
					//if(get_targetinBlockCheck == 0)
						//std::cout << "Ghost Block car in Turn check2." << std::endl;
				}
			}
			else if(roadPath_data[road_save[road_check]][6] == 0)
			{
				if(i < roadPath_data[road_save[road_check]][3])
				{
					for(int j_check = j+1; j_check < roadPath_data[road_save[road_check]][1]; j_check++)
					{
						if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check) != 0)
						{
							int k_check = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check)-1;

							marker[k] = marker[k_check];
							if(marker[k] == 1)
							{
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) = 0;
								*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j_check - 1) = k+1;
								cross_roundCheck[0] = 1;//Wait car move
							}
							get_targetinBlockCheck = 1;

							if(get_targetinBlockCheck)
								break;
						}
					}
					//if(get_targetinBlockCheck == 0)
						//std::cout << "Ghost Block car in Turn check3." << std::endl;
				}
				//else if(i >= roadPath_data[road_save[road_check]][3])
					//std::cout << "Wrong direction here." << std::endl;
			}
		}

	}
	return 0;//0:own-stop 1:wait 2:all-stop
}

void CrossFunction(int *cross_roundCheckEnsure, int *cross_roundCheck, int (*Q)[2], int *marker, std::vector<int*> roadMatrix_group, int *next_move, std::vector<int> car_speedChart, int *car_place, int *car_arrive, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	for(int cross_check = 0; cross_check < global_crossPath_row; cross_check++)
	{
#ifdef SHOW_IMFOR
		std::cout << "cross_check: " << cross_check << std::endl;
#endif
		std::vector<int>road_save;
		std::vector<int>road_save_direction;//0:north 1:east 2:south 3:west
		int road_connectNum = 0;
		for(int i = 1;i < 5; i++)
		{
			if(crossPath_data[cross_check][i] != -1)
			{
				for(int road_check = 0; road_check < global_roadPath_row; road_check++)
				{
					if(roadPath_data[road_check][0] == crossPath_data[cross_check][i])
					{
						road_save.emplace_back(road_check);//0-(n-1) -1
						road_save_direction.emplace_back(road_check);//0-(n-1) -1
					}
				}
				road_connectNum++;
			}
			else
			{
				road_save.emplace_back(crossPath_data[cross_check][i]);//save -1
				road_save_direction.emplace_back(crossPath_data[cross_check][i]);//save -1
			}
		}
		std::sort(road_save.begin(), road_save.end());
		
		//for(int i = 0; i<road_save.size(); i++)
		//{
		//	std::cout << road_save[i] << std::endl;
		//}
		
		int road_check = 0;
		int check_second = 0;//Ensure have checked all road
		int check_finishNum[4] = {0};
		int total_countCheck = 0;
		while(total_countCheck < road_connectNum)
		{
			//std::cout << "total_countCheck: " << total_countCheck << " " << road_connectNum << std::endl;
			//std::cout << "road_save[road_check]: " << roadPath_data[road_save[road_check]][0] << std::endl;
			//std::cout << "road_check: " << road_check << " " << crossPath_data[cross_check][0] << " " << std::endl;
			int check_nextRoadBreak_whole = 0;//IF the first priority entering cross can't go
			if(road_save[road_check] != -1)
			{
				std::vector<int>::iterator location_index;
				location_index = std::find(road_save_direction.begin(), road_save_direction.end(), road_save[road_check]);  //find函数
				int direction = location_index - road_save_direction.begin();//0:north 1:east 2:south 3:west
				Matrix_i2v2 road_position = roadMatrix_group[road_save[road_check]];
				if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
				{
					if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
					{
						int cross_priorityAffect = 0;
						for(int j = (roadPath_data[road_save[road_check]][1]) - 1; j >= 0; j--)
						{
							for(int i = roadPath_data[road_save[road_check]][3]; i < (roadPath_data[road_save[road_check]][3]*(roadPath_data[road_save[road_check]][6]+1)); i++)
							{
								int num_zero = 0;
								check_nextRoadBreak_whole = CrossWayCheck(cross_priorityAffect, cross_roundCheckEnsure, cross_roundCheck, num_zero, cross_check, road_save, road_save_direction, road_check, direction, road_position, i, j, Q, marker, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
								//std::cout << "check_nextRoadBreak_whole: " << road_save[road_check] << " " << check_nextRoadBreak_whole << std::endl;
								if(check_nextRoadBreak_whole)//0:own-stop 1:wait 2:all-stop
								{
									cross_priorityAffect = check_nextRoadBreak_whole;
									//break;
								}
							}
							//if(check_nextRoadBreak_whole)
								//break;
						}
						//if(check_nextRoadBreak_whole)
							//break;
					}
					else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-(n-1)(first):back
					{
						int cross_priorityAffect = 0;
						for(int j = 0; j < roadPath_data[road_save[road_check]][1]; j++)
						{
							for(int i = roadPath_data[road_save[road_check]][3]-1; i >= 0; i--)
							{
								int num_one = 1;
								check_nextRoadBreak_whole = CrossWayCheck(cross_priorityAffect, cross_roundCheckEnsure, cross_roundCheck, num_one, cross_check, road_save, road_save_direction, road_check, direction, road_position, i, j, Q, marker, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
								if(check_nextRoadBreak_whole)//0:own-stop 1:wait 2:all-stop
								{
									cross_priorityAffect = check_nextRoadBreak_whole;
									//break;
								}
							}
							//if(check_nextRoadBreak_whole)
								//break;
						}
						//if(check_nextRoadBreak_whole)
							//break;
					}
				}
				if (roadPath_data[road_save[road_check]][6] == 0)//One way
				{
					if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
					{
						int cross_priorityAffect = 0;
						for(int j = (roadPath_data[road_save[road_check]][1]) - 1; j >= 0; j--)
						{
							for(int i = 0; i < roadPath_data[road_save[road_check]][3]; i++)
							{
								int num_zero = 0;
								check_nextRoadBreak_whole = CrossWayCheck(cross_priorityAffect, cross_roundCheckEnsure, cross_roundCheck, num_zero, cross_check, road_save, road_save_direction, road_check, direction, road_position, i, j, Q, marker, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
								if(check_nextRoadBreak_whole)//0:own-stop 1:wait 2:all-stop
								{
									cross_priorityAffect = check_nextRoadBreak_whole;
									//break;
								}
							}
							//if(check_nextRoadBreak_whole)
								//break;
						}
						//if(check_nextRoadBreak_whole)
							//break;
					}
					//else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])
					//{
						//std::cout << "Can't enter oneway road in CrossFunction()." << std::endl;
						//std::cout << road_save[road_check] << std::endl;
					//}
				}
				
				
				//If any wait car left
				int get_target = 0;
				if (roadPath_data[road_save[road_check]][6] == 1)//Two way find one
				{
					if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//n(first)-(2n-1):ahead
					{
						/*std::cout << "here begin" << std::endl;
						for(int i = 0; i < (roadPath_data[road_save[road_check]][3]*(roadPath_data[road_save[road_check]][6]+1)); i++)
						{
							for(int j = 0; j < (roadPath_data[road_save[road_check]][1]); j++)
							{
								std::cout << *(road_position + i * (roadPath_data[road_save[road_check]][1]) + j) << " ";
							}
							std::cout << std::endl;
						}
						std::cout << marker[9487] << std::endl;
						std::cout << "next_move[9487]:" << next_move[9487] << " " << car_place[9487] << std::endl;
						*/
						/*int watchRID = 11;
						std::cout << marker[6] << marker[7322] << marker[2906] << marker[3129] << std::endl;
						std::cout << "next_move[2155]:" << next_move[6] << " " << car_place[6] << std::endl;
						Matrix_i2v2 road_position_test = roadMatrix_group[watchRID];
						for(int i_test = 0; i_test < (roadPath_data[watchRID][3]*(roadPath_data[watchRID][6]+1)); i_test++)
						{
							for(int j_test = 0; j_test < (roadPath_data[watchRID][1]); j_test++)
							{
								std::cout << *(road_position_test + i_test * (roadPath_data[watchRID][1]) + j_test) << " ";
							}
							std::cout << std::endl;
						}*/
						/*std::cout << isEnteringCross(carPath_data[86], car_place[86], roadMatrix_group, roadPath_data, crossPath_data) << std::endl;
						std::cout << marker[86] << std::endl;
						std::cout << next_move[86] << " " << car_place[86] << std::endl;*/
						/*int watchRID2 = 32;
						Matrix_i2v2 road_position_test2 = roadMatrix_group[watchRID2];
						for(int i_test = 0; i_test < (roadPath_data[watchRID][3]*(roadPath_data[watchRID][6]+1)); i_test++)
						{
							for(int j_test = 0; j_test < (roadPath_data[watchRID][1]); j_test++)
							{
								std::cout << *(road_position_test2 + i_test * (roadPath_data[watchRID][1]) + j_test) << " ";
							}
							std::cout << std::endl;
						}
						std::cout << marker[6505] << std::endl;
						std::cout << marker[9066] << std::endl;
						std::cout << "next_move[9066]:" << next_move[9066] << " " << car_place[9066] << std::endl;
						*/
						for(int i = roadPath_data[road_save[road_check]][3]; i < (roadPath_data[road_save[road_check]][3]*(roadPath_data[road_save[road_check]][6]+1)); i++)
						{
							for(int j = (roadPath_data[road_save[road_check]][1]) - 1; j >= 0; j--)
							{
								if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) != 0)
								{
									int k = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j)-1;

									if(marker[k] == 0 && isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
									{
										get_target = 1;//Not end
									}

								}
								if(get_target)
									break;
							}
							if(get_target)
								break;
						}
						if(get_target == 0)
							check_finishNum[road_check] = 1;
					}
					else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//0-n-1(first):back
					{
						/*std::cout << "There begin" << std::endl;
						for(int i = 0; i < (roadPath_data[road_save[road_check]][3]*(roadPath_data[road_save[road_check]][6]+1)); i++)
						{
							for(int j = 0; j < (roadPath_data[road_save[road_check]][1]); j++)
							{
								std::cout << *(road_position + i * (roadPath_data[road_save[road_check]][1]) + j) << " ";
							}
							std::cout << std::endl;
						}*/
						
						for(int i = roadPath_data[road_save[road_check]][3] - 1; i >= 0; i--)
						{
							for(int j = 0; j < (roadPath_data[road_save[road_check]][1]); j++)
							{
								if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) != 0)
								{
									int k = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j)-1;

									if(marker[k] == 0 && isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
									{
										get_target = 1;//Not end
									}

								}
								if(get_target)
									break;
							}
							if(get_target)
								break;
						}
						if(get_target == 0)
							check_finishNum[road_check] = 1;
					}
				}
				else if(roadPath_data[road_save[road_check]][6] == 0)//One way
				{
					if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][5])//0(first)-(n-1):ahead
					{
						/*
						for(int i = 0; i < (roadPath_data[road_save[road_check]][3]); i++)
						{
							for(int j = 0; j < (roadPath_data[road_save[road_check]][1]); j++)
							{
								std::cout << *(road_position + i * (roadPath_data[road_save[road_check]][1]) + j) << " ";
							}
							std::cout << std::endl;
						}
						*/
						for(int i = 0; i < (roadPath_data[road_save[road_check]][3]); i++)
						{
							for(int j = (roadPath_data[road_save[road_check]][1]) - 1; j >= 0; j--)
							{
								if(*(road_position + i*(roadPath_data[road_save[road_check]][1]) + j) != 0)
								{
									int k = *(road_position + i*(roadPath_data[road_save[road_check]][1]) + j)-1;

									if(marker[k] == 0 && isEnteringCross(k, carPath_data[k], car_place[k], roadMatrix_group, roadPath_data, crossPath_data))//marker: 0-wait 1-stop (-1)-arrive (-3)-inital
									{
										get_target = 1;//Not end
									}

								}
								if(get_target)
									break;
							}
							if(get_target)
								break;
						}
						if(get_target == 0)
							check_finishNum[road_check] = 1;
					}
					else if(crossPath_data[cross_check][0] == roadPath_data[road_save[road_check]][4])//Not enter direction, don't need to check
					{
						check_finishNum[road_check] = 1;
					}
				}
			}
			//std::cout << check_finishNum[0] << " " << check_finishNum[1] << " " << check_finishNum[2] << " " << check_finishNum[3] << std::endl;
			
			if(check_second == 1 && cross_roundCheckEnsure[0] == 1)
				break;
			
			total_countCheck = check_finishNum[0] + check_finishNum[1] + check_finishNum[2] + check_finishNum[3];
			road_check++;
			if(road_check >= 4)
			{
				road_check = 0;
				check_second = 1;
			}
		}
	}
}


void StartCarFunction(int *turn_overSpeed, int *car_arrive, int (*Q)[2], int *priority, std::vector<int*> roadMatrix_group, int *next_move, std::vector<int> car_speedChart, int *car_place, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	for (int check_turn = 0; check_turn < (int)car_speedChart.size(); check_turn++)//Give priority: 0-first 1-second 2-...
	{
		int finish_turn = 0;
		for(int car_check = 0; car_check < global_carPath_row; car_check++)
		{
			if(car_place[car_check] == -1 && priority[car_check] < check_turn)
			{
				finish_turn = 1;
				break;
			}
#ifdef FAST_CARFIRST
			if(priority[car_check] < check_turn && car_arrive[car_check] != 1)
			{
				finish_turn = 1;
				break;
			}
#endif
		}
		if(finish_turn)
			break;
		
		
		if(turn_overSpeed[0] <= check_turn)
		{
			turn_overSpeed[0] = check_turn;
		}
		else
		{
			continue;
		}
		
		for (int car_check = 0; car_check < global_carPath_row; car_check++)
		{
			if (car_place[car_check] == -1 && next_move[car_check] != -1)
			{
				if (priority[car_check] == check_turn)//Start!
				{
					if (!isEnteringCross(car_check, carPath_data[car_check], car_place[car_check], roadMatrix_group, roadPath_data, crossPath_data))
					{
						Matrix_i2v2 road_position = roadMatrix_group[next_move[car_check]];
						if (roadPath_data[next_move[car_check]][6] == 1)//Two way find one
						{
							if (carPath_data[car_check][1] == roadPath_data[next_move[car_check]][4])//n(first)-(2n-1):ahead
							{
								int lane_position = 0;//0-smallRID2bigRID 1-bigRID2smallRID
#ifdef NORMAL_STARTLIMIT
								int roadcar_num = CheckCarNum(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] * roadPath_data[next_move[car_check]][3] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef STRICT_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef ONEBYONE_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = LIMIT_NUM+(check_turn>2?1:0);//Wait for improve
#endif
								roadcar_limitation = roadcar_limitation < 1 ? 1 : roadcar_limitation;
								//std::cout << roadcar_num << " " << roadcar_limitation << std::endl;
								if (roadcar_num < roadcar_limitation)//When car speed is lower, limitation should be improve!!!
								{
									//To the first lane which no block.
									int isblock = 0;
									int isblock_row = 0;
									int get_target = 0;
									for (int i = roadPath_data[next_move[car_check]][3];
											i < (roadPath_data[next_move[car_check]][3])*(roadPath_data[next_move[car_check]][6] + 1); i++)
									{
										for (int j = 0; j < min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j++)
										{
											if (*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) != 0)
											{
												if(j != 0)//GO!!
												{
													*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j - 1) = car_check+1;
													car_place[car_check] = next_move[car_check];
													Q[car_place[car_check]][0]++;
													get_target = 1;
													break;
												}
												else
												{
													isblock = 1;//Something block in start place
													break;
												}
											}
										}
										if(get_target)
											break;//check next car
										if (isblock != 0)
										{
											isblock_row++;
											isblock = 0;
										}
										else//GO!!
										{
											*(road_position + i * (roadPath_data[next_move[car_check]][1]) + min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]) - 1) = car_check+1;
											car_place[car_check] = next_move[car_check];
											Q[car_place[car_check]][0]++;
											break;
										}
									}
									//if (isblock_row == roadPath_data[next_move[car_check]][3])
									//{
										//std::cout << "Start car is blocked1." << std::endl;
										/*std::cout << isblock_row << " " << next_move[car_check] << " " << car_check << std::endl;
										for (int i = roadPath_data[next_move[car_check]][3];
											i < (roadPath_data[next_move[car_check]][3])*(roadPath_data[next_move[car_check]][6] + 1); i++)
										{
											for (int j = 0; j < min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j++)
											{
												std::cout << *(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) << " ";
											}
											std::cout << std::endl;
										}
										exit(0);*/
									//}
								}
							}
							if (carPath_data[car_check][1] == roadPath_data[next_move[car_check]][5])//0-n-1(first):back
							{
								int lane_position = 1;//0-smallRID2bigRID 1-bigRID2smallRID
								//int roadcar_num = CheckCarNum(road_position, next_move[car_check], roadPath_data, lane_position);
								//int roadcar_limitation = roadPath_data[next_move[car_check]][1] * roadPath_data[next_move[car_check]][3] / roadPath_data[next_move[car_check]][2];//Wait for improve
#ifdef NORMAL_STARTLIMIT
								int roadcar_num = CheckCarNum(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] * roadPath_data[next_move[car_check]][3] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef STRICT_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef ONEBYONE_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = LIMIT_NUM+(check_turn>2?1:0);//Wait for improve
#endif
								roadcar_limitation = roadcar_limitation < 1 ? 1 : roadcar_limitation;
								if (roadcar_num < roadcar_limitation)
								{
									//To the first lane which no block
									int isblock = 0;
									int isblock_row = 0;
									int get_target = 0;
									for (int i = (roadPath_data[next_move[car_check]][3]) - 1; i >= 0; i--)
									{
										for (int j = roadPath_data[next_move[car_check]][1] - 1; j >= roadPath_data[next_move[car_check]][1] - min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j--)
										{
											if (*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) != 0)
											{
												if(j != roadPath_data[next_move[car_check]][1] - 1)//GO!!
												{
													*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j + 1) = car_check+1;
													car_place[car_check] = next_move[car_check];
													Q[car_place[car_check]][1]++;
													get_target = 1;
													break;
												}
												else
												{
													isblock = 1;//Something block in start place
													break;
												}
											}
										}
										if(get_target)
											break;//check next car
										if (isblock != 0)
										{
											isblock_row++;
											isblock = 0;
										}
										else//GO!!
										{
											*(road_position + i * (roadPath_data[next_move[car_check]][1]) + roadPath_data[next_move[car_check]][1] - min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3])) = car_check+1;
											car_place[car_check] = next_move[car_check];
											Q[car_place[car_check]][1]++;
											break;
										}
									}
									//if (isblock_row == roadPath_data[next_move[car_check]][3])
									//{
										//std::cout << "Start car is blocked2." << std::endl;
										/*std::cout << check_turn << " " << isblock_row << " " << next_move[car_check] << " " << car_check << " " << roadcar_num << "<" << roadcar_limitation << std::endl;
										for (int i = (roadPath_data[next_move[car_check]][3]) - 1; i >= 0; i--)
										{
											for (int j = roadPath_data[next_move[car_check]][1] - 1; j >= roadPath_data[next_move[car_check]][1] - min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j--)
											{
												std::cout << *(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) << " ";
											}
											std::cout << std::endl;
										}
										exit(0);*/
									//}
								}
							}
						}
						else if (roadPath_data[next_move[car_check]][6] == 0)//One way
						{
							if (carPath_data[car_check][1] == roadPath_data[next_move[car_check]][4])//0(first)-(n-1):ahead
							{
								int lane_position = 0;//0-smallRID2bigRID 1-bigRID2smallRID
								//int roadcar_num = CheckCarNum(road_position, next_move[car_check], roadPath_data, lane_position);
								//int roadcar_limitation = roadPath_data[next_move[car_check]][1] * roadPath_data[next_move[car_check]][3] / roadPath_data[next_move[car_check]][2];//Wait for improve
#ifdef NORMAL_STARTLIMIT
								int roadcar_num = CheckCarNum(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] * roadPath_data[next_move[car_check]][3] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef STRICT_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = roadPath_data[next_move[car_check]][1] / roadPath_data[next_move[car_check]][2];//Wait for improve
#endif
#ifdef ONEBYONE_STARTLIMIT
								int roadcar_num = CheckCarNum_InUseLandAverage(road_position, next_move[car_check], roadPath_data, lane_position);
								int roadcar_limitation = LIMIT_NUM+(check_turn>2?1:0);//Wait for improve
#endif
								roadcar_limitation = roadcar_limitation < 1 ? 1 : roadcar_limitation;
								if (roadcar_num < roadcar_limitation)
								{
									//To the first lane which no block
									int isblock = 0;
									int isblock_row = 0;
									int get_target = 0;
									for (int i = 0; i < (roadPath_data[next_move[car_check]][3]); i++)
									{
										for (int j = 0; j < min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j++)
										{
											if (*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) != 0)
											{
												if(j != 0)//GO!!
												{
													*(road_position + i * (roadPath_data[next_move[car_check]][1]) + j - 1) = car_check+1;
													car_place[car_check] = next_move[car_check];
													Q[car_place[car_check]][0]++;
													get_target = 1;
													break;
												}
												else
												{
													isblock = 1;//Something block in start place
													break;
												}
											}
										}
										if(get_target)
											break;//check next car
										if (isblock != 0)
										{
											isblock_row++;
											isblock = 0;
										}
										else//GO!!
										{
											*(road_position + i * (roadPath_data[next_move[car_check]][1]) + min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]) - 1) = car_check+1;
											car_place[car_check] = next_move[car_check];
											Q[car_place[car_check]][0]++;
											break;
										}
									}
									//if (isblock_row == roadPath_data[next_move[car_check]][3])
									//{
										//std::cout << "Start car is blocked3." << std::endl;
										/*std::cout << isblock_row << " " << next_move[car_check] << " " << car_check << std::endl;
										for (int i = 0; i < (roadPath_data[next_move[car_check]][3]); i++)
										{
											for (int j = 0; j < min_compare(roadPath_data[next_move[car_check]][2], carPath_data[car_check][3]); j++)
											{
												std::cout << *(road_position + i * (roadPath_data[next_move[car_check]][1]) + j) << " ";
											}
											std::cout << std::endl;
										}
										exit(0);*/
									//}
								}
							}
							/*else
							{
								std::cout << "Really one way in MarkFunction_StartCar()?" << std::endl;
								//std::cout << car_place[car_check] << " " << roadPath_data[next_move[car_check]][4] << " " << next_move[car_check] << std::endl;
								//exit(0);
							}*/
						}
						/*else
							std::cout << "How many ways?" << std::endl;*/
					}
					/*else
					{
						std::cout << "Start Car entering the cross?" << std::endl;;//ALL block
					}*/
				}
				//else//Wait for check in the last of MarkFunction()
				//{
				//	priority[car_check] = 1;
				//}
			}
			//else
				//std::cout << "Not start car." << std::endl;
		}
	}
}

int RoadFunction(int *turn_overSpeed, int (*Q)[2], int *priority, std::vector<int*> roadMatrix_group, int *next_move, std::vector<int> car_speedChart, int *car_place, int *car_arrive, int (*carPath_data)[5], int (*roadPath_data)[7], int (*crossPath_data)[5])
{
	int marker[global_carPath_row] = {0};//0-wait 1-stop (-1)-arrive (-3)-inital
	for(int i = 0;i < global_carPath_row;i++)
	{
		marker[i] = -3;
	}
	MarkFunction(marker, roadMatrix_group, next_move, car_speedChart, car_place, carPath_data, roadPath_data, crossPath_data);
	//std::cout << "Here" << std::endl;
	int cross_roundCheck[2] = {0};
	int cross_roundCheckEnsure[2] = {0};
	CrossFunction(cross_roundCheckEnsure, cross_roundCheck, Q, marker, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
	int dead_lock = 0;
	while(cross_roundCheckEnsure[0] == 1 && cross_roundCheck[0] == 1)
	{
		cross_roundCheck[0] = 0;//Any wait car move, to 1
		cross_roundCheckEnsure[0] = 0;//Any wait lock happen, to 1
		lockRoad.clear();
		CrossFunction(cross_roundCheckEnsure, cross_roundCheck, Q, marker, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
		dead_lock++;
		//std::cout << "Lock Check: " << dead_lock << std::endl;
		if(cross_roundCheckEnsure[0] == 1 && cross_roundCheck[0] == 0)
		{
			std::cout << "Dead lock: " << dead_lock << std::endl;
			return 1;
		}
	}
	//std::cout << "There" << std::endl;
	int has_newcar = 0;
	for (int car_check = 0; car_check < global_carPath_row; car_check++)//Check if there are cars that didn't start
	{
		if (car_place[car_check] == -1)
		{
			has_newcar = 1;
			//std::cout << "has_newcar." << std::endl;
			break;
		}
	}
	if(has_newcar)
		StartCarFunction(turn_overSpeed, car_arrive, Q, priority, roadMatrix_group, next_move, car_speedChart, car_place, carPath_data, roadPath_data, crossPath_data);
	return 0;
}



#endif
