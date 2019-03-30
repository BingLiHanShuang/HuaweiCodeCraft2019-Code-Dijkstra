#include "iostream"

#include <map>
#include <vector>
#include <string>
#include <fstream>

#include "ReadFile_tools.hpp"
#include "Dijkstra.hpp"
#include "RoadFunction_tools.hpp"

#define max_compare(a,b)	(((a) > (b)) ? (a) : (b))
#define TIMELIMIT	100000

#define LOCKSOLVE_NUM 4

typedef float* Matrix_f2v2;
typedef int* Matrix_i2v2;
const float INF = 9999;

int global_carPath_row = 0;
int global_roadPath_row = 0;
int global_crossPath_row = 0;

std::vector<int> lockRoad;

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);
	
	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;
	
	// TODO:read input filebuf
	std::ifstream carPath_read;
	const int carPath_row = CountLines(carPath) - 1;
	global_carPath_row = carPath_row;
	int carPath_data[carPath_row][5] = {0};
	readInToMatrix_5(carPath_read, carPath, carPath_data);
	
	std::ifstream roadPath_read;
	const int roadPath_row = CountLines(roadPath) - 1;
	global_roadPath_row = roadPath_row;
	int roadPath_data[roadPath_row][7] = {0};
	readInToMatrix_7(roadPath_read, roadPath, roadPath_data);//0-n-1(first):back n(first)-(2n-1):ahead
	
	std::ifstream crossPath_read;
	const int crossPath_row = CountLines(crossPath) - 1;
	global_crossPath_row = crossPath_row;
	int crossPath_data[crossPath_row][5] = {0};
	readInToMatrix_5(crossPath_read, crossPath, crossPath_data);
	
	// TODO:process
	//std::map<int, std::vector<>> answer;//Save answer into this vector
	std::vector<std::vector<int>> answer(carPath_row);//Save answer into this vector
	int recorded[carPath_row] = {0};
	
	std::vector<Matrix_i2v2> roadMatrix_group;
	for(int create_road = 0;create_road < roadPath_row;create_road++)//roadPath_row
	{
		//0-n-1(first):From roadPath_data[][5] to roadPath_data[][4], n(first)-(2n-1):From roadPath_data[][4] to roadPath_data[][5]
		Matrix_i2v2 road_create = (Matrix_i2v2)std::malloc(sizeof(int)*(roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1))*(roadPath_data[create_road][1]));//Create Single Road Matrix
		for(int i = 0;i < (roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1));i++)
		{
			for(int j = 0;j < (roadPath_data[create_road][1]);j++)
			{
				*(road_create + i*(roadPath_data[create_road][1]) + j) = 0;
			}
		}
		roadMatrix_group.emplace_back(road_create);
	}
	
	std::vector<Matrix_i2v2> roadMatrix_group_last;
	for(int create_road = 0;create_road < roadPath_row;create_road++)//roadPath_row
	{
		//0-n-1(first):From roadPath_data[][5] to roadPath_data[][4], n(first)-(2n-1):From roadPath_data[][4] to roadPath_data[][5]
		Matrix_i2v2 road_create = (Matrix_i2v2)std::malloc(sizeof(int)*(roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1))*(roadPath_data[create_road][1]));//Create Single Road Matrix
		for(int i = 0;i < (roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1));i++)
		{
			for(int j = 0;j < (roadPath_data[create_road][1]);j++)
			{
				*(road_create + i*(roadPath_data[create_road][1]) + j) = 0;
			}
		}
		roadMatrix_group_last.emplace_back(road_create);
	}
	
	//int callback = 0;
	int car_arrive[carPath_row] = {0};//Not_arrive:0 Arrive:1
	int car_place[carPath_row] = {0};//0-(carPath_row-1) Record which road the car stay in the present
	for(int i = 0;i < carPath_row;i++)
	{
		car_place[i] = -1;//Arrive:-2
	}
	int Q[roadPath_row][2] = {0};//Count number of cars that pass the cross from a road, 0:smallCross2bigCross 1:bigCross2smallCross
	int turn_overSpeed[2] = {0};
	
	//1. Calculate the priority of cars
	int priority[carPath_row] = {0};//0-first 1-second 2-...
	std::vector<int> car_speedAll;
	std::vector<int> car_speedChart;
	for(int i = 0;i < carPath_row;i++)
	{
		car_speedAll.emplace_back(carPath_data[i][3]);
	}
	std::sort(car_speedAll.begin(), car_speedAll.end());
	for (std::vector<int>::iterator it = car_speedAll.begin(); it != car_speedAll.end(); it++)
	{
		if(std::find(car_speedChart.begin(),car_speedChart.end(),*it) == car_speedChart.end())
		{
			car_speedChart.emplace_back(*it);
		}
	}
	std::sort(car_speedChart.begin(), car_speedChart.end());
	//for (std::vector<int>::iterator it = car_speedChart.begin(); it != car_speedChart.end(); it++)
		//std::cout << *it << std::endl;
	int decrease_priority = car_speedChart.size();
	std::vector<int> car_speedChart_copy(car_speedChart);
	for (int check_turn = 0; check_turn < decrease_priority-1; check_turn++)//Give priority: 0-first 1-second 2-...
	{
		car_speedChart_copy.pop_back();
		for (int car_check = 0; car_check < carPath_row; car_check++)//Give priority: 0-first 1-second 2-...
		{
			if (carPath_data[car_check][3] == car_speedChart_copy.back())
			{
				priority[car_check] = check_turn + 1;
			}
		}
	}
	
	//2. Calculate the cars which may pass the cross(including those are blocked) in the timestamp
	int next_move[carPath_row] = {0};//Road order:0-n-1
	for (int i = 0; i < carPath_row; i++)
	{
		next_move[i] = -1;
	}
	for(int timestamp = 0;timestamp < TIMELIMIT;timestamp++)//carPath_row
	{
		std::cout << "timestamp: " << timestamp << std::endl;
		int arrive_count = 0;
		for(int i = 0; i < carPath_row;i++)
			arrive_count += car_arrive[i];
		if(arrive_count == carPath_row)
		{
			std::cout << "Congratulation: " << timestamp << std::endl;
			break;
		}
		
		//Save last timestamp data for return
		for(int create_road = 0;create_road < roadPath_row;create_road++)//roadPath_row
		{
			std::memcpy(roadMatrix_group_last[create_road], roadMatrix_group[create_road], sizeof(int)*(roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1))*(roadPath_data[create_road][1]));
		}
		int car_arrive_last[carPath_row];
		int car_place_last[carPath_row];
		int Q_last[roadPath_row][2];
		int turn_overSpeed_last[2];
		std::memset(car_arrive_last, 0, carPath_row * sizeof(int));
		std::memcpy(car_arrive_last, car_arrive, carPath_row * sizeof(int));
		std::memset(car_place_last, 0, carPath_row * sizeof(int));
		std::memcpy(car_place_last, car_place, carPath_row * sizeof(int));
		std::memset(Q_last, 0, roadPath_row * 2 * sizeof(int));
		std::memcpy(Q_last, Q, roadPath_row * 2 * sizeof(int));
		std::memset(turn_overSpeed_last, 0, 2 * sizeof(int));
		std::memcpy(turn_overSpeed_last, turn_overSpeed, 2 * sizeof(int));
		
		int isDeadLock = 1;
		int isLock = 0;
		
		while(isDeadLock)
		{
			for(int car_check = 0;car_check < carPath_row;car_check++)//carPath_row
			{
				if(!car_arrive[car_check])
				{
					if(timestamp >= carPath_data[car_check][4])
					{
						//std::cout << "car_place: " << car_place[car_check] << std::endl;
						//std::cout << "car_check: " << car_check << " " << std::endl;
						int next_crossNum = Car_nextCrossCheck(car_check+1, car_place[car_check], roadMatrix_group, roadPath_data, crossPath_data, carPath_data[car_check][1]);
						//std::cout << "next_crossNum: " << next_crossNum << std::endl;
						int endPoint_order = -1;
						for(int search = 0; search < global_crossPath_row; search++)
						{
							if(crossPath_data[search][0] == carPath_data[car_check][2])
							{
								endPoint_order = search;
								break;
							}
						}
						if((car_place[car_check] == -1)
								|| (next_crossNum != endPoint_order && isEnteringCross(car_check, carPath_data[car_check], car_place[car_check], roadMatrix_group, roadPath_data, crossPath_data)))
						{
							int CrossOrder[crossPath_row] = {0};
							Matrix_f2v2 map = (Matrix_f2v2)std::malloc(sizeof(float)*crossPath_row*crossPath_row);//记录点到点
							
							//float distance[crossPath_row] = {0};//代表走到位置 i 需要最少走多远
							//bool flag[crossPath_row] = {0};//标记是否遍历过
							
							for(int i = 0;i < crossPath_row;i++)
							{
								for(int j = 0;j < crossPath_row;j++)
								{
									*(map + i*crossPath_row + j) = INF;
								}
							}
							/*for(int i = 0;i < crossPath_row;i++)
							{
								distance[i] = INF;
							}*/
							/*for(int i = 0;i < crossPath_row;i++)
							{
								flag[i] = false;
							}*/
							//std::cout << "here1" << std::endl;
							//3. Update the RoadWeight
							RoadWeight_Changeable(Q, roadMatrix_group, map, crossPath_row, crossPath_row, roadPath_data, carPath_data[car_check], crossPath_data);
							
							if(next_crossNum == -1)
								std::cout << "Car_nextCrossCheck Error" << std::endl;
							//std::cout << "here2" << std::endl;
							if(car_place[car_check] != -1)
							{
								int self_start = 0;
								int self_end = 0;
								for(int search = 0; search < global_crossPath_row; search++)
								{
									if(crossPath_data[search][0] == roadPath_data[car_place[car_check]][4])
									{
										self_start = search;
										break;
									}
								}
								for(int search = 0; search < global_crossPath_row; search++)
								{
									if(crossPath_data[search][0] == roadPath_data[car_place[car_check]][5])
									{
										self_end = search;
										break;
									}
								}
								*(map + self_start*crossPath_row + self_end) = INF;
								*(map + self_end*crossPath_row + self_start) = INF;
							}
							if(isLock == 1)
							{
								for(int map_check = 0; map_check < (int)lockRoad.size(); map_check++)
								{
									int lock_start = 0;
									int lock_end = 0;
									for(int search = 0; search < global_crossPath_row; search++)
									{
										if(crossPath_data[search][0] == roadPath_data[lockRoad[map_check]][4])
										{
											lock_start = search;
											break;
										}
									}
									for(int search = 0; search < global_crossPath_row; search++)
									{
										if(crossPath_data[search][0] == roadPath_data[lockRoad[map_check]][5])
										{
											lock_end = search;
											break;
										}
									}
									*(map + lock_start*crossPath_row + lock_end) = *(map + lock_start*crossPath_row + lock_end)*LOCKSOLVE_NUM;
									*(map + lock_end*crossPath_row + lock_start) = *(map + lock_end*crossPath_row + lock_start)*LOCKSOLVE_NUM;
									//std::cout << "lockRoad[map_check]:" << lockRoad[map_check] << std::endl;
								}
							}
							
							//std::cout << next_crossNum-1 << std::endl;
							int endPoint = 0;
							for(int search = 0; search < global_crossPath_row; search++)
							{
								if(crossPath_data[search][0] == carPath_data[car_check][2])
								{
									endPoint = search;
									break;
								}
							}
							//std::cout << "here1" << std::endl;
							//std::cout << "next_crossNum: " << next_crossNum << std::endl;
							//Dijkstra(CrossOrder, next_crossNum, endPoint, crossPath_row, map, distance, flag);
							Dijkstra_new(CrossOrder, next_crossNum, endPoint, crossPath_row, map);
							//std::cout << "here2" << std::endl;
							int AnswerRoadCount = 0;
							for(int i = 0;i < crossPath_row;i++)
							{
								if(CrossOrder[i])
								{
									AnswerRoadCount++;
									//std::cout << "CrossOrder[i]" << CrossOrder[i] << std::endl;
								}
							}
							//std::cout << std::endl;
							//std::cout << "AnswerRoadCount" << AnswerRoadCount << std::endl;
							int RoadOrder[AnswerRoadCount-1] = {0};//road ID
							for(int i = 0;i < crossPath_row-1;i++)
							{
								if(!CrossOrder[i])
									break;
								for(int j = 1;j < 5;j++)
								{
									for(int k = 1;k < 5;k++)
									{
										if((crossPath_data[CrossOrder[i]-1][j] == crossPath_data[CrossOrder[i+1]-1][k]) && (crossPath_data[CrossOrder[i]-1][j] != -1))
										{
											RoadOrder[i] = crossPath_data[CrossOrder[i]-1][j];
										}
									}
								}
							}
							
							//4. Record the Dijkstra result
							//next_move[car_check] = RoadOrder[0];
							for (int road_check = 0; road_check < roadPath_row; road_check++)//Check if road is full
							{
								if (roadPath_data[road_check][0] == RoadOrder[0])
								{
									next_move[car_check] = road_check;
								}
							}
							free(map);
						}
						else if(next_move[car_check] != -1)
						{
							//Remain
						}
						else
							next_move[car_check] = -1;
					}
					else
						next_move[car_check] = -1;
				}
				else
					next_move[car_check] = -1;
				//std::cout << next_move[car_check] << std::endl;
			}
			//std::cout << "next_move[17]:" << next_move[17] << " " << car_place[17] << std::endl;
			/*if(car_place[49] != -1)
			if(timestamp == 13)
			{
				exit(0);
			}*/
			
			for(int i = 0; i < roadPath_row; i++)//Clear Q
			{
				for(int j = 0; j < 2; j++)
				{
					Q[i][j] = 0;
				}
			}
			lockRoad.clear();
			//std::cout << "here3" << std::endl;
			//5. Run the Simulation
			isDeadLock = RoadFunction(turn_overSpeed, Q, priority, roadMatrix_group, next_move, car_speedChart, car_place, car_arrive, carPath_data, roadPath_data, crossPath_data);
			
			std::cout << "lockRoad.size():" << lockRoad.size() << std::endl;
			/*int watchRID = 59;
			Matrix_i2v2 road_position_test = roadMatrix_group[watchRID];
			for(int i_test = 0; i_test < (roadPath_data[watchRID][3]*(roadPath_data[watchRID][6]+1)); i_test++)
			{
				for(int j_test = 0; j_test < (roadPath_data[watchRID][1]); j_test++)
				{
					std::cout << *(road_position_test + i_test * (roadPath_data[watchRID][1]) + j_test) << " ";
				}
				std::cout << std::endl;
			}
			std::cout << "last: " << std::endl;
			Matrix_i2v2 road_position_test2 = roadMatrix_group_last[watchRID];
			for(int i_test = 0; i_test < (roadPath_data[watchRID][3]*(roadPath_data[watchRID][6]+1)); i_test++)
			{
				for(int j_test = 0; j_test < (roadPath_data[watchRID][1]); j_test++)
				{
					std::cout << *(road_position_test2 + i_test * (roadPath_data[watchRID][1]) + j_test) << " ";
				}
				std::cout << std::endl;
			}*/
			if(isDeadLock == 1)
			{
				isLock = 1;
				for(int create_road = 0;create_road < roadPath_row;create_road++)//roadPath_row
				{
					std::memcpy(roadMatrix_group[create_road], roadMatrix_group_last[create_road], sizeof(int)*(roadPath_data[create_road][3]*(roadPath_data[create_road][6]+1))*(roadPath_data[create_road][1]));
				}
				std::memcpy(car_arrive, car_arrive_last, carPath_row * sizeof(int));
				std::memcpy(car_place, car_place_last, carPath_row * sizeof(int));
				std::memcpy(Q, Q_last, roadPath_row * 2 * sizeof(int));
				std::memcpy(turn_overSpeed, turn_overSpeed_last, 2 * sizeof(int));
			}
			else
			{
				isLock = 0;
			}
		}
		
		for(int car_check = 0;car_check < carPath_row;car_check++)//carPath_row
		{
			if(car_arrive[car_check] == 1)
				continue;
			if(recorded[car_check] == 0 && car_place[car_check] != -1)
			{
				answer[car_check].emplace_back(carPath_data[car_check][0]);//Wait to improve
				answer[car_check].emplace_back(timestamp);
				recorded[car_check] = 1;
			}
			if(recorded[car_check] == 1 && car_place[car_check] != -1 && car_place[car_check] != -2 && (car_place[car_check]+5000) != answer[car_check].back())//Wait to improve
			{
				answer[car_check].emplace_back(roadPath_data[car_place[car_check]][0]);//Wait to improve
			}
		}
		
		/*if(timestamp == TIMELIMIT)
		{
			std::cout << "Reach Max TIMELIMIT" << std::endl;
		}*/
	}
	
	// TODO:write output file
	std::ofstream ofile;
	ofile.open(answerPath);
	ofile << "#(carId,StartTime,RoadId...)" << std::endl;
	
	for(int car_check = 0;car_check < carPath_row;car_check++)//carPath_row
	{
		ofile << "(";
		unsigned int i_out = 0;
		for(i_out = 0; i_out < answer[car_check].size()-1; i_out++)
		{
			ofile << answer[car_check][i_out] << ", ";
		}
		ofile << answer[car_check][i_out] << ")" << std::endl;
	}
	ofile.close();
	
	return 0;
}
