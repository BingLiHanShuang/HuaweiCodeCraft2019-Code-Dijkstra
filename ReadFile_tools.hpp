#ifndef _READFILE_TOOLS_HPP_
#define _READFILE_TOOLS_HPP_

#include <fstream>
#include <iostream>
#include <cstring>

int CountLines(std::string filename)
{
	std::ifstream ReadFile;
	int n=0;
	std::string temp;
	ReadFile.open(filename,std::ios::in);//ios::in 表示以只读的方式读取文件
	if(ReadFile.fail())//文件打开失败:返回0
	{
	   return 0;
	}
	else//文件存在
	{
		while(std::getline(ReadFile,temp))
		{
			n++;
		}
		return n;
	}
	ReadFile.close();
	return 0;
}

void readInToMatrix_5(std::ifstream &in, std::string FilePath, int (*data)[5]) 
{
	in.open(FilePath, std::ios::in);//打开一个file
	if(in.fail()) 
	{
		std::cout << "Can not find " << FilePath << std::endl;
	}
	std::string buff;
	int i = 0;//行数i
	while(std::getline(in, buff)) 
	{
		std::vector<double> nums;
		// string->char *
		
		buff.erase(buff.length()-1, 1);
		buff.erase(0, 1);
		
		char *s_input = (char *)buff.c_str();
		const char * split = ", ";
		// 以‘，’为分隔符拆分字符串
		char *p = std::strtok(s_input, split);
		int a;
		if(i != 0)
		{
			while(p != NULL) 
			{
				// char * -> int
				a = std::atoi(p);
				//std::cout << a << std::endl;
				nums.push_back(a);
				p = std::strtok(NULL, split);
			}//end while
			for(unsigned int b = 0; b < nums.size(); b++) 
			{
				data[i-1][b] = nums[b];
			}//end for
		}
		i++;
	}//end while
	in.close();
	std::cout << "_5:Get data" << std::endl;
}

void readInToMatrix_7(std::ifstream &in, std::string FilePath, int (*data)[7]) 
{
	in.open(FilePath, std::ios::in);//打开一个file
	if(in.fail()) 
	{
		std::cout << "Can not find " << FilePath << std::endl;
	}
	std::string buff;
	int i = 0;//行数i
	while(std::getline(in, buff)) 
	{
		std::vector<int> nums;
		// string->char *
		
		buff.erase(buff.length()-1, 1);
		buff.erase(0, 1);
		
		char *s_input = (char *)buff.c_str();
		const char * split = ", ";
		// 以‘，’为分隔符拆分字符串
		char *p = std::strtok(s_input, split);
		int a;
		if(i != 0)
		{
			while(p != NULL) 
			{
				// char * -> int
				a = std::atoi(p);
				//std::cout << "a: " << a << std::endl;
				nums.push_back(a);
				p = std::strtok(NULL, split);
			}//end while
			for(unsigned int b = 0; b < nums.size(); b++) 
			{
				data[i-1][b] = nums[b];
			}//end for
		}
		i++;
	}//end while
	in.close();
	std::cout << "_7:Get data" << std::endl;
}



#endif
