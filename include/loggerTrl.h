#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include "Eigen/Core"

namespace Trl{

std::string TimeStamp(void){
	std::time_t result = std::time(nullptr);
	std::ostringstream timeName;

#ifdef __linux__
	std::tm *pnow = std::localtime(&result);
	timeName << pnow->tm_year + 1900 << pnow->tm_mon + 1 << pnow->tm_mday
		<< pnow->tm_hour << pnow->tm_min << pnow->tm_sec;
#elif _WIN32
	std::tm pnow;
	localtime_s(&pnow,&result);
	timeName << pnow.tm_year + 1900 << pnow.tm_mon + 1 << pnow.tm_mday
		<< pnow.tm_hour << pnow.tm_min << pnow.tm_sec;
#else
	std::tm *pnow = std::localtime(&result);
#endif
	

	return std::string(timeName.str());
}

class Logger{
private:
	std::string fileName;
	std::ostringstream stack;
public:
	Logger();
	~Logger();
	Logger(const std::string name);

	void Write();

	void SetFileName(const std::string name);
	void SetData(Eigen::MatrixXd data);
	template <typename T> void SetData(const T data, const int num);
	template <typename T> void SetSentence(const T data);
};

Logger::Logger(){
	stack.str("");
}

Logger::~Logger(){
}

void Logger::Write(){
	if(fileName.size() == 0){
		std::cout << "*** file name no setting ***" << std::endl;
		std::cout << "*** cannot output data   ***" << std::endl;
	}

	fileName += "_" + TimeStamp() + ".txt";

	std::ofstream ofs(fileName.c_str(),std::ios::out);

	ofs << stack.str() << std::endl;

}

Logger::Logger(const std::string name){
	fileName = name;
}

void Logger::SetFileName(const std::string name){
	fileName = name;
}

void Logger::SetData(Eigen::MatrixXd data){
	for(int i = 0; i < data.rows();++i){
		for(int j = 0; j < data.cols(); ++j){
			stack << data(i,j) << "\t";
		}
		stack << std::endl;
	}
}

template <typename T> void Logger::SetSentence(const T data){
	stack << data << std::endl;
}

template <typename T> void Logger::SetData(const T data, const int num){
	for(int i = 0; i < num; ++i){
		stack << data[i];
		stack << "\t";
	}
	stack << std::endl;
}

}	//namespace Trl
