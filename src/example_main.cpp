/** 
*********************************************************************
	Author        : Mohith Sakthivel
	Date          : 04.06.2019
	Description   : Windows console application to implement 
					EGM control Robot and Track Setup
*********************************************************************
**/

#include <thread>
#include "egm_control.h"

const std::string ROB1_Name = "Robot 1";
const std::string TRACK1_Name = "Track 1";
const std::string ROB2_Name = "Robot 2";
const std::string TRACK2_Name = "Track 2";
const int ROB1_PORT = 7510;
const int TRACK1_PORT = 7520;
const int ROB2_PORT = 6510;
const int TRACK2_PORT = 6520;

// CSV File manipulation variables
std::vector<std::string> row;
std::string CSVfile = "ISRO Test Data.csv";
std::fstream inputFile;
std::string line, word;
bool IsReadable = true;

// Input file parameters
int msg_count = 1482;
int sampling_time = 4;

std::chrono::time_point<std::chrono::steady_clock> StartTime, EndTime, CycleStartTime, PrevCycleStartTime;

std::ofstream test;
float time_data[10] = { 0,0,0,0,0,0,0,0,0,0 };

bool readCSVinput (std::string filename) {
	if (!inputFile.is_open()) {
		inputFile.open(filename, std::fstream::in);
	}
	if (inputFile.good()) {
		row.clear();
		// read a line and store in row
		getline(inputFile, line);
		// used for breaking words
		std::stringstream rowstream(line);
		while (std::getline(rowstream, word, ',')) {
			row.push_back(word);
		}
		return true;
	}
	else {
		return false;
	}
}

inline void update_val (int count, Robot* sysRobot1, Track* sysTrack1, Robot* sysRobot2=nullptr, Track* sysTrack2=nullptr) {

	// print to screen to confirm write execution
	if (count % 50 == 0) {
		std::cout << ".";
		if (count % 1000 == 0) {
			std::cout << std::endl;
		}
	}

	// Read data from CSV file
	bool readfile = readCSVinput(CSVfile);
	if (readfile) {
		//calculate the next values
		if (sysTrack2 == nullptr) {
			sysTrack1->TrackJoint = stod(row[7]);
			sysTrack1->TrackJointSpeed = stod(row[10]);
			sysRobot1->RobotJoint[5] = sysTrack1->TrackJoint / 100;
			sysRobot1->RobotJointSpeed[5] = sysTrack1->TrackJointSpeed / 100;
		}
		else {
			sysTrack2->TrackJoint = sysTrack1->TrackJoint = stod(row[7]);
			sysTrack2->TrackJointSpeed = sysTrack1->TrackJointSpeed = stod(row[10]);
			sysRobot2->RobotJoint[5] = sysRobot1->RobotJoint[5] = sysTrack1->TrackJoint / 100;
			sysRobot2->RobotJointSpeed[5] = sysRobot1->RobotJointSpeed[5] = sysTrack1->TrackJointSpeed / 100;
		}

	}
	else {
		std::cout << "Read failed" << std::endl;
	}
}

inline void update_val_robo_only(int count, Robot* sysRobot1, Robot* sysRobot2 = nullptr) {

	// print to screen to confirm write execution
	if (count % 50 == 0) {
		std::cout << ".";
		if (count % 1000 == 0) {
			std::cout << std::endl;
		}
	}

	// Read data from CSV file
	bool readfile = readCSVinput(CSVfile);
	if (readfile) {
		//calculate the next values
		if (sysRobot2 == nullptr) {
			sysRobot1->RobotPos[2] = sysRobot1->initRobotPos[2] + stod(row[7]);
			sysRobot1->RobotPosSpeed[2] = stod(row[10]);
		}
		else {
			sysRobot1->RobotPos[2] = sysRobot1->initRobotPos[2] + stod(row[7]);
			sysRobot2->RobotPos[2] = sysRobot2->initRobotPos[2] + stod(row[7]);
			sysRobot2->RobotPosSpeed[2] = sysRobot1->RobotPosSpeed[2] = stod(row[10]);
		}
	}
	else {
		std::cout << "Read failed" << std::endl;
	}
}

int main(int argc, char* argv[]) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	initWinSock();

	test.open("test log.txt", std::ios::trunc);

	Robot* Rob1 = new  Robot(ROB1_Name, ROB1_PORT);
	Track* Track1 = new Track(TRACK1_Name, TRACK1_PORT);
	Robot* Rob2 = new  Robot(ROB2_Name, ROB2_PORT);
	Track* Track2 = new Track(TRACK2_Name, TRACK2_PORT);
	
	std::thread Robot1InitThread(&Robot::initRobot,Rob1);
	std::thread Track1InitThread(&Track::initTrack,Track1);
	std::thread Robot2InitThread(&Robot::initRobot, Rob2);
	std::thread Track2InitThread(&Track::initTrack, Track2);
	Robot1InitThread.join();
	Track1InitThread.join();
	Robot2InitThread.join();
	Track2InitThread.join();

	test << "Msg No, Total, Rob1 Read, Rob1 Write, Track1 Read, Track1 Write, Rob2 Read, Rob2 Write, Track2 Read, Track2 Write" << std::endl;
	
	while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - Rob1->StreamStartTime).count() / 1e6) <= sampling_time) {}
	
	PrevCycleStartTime = CycleStartTime = StartTime = std::chrono::steady_clock::now();


	for (int i = 1; i <= msg_count; i++) {

		// Read feedback through protobuf
		std::thread ReadRobot1([](Robot* Rob1, int i, float* TimeData, bool IsReadable) {Rob1->FeedbackCycleCartesian(i, TimeData, IsReadable); },Rob1,i,&time_data[1],IsReadable);
		std::thread ReadTrack1([](Track* Track1, int i, float* TimeData, bool IsReadable) {Track1->FeedbackCycleTrack(i, TimeData, IsReadable); },Track1,i, &time_data[3], IsReadable);
		std::thread ReadRobot2([](Robot* Rob2, int i, float* TimeData) {Rob2->FeedbackCycleCartesian(i, TimeData); }, Rob2, i, &time_data[5]);
		std::thread ReadTrack2([](Track* Track2, int i, float* TimeData) {Track2->FeedbackCycleTrack(i, TimeData); }, Track2, i, &time_data[7]);
		
		// wait for readthreads to join
		ReadRobot1.join();
		ReadTrack1.join();
		ReadRobot2.join();
		ReadTrack2.join();

		CycleStartTime = std::chrono::steady_clock::now();
		time_data[0] = float(std::chrono::duration_cast<std::chrono::nanoseconds>(CycleStartTime - PrevCycleStartTime).count() / 1e6);
		PrevCycleStartTime = CycleStartTime;

		update_val(i, Rob1, Track1, Rob2, Track2);

		// Send data through Protobuf
		std::thread WriteRobot1([](Robot* Rob1, float* TimeData) {Rob1->WriteCycleCartesian(TimeData); }, Rob1, &time_data[2]);
		std::thread WriteTrack1([](Track* Track1, float* TimeData) {Track1->WriteCycleTrack(TimeData); }, Track1, &time_data[4]);
		std::thread WriteRobot2([](Robot* Rob2, float* TimeData) {Rob2->WriteCycleJoint(TimeData); }, Rob2, &time_data[6]);
		std::thread WriteTrack2([](Track* Track2, float* TimeData) {Track2->WriteCycleTrack(TimeData); }, Track2, &time_data[8]);

		// wait for read and write threads to join
		WriteRobot1.join();
		WriteTrack1.join();
		WriteRobot2.join();
		WriteTrack2.join();


		// Hold the loop for sampling time
		//while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - CycleStartTime).count() / 1e6) <= sampling_time) {}
	}

	EndTime = std::chrono::steady_clock::now();
	std::cout << std::endl <<"Expected Runtime:" << ((msg_count*sampling_time)/1000.0) << std::endl <<
		"Execution Time:" << std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count() / 1e6 << " seconds" << std::endl;
	test << std::endl << std::endl << "Expected Runtime:" << ((msg_count * sampling_time) / 1000.0) << std::endl <<
		"Execution Time:" << std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count() / 1e6 << " seconds" << std::endl;

	delete Rob1;
	delete Track1;
	delete Rob2;
	delete Track2;
	std::cin.get();
	test.close();
	return 0;
}