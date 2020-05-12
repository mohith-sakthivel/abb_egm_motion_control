
#include "egm_control.h"

void initWinSock() {
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		fprintf(stderr, "Could not open Windows connection.\n");
		exit(0);
	}
}

MechanicalUnit::MechanicalUnit(const std::string id, int port_no) {
	port = port_no;
	MechID = id;
}

MechanicalUnit::~MechanicalUnit() {
	if (log.is_open()) {
		log.close();
	}
}

inline void MechanicalUnit::EGMRecieve() {
	// receive message from mechanical unit
	n_recieve = recvfrom(sock, ProtoMessage, 1400, 0, (struct sockaddr *)&Addr, &len_recieve);
	if (n_recieve < 0)
	{
		printf("Error receive message\n");
	}
	fromMechUnit = new abb::egm::EgmRobot();
	// deserialize inbound message
	fromMechUnit->ParseFromArray(ProtoMessage, n_recieve);
}

inline void MechanicalUnit::EGMSend() {
	// send message to the mechanical unit
	toMechUnit->SerializeToString(&MessageBuffer);
	n_recieve = sendto(sock, MessageBuffer.c_str(), MessageBuffer.length(), 0, (struct sockaddr *)&Addr, int(sizeof(Addr)));
	if (n_recieve < 0)
	{
		printf("Error send message\n");
	}
	delete toMechUnit;
}

void Robot::initRobot() {
	/**
		Initialize UDP communication
		Read initial pose and joint values from the robot
	**/

	// create socket to listen from Robot
	sock = ::socket(AF_INET, SOCK_DGRAM, 0);
	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(port);
	// listen on all interfaces
	::bind(sock, (struct sockaddr*) & serverAddr, sizeof(serverAddr));
	len_recieve = sizeof(ProtoMessage);
	int n = recvfrom(sock, ProtoMessage, 1400, 0, (struct sockaddr*) & Addr, &len_recieve);
	StreamStartTime = std::chrono::steady_clock::now();
	if (n < 0)
	{
		std::cout << "Communication with " << MechID << " failed.\n\n";
		return;
	}
	abb::egm::EgmRobot* initRobMessage = new abb::egm::EgmRobot();
	initRobMessage->ParseFromArray(ProtoMessage, n);
	// Store the initial joint and cartesian values in the class members
	nextRobotPos[0] = RobotPos[0] = initRobotPos[0] = initRobMessage->feedback().cartesian().pos().x();
	nextRobotPos[1] = RobotPos[1] = initRobotPos[1] = initRobMessage->feedback().cartesian().pos().y();
	nextRobotPos[2] = RobotPos[2] = initRobotPos[2] = initRobMessage->feedback().cartesian().pos().z();
	nextRobotEuler[0] = RobotEuler[0] = initRobotEuler[0] = initRobMessage->feedback().cartesian().euler().x();
	nextRobotEuler[1] = RobotEuler[1] = initRobotEuler[1] = initRobMessage->feedback().cartesian().euler().y();
	nextRobotEuler[2] = RobotEuler[2] = initRobotEuler[2] = initRobMessage->feedback().cartesian().euler().z();
	nextRobotJoint[0] = RobotJoint[0] = initRobotJoint[0] = initRobMessage->feedback().joints().joints(0);
	nextRobotJoint[1] = RobotJoint[1] = initRobotJoint[1] = initRobMessage->feedback().joints().joints(1);
	nextRobotJoint[2] = RobotJoint[2] = initRobotJoint[2] = initRobMessage->feedback().joints().joints(2);
	nextRobotJoint[3] = RobotJoint[3] = initRobotJoint[3] = initRobMessage->feedback().joints().joints(3);
	nextRobotJoint[4] = RobotJoint[4] = initRobotJoint[4] = initRobMessage->feedback().joints().joints(4);
	nextRobotJoint[5] = RobotJoint[5] = initRobotJoint[5] = initRobMessage->feedback().joints().joints(5);
	delete initRobMessage;
	log.open(MechID.append(" - log.txt"), std::ios::trunc);
}

inline void Robot::RobotSetCartesian(bool SetSpeed) {
	/**
		Adds cartesian position and speed values to
		a PC outbound message to guide the robot
	**/
	toMechUnit = new abb::egm::EgmSensor();

	abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(seq_no++);
	header->set_tm((unsigned __int32)GetTickCount64());

	toMechUnit->set_allocated_header(header);

	//Set cartesian space position values for the robot
	abb::egm::EgmCartesian *robc = new abb::egm::EgmCartesian();
	robc->set_x(RobotPos[0]);
	robc->set_y(RobotPos[1]);
	robc->set_z(RobotPos[2]);
	abb::egm::EgmEuler *robeu = new abb::egm::EgmEuler();
	robeu->set_x(RobotEuler[0]);
	robeu->set_y(RobotEuler[1]);
	robeu->set_z(RobotEuler[2]);
	abb::egm::EgmPose *robcartesian = new abb::egm::EgmPose();
	robcartesian->set_allocated_euler(robeu);
	robcartesian->set_allocated_pos(robc);
	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	planned->set_allocated_cartesian(robcartesian);
	toMechUnit->set_allocated_planned(planned);

	if (SetSpeed) {
		//Set cartesian space speed values for the robot
		abb::egm::EgmCartesianSpeed *CartesianSpeed = new abb::egm::EgmCartesianSpeed();
		CartesianSpeed->add_value(RobotPosSpeed[0]);
		CartesianSpeed->add_value(RobotPosSpeed[1]);
		CartesianSpeed->add_value(RobotPosSpeed[2]);
		CartesianSpeed->add_value(RobotEulerSpeed[0]);
		CartesianSpeed->add_value(RobotEulerSpeed[1]);
		CartesianSpeed->add_value(RobotEulerSpeed[2]);
		abb::egm::EgmSpeedRef *speedRef = new abb::egm::EgmSpeedRef();
		speedRef->set_allocated_cartesians(CartesianSpeed);
		toMechUnit->set_allocated_speedref(speedRef);
	}
}

inline void Robot::RobotSetJoint(bool SetSpeed) {
	/**
		Add joint values and joint speed to a
		PC outbound message to guide the robot
	**/
	toMechUnit = new abb::egm::EgmSensor();

	abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(seq_no++);
	header->set_tm((unsigned __int32)GetTickCount64());

	toMechUnit->set_allocated_header(header);

	//Set joint values for the robot
	abb::egm::EgmJoints *rob_ax = new abb::egm::EgmJoints();
	rob_ax->add_joints(RobotJoint[0]);
	rob_ax->add_joints(RobotJoint[1]);
	rob_ax->add_joints(RobotJoint[2]);
	rob_ax->add_joints(RobotJoint[3]);
	rob_ax->add_joints(RobotJoint[4]);
	rob_ax->add_joints(RobotJoint[5]);

	//Set external axis joint value
	abb::egm::EgmJoints* track_eax = new abb::egm::EgmJoints();
	track_eax->add_joints(RobotJoint[0]);


	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	planned->set_allocated_joints(rob_ax);
	planned->set_allocated_externaljoints(track_eax);
	toMechUnit->set_allocated_planned(planned);

	if (SetSpeed) {
		//Set joint speed values for the robot
		abb::egm::EgmJoints *rob_ax_speed = new abb::egm::EgmJoints();
		rob_ax_speed->add_joints(RobotJointSpeed[0]);
		rob_ax_speed->add_joints(RobotJointSpeed[1]);
		rob_ax_speed->add_joints(RobotJointSpeed[2]);
		rob_ax_speed->add_joints(RobotJointSpeed[3]);
		rob_ax_speed->add_joints(RobotJointSpeed[4]);
		rob_ax_speed->add_joints(RobotJointSpeed[5]);

		abb::egm::EgmJoints* ExtJointSpeed = new abb::egm::EgmJoints();
		ExtJointSpeed->add_joints(RobotJointSpeed[0]);

		abb::egm::EgmSpeedRef *speedRef = new abb::egm::EgmSpeedRef();
		speedRef->set_allocated_joints(rob_ax_speed);
		speedRef->set_allocated_externaljoints(ExtJointSpeed);
		toMechUnit->set_allocated_speedref(speedRef);
	}
}

inline void Robot::LogCartesianData(int msg_no, bool Readable) {
	/**
		Writes the cartesian feedback data from the inbound robot message
		Inputs:
			msg_no - number of the message
	**/
	if (msg_no == 1) {
		first_msg_time = fromMechUnit->header().tm();
		if (!Readable) {
			log << "Msg No.,Robot Seq No.,Time,set x, set y, set z, set eux, set euy, set euz, planned x, planned y, planned z, planned eux, planned euy, planned euz, Feedback x, Feedback y, Feedback z, Feedback eux, Feedback euy, Feedback euz " << std::endl;
		}
	}
	if (fromMechUnit->has_header() && fromMechUnit->header().has_seqno() &&
		fromMechUnit->header().has_tm() && fromMechUnit->header().has_mtype())
	{
		float time_s = float((fromMechUnit->header().tm() - first_msg_time) / 1000.0);
		if (Readable) {
			//print header
			log << "Msg No. = " << msg_no << "\t Robot Seq No. = " << fromMechUnit->header().seqno() << "\t Time = " << time_s << std::endl;
			//print set value cartesian value
			log << "set value :\n" << std::fixed << std::setprecision(4)
				<< "x: " << RobotPos[0]
				<< "\ty: " << RobotPos[1]
				<< "\tz: " << RobotPos[2] << std::endl
				<< "eu x: " << RobotEuler[0]
				<< "\teu y: " << RobotEuler[1]
				<< "\teu z: " << RobotEuler[2] << std::endl;
			//print planned cartesian value
			log << "planned :\n" << std::fixed << std::setprecision(4)
				<< "x: " << fromMechUnit->planned().cartesian().pos().x()
				<< "\ty: " << fromMechUnit->planned().cartesian().pos().y()
				<< "\tz: " << fromMechUnit->planned().cartesian().pos().z() << std::endl
				<< "eu x: " << fromMechUnit->planned().cartesian().euler().x()
				<< "\teu y: " << fromMechUnit->planned().cartesian().euler().y()
				<< "\teu z: " << fromMechUnit->planned().cartesian().euler().z() << std::endl;
			//print feedback cartesian value
			log << "feedback :\n" << std::fixed << std::setprecision(4)
				<< "x: " << fromMechUnit->feedback().cartesian().pos().x()
				<< "\ty: " << fromMechUnit->feedback().cartesian().pos().y()
				<< "\tz: " << fromMechUnit->feedback().cartesian().pos().z() << std::endl
				<< "eu x: " << fromMechUnit->feedback().cartesian().euler().x()
				<< "\teu y: " << fromMechUnit->feedback().cartesian().euler().y()
				<< "\teu z: " << fromMechUnit->feedback().cartesian().euler().z() << std::endl << std::endl;
		}
		else {
			log << msg_no << "," << fromMechUnit->header().seqno() << "," << std::fixed << std::setprecision(4) << time_s << "," 
				<< RobotPos[0] << ","
				<< RobotPos[1] << ","
				<< RobotPos[2] << ","
				<< RobotEuler[0] << ","
				<< RobotEuler[1] << ","
				<< RobotEuler[2] << ","
				<< fromMechUnit->planned().cartesian().pos().x() << "," 
				<< fromMechUnit->planned().cartesian().pos().y() << ","
				<< fromMechUnit->planned().cartesian().pos().z() << "," 
				<< fromMechUnit->planned().cartesian().euler().x() << ","
				<< fromMechUnit->planned().cartesian().euler().y() << "," 
				<< fromMechUnit->planned().cartesian().euler().z() << "," 
				<< fromMechUnit->feedback().cartesian().pos().x() << "," 
				<< fromMechUnit->feedback().cartesian().pos().y() << "," 
				<< fromMechUnit->feedback().cartesian().pos().z() << "," 
				<< fromMechUnit->feedback().cartesian().euler().x() << ","
				<< fromMechUnit->feedback().cartesian().euler().y() << "," 
				<< fromMechUnit->feedback().cartesian().euler().z() << std::endl;
		}
	}
}

inline void Robot::LogJointData(int msg_no, bool Readable) {
	/**
		Writes the Joint space feedback data from the inbound robot message
		Inputs:
			msg_no - number of the message
	**/
	if (msg_no == 1) {
		first_msg_time = fromMechUnit->header().tm();
		if (!Readable) {
			log << "Msg No.,Robot Seq No.,Time, set J1, set J2, set J3, set J4, set J5, set J6, planned J1, planned J2, planned J3, planned J4, planned J5, planned J6,Feedback J1, Feedback J2,Feedback J3, Feedback J4, Feedback J5, Feedback J6 " << std::endl;
		}
	}
	if (fromMechUnit->has_header() && fromMechUnit->header().has_seqno() &&
		fromMechUnit->header().has_tm() && fromMechUnit->header().has_mtype())
	{
		float time_s = float((fromMechUnit->header().tm() - first_msg_time) / 1000.0);
		if (Readable) {
			//print header
			log << "Msg No. = " << msg_no << "\t Robot Seq No. = " << fromMechUnit->header().seqno() << "\t Time = " << time_s << std::endl;
			//print set joint values
			log << "set values :\n" << std::fixed << std::setprecision(4)
				<< "J1:  " << RobotJoint[0]
				<< "\tJ2:  " << RobotJoint[1]
				<< "\tJ3:  " << RobotJoint[2]
				<< "\nJ4:  " << RobotJoint[3]
				<< "\tJ5:  " << RobotJoint[4]
				<< "\tJ6:  " << RobotJoint[5] << std::endl;
			//print planned joint values
			log << "planned :\n" << std::fixed << std::setprecision(4)
				<< "J1:  " << fromMechUnit->planned().joints().joints(0)
				<< "\tJ2:  " << fromMechUnit->planned().joints().joints(1)
				<< "\tJ3:  " << fromMechUnit->planned().joints().joints(2)
				<< "\nJ4:  " << fromMechUnit->planned().joints().joints(3)
				<< "\tJ5:  " << fromMechUnit->planned().joints().joints(4)
				<< "\tJ6:  " << fromMechUnit->planned().joints().joints(5) << std::endl;
			//print feedback joint values
			log << "feedback :\n" << std::fixed << std::setprecision(4)
				<< "J1:  " << fromMechUnit->feedback().joints().joints(0)
				<< "\tJ2:  " << fromMechUnit->feedback().joints().joints(1)
				<< "\tJ3:  " << fromMechUnit->feedback().joints().joints(2)
				<< "\nJ4:  " << fromMechUnit->feedback().joints().joints(3)
				<< "\tJ5:  " << fromMechUnit->feedback().joints().joints(4)
				<< "\tJ6:  " << fromMechUnit->feedback().joints().joints(5) << std::endl << std::endl;
		}
		else {
			log << msg_no << "," << fromMechUnit->header().seqno() << "," << std::fixed << std::setprecision(4) << time_s << "," 
				<< RobotJoint[0] << ","
				<< RobotJoint[1] << ","
				<< RobotJoint[2] << ","
				<< RobotJoint[3] << ","
				<< RobotJoint[4] << ","
				<< RobotJoint[5] << ","
				<< fromMechUnit->planned().joints().joints(0) << "," 
				<< fromMechUnit->planned().joints().joints(1) << "," 
				<< fromMechUnit->planned().joints().joints(2) << "," 
				<< fromMechUnit->planned().joints().joints(3) << ","
				<< fromMechUnit->planned().joints().joints(4) << "," 
				<< fromMechUnit->planned().joints().joints(5) << "," 
				<< fromMechUnit->feedback().joints().joints(0) << "," 
				<< fromMechUnit->feedback().joints().joints(1) << "," 
				<< fromMechUnit->feedback().joints().joints(2) << "," 
				<< fromMechUnit->feedback().joints().joints(3) << ","
				<< fromMechUnit->feedback().joints().joints(4) << "," 
				<< fromMechUnit->feedback().joints().joints(5) << std::endl;
		}
	}
}

void Robot::WriteCycleCartesian(float* cycle_time) {
	std::chrono::time_point<std::chrono::steady_clock> start_write_cartesian = std::chrono::steady_clock::now();
	RobotSetCartesian();
	EGMSend();
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_write_cartesian).count() / 1000000.0);
}

void Robot::FeedbackCycleCartesian(int msg_no,float* cycle_time, bool Readable) {
	std::chrono::time_point<std::chrono::steady_clock> start_feedback_cartesian = std::chrono::steady_clock::now();
	EGMRecieve();
	LogCartesianData(msg_no, Readable);
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_feedback_cartesian).count() / 1000000.0);
}

void Robot::WriteCycleJoint(float* cycle_time) {
	std::chrono::time_point<std::chrono::steady_clock> start_write_joint = std::chrono::steady_clock::now();
	RobotSetJoint();
	EGMSend();
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_write_joint).count() / 1000000.0);
}

void Robot::FeedbackCycleJoint(int msg_no, float* cycle_time, bool Readable) {
	std::chrono::time_point<std::chrono::steady_clock> start_feedback_joint = std::chrono::steady_clock::now();
	EGMRecieve();
	LogJointData(msg_no,Readable);
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_feedback_joint).count() / 1000000.0);
}

void Track::initTrack() {
	/**
		Initialize UDP communication
		Read the inital joint value from the track
	**/

	// create socket to listen from Robot
	sock = ::socket(AF_INET, SOCK_DGRAM, 0);
	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(port);
	// listen on all interfaces
	::bind(sock, (struct sockaddr*) & serverAddr, sizeof(serverAddr));
	len_recieve = sizeof(ProtoMessage);
	int n = recvfrom(sock, ProtoMessage, 1400, 0, (struct sockaddr*) & Addr, &len_recieve);
	if (n < 0)
	{
		std::cout << "Communication with " << MechID << " failed.\n\n";
		return;
	}
	abb::egm::EgmRobot* initTrackMessage = new abb::egm::EgmRobot();
	initTrackMessage->ParseFromArray(ProtoMessage, n);
	// Store the initial joint and cartesian values in the class members
	nextTrackJoint = TrackJoint = initTrackJoint = initTrackMessage->feedback().externaljoints().joints(0);
	delete initTrackMessage;
	log.open(MechID.append(" - log.txt"), std::ios::trunc);
}

inline void Track::TrackSetJoint(bool SetSpeed) {
	/**
		Add an exteral axis joint value and speed 
		to a PC outbound message
	**/
	toMechUnit = new abb::egm::EgmSensor();

	abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(seq_no++);
	header->set_tm((unsigned __int32)GetTickCount64());
	toMechUnit->set_allocated_header(header);

	//Set external axis joint value
	abb::egm::EgmJoints *track_eax = new abb::egm::EgmJoints();
	track_eax->add_joints(TrackJoint);
	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	planned->set_allocated_externaljoints(track_eax);
	toMechUnit->set_allocated_planned(planned);

	if (SetSpeed) {
		//Set external axis joint speed
		abb::egm::EgmJoints *ExtJointSpeed = new abb::egm::EgmJoints();
		ExtJointSpeed->add_joints(TrackJointSpeed);
		abb::egm::EgmSpeedRef *setSpeed = new abb::egm::EgmSpeedRef();
		setSpeed->set_allocated_externaljoints(ExtJointSpeed);
		toMechUnit->set_allocated_speedref(setSpeed);
	}
}

inline void Track::LogTrackData(int msg_no, bool Readable) {
	/**
		Displays the joint data from the inbound track message
	**/
	if (msg_no == 1) {
		first_msg_time = fromMechUnit->header().tm();
		if (!Readable){
			//log << "Msg No, Track Seq No., Time, planned, feedback" << std::endl;
		}
	}
	if (fromMechUnit->has_header() && fromMechUnit->header().has_seqno() &&
		fromMechUnit->header().has_tm() && fromMechUnit->header().has_mtype())
	{
		float time_ms = float((fromMechUnit->header().tm() - first_msg_time) / 1000.0);
		if (Readable) {
			// print header
			log << "Msg No. = " << msg_no << "\t Track Seq No. = " << fromMechUnit->header().seqno() 
				<< "\t Time =" << time_ms << std::endl;
			//print planned and feedback external axis values
			log << std::fixed << std::setprecision(4) << "planned: " << fromMechUnit->planned().externaljoints().joints(0) 
				<< "\tfeedback: " << fromMechUnit->feedback().externaljoints().joints(0) << std::endl << std::endl;
		}
		else {
			log << msg_no << "," << fromMechUnit->header().seqno() << "," << std::fixed << std::setprecision(4) << time_ms << "," 
				<< fromMechUnit->planned().externaljoints().joints(0) << "," 
				<< fromMechUnit->feedback().externaljoints().joints(0) << std::endl;
		}
		}
	else
	{
		log << "No track header\n\n";
	}
	delete fromMechUnit;
}

void Track::WriteCycleTrack(float* cycle_time) {
	std::chrono::time_point<std::chrono::steady_clock> start_write = std::chrono::steady_clock::now();
	TrackSetJoint();
	EGMSend();
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_write).count() / 1000000.0);
}

void Track::FeedbackCycleTrack(int msg_no, float* cycle_time, bool Readable) {
	std::chrono::time_point<std::chrono::steady_clock> start_feedback = std::chrono::steady_clock::now();
	EGMRecieve();
	LogTrackData(msg_no, Readable);
	if (cycle_time != nullptr)
		*cycle_time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_feedback).count() / 1000000.0);
}