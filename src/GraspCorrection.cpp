/*
 * GraspCorrection.cpp
 *
 *  Created on: Oct 9, 2013
 *      Author: miao
 */

#include "GraspCorrection.h"
//using namespace std;

ofstream FileTacF1;
ofstream FileTacF2;
ofstream FileTacF3;

ofstream FilePosF1;
ofstream FilePosF2;
ofstream FilePosF3;

ofstream FileStiff;

ofstream FileVF;

ofstream FileFinJnt;

GraspCorrectionClass::GraspCorrectionClass() {

	int argc = 0;
	char* argv = NULL;

	ros::init(argc, &argv, "RobustGrasp");

	if (ros::master::check()) {
		nH = new ros::NodeHandle;
		STacF1 = nH->subscribe("/finger1", 10,
				&GraspCorrectionClass::STacF1CallBack, this);
		//		cout<<"finger1"<<endl;
	}

	if (ros::master::check()) {
		STacF2 = nH->subscribe("/finger2", 10,
				&GraspCorrectionClass::STacF2CallBack, this);
		//		cout<<"finger1"<<endl;
	}

	if (ros::master::check()) {
		STacF3 = nH->subscribe("/finger3", 10,
				&GraspCorrectionClass::STacF3CallBack, this);
		//		cout<<"finger1"<<endl;
	}

	if (ros::master::check()) {
		UserCmd = nH->subscribe("/cmd_topic", 100,
				&GraspCorrectionClass::UserCmdCallBack, this);
		//		cout<<"cmdtopic"<<endl;
	}

	if (ros::master::check()) {
		SPosF1 = nH->subscribe("/objimpctrl/finger1_state", 100,
				&GraspCorrectionClass::SPosF1CallBack, this);
		//		cout<<"finger1Pos"<<endl;
	}
	if (ros::master::check()) {
		SPosF2 = nH->subscribe("/objimpctrl/finger2_state", 100,
				&GraspCorrectionClass::SPosF2CallBack, this);
		//		cout<<"finger2Pos"<<endl;
	}
	if (ros::master::check()) {
		SPosF3 = nH->subscribe("/objimpctrl/finger3_state", 100,
				&GraspCorrectionClass::SPosF3CallBack, this);
		//		cout<<"finger3Pos"<<endl;
	}
	if (ros::master::check()) {
		SVF = nH->subscribe("/objimpctrl/obj_pose_state", 100,
				&GraspCorrectionClass::SVFCallBack, this);
		//		cout<<"obj_pose_state"<<endl;
	}
	if (ros::master::check()) {
		SFinJnt = nH->subscribe("/allegro/joint_state", 100,
				&GraspCorrectionClass::SFinJntCallBack, this);
		//		cout<<"joint_state"<<endl;
	}

	if(ros::master::check()){
		GraspCorrectionPub=nH->advertise<GraspCorrection::GraspCorrectionMsg>("/GraspCorrectionTopic", 10);
	}

	if(ros::master::check()){
		GraspCorrectionPy=nH->advertise<std_msgs::String>("/GraspCorrectionTopicPy", 10);
	}

	// read the file for GraspCorrection;
	GraspStability = new Gaussians(13, 14,
			"./data/model_2/GraspCorrect_mu.txt",
			"./data/model_2/GraspCorrect_sigma.txt",
			"./data/model_2/GraspCorrect_prio.txt");
	GraspStability->InitFastGMR(0, 10, 11, 13);

	COEFF.Resize(57, 8);
	COEFF.Zero();
	ifstream input("./data/model_2/COEFF.txt");

	if (!input) {
		cout << "can not find the file COEFF!!" << endl;
	}
	char line[400];

	for (int linenb = 0; linenb < 57; linenb++) {

		input.getline(line, 400);
		istringstream strparse(line);
		for (int i = 0; i < 8; i++) // we only use the first 8 principle components
			strparse >> COEFF(linenb, i);
	}
	input.close();

	DataTacMean.Resize(57);
	input.open("./data/model_2/DataTacMean.txt");
	input.getline(line, 400);
	istringstream strparseMean(line);

	for (int i = 0; i < 57; i++)
		strparseMean >> DataTacMean(i);
	input.close();

	DataGMMMean.Resize(13,14);
	DataGMMMean.Load("./data/model_2/GraspCorrect_mu.txt");

	t1.Resize(57);
	input.open("./data/model_2/t1.txt");
	input.getline(line, 400);
	istringstream strparset1(line);
	for (int i = 0; i < 57; i++)
		strparset1 >> t1(i);
	input.close();

	t2.Resize(3);
	input.open("./data/model_2/t2.txt");
	input.getline(line, 400);
	istringstream strparset2(line);
	for (int i = 0; i < 3; i++)
		strparset2 >> t2(i);
	input.close();

	t3.Resize(3);
	input.open("./data/model_2/t3.txt");
	input.getline(line, 400);
	istringstream strparset3(line);
	for (int i = 0; i < 3; i++)
		strparset3 >> t3(i);
	input.close();

	ts.Resize(13);
	ts.Zero();
	input.open("./data/model_2/ts.txt");
	input.getline(line, 400);
	istringstream strparsets(line);
	for (int i = 0; i < 13; i++)
		strparset3 >> ts(i);
	input.close();

	tsDist.Resize(13);
	tsDist.Zero();

	MathLib::Vector tmptac;
	tmptac.Resize(19);

	VecTacF1Base.Resize(19);
	VecTacF1Base.Zero();
	input.open("./data/model_2/VecTacF1_0.txt");
	for (int linenb = 0; linenb < 100; linenb++) {
		input.getline(line, 400);
		istringstream strparseTacF1(line);
		for (int i = 0; i < 19; i++) // we only use the first 8 principle components
		{
			strparseTacF1 >> tmptac(i);
		}
		VecTacF1Base = VecTacF1Base + tmptac;
	}
	input.close();
	VecTacF1Base = VecTacF1Base / 100;

	VecTacF2Base.Resize(19);
	VecTacF2Base.Zero();
	input.open("./data/model_2/VecTacF2_0.txt");
	for (int linenb = 0; linenb < 100; linenb++) {
		input.getline(line, 400);
		istringstream strparseTacF2(line);
		for (int i = 0; i < 19; i++) // we only use the first 8 principle components
		{
			strparseTacF2 >> tmptac(i);
		}
		VecTacF2Base = VecTacF2Base + tmptac;
	}
	input.close();
	VecTacF2Base = VecTacF2Base / 100;

	VecTacF3Base.Resize(19);
	VecTacF3Base.Zero();
	input.open("./data/model_2/VecTacF3_0.txt");
	for (int linenb = 0; linenb < 100; linenb++) {
		input.getline(line, 400);
		istringstream strparseTacF3(line);
		for (int i = 0; i < 19; i++) // we only use the first 8 principle components
		{
			strparseTacF3 >> tmptac(i);
		}
		VecTacF3Base = VecTacF3Base + tmptac;
	}
	input.close();
	VecTacF3Base = VecTacF3Base / 100;


	//

	VecTacF1.Resize(23); //data from SynTouch: E pac pdc tac tdc
	VecTacF2.Resize(23); //
	VecTacF3.Resize(23); //

	VecPosF1.Resize(3); //The finger-tip position in the hand ref. frame
	VecPosF2.Resize(3);
	VecPosF3.Resize(3);

	VecVF.Resize(7); // The Virtual Frame;
	VecFinJnt.Resize(16); // The finger joint angles;

	ModelInput.Resize(14);
	ModelInput.Zero();
	bGrasp = false;
	bHandOpen = false;
	bRecord = false;
	bImp = false;
	bCorrect = false;


	//init Hang's compExploration
	string RFile = "./data/exploration/data/code.bin";
	string objFile = "./data/exploration/models/data_scanner/milk/milk_labeled.bin";
	string graspFile = "./data/exploration/tmp/labels_milk.bin";

	MatrixXd labels = BinaryData::readMatrix(graspFile);

	//	cout<<"here 00"<<endl;
	//cout << "Finished advertising" << endl;
	compEXP = new compExploration(RFile, objFile, graspFile);
	//cout << "Finished init fast GMR" << endl;
	compEXP->setAlpha(0.4);
	//	cout << compEXP->getContacts()[0].pos.transpose() << endl;

	HandInObj.Resize(4,4);
	//	HandInObj.Identity();
	HandInObj.Load("./data/DataHand/handTrans.dat");

}

GraspCorrectionClass::~GraspCorrectionClass() {
	delete nH;
	delete GraspStability;
	delete compEXP;

}

void GraspCorrectionClass::STacF1CallBack(
		const SynTouchPublisher::biotac_message::ConstPtr& SynPression) {

	for (int i = 0; i < 19; i++) {
		VecTacF1(i) = SynPression->E[i];
	}

	VecTacF1(19) = SynPression->Pac[0];
	VecTacF1(20) = SynPression->Pdc;
	VecTacF1(21) = SynPression->Tac;
	VecTacF1(22) = SynPression->Tdc;

}

void GraspCorrectionClass::STacF2CallBack(
		const SynTouchPublisher::biotac_message::ConstPtr& SynPression) {

	for (int i = 0; i < 19; i++) {
		VecTacF2(i) = SynPression->E[i];
	}

	VecTacF2(19) = SynPression->Pac[0];
	VecTacF2(20) = SynPression->Pdc;
	VecTacF2(21) = SynPression->Tac;
	VecTacF2(22) = SynPression->Tdc;

}

void GraspCorrectionClass::STacF3CallBack(
		const SynTouchPublisher::biotac_message::ConstPtr& SynPression) {

	for (int i = 0; i < 19; i++) {
		VecTacF3(i) = SynPression->E[i];
	}

	VecTacF3(19) = SynPression->Pac[0];
	VecTacF3(20) = SynPression->Pdc;
	VecTacF3(21) = SynPression->Tac;
	VecTacF3(22) = SynPression->Tdc;

}

void GraspCorrectionClass::SPosF1CallBack(const geometry_msgs::PointStamped& F1Pos) {
	VecPosF1(0) = F1Pos.point.x;
	VecPosF1(1) = F1Pos.point.y;
	VecPosF1(2) = F1Pos.point.z;
}

void GraspCorrectionClass::SPosF2CallBack(const geometry_msgs::PointStamped& F2Pos) {
	VecPosF2(0) = F2Pos.point.x;
	VecPosF2(1) = F2Pos.point.y;
	VecPosF2(2) = F2Pos.point.z;
}
void GraspCorrectionClass::SPosF3CallBack(const geometry_msgs::PointStamped& F3Pos) {

	VecPosF3(0) = F3Pos.point.x;
	VecPosF3(1) = F3Pos.point.y;
	VecPosF3(2) = F3Pos.point.z;
}
void GraspCorrectionClass::SVFCallBack(const geometry_msgs::PoseStamped& VF) {

	VecVF(0) = VF.pose.position.x;
	VecVF(1) = VF.pose.position.y;
	VecVF(2) = VF.pose.position.z;

	VecVF(3) = VF.pose.orientation.w;
	VecVF(4) = VF.pose.orientation.x;
	VecVF(5) = VF.pose.orientation.y;
	VecVF(6) = VF.pose.orientation.z;

}

void GraspCorrectionClass::SFinJntCallBack(const sensor_msgs::JointState& Jnt) {
	for (int i = 0; i < 16; i++) {

		VecFinJnt[i] = Jnt.position[i];

	}
}

void GraspCorrectionClass::UserCmdCallBack(const std_msgs::String& cmd) {

	if (cmd.data == "HandOpen") {
		// open the finger to some initial position
		bHandOpen = true;
		bImp = false;
		bGrasp = false;
		bRecord = false;
		bCorrect = false;
	}

	if (cmd.data == "Imp") {
		bImp = true;
		bHandOpen = false;
		bGrasp = false;
		bRecord = false;
		bCorrect = false;
	}

	if (cmd.data == "grasp") {
		// close the finger with initial grasp

		bGrasp = true;
		bImp = false;
		bHandOpen = false;
		bRecord = false;
		bCorrect = false;
	}

	if (cmd.data == "Correct") {

		//cout << "start to correct grasp" << endl;
		bGrasp = true;
		bImp = false;
		bHandOpen = false;
		bRecord = false;
		bCorrect = true;

	}

	char reccmd[1024];
	char fname[1024];
	int param;

	sscanf(cmd.data.c_str(), "%s %d", reccmd, &param);

	if (strcmp(reccmd, "rec") == 0) {

		// start to record the data

		sprintf(fname, "./data/VecTacF1_%d.txt", param);
		FileTacF1.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecTacF2_%d.txt", param);
		FileTacF2.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecTacF3_%d.txt", param);
		FileTacF3.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecPosF1_%d.txt", param);
		FilePosF1.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecPosF2_%d.txt", param);
		FilePosF2.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecPosF3_%d.txt", param);
		FilePosF3.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecVF_%d.txt", param);
		FileVF.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecFinJnt_%d.txt", param);
		FileFinJnt.open(fname, ios::out | ios::app | ios::binary);

		sprintf(fname, "./data/VecStiff_%d.txt", param);
		FileStiff.open(fname, ios::out | ios::app | ios::binary);

		bGrasp = true;
		bImp = false;
		bHandOpen = false;
		bRecord = true;
		bCorrect = false;

	}
	if (cmd.data == "srec") {
		// stop to record the data

		FileTacF1.close();
		FileTacF2.close();
		FileTacF3.close();

		FilePosF1.close();
		FilePosF2.close();
		FilePosF3.close();

		FileVF.close();
		FileFinJnt.close();
		FileStiff.close();

		bHandOpen = false; // open the hand to initial position;
		bImp = false; // change to impedance mode;
		bGrasp = true; // the initial grasp;
		bRecord = false; // start to record data with varying impedance;
		bCorrect = false;

	}

}

template <typename T>
string GraspCorrectionClass::num2str(T n){
	stringstream ss;
	ss << n;
	return ss.str();
}

int main() {
	GraspCorrectionClass* RobustGrasp;
	RobustGrasp = new GraspCorrectionClass;

	RobustGrasp->VecFinJnt.Zero();
	RobustGrasp->VecPosF1.Zero();
	RobustGrasp->VecPosF2.Zero();
	RobustGrasp->VecPosF3.Zero();

	RobustGrasp->VecTacF1.Zero();
	RobustGrasp->VecTacF2.Zero();
	RobustGrasp->VecTacF3.Zero();

	RobustGrasp->VecVF.Zero();

	RobustGrasp->GraspStiffX = 10;
	RobustGrasp->GraspStiffY = 2;
	RobustGrasp->GraspStiffZ = 2;

	double StiffNewX = RobustGrasp->GraspStiffX;
	double StiffNewY = RobustGrasp->GraspStiffY;
	double StiffNewZ = RobustGrasp->GraspStiffZ;

	int samplenb = 0;
	int scnt = 0;
	int stiffcnt = 1;
	int nbcorrection = 0;
	bool localexp=false;


	RobustGrasp->CorrInfo.stability=" stable";
	RobustGrasp->CorrInfo.correction="regression ";
	RobustGrasp->CorrInfo.kx=RobustGrasp->GraspStiffX;
	RobustGrasp->CorrInfo.ky=RobustGrasp->GraspStiffY;
	RobustGrasp->CorrInfo.kz=RobustGrasp->GraspStiffZ;
	RobustGrasp->CorrInfo.px=0;
	RobustGrasp->CorrInfo.py=0;
	RobustGrasp->CorrInfo.pz=0;

	ros::NodeHandle ns;
	ros::ServiceClient client = ns.serviceClient<
			ObjImpManipulation::ObjImpManipulationCmd> ("/objimpctrl/node_cmd");
	ros::Rate r(50);


	while (ros::ok()) {
		ros::spinOnce();
		RobustGrasp->GraspCorrectionPub.publish(RobustGrasp->CorrInfo);    // publish the corrective msg;

		RobustGrasp->GraspCorrectionPy.publish(RobustGrasp->msgPy);   // publish for python;

		double RL1=(RobustGrasp->VecPosF1(0)-RobustGrasp->VecVF(0))*(RobustGrasp->VecPosF1(0)-RobustGrasp->VecVF(0))+
				(RobustGrasp->VecPosF1(1)-RobustGrasp->VecVF(1))*(RobustGrasp->VecPosF1(1)-RobustGrasp->VecVF(1))+
				(RobustGrasp->VecPosF1(2)-RobustGrasp->VecVF(2))*(RobustGrasp->VecPosF1(2)-RobustGrasp->VecVF(2));

		double RL2=(RobustGrasp->VecPosF2(0)-RobustGrasp->VecVF(0))*(RobustGrasp->VecPosF2(0)-RobustGrasp->VecVF(0))+
				(RobustGrasp->VecPosF2(1)-RobustGrasp->VecVF(1))*(RobustGrasp->VecPosF2(1)-RobustGrasp->VecVF(1))+
				(RobustGrasp->VecPosF2(2)-RobustGrasp->VecVF(2))*(RobustGrasp->VecPosF2(2)-RobustGrasp->VecVF(2));

		double RL3=(RobustGrasp->VecPosF3(0)-RobustGrasp->VecVF(0))*(RobustGrasp->VecPosF3(0)-RobustGrasp->VecVF(0))+
				(RobustGrasp->VecPosF3(1)-RobustGrasp->VecVF(1))*(RobustGrasp->VecPosF3(1)-RobustGrasp->VecVF(1))+
				(RobustGrasp->VecPosF3(2)-RobustGrasp->VecVF(2))*(RobustGrasp->VecPosF3(2)-RobustGrasp->VecVF(2));


		if (RobustGrasp->bHandOpen) {
			ObjImpManipulation::ObjImpManipulationCmd srv;
			srv.request.cmd = "home";
			client.call(srv);
			cout << "received hand call" << endl;
			RobustGrasp->bHandOpen = false;
		}
		if (RobustGrasp->bImp) {
			ObjImpManipulation::ObjImpManipulationCmd srv;
			srv.request.cmd = "imp_mode";
			client.call(srv);
			cout << "imp mode" << endl;
			RobustGrasp->bImp = false;
		}
		if (RobustGrasp->bGrasp) {
			ObjImpManipulation::ObjImpManipulationCmd srv;
			srv.request.cmd = "ctrl_cmd";

			geometry_msgs::Vector3 graspstiff;

			graspstiff.x = RobustGrasp->GraspStiffX;
			graspstiff.y = RobustGrasp->GraspStiffY;
			graspstiff.z = RobustGrasp->GraspStiffZ;

			srv.request.parms.grasp_stiff.push_back(graspstiff);
			srv.request.parms.grasp_stiff.push_back(graspstiff);
			srv.request.parms.grasp_stiff.push_back(graspstiff);

			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);//
			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);//
			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);//

			if(localexp){
				geometry_msgs::Point desPos1;                         // The desired position of F1;
				geometry_msgs::Point desPos2;                         //The desired position of F2;

				MathLib::Vector tmppos;                               // relative movement of the adapted finger;
				tmppos.Resize(3);

				// modify for compExploration
				MathLib::Vector tmpVF;
				tmpVF.Resize(3);
				tmpVF.Zero();
				tmpVF(0) = RobustGrasp->VecVF(0);
				tmpVF(1) = RobustGrasp->VecVF(1);
				tmpVF(2) = RobustGrasp->VecVF(2);

				MathLib::Vector tmpRL;
				tmpRL.Resize(3);
				tmpRL.Zero();
				tmpRL(0) = (tmpVF - RobustGrasp->VecPosF1).Norm()/ RobustGrasp->t2(0);
				tmpRL(1) = (tmpVF - RobustGrasp->VecPosF2).Norm()/ RobustGrasp->t2(1);
				tmpRL(2) =(tmpVF - RobustGrasp->VecPosF3).Norm() / RobustGrasp->t2(2);

				MathLib::Matrix DataRLMean;
				DataRLMean.Resize(3,13);
				DataRLMean=RobustGrasp->DataGMMMean.GetMatrix(8,10,0,12);
				int bestPos = -1;
				double tempL = 10e9;
				for (int i = 0; i < 13; ++i)
				{
					double t = (tmpRL - DataRLMean.GetColumn(i)).Norm();
					if ( t < tempL){
						tempL = t;
						bestPos = i;
					}
				}
				//				cout<<"bestPos: "<< bestPos <<endl;

				tmpRL(0) = DataRLMean(0,bestPos)*RobustGrasp->t2(0);
				tmpRL(1) = DataRLMean(1,bestPos)*RobustGrasp->t2(1);
				tmpRL(2) = DataRLMean(2,bestPos)*RobustGrasp->t2(2);

				Vector3d desiredRL(tmpRL(0),tmpRL(1),tmpRL(2));

				Move adaMove = RobustGrasp->compEXP->nextMove(desiredRL);

				MathLib::Vector desPosInObj;
				desPosInObj.Resize(3);
				MathLib::Vector desPosInHand;
				desPosInHand.Resize(3);

				if (adaMove.fId == 0) // move finger 1
				{
					//transform from obj frame to hand frame;
					desPosInObj(0) = adaMove.position.pos(0);
					desPosInObj(1) = adaMove.position.pos(1);
					desPosInObj(2) = adaMove.position.pos(2);

					desPosInObj.Print("Fin 1 des position in Obj");

					MathLib::Vector tmp(3);
					tmp(0)=RobustGrasp->HandInObj(0,3);
					tmp(1)=RobustGrasp->HandInObj(1,3);
					tmp(2)=RobustGrasp->HandInObj(2,3);
					desPosInHand = RobustGrasp->HandInObj.GetMatrix(0,2,0,2).TransposeMult(desPosInObj)-
							RobustGrasp->HandInObj.GetMatrix(0,2,0,2).TransposeMult(tmp);

					double Direction = 1; // moving direction
					if((desPosInHand-RobustGrasp->VecPosF2).Norm() < (RobustGrasp->VecPosF2-RobustGrasp->VecPosF1).Norm())
					{
						Direction= -1;
					}else{
						Direction= 1;
					}

					if((RobustGrasp->VecPosF2-RobustGrasp->VecPosF1).Norm() > 0.05 && Direction == 1){
						Direction =-1;
					}



					//					desPos1.x = desPosInHand(0);
					//					desPos1.y = desPosInHand(1);
					//					desPos1.z = desPosInHand(2)+0.01;

					//					desPos1.x = RobustGrasp->VecPosF1(0);
					//					desPos1.y = RobustGrasp->VecPosF1(1)+0.04;
					//					desPos1.z = RobustGrasp->VecPosF1(2)+0.01;

					//					double k=0.60;
					//					srv.request.parms.FinCorrectionStiffness.push_back(k*100);
					//					srv.request.parms.FinDesiredCorrection.push_back(desPos1);

					/////Miao's simple adaptation
					MathLib::Vector tmppos;
					tmppos.Resize(3);
					MathLib::Vector tmpvf;
					tmpvf.Resize(3);
					tmpvf(0)=RobustGrasp->VecPosF3(0);
					tmpvf(1)=RobustGrasp->VecPosF3(1);
					tmpvf(2)=RobustGrasp->VecPosF3(2);
					double k=0.10;
					// simple adaption used here.
					tmppos=(RobustGrasp->VecPosF1-RobustGrasp->VecPosF2)/( (RobustGrasp->VecPosF1-RobustGrasp->VecPosF2).Norm())*k*Direction+
							(RobustGrasp->VecPosF1-tmpvf)/((RobustGrasp->VecPosF1-tmpvf).Norm())*0.03;

					desPos1.x = tmppos(0)+RobustGrasp->VecPosF1(0);
					desPos1.y = tmppos(1)+RobustGrasp->VecPosF1(1);
					desPos1.z = tmppos(2)+RobustGrasp->VecPosF1(2);


					srv.request.parms.FinCorrectionStiffness.push_back(28);
					srv.request.parms.FinDesiredCorrection.push_back(desPos1);
					////Miao's adaptation

					// msg for GUI
					RobustGrasp->CorrInfo.stability="exploration";
					RobustGrasp->CorrInfo.correction="1";
					RobustGrasp->CorrInfo.kx=RobustGrasp->GraspStiffX;
					RobustGrasp->CorrInfo.ky=RobustGrasp->GraspStiffY;
					RobustGrasp->CorrInfo.kz=RobustGrasp->GraspStiffZ;
					RobustGrasp->CorrInfo.px=desPosInObj(0);
					RobustGrasp->CorrInfo.py=desPosInObj(1);
					RobustGrasp->CorrInfo.pz=desPosInObj(2);

					//msg for python;
					string msgTmp = "unstable | exploration | 1 | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(0));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(1));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(2));

					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL1);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL2);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL3);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(0));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(1));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(2));
					RobustGrasp->msgPy.data = msgTmp;


				} else if (adaMove.fId == 1){
					// move finger 2
					// transform from obj frame to hand frame;
					desPosInObj(0) = adaMove.position.pos(0);
					desPosInObj(1) = adaMove.position.pos(1);
					desPosInObj(2) = adaMove.position.pos(2);

					desPosInObj.Print("Fin 2 des position in Obj");

					MathLib::Vector tmp(3);
					tmp(0)=RobustGrasp->HandInObj(0,3);
					tmp(1)=RobustGrasp->HandInObj(1,3);
					tmp(2)=RobustGrasp->HandInObj(2,3);
					desPosInHand = RobustGrasp->HandInObj.GetMatrix(0,2,0,2).TransposeMult(desPosInObj)-
							RobustGrasp->HandInObj.GetMatrix(0,2,0,2).TransposeMult(tmp);

					desPos1.x = RobustGrasp->VecPosF1(0);   // keep finger 1 the same position and only adapt finger 2;
					desPos1.y = RobustGrasp->VecPosF1(1);
					desPos1.z = RobustGrasp->VecPosF1(2);

					double Direction = 1; // moving direction
					if((desPosInHand-RobustGrasp->VecPosF1).Norm() < (RobustGrasp->VecPosF2-RobustGrasp->VecPosF1).Norm())
					{
						Direction=1;
					}else{
						Direction=-1;
					}


					//					desPos2.x = desPosInHand(0);
					//					desPos2.y = desPosInHand(1);
					//					desPos2.z = desPosInHand(2)+0.01;

					//					desPos2.x = RobustGrasp->VecPosF2(0);
					//					desPos2.y = RobustGrasp->VecPosF2(1)-0.04;
					//					desPos2.z = RobustGrasp->VecPosF2(2)+0.01;


					//					double k=0.50;
					//					srv.request.parms.FinCorrectionStiffness.push_back(0.0);
					//					srv.request.parms.FinCorrectionStiffness.push_back(k*100);
					//
					//					srv.request.parms.FinDesiredCorrection.push_back(desPos1);
					//					srv.request.parms.FinDesiredCorrection.push_back(desPos2);


					/////Miao's simple adaptation
					MathLib::Vector tmppos;
					tmppos.Resize(3);
					MathLib::Vector tmpvf;
					tmpvf.Resize(3);
					tmpvf(0)=RobustGrasp->VecPosF3(0);
					tmpvf(1)=RobustGrasp->VecPosF3(1);
					tmpvf(2)=RobustGrasp->VecPosF3(2);
					double k=0.2;
					// simple adaption used here.
					tmppos=(RobustGrasp->VecPosF1-RobustGrasp->VecPosF2)/( (RobustGrasp->VecPosF1-RobustGrasp->VecPosF2).Norm())*k*Direction+
							(RobustGrasp->VecPosF2-tmpvf)/((RobustGrasp->VecPosF2-tmpvf).Norm())*0.04;
					desPos2.x = tmppos(0)+RobustGrasp->VecPosF2(0);
					desPos2.y = tmppos(1)+RobustGrasp->VecPosF2(1);
					desPos2.z = tmppos(2)+RobustGrasp->VecPosF2(2);

					srv.request.parms.FinCorrectionStiffness.push_back(0.0);
					srv.request.parms.FinCorrectionStiffness.push_back(28);
					srv.request.parms.FinDesiredCorrection.push_back(desPos1);
					srv.request.parms.FinDesiredCorrection.push_back(desPos2);
					////Miao's adaptation

					// msg for GUI
					RobustGrasp->CorrInfo.stability = "exploration";
					RobustGrasp->CorrInfo.correction = "2";
					RobustGrasp->CorrInfo.kx=RobustGrasp->GraspStiffX;
					RobustGrasp->CorrInfo.ky=RobustGrasp->GraspStiffY;
					RobustGrasp->CorrInfo.kz=RobustGrasp->GraspStiffZ;
					RobustGrasp->CorrInfo.px=desPosInObj(0);
					RobustGrasp->CorrInfo.py=desPosInObj(1);
					RobustGrasp->CorrInfo.pz=desPosInObj(2);

					//msg for python;
					string msgTmp = "unstable | exploration | 2 | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(0));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(1));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(desPosInObj(2));

					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL1);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL2);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(RL3);
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(0));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(1));
					msgTmp +=" | ";
					msgTmp += RobustGrasp->num2str(tmpRL(2));

					RobustGrasp->msgPy.data = msgTmp;
				}


			}else{

				srv.request.parms.FinCorrectionStiffness.push_back(5); // close the local exploration;
				srv.request.parms.FinCorrectionStiffness.push_back(5); // close the local exploration; for the second finger;

			}

			client.call(srv);

		}

		if (RobustGrasp->bCorrect) {

			MathLib::Vector tmp;
			tmp.Resize(57);

			for (int i = 0; i < 19; i++) {
				tmp(i) = (RobustGrasp->VecTacF1(i)-RobustGrasp->VecTacF1Base(i)) / RobustGrasp->t1(i);
			}
			for (int i = 19; i < 38; i++) {
				tmp(i) = (RobustGrasp->VecTacF2(i)-RobustGrasp->VecTacF2Base(i-19)) / RobustGrasp->t1(i);
			}
			for (int i = 38; i < 57; i++) {
				tmp(i) = (RobustGrasp->VecTacF3(i)-RobustGrasp->VecTacF3Base(i-38))/ RobustGrasp->t1(i);
			}

			for (int i = 0; i < 57; i++) {
				tmp(i) = tmp(i) - RobustGrasp->DataTacMean(i);
			}

			MathLib::Vector TacPCA;
			TacPCA.Resize(8);
			TacPCA = RobustGrasp->COEFF.Transpose().Mult(tmp);

			for (int i = 0; i < 8; i++) {
				RobustGrasp->ModelInput(i) = TacPCA(i);
			}

			MathLib::Vector tmpVF;
			tmpVF.Resize(3);
			tmpVF(0) = RobustGrasp->VecVF(0);
			tmpVF(1) = RobustGrasp->VecVF(1);
			tmpVF(2) = RobustGrasp->VecVF(2);
			//			RobustGrasp->VecVF.Print("VF");

			RobustGrasp->ModelInput(8) = (tmpVF - RobustGrasp->VecPosF1).Norm()
																							/ RobustGrasp->t2(0);
			//			cout<<"Length: "<<RobustGrasp->ModelInput(8)<<endl;

			RobustGrasp->ModelInput(9) = (tmpVF - RobustGrasp->VecPosF2).Norm()
																							/ RobustGrasp->t2(1);
			RobustGrasp->ModelInput(10)
																							= (tmpVF - RobustGrasp->VecPosF3).Norm() / RobustGrasp->t2(
																									2);

			RobustGrasp->ModelInput(11) = RobustGrasp->GraspStiffX
					/ RobustGrasp->t3(0);
			RobustGrasp->ModelInput(12) = RobustGrasp->GraspStiffY
					/ RobustGrasp->t3(1);
			RobustGrasp->ModelInput(13) = RobustGrasp->GraspStiffZ
					/ RobustGrasp->t3(2);
			//			RobustGrasp->ModelInput.Print("ModelInput");

			bool bDist = false;
			double lik = RobustGrasp->GraspStability->GaussianProbFast(
					RobustGrasp->ModelInput); //

		     cout<<"Likelihood"<<log(lik)<<endl;
			// check if the input is close enough to some of the Gaussian Components;

			for (int i = 0; i < RobustGrasp->tsDist.Size(); i++) {

				RobustGrasp->tsDist(i) = RobustGrasp->GraspStability->GaussianPDFFast(i, RobustGrasp->ModelInput);

				RobustGrasp->tsDist(i) = log(RobustGrasp->tsDist(i));

				if (RobustGrasp->tsDist(i) > -500) { // Here the distance comparison should be "ts(i)";
					bDist = true;                    // If the distance is still close to the GMM, still can use GMR; Otherwise, jump to local exploration.
					break;
				}
			}

			//RobustGrasp->tsDist.Print("Distance");

			MathLib::Vector tmpstiff;
			tmpstiff.Resize(3);
			localexp=false;

			if (log(lik) > -120) {
				nbcorrection = 0;
				RobustGrasp->CorrInfo.stability="stable";
				RobustGrasp->CorrInfo.correction="Not Required";
				RobustGrasp->CorrInfo.kx=RobustGrasp->GraspStiffX;
				RobustGrasp->CorrInfo.ky=RobustGrasp->GraspStiffY;
				RobustGrasp->CorrInfo.kz=RobustGrasp->GraspStiffZ;
				RobustGrasp->CorrInfo.px=0;
				RobustGrasp->CorrInfo.py=0;
				RobustGrasp->CorrInfo.pz=0;

				//msg for python;
				string msgTmp = "stable | stable | 0 | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffX);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffY);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffZ);

				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL1);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL2);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL3);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL1);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL2);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL3);
				RobustGrasp->msgPy.data = msgTmp;

			} else if (bDist && nbcorrection > 5) {

				nbcorrection++;

				RobustGrasp->GraspStability->Regression(
						RobustGrasp->ModelInput, tmpstiff);
				//tmpstiff.Print("Stiffness from Regression!");
				RobustGrasp->GraspStiffX=tmpstiff(0)*RobustGrasp->t3(0);    //Miao: change to random forest if needed;
				RobustGrasp->GraspStiffY=tmpstiff(0)*RobustGrasp->t3(1);
				RobustGrasp->GraspStiffZ=tmpstiff(0)*RobustGrasp->t3(2);

				RobustGrasp->CorrInfo.stability="stiffness";
				RobustGrasp->CorrInfo.correction="Adapt Stiffness";
				RobustGrasp->CorrInfo.kx=RobustGrasp->GraspStiffX;
				RobustGrasp->CorrInfo.ky=RobustGrasp->GraspStiffY;
				RobustGrasp->CorrInfo.kz=RobustGrasp->GraspStiffZ;
				RobustGrasp->CorrInfo.px=0;
				RobustGrasp->CorrInfo.py=0;
				RobustGrasp->CorrInfo.pz=0;

				//msg for python;
				string msgTmp = "unstable | stiffness | 0 | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffX);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffY);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RobustGrasp->GraspStiffZ);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL1);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL2);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL3);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL1);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL2);
				msgTmp +=" | ";
				msgTmp += RobustGrasp->num2str(RL3);

				RobustGrasp->msgPy.data = msgTmp;

			} else{
				nbcorrection++;

				if(nbcorrection > 3){
					localexp=true;
					cout<<": Local Exploration"<<endl;
				}
			}

		}
		// Miao 2013-11-4

		if (RobustGrasp->bRecord) {
			//start to change the grasp stiffness and record the data;
			//increase the grasp stiff 25%
			scnt = scnt + 1;

			if (scnt > 50 && stiffcnt < 50) {

				double stmp = 0.1 * rand() / RAND_MAX; //add some variance to the impedance parameter;

				RobustGrasp->GraspStiffX = StiffNewX * (1 + stiffcnt * 0.005)
																								* (1 + stmp);
				RobustGrasp->GraspStiffY = StiffNewY * (1 + stiffcnt * 0.005)
																								* (1 + stmp);
				RobustGrasp->GraspStiffZ = StiffNewZ * (1 + stiffcnt * 0.005)
																								* (1 + stmp);
				scnt = 0;
				stiffcnt = stiffcnt + 1;

			}

			ObjImpManipulation::ObjImpManipulationCmd srv;
			srv.request.cmd = "ctrl_cmd";
			geometry_msgs::Vector3 graspstiff;

			graspstiff.x = RobustGrasp->GraspStiffX;
			graspstiff.y = RobustGrasp->GraspStiffY;
			graspstiff.z = RobustGrasp->GraspStiffZ;

			srv.request.parms.grasp_stiff.push_back(graspstiff);
			srv.request.parms.grasp_stiff.push_back(graspstiff);
			srv.request.parms.grasp_stiff.push_back(graspstiff);

			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);//
			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);
			srv.request.parms.grasp_rest_len_ratio.push_back(-2.0);
			client.call(srv);
			if (samplenb < 2500) {

				FileStiff << RobustGrasp->GraspStiffX << " "
						<< RobustGrasp->GraspStiffY << " "
						<< RobustGrasp->GraspStiffZ << "";

				FileStiff << endl;

				for (int i = 0; i < int(RobustGrasp->VecTacF1.Size()); i++) {
					FileTacF1 << RobustGrasp->VecTacF1[i] << " ";

				}
				FileTacF1 << endl;

				for (int i = 0; i < int(RobustGrasp->VecTacF2.Size()); i++) {
					FileTacF2 << RobustGrasp->VecTacF2[i] << " ";

				}
				FileTacF2 << endl;

				for (int i = 0; i < int(RobustGrasp->VecTacF3.Size()); i++) {
					FileTacF3 << RobustGrasp->VecTacF3[i] << " ";
				}
				FileTacF3 << endl;

				for (int i = 0; i < int(RobustGrasp->VecPosF1.Size()); i++) {
					FilePosF1 << RobustGrasp->VecPosF1[i] << " ";
				}
				FilePosF1 << endl;

				for (int i = 0; i < int(RobustGrasp->VecPosF2.Size()); i++) {
					FilePosF2 << RobustGrasp->VecPosF2[i] << " ";
				}
				FilePosF2 << endl;

				for (int i = 0; i < int(RobustGrasp->VecPosF3.Size()); i++) {
					FilePosF3 << RobustGrasp->VecPosF3[i] << " ";
				}
				FilePosF3 << endl;

				for (int i = 0; i < int(RobustGrasp->VecVF.Size()); i++) {
					FileVF << RobustGrasp->VecVF[i] << " ";
				}
				FileVF << endl;

				for (int i = 0; i < int(RobustGrasp->VecFinJnt.Size()); i++) {
					FileFinJnt << RobustGrasp->VecFinJnt[i] << " ";
				}
				FileFinJnt << endl;
				samplenb = samplenb + 1;
				cout << "received record" << endl;

			}
		}

	}

	return 0;
}

