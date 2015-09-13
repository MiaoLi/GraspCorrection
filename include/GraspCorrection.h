/*
 * GraspCorrection.h
 *
 *  Created on: Oct 9, 2013
 *      Author: miao
 */

#ifndef GRASPCORRECTION_H_
#define GRASPCORRECTION_H_

#include <iostream>
#include "ros/ros.h"
#include "MathLib/MathLib.h"
#include "SynTouchPublisher/biotac_message.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "ObjImpManipulation/ObjImpManipulationCmd.h"
#include <fstream>
#include <sstream>
#include "Gaussians.h"
#include "GraspCorrection/GraspCorrectionMsg.h"

#include <vector>
#include <Eigen/Dense>
#include <utility>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include "BinaryData.h"
#include "compExploration.hpp"

class GraspCorrectionClass{

private:

	ros::NodeHandle* nH;

	ros::Subscriber STacF1;   //read data from finger 1;
	ros::Subscriber STacF2;   //read data from finger 2;
	ros::Subscriber STacF3;   //read data from finger 3;

	ros::Subscriber SPosF1;   // read fingertip's position for finger 1;
	ros::Subscriber SPosF2;
	ros::Subscriber SPosF3;

	ros::Subscriber SVF;      //read data of Virtual Frame;


    ros::Subscriber UserCmd;

    ros::Subscriber SFinJnt; //FingerJoint;






public:


	bool bHandOpen;         // open the hand to initial position
	bool bImp;              // change to impedance mode
	bool bRecord;           // start to record data
	bool bGrasp;            // grasp the object with specified impedance
	bool bCorrect;          // predict the grasp stability and correct possible failed grasps.

	MathLib::Vector VecTacF1;
	MathLib::Vector VecTacF1Base;

	MathLib::Vector VecTacF2;
	MathLib::Vector VecTacF2Base;
	MathLib::Vector VecTacF3;
	MathLib::Vector VecTacF3Base;

	MathLib::Vector VecPosF1;
	MathLib::Vector VecPosF2;
	MathLib::Vector VecPosF3;

	MathLib::Vector VecVF;
	MathLib::Vector VecFinJnt;

	MathLib::Matrix COEFF;

	MathLib::Vector DataTacMean;
	MathLib::Matrix DataGMMMean;       //The mean of GMM for rest length;

	MathLib::Vector t1;
	MathLib::Vector t2;
	MathLib::Vector t3;
	MathLib::Vector ts;
	MathLib::Vector tsDist;  // compute the distance to each Gaussian Components;

	MathLib::Vector ModelInput;  // The input variable for the GMM model;

	double GraspStiffX;
	double GraspStiffY;
	double GraspStiffZ;

	GraspCorrection::GraspCorrectionMsg CorrInfo;
	ros::Publisher GraspCorrectionPub;

	ros::Publisher GraspCorrectionPy;
	std_msgs::String msgPy;

    Gaussians* GraspStability;
    compExploration* compEXP;

    MathLib::Matrix HandInObj;


   GraspCorrectionClass();
   ~GraspCorrectionClass();

   template <typename T>
   string num2str(T n);

   void STacF1CallBack(const SynTouchPublisher::biotac_message::ConstPtr& );
   void STacF2CallBack(const SynTouchPublisher::biotac_message::ConstPtr& );
   void STacF3CallBack(const SynTouchPublisher::biotac_message::ConstPtr& );

   void SPosF1CallBack(const geometry_msgs::PointStamped&);
   void SPosF2CallBack(const geometry_msgs::PointStamped&);
   void SPosF3CallBack(const geometry_msgs::PointStamped&);

   void SVFCallBack(const geometry_msgs::PoseStamped&);

   void UserCmdCallBack(const std_msgs::String&);

   void SFinJntCallBack(const sensor_msgs::JointState&);

};

#endif /* GRASPCORRECTION_H_ */
