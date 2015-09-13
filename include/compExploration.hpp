/*
Author: Hang Kaiyu 
		kaiyuh@kth.se

		KTH Royal Institute of Technology, Sweden


*/


#ifndef COMP_EXPLORATION
#define COMP_EXPLORATION

#include <vector>
#include <Eigen/Dense>
#include <utility>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include "BinaryData.h"

struct Contact
{
	int level; //0-indexed
	std::vector<int> cluster; // 1-indexed
	Vector3d pos;
	Vector3d norm;

};

struct Move
{
	int fId;
	Contact position;
	double residual; // this is only used in optimization

};



class compExploration
{
public:
	compExploration(string RFile, string objFile, string graspFile);
	~compExploration();

	Move nextMove(Vector3d RL);
	std::vector<Contact> getContacts();
	Vector3d getRestLength();
	void setAlpha(double);

	// debugging
	void test();





private:
	// private methods
	void initContacts(std::vector< std::vector<int> > contactLabels);
	flann::Matrix<float> mat2Flann(const MatrixXd input);
	double calResidule(std::vector<double>);
	Vector3d calRestLength();
	Vector3d calRestLength(std::vector<Contact> contacts);
	Move optimizeContact(int fId, Vector3d targetRL);
	std::vector<int> findEntries(std::vector<int> label, std::vector<int> indx);
	void getLabels();
	std::vector<Vector3d> clusterRepresentative(std::vector<int> indx);
	std::vector<double> encodeGrasp(std::vector<Contact> contacts);
	std::vector<double> encodePair(std::vector<int> idx, std::vector<Contact> contacts);

	template <typename T>
	inline void printStdVec(std::vector<T> v){
		for (int i = 0; i < v.size(); ++i)
			cout << v[i] << " ";
		cout << endl;
	}


	// private attributes

	int _k;
	flann::Matrix<float> _RTable;
	MatrixXd _objData; // NOTE: cluster is 1-indexed as in MATLAB
	std::vector<Contact> _contacts;
	flann::Index< flann::ChiSquareDistance<float> >* _RTree;
	Vector3d _restLength;
	std::vector< std::vector<int> > _labels;
	std::vector<int> _clusterNumbers;
	double _alpha;


};


#endif