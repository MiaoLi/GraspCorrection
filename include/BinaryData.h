///////////////////////////////////////////////////////////////////////////
//  author   :   Florian T. Pokorny, fpokorny@kth.se
//               KTH Royal Institute of Technology, Sweden
//               http://www.csc.kth.se/~fpokorny/
//
//  created  :   06 Dec 2011 at 21:35:45
//  modified :   30 Dec 2011 at 12:48:54
///////////////////////////////////////////////////////////////////////////
#pragma once
#include <Eigen/Dense>
#include <stdio.h>
using namespace Eigen;
using namespace std;

class BinaryData
{
public:
		static MatrixXd readMatrix(string file);
		static void writeMatrix(MatrixXd m, string file);
		static void writeVector(VectorXd m, string file);
};
