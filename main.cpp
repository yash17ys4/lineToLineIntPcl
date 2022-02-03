#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl-1.12/pcl/common/intersections.h>

// Program to find the point of intersection of two lines
// Input: Two points of each line
// Output: One point of intersection (if exists)

// Global variable to check whether in the test mode
int isTestMode = 0;
// Global variable for error tolerance
double _epsilon = 0.01;

// Function that takes x y z values for 4 points (total 12 args) and
//  outputs pair (whether line intersect, the point of intersection)
std::pair <bool, pcl::PointXYZ> intersectionOfTwoLines(double x00, double y00, double z00,
															double x01, double y01, double z01,
															double x10, double y10, double z10,
															double x11, double y11, double z11) {
	// Calculating the parameters from lines
	// _lambda for line 1
	// _mmu for line 2
	Eigen::Matrix<double, 2, 2> A;
	A(0, 0) = x00 - x01;
	A(0, 1) = x11 - x10;
	A(1, 0) = y00 - y01;
	A(1, 1) = y11 - y10;
	Eigen::Matrix<double, 2, 1> B;
	B(0, 0) = x11 - x01;
	B(1, 0) = y11 - y01;
	Eigen::Matrix<double, 2, 1> x = A.inverse() * B;
	double _lambda = x(0, 0);
	double _mu = x(1, 0);

	// Calculating the point using the parameter
	// for line 1
	double x0 = x01 + _lambda * (x00 - x01);
	double y0 = y01 + _lambda * (y00 - y01);
	double z0 = z01 + _lambda * (z00 - z01);
	pcl::PointXYZ p0 (x0, y0, z0);
	// for line 2
	double x1 = x11 + _mu * (x10 - x11);
	double y1 = y11 + _mu * (y10 - y11);
	double z1 = z11 + _mu * (z10 - z11);
	pcl::PointXYZ p1 (x1, y1, z1);

	std::cout<<"Point are "<<p0<<" "<<p1<<std::endl;

	// Calculating distance between the points
	// Rejecting the intersection if distance is greater then threshold
	double _delta = pcl::euclideanDistance(p0, p1);
	std::cout<<"Point dist: "<<_delta<<std::endl;

	if(_delta < _epsilon) {
		return {true, p0};
	} else {
		// returning false if no intersection and origin
		return {false, pcl::PointXYZ (0,0,0)};
	}
}

int main(int argc, char *argv[]) {
	// Checking the input arguments
	if(argc >= 2) {
		isTestMode = 1;
	}
	double x00, y00, z00,
			x01, y01, z01,
			x10, y10, z10,
			x11, y11, z11;
	if(isTestMode >= 1) {
		//Opening test file
		std::fstream file;
		file.open("../testCases.txt", std::ios::in);
		// If file not opened, abort
		if(!file) {
			std::cout<<"Test cases file not opened. Please try again"<<std::endl;
			return -1;
		}
		//Testing on the test cases
		int testCases;
		int testCasesPassed = 0;
		file>>testCases;
		for(int t = 0; t < testCases; t++) {
			file>>x00>>y00>>z00
				>>x01>>y01>>z01
				>>x10>>y10>>z10
				>>x11>>y11>>z11;
			// My function called
			std::pair <bool, pcl::PointXYZ> answer = intersectionOfTwoLines(x00, y00, z00, x01, y01, z01,
																			x10, y10, z10, x11, y11, z11);
			// Comparing with the pcl function result
			double drx0 = x00-x01, dry0 = y00-y01, drz0 = z00-z01;
			double drx1 = x10-x11, dry1 = y10-y11, drz1 = z10-z11; 
			Eigen::VectorXf line0(6);
			line0<<x01, y01, z01, drx0, dry0, drz0;
			Eigen::VectorXf line1(6);
			line1<<x11, y11, z11, drx1, dry1, drz1;
			Eigen::Vector4f pt;
			bool doesIntersect = pcl::lineWithLineIntersection(line0, line1, pt, _epsilon);
			pcl::PointXYZ ptFromPCL(pt(0), pt(1), pt(2));
			std::cout<<"Test case #"<<t+1<<std::endl;
			std::cout<<"Pt of intersection from pcl "<<ptFromPCL<<std::endl;
			if(answer.first == true && doesIntersect == true) {
				std::cout<<"Result from my solution "<<answer.second<<std::endl;
				std::cout<<"Result from pcl's algo "<<ptFromPCL<<std::endl;
			} else {
				std::cout<<"No intersection"<<std::endl;
			}
			std::cout<<std::endl;
			if(doesIntersect == answer.first) {
				if(answer.first == true && pcl::euclideanDistance(ptFromPCL, answer.second) <= _epsilon) {
					testCasesPassed++;
				}
				if(answer.first == false) {
					testCasesPassed++;
				}
			}
		}
		std::cout<<"Number of test cases passed "<<testCasesPassed<<" out of "<<testCases<<std::endl;
	} else {
		// Taking input from the console
		std::cout<<"Please enter the point of line 1: "<<std::endl;
		// input for line 1
		std::cout<<"Enter (x, y, z) for point 1 of line 1: "<<std::endl;
		std::cin>>x00>>y00>>z00;
		std::cout<<"Enter (x, y, z) for point 2 of line 1: "<<std::endl;
		std::cin>>x01>>y01>>z01;
		// input for line 2
		std::cout<<"Enter (x, y, z) for point 1 of line 2: "<<std::endl;
		std::cin>>x10>>y10>>z10;
		std::cout<<"Enter (x, y, z) for point 2 of line 2: "<<std::endl;
		std::cin>>x11>>y11>>z11;
		// Calling my function
		std::pair <bool, pcl::PointXYZ> answer = intersectionOfTwoLines(x00, y00, z00, x01, y01, z01,
																			x10, y10, z10, x11, y11, z11);
		if(answer.first == true) {
			std::cout<<"The point of intersection is:"<<std::endl;
			std::cout<<answer.second<<std::endl;
		} else {
			std::cout<<"There is no point of intersection"<<std::endl;
			std::cout<<"The lines are skewed"<<std::endl;
		}
	}
	return 0;
}