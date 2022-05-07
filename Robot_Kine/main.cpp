#include "NeuroRobot/NeuroRobot.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include "eigen3/Eigen/Dense"
using namespace std;



void load_ep(const string& path, vector<Eigen::Vector3d>& entry_points)
{
	entry_points.clear();
	ifstream inFile(path);

	if (inFile.is_open())
	{
		Eigen::Vector3d p;

		string line;
		while( std::getline(inFile,line) )
		{
			stringstream _line(line);
			string value;
			// get xyz
			p = {};
			for (int i=0; std::getline(_line,value,' '); ++i )
			{
				// to read negative values correctly
				if (value.at(0) == '-')
				{
					p(i) = std::stod( value.substr(1) );
					p(i) *= -1;
				}
				p(i) = std::stod(value);
			}
			// append xyz to vector
			entry_points.push_back(p);
		}
	}
	inFile.close();
}




int main()
{
	
	NeuroRobot robot;
	robot.RunFK();
	robot.RunInverseKinematics();
//	robot.CreateWorkspace();

//	vector<Eigen::Vector3d> entry_points;
//	load_ep("/home/emmanuel/workspace/ros-workspace/RBE580_ws/src/rbe580_neuro_robot/prelims/vtk/RBE580_project/build2/entryPoints.txt", entry_points);




	cout << "Done" << endl;
	return 0;
}
