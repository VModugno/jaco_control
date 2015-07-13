/*
 * head.hpp
 *
 *  Created on: Jul 9, 2015
 *      Author: vale
 */

#ifndef HEAD_HPP_
#define HEAD_HPP_




#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "KinovaTypes.h"
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <armadillo>


typedef arma::vec         State;


extern double DEG;
extern boost::atomic<bool>  EXEC;

// template function
template<typename T, size_t N>
 T * End(T (&ra)[N]) {
    return ra + N;
};

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
	//FIX THIS!!!
    //std::assert( a.size() == b.size() );

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(),
                   std::back_inserter(result), std::plus<T>());
    return result;
}


inline int ReadFile(std::string namefile,std::vector< State > & value)
{
	std::ifstream infile;
	int task_space_dim;

	try
	{
	  infile.open(namefile.c_str(),std::ifstream::in);

	  std::string line;
	  // in the first line is defined the taskspacedimension
	  std::getline(infile, line);
	  std::stringstream ss1(line);
	  if ( !(ss1 >> task_space_dim) )
	  {
		  std::cout<<"problem reading number of joint"<< std::endl;
	  }

	  while (std::getline(infile, line))
	  {
			std::stringstream ss(line);
			State app(task_space_dim);
			int index = 0;
			while( !ss.eof() )
			{
				double ff;
				if ( ss >> ff)
				{
				   app[index] = ff;
				   index++;
				}

			}
			if(!app.empty())
			{
				value.push_back(app);
			}


	  }
	  infile.close();
	}
	catch (std::ifstream::failure e)
	{
	  std::cerr << "Exception opening/reading/closing file\n";
	}

	return task_space_dim;
};

inline void  WriteFile(std::vector<AngularPosition> log,std::string namefile)
{
	std::ofstream myfile(namefile.c_str());

	for(unsigned int i =0;i<log.size();i++)
	{
		myfile<<log[i].Actuators.Actuator1<<" "<<log[i].Actuators.Actuator2<<" ";
		myfile<<log[i].Actuators.Actuator3<<" "<<log[i].Actuators.Actuator4<<" "<<log[i].Actuators.Actuator5<<" "<<log[i].Actuators.Actuator6<<"\n";
	}

	myfile.close();

};


inline void  WriteFile(std::vector<CartesianPosition> log,std::string namefile)
{
	std::ofstream myfile(namefile.c_str());

	for(unsigned int i =0;i<log.size();i++)
	{
		myfile<<log[i].Coordinates.X<<" "<<log[i].Coordinates.Y<<" ";
		myfile<<log[i].Coordinates.Z<<" "<<log[i].Coordinates.ThetaX<<" "<<log[i].Coordinates.ThetaY<<" "<<log[i].Coordinates.ThetaZ<<"\n";
	}

	myfile.close();

};

inline void  WriteFile(std::vector<int> log,std::string namefile)
{
	std::ofstream myfile(namefile.c_str());

	for(unsigned int i =0;i<log.size();i++)
	{
		myfile<<log[i]<<"\n";
	}

	myfile.close();
};


arma::mat J0(State & q,std::string type);
void EmergencyStop();




#endif /* HEAD_HPP_ */
