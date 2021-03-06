#include "safetycheck.hpp"

using namespace std;

int main()
{
	 int exit;

	 //Handle for the library's command layer.
	 void * commandLayer_handle;
	 boost::thread* safety_check;
	 bool INITIALIZE =	true; // if you want to initialize
	 std::string CONTROL_TYPE = "joint"; // "joint" "cartesian"
	 bool save_torque = true;
	 boost::chrono::high_resolution_clock::time_point time_reference;
	 boost::chrono::milliseconds cur_time;
	 int total_time = 20000; // 20 s
	 int time_step = 1; // ms
	 int index = 0;
	 int refresh_threshold = 0;
	 int time_refresh_threshold = 0;
	 int counter = 0;


	//Function pointers to the functions we need
	int (*MyInitAPI)();
	int (*MyCloseAPI)();
	int (*MySendBasicTrajectory)(TrajectoryPoint command);
	int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
	int (*MyGetAngularPosition)(AngularPosition &);
	int (*MyGetAngularVelocity)(AngularPosition &);
	int (*MyGetCartesianPosition)(CartesianPosition &);
	int (*MyGetAngularForce)(AngularPosition &);
	int (*MyEraseAllTrajectories)();
	int (*MyStartControlAPI)();

	//We load the library (Under Windows, use the function LoadLibrary)
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW);

	//We load the functions from the library (Under Windows, use GetProcAddress)
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
	MyGetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MySendAdvanceTrajectory=(int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
	MyGetCartesianPosition = (int (*)(CartesianPosition &))dlsym(commandLayer_handle,"GetCartesianPosition");
	MyGetAngularForce    = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularForce");
	MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
	MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");

	const char * _name_start_point_joint[] = {"start_joint_pos.txt"};
	std::vector<std::string> name_start_point_joint (_name_start_point_joint,End(_name_start_point_joint));
	const char * _name_start_point_cart[] = {"start_cart_pos.txt"};
	std::vector<std::string> name_start_point_cart (_name_start_point_cart,End(_name_start_point_cart));
	const char * _name_joint_vel[] = {"joint_vel.txt"};
	std::vector<std::string> name_joint_vel (_name_start_point_cart,End(_name_start_point_cart));
	const char * _namefiles[] = {"cart_pos.txt","cart_vel.txt","joint_vel.txt"};
	std::vector<std::string> namefile (_namefiles,End(_namefiles));

	std::vector<AngularPosition> Log_position;
	std::vector<CartesianPosition> Log_cartesian;
	std::vector<AngularPosition> Log_tau;
	std::vector<int> Log_index;

	double lambda = 0.001;
	double P = 5;
	int limitation = 1;
	std::vector<std::vector<State> > ff;
	std::vector<State> desired_values;


	// checking module
	bool check=false;
	// define bounding box
	const double bb_point[] = {-0.6,-0.8,-0.2};
	const double bb_dims[]  = {1.2,1.6,0.22};
	std::vector<double> bb_p(bb_point,End(bb_point)),bb_d(bb_dims,End(bb_dims));

	// define all the limit
	const char *cl[] = {"j_pos","j_tau"};
	std::vector<std::string> chekclist(cl,End(cl));
	const double joint_min[] = {-10000*DEG,47*DEG,19*DEG,-10000*DEG,-10000*DEG,-10000*DEG}; // rad
	const double joint_max[] = {10000*DEG,313*DEG,341*DEG,10000*DEG,10000*DEG,10000*DEG}; // rad
	const double tau_min[] = {-20}; // Nm
	const double tau_max[] = {20};  // nm
	std::vector<double> j_min(joint_min,End(joint_min)),j_max(joint_max,End(joint_max));
	std::vector<double> t_min(tau_min,End(tau_min)),t_max(tau_max,End(tau_max));

	std::vector<std::vector<double> > l_down_left_corner,l_dims,l_min,l_max;
	l_down_left_corner.push_back(bb_p);l_dims.push_back(bb_d);
	l_min.push_back(j_min);l_min.push_back(t_min);
	l_max.push_back(j_max);l_max.push_back(t_max);

	safetycheck checker(l_down_left_corner,l_dims,l_min,l_max,chekclist);

	//If the was loaded correctly
	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) || (MyStartControlAPI == NULL))
	{
		cout << "Unable to initialize the command layer." << endl;
	}
	else
	{
		cout << "The command has been initialized correctly." << endl << endl;

		cout << "Calling the method InitAPI()" << endl;

		exit = (*MyInitAPI)();
		if(exit != 1)
		{
			std::cout << "exit " << exit << std::endl;
			std::cout<< "probably the usb cable is not connected"<<std::endl;
			return 0;
		}
		cout << "We take control of the robotic arm." << endl;
		exit = (*MyStartControlAPI)();

		for(unsigned int i =0;i<namefile.size();i++)
		{
			std::vector<State> app;
			ReadFile(namefile[i],app);
			ff.push_back(app);
		}

		std::vector<State> start_point;
		if(CONTROL_TYPE.compare("joint") == 0 )
		{
			ReadFile(_name_start_point_joint[0],start_point);
		}
		else if(CONTROL_TYPE.compare("cartesian") == 0 )
		{
			ReadFile(_name_start_point_cart[0],start_point);
		}

		//DEBUG
		std::cout<< "start value"<<std::endl;
		for(unsigned int ik =0;ik<start_point[0].size();ik++)
				std::cout<<start_point[0][ik]<<" ";
		std::cout<<std::endl;
		//---

        // INITIALIZE STRUCTURE FOR COMMUNICATE WITH ROBOT
		AngularPosition   position;
		CartesianPosition cart_pos;
		AngularPosition   joint_tau;
		TrajectoryPoint trajectoryPoint;
		trajectoryPoint.InitStruct();
		trajectoryPoint.Position.HandMode = HAND_NOMOVEMENT;
		//DEFINE LIMITATIONS HERE
		trajectoryPoint.Limitations.speedParameter1 = 8.0f;//We limit the translation velocity to 8 cm per second.
		trajectoryPoint.Limitations.speedParameter2 = 8.0f; //We limit the orientation velocity to 0.6 RAD per second
		trajectoryPoint.Limitations.speedParameter3 = 0.08f;

		if(CONTROL_TYPE.compare("joint") == 0 )
		{
			trajectoryPoint.Position.Type = ANGULAR_POSITION;
			trajectoryPoint.LimitationsActive = limitation;
			// covenrsion from rad to deg
			start_point[0] = start_point[0]*(1/DEG);
			// conversione to kinova
			trajectoryPoint.Position.Actuators.Actuator1 = start_point[0][0];
			trajectoryPoint.Position.Actuators.Actuator2 = start_point[0][1];
			trajectoryPoint.Position.Actuators.Actuator3 = start_point[0][2];
			trajectoryPoint.Position.Actuators.Actuator4 = start_point[0][3];
			trajectoryPoint.Position.Actuators.Actuator5 = start_point[0][4];
			trajectoryPoint.Position.Actuators.Actuator6 = start_point[0][5];
		}
		else if(CONTROL_TYPE.compare("cartesian") == 0 )
		{
			trajectoryPoint.Position.Type = CARTESIAN_POSITION;
			trajectoryPoint.LimitationsActive = limitation;

			// conversione to kinova
			trajectoryPoint.Position.CartesianPosition.X = start_point[0][0];
			trajectoryPoint.Position.CartesianPosition.Y = start_point[0][1];
			trajectoryPoint.Position.CartesianPosition.Z = start_point[0][2];
			trajectoryPoint.Position.CartesianPosition.ThetaX = start_point[0][3];
			trajectoryPoint.Position.CartesianPosition.ThetaY = start_point[0][4];
			trajectoryPoint.Position.CartesianPosition.ThetaZ = start_point[0][5];
		}

		(*MySendAdvanceTrajectory)(trajectoryPoint);
		std::cout<< "Waiting for the completion of the initialization. When is done press i"<<std::endl;
		// waiting for the completion of the initialization to start the trajectory tracking
		while(INITIALIZE)
		{
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::I))
			{
				std::cout<< "finish initialization"<<std::endl;
				INITIALIZE = false;

			}
		}
		// after init i launch the security check thread
		safety_check = new boost::thread(boost::bind(&EmergencyStop));

		// change the controller type per il controllore in uso
		if(CONTROL_TYPE.compare("joint") == 0 )
			trajectoryPoint.Position.Type = ANGULAR_VELOCITY;
		else if(CONTROL_TYPE.compare("cartesian") == 0 )
			trajectoryPoint.Position.Type = CARTESIAN_VELOCITY;
		// inzializzo desired value
		State init;
		desired_values.push_back(init);
		desired_values.push_back(init);
		desired_values.push_back(init);
		// initialize time reference
		time_reference = boost::chrono::high_resolution_clock::now();
		cur_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - time_reference);

		while( cur_time.count() < total_time)
		//while(true)
		{
			boost::chrono::high_resolution_clock::time_point begin = boost::chrono::high_resolution_clock::now();

			// read from robot

			(*MyGetCartesianPosition)(cart_pos);
			(*MyGetAngularPosition)(position);
			if(save_torque)
			{
				(MyGetAngularForce)(joint_tau);
			}


			// save data
			Log_cartesian.push_back(cart_pos);
			Log_position.push_back(position);
			if(save_torque)
				Log_tau.push_back(joint_tau);




			// trascrivo joint
			State joint_pos(6);
			joint_pos[0]=position.Actuators.Actuator1;
			joint_pos[1]=position.Actuators.Actuator2;
			joint_pos[2]=position.Actuators.Actuator3;
			joint_pos[3]=position.Actuators.Actuator4;
			joint_pos[4]=position.Actuators.Actuator5;
			joint_pos[5]=position.Actuators.Actuator6;

			joint_pos = joint_pos*DEG;
			// trascrivo cartesian
			State cart(3);

			cart[0] = cart_pos.Coordinates.X;
			cart[1] = cart_pos.Coordinates.Y;
			cart[2] = cart_pos.Coordinates.Z;
			//cart[3] = cart_pos.Coordinates.ThetaX;
			//cart[4] = cart_pos.Coordinates.ThetaY;
			//cart[5] = cart_pos.Coordinates.ThetaZ;

			if(desired_values[0].size()==3)
			{
				cart.resize(3);
			}

			//check block
			/*std::vector<State> check_val;
			check_val.push_back(joint_pos);
			check_val.push_back(tau);
			check_val.push_back(cart);
			check = checker.VerifyViolation(check_val);*/
			// add control to stop robot if i violate something
			if(check)
			{
				std::cout<<"something went wrong"<<std::endl;
				break;
			}
			std::cout << "reading and checking time: " << boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - begin).count() << " ms\n";

			// update cur time
			cur_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - time_reference);
			index = boost::chrono::round<boost::chrono::milliseconds>(cur_time).count();
			if(index > (total_time-1) )
				index = total_time - 1;

			// i save the value of the index to compute mean and average error on the position error
			Log_index.push_back(index + 1);

			// DEBUG
			std::cout<< "index = " << index << std::endl;
			//
			//controllo
			desired_values[0] = ff[0][index];
			desired_values[1] = ff[1][index];
			desired_values[2] = ff[2][index];
			State result;

			//DEBUG
			//std::cout<< "desired value 1"<<std::endl;
			//for(unsigned int ik =0;ik<desired_values[1].size();ik++)
			//		std::cout<<desired_values[1][ik]<<" ";
			//std::cout<<std::endl;

			//std::cout<< "desired value 1"<<std::endl;
			//for(unsigned int ik =0;ik<desired_values[1].size();ik++)
			//		std::cout<<desired_values[1][ik]<<" ";
			//std::cout<<std::endl;
			//---

			begin = boost::chrono::high_resolution_clock::now();

			if(CONTROL_TYPE.compare("joint") == 0 )
			{
				// controllo nei giunti (velocita)
				arma::mat J;
				if(desired_values[0].size()==3)
				{
					J = J0(joint_pos,"trasl");
				}
				else
				{
					J = J0(joint_pos," ");
				}

				arma::mat I=arma::eye(J.n_rows,J.n_rows);
				arma::mat J_brack = arma::inv(J*J.t() + I*lambda);
				arma::mat J_damp = J.t()*(J_brack);
				result = J_damp*(P*(desired_values[0] - cart));// + desired_values[1]);
				result[0] = -result[0];
				//arma::mat J_sub_inv = arma::pinv(J_sub);
				//result = J_sub_inv*(P*(desired_values[0] - cart) + desired_values[1]);

				result = result + ff[2][index];

				result = result*(1/DEG);

				//DEBUG
				//State error = desired_values[0] - cart;
				//std::cout<< "result"<<std::endl;
				//for(unsigned int ik =0;ik<result.size();ik++)
				//		std::cout<<result[ik]<<" ";
				//std::cout<<std::endl;
				//----

				// to manage error in the generation of the jacobian
				//result[0] = -result[0];

				//limitazioni
				//for(unsigned int j =0;j<result.size();j++)
				//{
				//	if(result[j]>30)
				//		result[j] = 30;
				//	if(result[j]<-30)
				//		result[j] = -30;
				//}

				// conversione to kinova
				trajectoryPoint.Position.Actuators.Actuator1 = result[0];
				trajectoryPoint.Position.Actuators.Actuator2 = result[1];
				trajectoryPoint.Position.Actuators.Actuator3 = result[2];
				trajectoryPoint.Position.Actuators.Actuator4 = result[3];
				trajectoryPoint.Position.Actuators.Actuator5 = result[4];
				trajectoryPoint.Position.Actuators.Actuator6 = result[5];
			}
			else if(CONTROL_TYPE.compare("cartesian") == 0 )
			{
				//controllo cartesiano
				result = desired_values[1];
				trajectoryPoint.Position.CartesianPosition.X = result[0];
				trajectoryPoint.Position.CartesianPosition.Y = result[1];
				trajectoryPoint.Position.CartesianPosition.Z = result[2];

				//DEBUG
				std::cout<< trajectoryPoint.Position.CartesianPosition.X <<" ";
				std::cout<< trajectoryPoint.Position.CartesianPosition.Y <<" ";
				std::cout<< trajectoryPoint.Position.CartesianPosition.Z <<" ";
				std::cout<<std::endl;
				//---
			}

			// in this way i can apply MyEraseAllTrajectories from a specific time
			/*if (index > time_refresh_threshold)
			{
				if(counter >= refresh_threshold)
				{*/
					(*MyEraseAllTrajectories)();
					/*counter = 0;
				}
			}*/
			(*MySendBasicTrajectory)(trajectoryPoint);

			std::cout << "control time : " << boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - begin).count() << " ms\n";

			if (!EXEC.load(boost::memory_order_acquire))
			{
				(*MyEraseAllTrajectories)();
				break;

			}
			// update cur time
			cur_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - time_reference);
			// update counter
			counter++;

		}
		// close and stop execution
		(*MyEraseAllTrajectories)();
		safety_check->join();
		cout << endl << "Calling the method CloseAPI()" << endl;
		exit = (*MyCloseAPI)();
		cout << "result of CloseAPI() = " << exit << endl;
		//dump result on a file
		if(save_torque)
			WriteFile(Log_tau,"joint_tau.mat");

		WriteFile(Log_position,"joint_pos.mat");
		WriteFile(Log_position,"joint_pos.mat");
		WriteFile(Log_cartesian,"cart_pos.mat");
		WriteFile(Log_index,"index.mat");

	}

	return 0;
}
