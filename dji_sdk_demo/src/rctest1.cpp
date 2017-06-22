/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "rcphone/rpydata.h"
#include "rcphone/switchdata.h"
#include "rcphone/zdata.h"
using namespace DJI::onboardSDK;

//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);

//! Control msg Callback
void StateCallback(rcphone::switchdata msg);
void AttitudeCallback(rcphone::rpydata msg);
void VerticalCallback(rcphone::zdata msg);

//! Global Viriable
int operate_code = -1;
int m100_state=0;
float roll_angle=0.0;
float pitch_angle=0.0;
float yaw_angle=0.0;
float fix_yaw_angle=0.0;
float z_velocity=0.0;
float roll_vel=0.0;
float pitch_vel=0.0;


static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

   
int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("Begin!");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;


	//control data
	


    ros::spinOnce();
    
    //! Setting functions to be called for Mobile App Commands mode 
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);

	//Set subscriber
	ros::Subscriber sub=nh.subscribe("switchdata",5,StateCallback);
	ros::Subscriber sub1=nh.subscribe("rpydata",50,AttitudeCallback);
	ros::Subscriber sub2=nh.subscribe("zdata",5,VerticalCallback);
	
    //Display_Main_Menu();
	drone->request_sdk_permission_control();
	sleep(1);
    while(1)
    {
        ros::spinOnce();
	
        //std::cout << "Enter Input Val: ";
        //while(!(std::cin >> temp32)){
        //std::cin.clear();
        //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        //std::cout << "Invalid input.  Try again: ";
	//}

        /*
		if(temp32>0 && temp32<38)
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
		*/
	//ROS_INFO("Hello!");

	//m100_state=0:On the ground
	//m100_state=1:hold x y z
	//m100_state=2:rp velocity control by phone
	//m100_state=3:yaw angle control by phone
	//0x42: 
	//VERT_VEL -4m/s~4m/s
	//HORI_ATTI_TILT_ANG -30deg~30deg
	//YAW_ANGLE -180deg~180deg
	//Coordinate_Frame:Body Frame
        switch(operate_code)
        {
				case 1:
						//take off
						drone->takeoff();
						sleep(3);
						m100_state=1;
						operate_code=-1;
						ROS_INFO("Take OFF Successfully!");
						break;

				case 2:
						//landing
						drone->landing();
						m100_state=0;
						operate_code=-1;
						ROS_INFO("Landing Successfully!");
						break;
				case 3:
						//vertical control
						for(int i=0;i<10;i++)
						{
							drone->attitude_control(0x42,0,0,z_velocity,fix_yaw_angle);//float x float y float z float yaw
							usleep(20000);//50hz
						}
						z_velocity=0.0;
						operate_code=6;
						ROS_INFO("Height Control Successfully!");
						break;

				case 4:
						//rp control
						for(int i=0;i<5;i++)
						{
							ROS_INFO("roll:%f;pitch:%f;",roll_vel,pitch_vel);
							drone->attitude_control(0x42,-pitch_vel,roll_vel,0,fix_yaw_angle);
							usleep(20000);
						}
						m100_state=2;
						break;
				case 5:
						//yaw control
						for(int i=0;i<5;i++)
						{
							ROS_INFO("yaw:%f.",yaw_angle/180*3.14);
							drone->attitude_control(0x42,0,0,0,yaw_angle);
							usleep(20000);
						}
						fix_yaw_angle=yaw_angle;
						m100_state=3;
						break;
				case 6:

						//stable mode
						ROS_INFO("fix_yaw:%f.",fix_yaw_angle/180*3.14);
						for(int i=0;i<20;i++)
						{
							drone->attitude_control(0x42,0,0,0,fix_yaw_angle);
							usleep(20000);
						}
						m100_state=1;
						operate_code=-1;
						ROS_INFO("Stable Mode Successfully!");
						break;

				default:
						break;
		}
        //main_operate_code = -1;
        //Display_Main_Menu();
    }
    return 0;
}
//! Callback functions for Phone Commands
	//m100_state=0:On the ground
	//m100_state=1:hold x y z
	//m100_state=2:rp velocity control by phone
	//m100_state=3:yaw angle control by phone
	void StateCallback(rcphone::switchdata msg)
{
		ROS_INFO("Switchdata Received.");
		switch (msg.cmd)
		{
				case 0x01:
						ROS_INFO("Take OFF msg");
						if(m100_state==0)
						{
								operate_code=1;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				case 0x02:
						ROS_INFO("Landing msg");
						if(m100_state==1)
						{
								operate_code=2;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				case 0x03:
						ROS_INFO("Height Control msg");
						if(m100_state==1)
						{
								operate_code=3;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				case 0x04:
						ROS_INFO("RP Velocity Control msg");
						if(m100_state==1)
						{
								operate_code=4;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				case 0x05:
						ROS_INFO("Yaw Angle Control msg");
						if(m100_state==1)
						{
								operate_code=5;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				case 0x06:
						if(m100_state==2 || m100_state==3)
						{
								operate_code=6;
								ROS_INFO("Set Operate Code Successfully!");
						}
						break;
				default:
						operate_code=-1;
						break;
		}
		
}
void AttitudeCallback(rcphone::rpydata msg)
{

		float rt=0.0;
		float pt=0.0;
		float yt=0.0;
		rt=msg.roll;
		pt=msg.pitch;
		yt=msg.yaw;
		if(rt<-20)
				rt=-20;
		if(rt>20)
				rt=20;
		if(pt<-20)
				pt=-20;
		if(pt>20)
				pt=20;
		if(yt<-45)
				yt=-45;
		if(yt>45)
				yt=45;
		if(abs(rt)<=5)
				rt=0;
		if(abs(pt)<=5)
				pt=0;
		if(abs(yt)<=5)
				yt=0;
		roll_angle=rt;
		roll_vel=roll_angle/10.0;
		pitch_angle=pt;
		pitch_vel=pitch_angle/10.0;
		yaw_angle=yt;
		//ROS_INFO("roll:%f;pitch:%f;yaw:%f.",rt,pt,yt);
}
		
void VerticalCallback(rcphone::zdata msg)
{
		if(msg.z_cmd>0)
		{
				z_velocity=0.5;
		}
		else
		{
				if(msg.z_cmd<0)
				{
						z_velocity=-0.5;
				}
				else
				{
						z_velocity=0.0;
				}

		}
		ROS_INFO("zdata Received.");
		operate_code=3;
}

//! Callback functions for Mobile Commands
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local; 

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {   
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }
        

        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {   
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
        /* attitude control sample*/
        drone->takeoff();
        sleep(8);


        for(int i = 0; i < 100; i ++)
        {
            if(i < 90)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, -2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, -2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, 0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, -0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, 90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, -90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        drone->landing();

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {
        
    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw 
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{ 
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}
