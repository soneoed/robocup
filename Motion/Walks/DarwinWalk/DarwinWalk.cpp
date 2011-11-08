/*! @file DarwinWalk.cpp
    @brief Implementation of DarwinWalk class

    @author Aaron Wong,  Jason Kulk
 
 Copyright (c) 2009,2011 Jason Kulk, Aaron Wong
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DarwinWalk.h"
//using namespace Kinematics;
//From Darwin Library:
#include <Walking.h>	
//#include <LinuxCM730.h>	
#include "minIni.h"
#include "MotionStatus.h"

#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/Platforms/Darwin/DarwinPlatform.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"


#include <math.h>
#include <list>

//! @todo TODO: put M_PI, NORMALISE, NUM_JOINTS somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

#define GYRO_WINDOW_SIZE    100
#define MARGIN_OF_SD        2.0

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}


/*! Creates a module to walk using Darwin's walk engine
 */
DarwinWalk::DarwinWalk(NUSensorsData* data, NUActionatorsData* actions) :  NUWalk(data, actions)
{
    m_CM730 = dynamic_cast<DarwinPlatform*>(Platform)->cm730;
    m_CalibrationStatus = 0;
	m_FBGyroCenter = 512;
	m_RLGyroCenter = 512;
    
    DarwinWalkEngine = Robot::Walking::GetInstance();
    minIni* ini = new minIni("config.ini");
	Robot::Walking::GetInstance()->LoadINISettings(ini);
	Robot::Walking::GetInstance()->Initialize();
    m_walk_parameters.load("DarwinWalkDefault");
    
    initInitialPosition();
}

/*! @brief Destructor for walk module
 */
DarwinWalk::~DarwinWalk()
{
}

void DarwinWalk::initInitialPosition()
{
    // arm positions can be hard-coded because they never change
    float L_arms[] = {-0.29, 0.833+1.5707963, -0.5-1.5707963};
	float R_arms[] = {0.29, -0.837-1.5707963, 0.5+1.5707963};
    m_initial_larm = vector<float>(L_arms, L_arms + sizeof(L_arms)/sizeof(*L_arms));
    m_initial_rarm = vector<float>(R_arms, R_arms + sizeof(R_arms)/sizeof(*R_arms));
    
    // leg positions need to be taken from the walk engine
    m_initial_lleg = vector<float>(6,0);
    m_initial_lleg[0] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_ROLL);
    m_initial_lleg[1] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_PITCH);
    m_initial_lleg[2] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_YAW);
    m_initial_lleg[3] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_KNEE);
    m_initial_lleg[4] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_ROLL);
    m_initial_lleg[5] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_PITCH);
    m_initial_rleg = vector<float>(6,0);
    m_initial_rleg[0] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_ROLL);
    m_initial_rleg[1] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_PITCH);
    m_initial_rleg[2] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_YAW);
    m_initial_rleg[3] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_KNEE);
    m_initial_rleg[4] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_ROLL);
    m_initial_rleg[5] = DarwinWalkEngine->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_PITCH);
}

void DarwinWalk::doWalk()
{
	//SET THE MOTOR POSITIONS IN THE WALK ENGINE:
	updateWalkEngineSensorData();
	//TELL THE WALK ENGINE THE NEW COMMAND
	if(m_speed_x==0  && m_speed_y==0  && m_speed_yaw ==0 )
		Robot::Walking::GetInstance()->Stop();
	else
		Robot::Walking::GetInstance()->Start();

	Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE = m_speed_x;        
	Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_speed_y;
	Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE = m_speed_yaw*57.295;       // convert to degrees per second

	//cout << "Walk Commands: " << Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE << endl;
	Robot::Walking::GetInstance()->Process();

	//GET THE NEW TARGET POSITIONS FROM THE WALK ENGINE
	updateActionatorsData();
	return;
}
void DarwinWalk::updateWalkEngineSensorData()
{
	//Joint Order is same as platform
	vector<float> nu_jointpositions;
    m_data->getPosition(NUSensorsData::All, nu_jointpositions);

	//Arms:
	SetDarwinSensor(Robot::JointData::ID_L_SHOULDER_ROLL,	nu_jointpositions[2]);
	SetDarwinSensor(Robot::JointData::ID_L_SHOULDER_PITCH,	nu_jointpositions[3]);
	SetDarwinSensor(Robot::JointData::ID_L_ELBOW,			nu_jointpositions[4]);
	SetDarwinSensor(Robot::JointData::ID_R_SHOULDER_ROLL,	nu_jointpositions[5]);
	SetDarwinSensor(Robot::JointData::ID_R_SHOULDER_PITCH,	nu_jointpositions[6]);
	SetDarwinSensor(Robot::JointData::ID_R_ELBOW,			nu_jointpositions[7]);

	//Leg:
	SetDarwinSensor(Robot::JointData::ID_L_HIP_ROLL,		nu_jointpositions[8]);
	SetDarwinSensor(Robot::JointData::ID_L_HIP_PITCH,		nu_jointpositions[9]);
	SetDarwinSensor(Robot::JointData::ID_L_HIP_YAW,			nu_jointpositions[10]);
	SetDarwinSensor(Robot::JointData::ID_L_KNEE,			nu_jointpositions[11]);
	SetDarwinSensor(Robot::JointData::ID_L_ANKLE_ROLL,		nu_jointpositions[12]);
	SetDarwinSensor(Robot::JointData::ID_L_ANKLE_PITCH,		nu_jointpositions[13]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_ROLL,		nu_jointpositions[14]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_PITCH,		nu_jointpositions[15]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_YAW,			nu_jointpositions[16]);
	SetDarwinSensor(Robot::JointData::ID_R_KNEE,			nu_jointpositions[17]);
	SetDarwinSensor(Robot::JointData::ID_R_ANKLE_ROLL,		nu_jointpositions[18]);
	SetDarwinSensor(Robot::JointData::ID_R_ANKLE_PITCH,		nu_jointpositions[19]);

	//Update walk engine gyro:
    
    // The following gyro calibration code comes from the Darwin's MotionManager::Process() function
    // It is included here to keep the walk engine happy
    // calibrate gyro sensor
    if(m_CalibrationStatus == 0 || m_CalibrationStatus == -1)
    {
        static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;
        
        if(buf_idx < GYRO_WINDOW_SIZE)
        {
            if(m_CM730->m_BulkReadData[Robot::CM730::ID_CM].error == 0)
            {
                fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Y_L);
                rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_X_L);
                buf_idx++;
            }
        }
        else
        {
            double fb_sum = 0.0, rl_sum = 0.0;
            double fb_sd = 0.0, rl_sd = 0.0;
            double fb_diff, rl_diff;
            double fb_mean = 0.0, rl_mean = 0.0;
            
            buf_idx = 0;
            
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_sum += fb_gyro_array[i];
                rl_sum += rl_gyro_array[i];
            }
            fb_mean = fb_sum / GYRO_WINDOW_SIZE;
            rl_mean = rl_sum / GYRO_WINDOW_SIZE;
            
            fb_sum = 0.0; rl_sum = 0.0;
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_diff = fb_gyro_array[i] - fb_mean;
                rl_diff = rl_gyro_array[i] - rl_mean;
                fb_sum += fb_diff * fb_diff;
                rl_sum += rl_diff * rl_diff;
            }
            fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
            rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);
            
            if(fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD)
            {
                m_FBGyroCenter = (int)fb_mean;
                m_RLGyroCenter = (int)rl_mean;
                m_CalibrationStatus = 1;
            }
            else
            {
                m_FBGyroCenter = 512;
                m_RLGyroCenter = 512;
                m_CalibrationStatus = -1;
            }
        }
    }
    
    if(m_CalibrationStatus == 1)
    {
        if(m_CM730->m_BulkReadData[Robot::CM730::ID_CM].error == 0)
        {
            Robot::MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            Robot::MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_X_L) - m_RLGyroCenter;
            Robot::MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_X_L);
            Robot::MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_Y_L);
        }
    }
}

void DarwinWalk::SetDarwinSensor(int id, float angle)
{
	Robot::Walking::GetInstance()->m_Joint.SetRadian(id,angle);
}
void DarwinWalk::updateActionatorsData()
{
	//UPDATE ARMS:
	static vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 1);
	static vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 1);

	vector<vector<float> >& armgains = m_walk_parameters.getArmGains();

	nu_nextRightArmJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_SHOULDER_ROLL);
	nu_nextRightArmJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_SHOULDER_PITCH)-1.5707963;
	nu_nextRightArmJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ELBOW);

	nu_nextLeftArmJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_SHOULDER_ROLL);
	nu_nextLeftArmJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_SHOULDER_PITCH)+1.5707963;
	nu_nextLeftArmJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ELBOW);

	m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, armgains[0]);
	m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, armgains[0]);

	//UPDATE LEGS:
	static vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0);
	static vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0);

	vector<vector<float> >& leggains = m_walk_parameters.getLegGains();

	nu_nextRightLegJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_ROLL);
	nu_nextRightLegJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_PITCH);	
	nu_nextRightLegJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_YAW);
	nu_nextRightLegJoints[3] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_KNEE);
	nu_nextRightLegJoints[4] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_ROLL);
	nu_nextRightLegJoints[5] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_PITCH);
	
	nu_nextLeftLegJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_ROLL);
	nu_nextLeftLegJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_PITCH);
	nu_nextLeftLegJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_YAW);
	nu_nextLeftLegJoints[3] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_KNEE);
	nu_nextLeftLegJoints[4] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_ROLL);
	nu_nextLeftLegJoints[5] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_PITCH);
	
	m_actions->add(NUActionatorsData::RLeg, Platform->getTime(), nu_nextRightLegJoints, leggains[0]);
	m_actions->add(NUActionatorsData::LLeg, Platform->getTime(), nu_nextLeftLegJoints, leggains[0]);


	return;
}


void DarwinWalk::setWalkParameters(const WalkParameters& walkparameters)
{
    debug << walkparameters << endl;
    NUWalk::setWalkParameters(walkparameters);
    vector<Parameter>& parameters = m_walk_parameters.getParameters(); 
    int idx = 0;
    
    DarwinWalkEngine->PERIOD_TIME = 1000.0/parameters[idx++].get();       // step frequency (Hz to period in ms)
    
    DarwinWalkEngine->Z_MOVE_AMPLITUDE = 10*parameters[idx++].get();      // step height (cm to mm)
    DarwinWalkEngine->Y_SWAP_AMPLITUDE = 10*parameters[idx++].get();      // step left-right (cm to mm)
    DarwinWalkEngine->Z_SWAP_AMPLITUDE = 10*parameters[idx++].get();      // swing top-down (cm to mm)
    
    DarwinWalkEngine->X_OFFSET = 10*parameters[idx++].get();
    DarwinWalkEngine->Y_OFFSET = 10*parameters[idx++].get();
    DarwinWalkEngine->Z_OFFSET = 10*parameters[idx++].get();
    DarwinWalkEngine->R_OFFSET = parameters[idx++].get();
    DarwinWalkEngine->P_OFFSET = parameters[idx++].get();
    DarwinWalkEngine->HIP_PITCH_OFFSET = parameters[idx++].get();
    DarwinWalkEngine->PELVIS_OFFSET = parameters[idx++].get();
    
    DarwinWalkEngine->DSP_RATIO = parameters[idx++].get();
    DarwinWalkEngine->STEP_FB_RATIO = parameters[idx++].get();
    
    DarwinWalkEngine->BALANCE_KNEE_GAIN = parameters[idx++].get();
    DarwinWalkEngine->BALANCE_ANKLE_PITCH_GAIN = parameters[idx++].get();
    DarwinWalkEngine->BALANCE_HIP_ROLL_GAIN = parameters[idx++].get();
    DarwinWalkEngine->BALANCE_ANKLE_ROLL_GAIN = parameters[idx++].get();
}
