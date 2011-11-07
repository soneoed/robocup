/*! @file NUSensors.h
    @brief Declaration of a base sensor class to store sensor data in a platform independent way
    @author Jason Kulk
 
    @class NUSensors
    @brief The base Sensors class
 
    How to organise all of this data has been the topic of much thought. At the moment
    I think my best idea is to group similar sensors into a sensor_t, and then have a
    vector of sensor_t's. This is OK, except I would like to be able to access the data
    based on well defined names.
 
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NUSENSORS_H
#define NUSENSORS_H

class NUSensorsData;
class EndEffectorTouch;
class Kinematics;
class OrientationUKF;
class OdometryEstimator;

#include <vector>
using namespace std;

/*! @brief Base sensor storage class
 */
class NUSensors
{
public:
    NUSensors();
    virtual ~NUSensors();
    
    void update();
    NUSensorsData* getNUSensorsData();
    
protected:
    virtual void copyFromHardwareCommunications();
    
    virtual void calculateSoftSensors();
    
    void calculateKinematics();
    
    void calculateOrientation();
    void calculateHorizon();
    void calculateButtonDurations();
    
    void calculateZMP();
    void calculateFallSense();
    void calculateOdometry();
    void calculateGaitPhase();
    void calculateCameraHeight();
    
private:
protected:
    NUSensorsData* m_data;
    double m_current_time;
    double m_previous_time;
    
    vector<float> m_previous_joint_positions;
    vector<float> m_previous_joint_velocities;
    
    EndEffectorTouch* m_touch;
    Kinematics* m_kinematicModel;
    OrientationUKF* m_orientationFilter;
    OdometryEstimator* m_odometry;
private:
};

#endif


