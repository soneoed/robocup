/*! @file QSController.cpp
    @brief An abstract qs controller
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#include "QSController.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include <cmath>


QSController::QSController(const NUData::id_t& joint)
{
    m_joint = joint;
    m_initialised = false;
}

QSController::~QSController()
{
}
    
void QSController::updateFilter(NUSensorsData* data)
{
    if (not m_initialised)
    {   // we need a special initialisation so the m_position is set to the proper value the first time
        data->getPosition(m_joint, m_position);
        m_velocity = 0;
        m_acceleration = 0;
        m_initialised = true;
    }
    else 
    {
        // so im going to need to use an alpha-beta-gamma filter because I need the acceleration too :(
        float beta = 0.015;
        float alpha = 2*sqrt(beta/2) - beta/2;              // coefficents taken from (Arcasoy, 1997) (Painter, 1990)
        float gamma = 0.3*alpha*beta/(2-alpha);              
        
        // Measure the position
        float measuredposition;
        data->getPosition(m_joint, measuredposition);
        double dt = (data->CurrentTime - data->PreviousTime)/1000;
        
        // Do the prediction
        float predictedposition = m_position + m_velocity*dt + 0.5*m_acceleration*pow(dt,2);
        float predictedvelocity = m_velocity + m_acceleration*dt;
        float predictedacceleration = m_acceleration;
        
        // Update the estimates
        float change = measuredposition - predictedposition;
        m_position = predictedposition + alpha*change;
        m_velocity = predictedvelocity + (beta/dt)*change;
        m_acceleration = predictedacceleration + (gamma/(2*pow(dt,2)))*change;
    }
}

float QSController::estimateGravity(NUSensorsData* data)
{
    float m = 2.8;
    float g = 9.81;
    float h = 0.25;
    float o = -0.02;
    float b = 3.3;
    float K = 1.0;          // K is a constant to compensate for calculation errors in the torque.
    
    float u, v, a, t;
    u = m_position;
    v = m_velocity;
    a = m_acceleration;
    data->getTorque(m_joint, t);
    
    return - asin((m*pow(h,2)*a + b*v - K*t)/(m*g*h));
}



