/*! @file QSPIDController.cpp
    @brief A simple pid controller for quiet stance
 
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

#include "QSPIDController.h"
#include "Motion/Tools/PIDController.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

QSPIDController::QSPIDController(const NUData::id_t& joint) : QSController(joint)
{
    m_controller = new PIDController(m_joint.Name, 2.5, 10, 2.0, 0.2, -0.60, 0.60);
    m_controller->set(-0.02);
}

QSPIDController::~QSPIDController()
{
    delete m_controller;
}

void QSPIDController::process(NUSensorsData* data, NUActionatorsData* actions)
{
    updateFilter(data);
    float t = data->CurrentTime;
    float p = 0;
    /*float Kp, Ki, Kd;
    m_gains.open("/root/pidgains.txt");
    m_gains >> Kp >> Ki >> Kd;
    m_gains.close();
    m_controller->set(Kp, Ki, Kd);*/
    cout << estimateGravity(data) << endl;
    m_controller->set(-0.02 - estimateGravity(data));
    if (data->getPosition(m_joint, p))
        actions->add(m_joint, t, m_controller->doIt(t, p));
}

