/*! @file QuietStanceProvider.cpp
    @brief Implementation of behaviour for quasi-quiet stance controller
 
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

#include "QuietStanceProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "QSBallisticController.h"
#include "QSPIDController.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

QuietStanceProvider::QuietStanceProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QuietStanceProvider::QuietStanceProvider" << endl;
    #endif
    //m_lankle = new QSBallisticController(NUData::LAnklePitch);
    //m_rankle = new QSBallisticController(NUData::RAnklePitch);
    m_lankle = new QSPIDController(NUData::LAnklePitch);
    m_rankle = new QSPIDController(NUData::RAnklePitch);
    
    initPosition();
}

QuietStanceProvider::~QuietStanceProvider()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QuietStanceProvider::~QuietStanceProvider" << endl;
    #endif
    delete m_lankle;
    delete m_rankle;
}

void QuietStanceProvider::initPosition()
{
    Blackboard->Actions->add(NUData::LLeg, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::RLeg, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::LShoulderRoll, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::LElbowYaw, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::LElbowPitch, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::RShoulderRoll, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::RElbowYaw, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::RElbowPitch, Blackboard->Actions->CurrentTime + 3000, 0, 100);
    Blackboard->Actions->add(NUData::Torso, Blackboard->Actions->CurrentTime + 3000, vector<float>(2,0), 100);
}

void QuietStanceProvider::doBehaviour()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QuietStanceProvider::doBehaviour" << endl;
    #endif
    m_lankle->process(m_data, m_actions);
    m_rankle->process(m_data, m_actions);
}
 

                             
    


