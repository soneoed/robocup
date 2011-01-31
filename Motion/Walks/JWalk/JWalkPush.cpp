/*! @file JWalkPush.cpp
    @brief Implementation of a jwalk's push state
 
    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#include "JWalkPush.h"

JWalkPush::JWalkPush(const NUData::id_t& leg) : JWalkState("Push", leg)
{
    
}

JWalkPush::~JWalkPush()
{
    
}

void JWalkPush::doIt()
{
    // The push state is always an extension. For a walk the extension is done using the
    // ankle, for a run/sprint the extension is done using both the ankle and knee.
    // The distance between the two feet specifies the amount of extension
    // The step frequency will specify the transfer speed
    
    vector<float> thisleg, otherleg;
    bool valid = true;
    valid &= Blackboard->Sensors->getEndPosition(m_leg, thisleg);
    valid &= Blackboard->Sensors->getEndPosition(m_other_leg, otherleg);
    if (valid)
    {
        float d = sqrt(pow(thisleg[0] - otherleg[0], 2) + pow(thisleg[1] - otherleg[1], 2));
        float l1 = -thisleg[2];
        float l2 = -otherleg[2];
        float alpha = atan(5/l2);
        float ext = sqrt(pow(l2,2) + pow(d,2) - 2*d*l2*cos(1.571 + alpha)) - l1;
        cout << ext << endl;
        
        static bool done = false;
        if (not done)
        {
            Blackboard->Actions->add(NUData::LAnklePitch, Blackboard->Actions->CurrentTime + 4000, acos(ext/11));
            Blackboard->Actions->add(NUData::LHipPitch, Blackboard->Actions->CurrentTime + 4000, -4*acos((ext - l1)/l1));
            Blackboard->Actions->add(NUData::LKneePitch, Blackboard->Actions->CurrentTime + 4000, 8*acos((ext - l1)/l1));
            done = true;
        }
    }
}

JWalkPush* JWalkPush::next()
{
    return this;
}

