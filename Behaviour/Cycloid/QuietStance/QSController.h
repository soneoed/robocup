/*! @file QSController.h
    @brief An abstract qs controller
 
    @class QSController
    @brief An abstract class for quiet stance controllers.
           Every controller need only have a constructor taking in the joint to control,
           and a process function which actually does the control itself
 
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

#ifndef QSController_H
#define QSController_H

#include "Infrastructure/NUData.h"
class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

class QSController
{
public:
    QSController(const NUData::id_t& joint);
    virtual ~QSController();
    
    virtual void process(NUSensorsData* data, NUActionatorsData* actions) = 0;
protected:
    void updateFilter(NUSensorsData* data);
    float estimateGravity(NUSensorsData* data);
protected:
    NUData::id_t m_joint;
    
    bool m_initialised;
    float m_position;
    float m_velocity;
    float m_acceleration;
};

#endif

