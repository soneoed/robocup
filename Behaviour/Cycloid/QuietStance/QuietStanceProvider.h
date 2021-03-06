/*! @file QuietStanceProvider.h
    @brief Declaration of a quiet stance provider for the cycloid robot
 
    @class QuietStanceProvider
    @brief A special behaviour for testing quiet stance controllers for the cycloid
 
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

#ifndef QUIETSTANCE_PROVIDER_H
#define QUIETSTANCE_PROVIDER_H

#include "../../BehaviourProvider.h"
class QSController;

class QuietStanceProvider : public BehaviourProvider
{
public:
    QuietStanceProvider(Behaviour* manager);
    ~QuietStanceProvider();
protected:
    void doBehaviour();
private:
    void initPosition();
private:
    QSController* m_lankle;
    QSController* m_rankle;
};

#endif

