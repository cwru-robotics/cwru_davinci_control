/******************************************************************************
**
**   Joint Name definitions for the dvrk.
**   Copyright (C) 2017  Tom Shkurti, Russell Jackson, & Wyatt Newman
**   Case Western Reserve University
**
**   This program is free software: you can redistribute it and/or modify
**   it under the terms of the GNU General Public License as published by
**   the Free Software Foundation, either version 3 of the License, or
**   (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**   but WITHOUT ANY WARRANTY; without even the implied warranty of
**   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**   GNU General Public License for more details.
**
**   You should have received a copy of the GNU General Public License
**   along with this program.  If not, see <http://www.gnu.org/licenses/>.
**
******************************************************************************/
#ifndef CWRU_DAVINCI_CONTROL_JOINT_NAMES_H
#define CWRU_DAVINCI_CONTROL_JOINT_NAMES_H

#include <string>
#include <vector>

namespace dvrk_control
{
  const std::vector<std::string> joint_names =
  {
    // These should always reflect the joint names given in cwru_davinci_geometry_models/model
    // This is a hard-coded joint labeling.
    "outer_yaw",
    "outer_pitch",
    "outer_insertion",
    "outer_roll",
    "outer_wrist_pitch",
    "outer_wrist_yaw",
    "jaw"
  };
};  // namespace dvrk_control

#endif  // CWRU_DAVINCI_CONTROL_JOINT_NAMES_H
