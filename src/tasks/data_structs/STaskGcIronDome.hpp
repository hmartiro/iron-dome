/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * STaskGcIronDome.hpp
 *
 *  Created on: Oct 20, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKGCIRONDOME_HPP_
#define STASKGCIRONDOME_HPP_

#include <scl/control/task/data_structs/STaskBase.hpp>

namespace scl_app
{

  class STaskGcIronDome : public scl::STaskBase
  {
  public:
    STaskGcIronDome(){}
    virtual ~STaskGcIronDome(){}

    /** Processes the task's non standard parameters, which the
     * init() function stores in the task_nonstd_params_.
     *
     * NOTE : This is a sample task and hence doesn't have any non
     * standard params. You may extend it to add yours if you like. */
    virtual bool initTaskParams()
    { return true;  }
  };

} /* namespace scl_app */
#endif /* STASKGCIRONDOME_HPP_ */
