// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <stdexcept>
#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include "dynamic-hrp2_14.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

namespace dynamicgraph
{
  namespace sot
  {
    namespace hrp2
    {

      DynamicHrp2_14::DynamicHrp2_14 (const std::string & name)
	: Dynamic (name, false)
      {
	sotDEBUGIN(15);
	DynamicHrp2_14::buildModelHrp2 ();
	sotDEBUGOUT(15);
      }

      void
      DynamicHrp2_14::buildModelHrp2 ()
      {
	sotDEBUGIN(15);
	dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

	Chrp2OptHumanoidDynamicRobot* aHDR =
	  new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);

	m_HDR = aHDR;

	if (!aHDR)
	  throw std::runtime_error ("dynamic cast on HDR failed ");
	sotDEBUGOUT(15);
      }

      DynamicHrp2_14::~DynamicHrp2_14 ()
      {
	sotDEBUGINOUT(5);
	return;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicHrp2_14,"DynamicHrp2_14");
    } // end of namespace hrp2.
  } // end of namespace sot.
} // end of namespace dynamicgraph.
