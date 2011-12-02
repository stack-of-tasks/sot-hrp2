// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of sot-hrp2.
// sot-hrp2 is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-hrp2 is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-hrp2. If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_HRP2_DYNAMIC_HRP2_14_HH
# define SOT_HRP2_DYNAMIC_HRP2_14_HH
# include <sot-dynamic/dynamic.h>
# include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

# if defined (WIN32)
#   if defined (dynamic_hrp2_14_EXPORTS)
#     define DYNAMICHRP2_14_EXPORT __declspec(dllexport)
#   else
#     define DYNAMICHRP2_14_EXPORT __declspec(dllimport)
#   endif
# else
#   define DYNAMICHRP2_14_EXPORT
# endif

namespace dynamicgraph
{
  namespace sot
  {
    namespace hrp2
    {
      /// \brief Optimized sot-dynamics entity using algorithms
      /// dedicated to the hrp2-14 kinematics structure.
      ///
      /// See hrp2_14_optimized package for more information.
      class DYNAMICHRP2_14_EXPORT DynamicHrp2_14 : public Dynamic
      {
	DYNAMIC_GRAPH_ENTITY_DECL ();
      public:
	explicit DynamicHrp2_14 (const std::string& name);
	virtual ~DynamicHrp2_14 ();
	void buildModelHrp2 ();
      };
    } // end of namespace hrp2.
  } // end of namespace sot.
} // end of namespace dynamicgraph.


#endif //! SOT_HRP2_DYNAMIC_HRP2_14_HH
