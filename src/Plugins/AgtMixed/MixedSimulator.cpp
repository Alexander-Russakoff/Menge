/*

License

Menge
Copyright � and trademark � 2012-14 University of North Carolina at Chapel Hill.
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation
for educational, research, and non-profit purposes, without fee, and without a
written agreement is hereby granted, provided that the above copyright notice,
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North
Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
without any accompanying services from the University of North Carolina at Chapel
Hill or the authors. The University of North Carolina at Chapel Hill and the
authors do not warrant that the operation of the program will be uninterrupted
or error-free. The end-user understands that the program was developed for research
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the authors {menge,geom}@cs.unc.edu

*/

#include "MixedSimulator.h"
#include "MengeCore/Runtime/Utils.h"

namespace Mixed {

using Menge::toFloat;
using Menge::toInt; //TODO - add to pull boolean from xml
using Menge::UtilException;
using Menge::Agents::SimulatorBase;
using Menge::Agents::XMLParamException;

////////////////////////////////////////////////////////////////
//					Implementation of Johansson::Simulator
////////////////////////////////////////////////////////////////

// These values come directly from the Johansson 2007 paper
float Simulator::AGENT_SCALE = 0.11f;
float Simulator::OBST_SCALE = 0.11f;
float Simulator::REACTION_TIME = 0.5f;
float Simulator::FORCE_DISTANCE = 1.19f;  // This is a HUGE value, but specified in the paper
float Simulator::STRIDE_TIME = 0.5f;
float Simulator::ORCA_AGENT = 0; //TODO default to not use orca

////////////////////////////////////////////////////////////////

bool Simulator::setExpParam(const std::string& paramName,
                            const std::string& value) throw(XMLParamException) {
  try {
    std::cout << "HERE!";
    if (paramName == "agent_scale") {
      AGENT_SCALE = toFloat(value);
    } else if (paramName == "obstacle_scale") {
      OBST_SCALE = toFloat(value);
    } else if (paramName == "reaction_time") {
      REACTION_TIME = toFloat(value);
    } else if (paramName == "stride_time") {
      STRIDE_TIME = toFloat(value);
    } else if (paramName == "force_distance") {
      FORCE_DISTANCE = toFloat(value);
    }
    /**
    //TODO - lets add orca param here
    else if (paramName == "orca_agent"){
      ORCA_AGENT = toFloat(value);
    }**/
    //TODO - end addition

    else if (!SimulatorBase<Agent>::setExpParam(paramName, value)) {
      // Simulator base didn't recognize the parameter either
      return false;
    }

  } catch (UtilException) {
    throw XMLParamException(std::string("Mixed parameter \"") + paramName +
                            std::string("\" value couldn't be converted to the correct "
                                        "type.  Found the value: ") +
                            value);
  }
  return true;
}
}  // namespace Mixed
