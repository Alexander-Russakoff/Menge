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

#include "MixedInitializer.h"
#include "MixedAgent.h"
#include "MengeCore/Math/RandGenerator.h"
#include "MengeCore/Runtime/Logger.h"

namespace Mixed {

using Menge::Logger;
using Menge::logger;
using Menge::Agents::BaseAgent;
using Menge::Math::ConstFloatGenerator;

////////////////////////////////////////////////////////////////
//			Implementation of Mixed::AgentInitializer
////////////////////////////////////////////////////////////////

// Default values jo
const float DIR_WEIGHT = 0.16f;  ///<	The default field-of-view weight.
// Default values orca
const float TAU = 2.5f;  ///< The default value for tau (the time horizon w.r.t. other agents).
const float TAU_OBST =
    0.15f;  ///< The default value for tau obstacles (the time horizon w.r.t. obstacles).

// for adapter
const float ORCA_AGENT = 0;
////////////////////////////////////////////////////////////////

AgentInitializer::AgentInitializer() : Menge::Agents::AgentInitializer() {
  //init jo
  _dirWeight = new ConstFloatGenerator(DIR_WEIGHT);
  //init orca
  _timeHorizon = new ConstFloatGenerator(TAU);
  _timeHorizonObst = new ConstFloatGenerator(TAU_OBST);

  _orcaAgent = new ConstFloatGenerator(ORCA_AGENT);
}

////////////////////////////////////////////////////////////////

AgentInitializer::AgentInitializer(const AgentInitializer& init)
    : Menge::Agents::AgentInitializer(init) {
  //init jo
  _dirWeight = init._dirWeight->copy();
  //init orca
  _timeHorizon = init._timeHorizon->copy();
  _timeHorizonObst = init._timeHorizonObst->copy();

  _orcaAgent = init._orcaAgent->copy();
}

////////////////////////////////////////////////////////////////

AgentInitializer::~AgentInitializer() {
  //destruct jo
  delete _dirWeight;
  //destruct orca
  delete _timeHorizon;
  delete _timeHorizonObst;

  delete _orcaAgent;
}

////////////////////////////////////////////////////////////////

bool AgentInitializer::setProperties(BaseAgent* agent) {
  Agent* a = dynamic_cast<Agent*>(agent);
  if (a == 0x0) return false;
  //set jo props
  a->_dirWeight = _dirWeight->getValue();
  //set orca props
  a->_timeHorizon = _timeHorizon->getValue();
  a->_timeHorizonObst = _timeHorizonObst->getValue();

  a->_orcaAgent = _orcaAgent->getValue();
  return Menge::Agents::AgentInitializer::setProperties(agent);
}

////////////////////////////////////////////////////////////////

bool AgentInitializer::isRelevant(const ::std::string& tagName) {
  return (tagName == "Mixed") || Menge::Agents::AgentInitializer::isRelevant(tagName);
}

////////////////////////////////////////////////////////////////

Menge::Agents::AgentInitializer::ParseResult AgentInitializer::setFromXMLAttribute(
    const ::std::string& paramName, const ::std::string& value) {
  ParseResult result = IGNORED;
  if (paramName == "fov_weight") {
    result = constFloatGenerator(_dirWeight, value);
  } else if (paramName == "tau") {
    result = constFloatGenerator(_timeHorizon, value);
  } else if (paramName == "tauObst") {
    result = constFloatGenerator(_timeHorizonObst, value);
  } else if (paramName == "orca_agent"){
    result = constFloatGenerator(_orcaAgent,value);
  }

  if (result == FAILURE) {
    logger << Logger::WARN_MSG << "Attribute \"" << paramName;
    logger << "\" had an incorrectly formed value: \"" << value;
    logger << "\".  Using default value.";
    result = ACCEPTED;
  } else if (result == IGNORED) {
    return Menge::Agents::AgentInitializer::setFromXMLAttribute(paramName, value);
  }
  return result;
}

////////////////////////////////////////////////////////////////

AgentInitializer::ParseResult AgentInitializer::processProperty(::std::string propName,
                                                                TiXmlElement* node) {
  ParseResult result = IGNORED;
  if (propName == "fov_weight") { //begin jo
    result = getFloatGenerator(_dirWeight, node);
  } else if (propName == "tau") { //begin orca
    result = getFloatGenerator(_timeHorizon, node);
  } else if (propName == "tauObst") {
    result = getFloatGenerator(_timeHorizonObst, node);
  } else if (propName == "orca_agent"){
    result = getFloatGenerator(_orcaAgent,node);
  }

  if (result == FAILURE) {
    logger << Logger::ERR_MSG << "Error extracting value distribution from Property " << propName
           << ".";
    return result;
  } else if (result == IGNORED) {
    return Menge::Agents::AgentInitializer::processProperty(propName, node);
  }
  return result;
}

////////////////////////////////////////////////////////////////

void AgentInitializer::setDefaults() {
  //set default jo props
  if (_dirWeight) delete _dirWeight;
  _dirWeight = new ConstFloatGenerator(DIR_WEIGHT);
  //set default orca props
  if (_timeHorizon) delete _timeHorizon;
  _timeHorizon = new ConstFloatGenerator(TAU);
  if (_timeHorizonObst) delete _timeHorizonObst;
  _timeHorizonObst = new ConstFloatGenerator(TAU_OBST);

  if (_orcaAgent) delete _orcaAgent;
  _orcaAgent=new ConstFloatGenerator(ORCA_AGENT);

  Menge::Agents::AgentInitializer::setDefaults();
}

////////////////////////////////////////////////////////////////

}  // namespace Mixed
