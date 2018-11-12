///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2014, Research Center "E. Piaggio"
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// \author Adolfo Rodriguez Tsouroukdissian

// \author Carlos Rosales, Hamal Marino, based on templates of other transmissions

#ifndef TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_LOADER_H
#define TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_LOADER_H

// TinyXML
#include <tinyxml.h>

// ros_control
#include <transmission_interface/transmission_loader.h>

namespace adaptive_transmission_interface
{

using namespace transmission_interface;

/**
 * \brief Class for loading a Adaptive Synergy transmission instance from configuration data.
 */
class AdaptiveSynergyTransmissionLoader : public TransmissionLoader
{
public:

  TransmissionSharedPtr load(const TransmissionInfo& transmission_info);

private:
  static bool getActuatorConfig(const TransmissionInfo& transmission_info,
                                std::vector<double>&    actuator_reduction);

  static bool getJointConfig(const TransmissionInfo& transmission_info,
                             std::vector<double>&    joint_reduction,
                             std::vector<double>&    joint_elastic,
                             std::vector<double>&    joint_offset);

  // specific for SoftHand elastic joints
  static ParseStatus getJointElastic(const TiXmlElement& parent_el,
                                     const std::string&  joint_name,
                                     const std::string&  transmission_name,
                                     bool                required,
                                     double&             elastic);
};

} // namespace

#endif //header guard
