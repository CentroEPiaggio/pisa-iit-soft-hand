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

// ROS
#include <ros/console.h>

// Pluginlib
#include <pluginlib/class_list_macros.h>

// ros_control
#include <hardware_interface/internal/demangle_symbol.h>
#include <adaptive_transmission/adaptive_synergy_transmission.h>
#include <adaptive_transmission/adaptive_synergy_transmission_loader.h>
#include <boost/lexical_cast.hpp>

namespace adaptive_transmission_interface
{

using namespace transmission_interface;

TransmissionSharedPtr
AdaptiveSynergyTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  // Transmission should contain only one actuator/joint
  if (!checkActuatorDimension(transmission_info, 1)) {return TransmissionSharedPtr();}
  if (!checkJointDimension(transmission_info,    19)) {return TransmissionSharedPtr();}

  // Get actuator and joint configuration sorted by role: [actuator1] and [joint1,..., joint19]
  std::vector<double> act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok) {return TransmissionSharedPtr();}

  std::vector<double> jnt_reduction;
  std::vector<double> jnt_elastic;
  std::vector<double> jnt_offset;
  const bool jnt_config_ok = getJointConfig(transmission_info,
                                            jnt_reduction,
                                            jnt_elastic,
                                            jnt_offset);

  if (!jnt_config_ok) {return TransmissionSharedPtr();}

  // Transmission instance
  try
  {
    TransmissionSharedPtr transmission(new AdaptiveSynergyTransmission(act_reduction,
                                                              jnt_reduction,
                                                              jnt_elastic,
                                                              jnt_offset));
    return transmission;
  }
  catch(const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" << transmission_info.name_ << "' of type '" <<
                           demangledTypeName<AdaptiveSynergyTransmission>()<< "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

bool AdaptiveSynergyTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                       std::vector<double>&    actuator_reduction)
{
  const std::string ACTUATOR1_ROLE = "actuator1";
  
  std::vector<TiXmlElement> act_elements(1,"");
  std::vector<std::string>  act_names(1);
  std::vector<std::string>  act_roles(1);

  for (unsigned int i = 0; i < 1; ++i)
  {
    // Actuator name
    act_names[i] = transmission_info.actuators_[i].name_;

    // Actuator xml element
    act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

    // Populate role string
    std::string& act_role = act_roles[i];
    const ParseStatus act_role_status = getActuatorRole(act_elements[i],
                                                        act_names[i],
                                                        transmission_info.name_,
                                                        false, // Required
                                                        act_role);
    if (act_role_status != SUCCESS) {return false;}

    // Validate role string
    if (ACTUATOR1_ROLE != act_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << act_names[i] << "' of transmission '" << transmission_info.name_ <<
                             "' does not specify a valid <role> element. Got '" << act_role << "', expected '" <<
                             ACTUATOR1_ROLE << "'.");
      return false;
    }
  }

  // Parse required mechanical reductions
  actuator_reduction.resize(1);

  const ParseStatus reduction_status = getActuatorReduction(act_elements[0],
                                                              act_names[0],
                                                              transmission_info.name_,
                                                              true, // Required
                                                              actuator_reduction[0]);
  if (reduction_status != SUCCESS) {return false;}

  return true;
}

bool AdaptiveSynergyTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                    std::vector<double>&    joint_reduction,
                                                    std::vector<double>&    joint_elastic,
                                                    std::vector<double>&    joint_offset)
{
  const std::string JOINT1_ROLE = "joint1";
  const std::string JOINT2_ROLE = "joint2";
  const std::string JOINT3_ROLE = "joint3";
  const std::string JOINT4_ROLE = "joint4";
  const std::string JOINT5_ROLE = "joint5";
  const std::string JOINT6_ROLE = "joint6";
  const std::string JOINT7_ROLE = "joint7";
  const std::string JOINT8_ROLE = "joint8";
  const std::string JOINT9_ROLE = "joint9";
  const std::string JOINT10_ROLE = "joint10";
  const std::string JOINT11_ROLE = "joint11";
  const std::string JOINT12_ROLE = "joint12";
  const std::string JOINT13_ROLE = "joint13";
  const std::string JOINT14_ROLE = "joint14";
  const std::string JOINT15_ROLE = "joint15";
  const std::string JOINT16_ROLE = "joint16";
  const std::string JOINT17_ROLE = "joint17";
  const std::string JOINT18_ROLE = "joint18";
  const std::string JOINT19_ROLE = "joint19";
    
  std::vector<TiXmlElement> jnt_elements(19,"");
  std::vector<std::string>  jnt_names(19);
  std::vector<std::string>  jnt_roles(19);

  for (unsigned int i = 0; i < 19; ++i)
  {
    // Joint name
    jnt_names[i] = transmission_info.joints_[i].name_;

    // Joint xml element
    jnt_elements[i] = loadXmlElement(transmission_info.joints_[i].xml_element_);

    // Populate role string
    std::string& jnt_role = jnt_roles[i];
    const ParseStatus jnt_role_status = getJointRole(jnt_elements[i],
                                                     jnt_names[i],
                                                     transmission_info.name_,
                                                     true, // Required
                                                     jnt_role);
    if (jnt_role_status != SUCCESS) {return false;}

    // Validate role string
    if (JOINT1_ROLE != jnt_role && JOINT2_ROLE != jnt_role && JOINT3_ROLE != jnt_role && JOINT4_ROLE != jnt_role
      && JOINT5_ROLE != jnt_role && JOINT6_ROLE != jnt_role && JOINT7_ROLE != jnt_role && JOINT8_ROLE != jnt_role
      && JOINT9_ROLE != jnt_role && JOINT10_ROLE != jnt_role && JOINT11_ROLE != jnt_role && JOINT12_ROLE != jnt_role 
      && JOINT13_ROLE != jnt_role && JOINT14_ROLE != jnt_role && JOINT15_ROLE != jnt_role && JOINT16_ROLE != jnt_role
      && JOINT17_ROLE != jnt_role && JOINT18_ROLE != jnt_role && JOINT19_ROLE != jnt_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Joint '" << jnt_names[i] << "' of transmission '" << transmission_info.name_ <<
                             "' does not specify a valid <role> element. Got '" << jnt_role << "', expected '" <<
                             JOINT1_ROLE << "' to '" << JOINT19_ROLE << "'.");
      return false;
    }
  }

  // TODO: check that roles are different
  // Roles must be different
/*  if (jnt_roles[0] == jnt_roles[1])
  {
    ROS_ERROR_STREAM_NAMED("parser", "Joints '" << jnt_names[0] << "' and '" << jnt_names[1] <<
                           "' of transmission '" << transmission_info.name_ <<
                           "' must have different roles. Both specify '" << jnt_roles[0] << "'.");
    return false;
  }*/

  // Joint configuration
  joint_reduction.resize(19, 1.0);
  joint_elastic.resize(19, 1.0);
  joint_offset.resize(19, 0.0);
  for (unsigned int i = 0; i < 19; ++i)
  {

    // Parse optional mechanical reductions. Even though it's optional --and to avoid surprises-- we fail if the element
    // is specified but is of the wrong type
    const ParseStatus reduction_status = getJointReduction(jnt_elements[i],
                                                           jnt_names[i],
                                                           transmission_info.name_,
                                                           false, // Optional
                                                           joint_reduction[i]);
    if (reduction_status == BAD_TYPE) {return false;}

    // Parse optional mechanical elasticities. Even though it's optional --and to avoid surprises-- we fail if the element
    // is specified but is of the wrong type
    const ParseStatus elastic_status = getJointElastic(jnt_elements[i],
                                                           jnt_names[i],
                                                           transmission_info.name_,
                                                           false, // Optional
                                                           joint_elastic[i]);
    if (elastic_status == BAD_TYPE) {return false;}

    // Parse optional joint offset. Even though it's optional --and to avoid surprises-- we fail if the element is
    // specified but is of the wrong type
    const ParseStatus offset_status = getJointOffset(jnt_elements[i],
                                                     jnt_names[i],
                                                     transmission_info.name_,
                                                     false, // Optional
                                                     joint_offset[i]);
    if (offset_status == BAD_TYPE) {return false;}
  }

  return true;
}

TransmissionLoader::ParseStatus AdaptiveSynergyTransmissionLoader::getJointElastic(const TiXmlElement& parent_el, const std::string& joint_name, const std::string& transmission_name, bool required, double& elastic)
{
    // Get XML element
    const TiXmlElement* elastic_el = parent_el.FirstChildElement("mechanicalElasticity");
    if(!elastic_el)
    {
        if (required)
        {
            ROS_ERROR_STREAM_NAMED("parser", "Joint '" << joint_name << "' of transmission '" << transmission_name <<
            "' does not specify the required <mechanicalElasticity> element.");
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("parser", "Joint '" << joint_name << "' of transmission '" << transmission_name <<
            "' does not specify the optional <mechanicalElasticity> element.");
        }
        return NO_DATA;
    }
    // Cast to number
    try {elastic = boost::lexical_cast<double>(elastic_el->GetText());}
    catch (const boost::bad_lexical_cast&)
    {
        ROS_ERROR_STREAM_NAMED("parser", "Joint '" << joint_name << "' of transmission '" << transmission_name <<
        "' specifies the <mechanicalElasticity> element, but is not a number.");
        return BAD_TYPE;
    }
    return SUCCESS;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(adaptive_transmission_interface::AdaptiveSynergyTransmissionLoader,
                       transmission_interface::TransmissionLoader)
