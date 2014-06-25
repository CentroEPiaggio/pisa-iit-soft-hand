///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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

/// \author Adolfo Rodriguez Tsouroukdissian
/// \ modified by Marco Bartolomei 2014

#ifndef TRANSMISSION_INTERFACE_ADAPTIVESINERGY_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_ADAPTIVESINERGY_TRANSMISSION_H

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace AdaptiveSinergy_interface
{

/**
* \brief Implementation of a differential transmission.
*
* This transmission relates <b>two actuators</b> and <b>two joints</b> through a differential mechanism, as illustrated
* below.
* \image html differential_transmission.png
*
* <CENTER>
* <table>
* <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
* <tr><td>
* <b> Actuator to joint </b>
* </td>
* <td>
* \f{eqnarray*}{
* \tau_{j_1} & = & n_{j_1} ( n_{a_1} \tau_{a_1} + n_{a_2} \tau_{a_2} ) \\[2.5em]
* \tau_{j_2} & = & n_{j_2} ( n_{a_1} \tau_{a_1} + n_{a_2} \tau_{a_2} )
* \f}
* </td>
* <td>
* \f{eqnarray*}{
* \dot{x}_{j_1} & = & \frac{ \dot{x}_{a_1} / n_{a_1} + \dot{x}_{a_2} / n_{a_2} }{2 n_{j_1}} \\[1em]
* \dot{x}_{j_2} & = & \frac{ \dot{x}_{a_1} / n_{a_1} - \dot{x}_{a_2} / n_{a_2} }{2 n_{j_2}}
* \f}
* </td>
* <td>
* \f{eqnarray*}{
* x_{j_1} & = & \frac{ x_{a_1} / n_{a_1} + x_{a_2} / n_{a_2} }{2 n_{j_1}} + x_{off_1} \\[1em]
* x_{j_2} & = & \frac{ x_{a_1} / n_{a_1} - x_{a_2} / n_{a_2} }{2 n_{j_1}} + x_{off_2}
* \f}
* </td>
* </tr>
* <tr><td>
* <b> Joint to actuator </b>
* </td>
* <td>
* \f{eqnarray*}{
* \tau_{a_1} & = & \frac{ \tau_{j_1} / n_{j_1} + \tau_{j_2} / n_{j_2} }{2 n_{a_1}} \\[1em]
* \tau_{a_2} & = & \frac{ \tau_{j_1} / n_{j_1} - \tau_{j_2} / n_{j_2} }{2 n_{a_1}}
* \f}
* </td>
* <td>
* \f{eqnarray*}{
* \dot{x}_{a_1} & = & n_{a_1} ( n_{j_1} \dot{x}_{j_1} + n_{j_2} \dot{x}_{j_2} ) \\[2.5em]
* \dot{x}_{a_2} & = & n_{a_2} ( n_{j_1} \dot{x}_{j_1} - n_{j_2} \dot{x}_{j_2} )
* \f}
* </td>
* <td>
* \f{eqnarray*}{
* x_{a_1} & = & n_{a_1} \left[ n_{j_1} (x_{j_1} - x_{off_1}) + n_{j_2} (x_{j_2} - x_{off_2}) \right] \\[2.5em]
* x_{a_2} & = & n_{a_2} \left[ n_{j_1} (x_{j_1} - x_{off_1}) - n_{j_2} (x_{j_2} - x_{off_2}) \right]
* \f}
* </td></tr></table>
* </CENTER>
*
* where:
* - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables, respectively.
* - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space variables, respectively.
* - \f$ x_{off}\f$ represents the offset between motor and joint zeros, expressed in joint position coordinates
* (cf. SimpleTransmission class documentation for a more detailed descrpition of this variable).
* - \f$ n \f$ represents a transmission ratio. Reducers/amplifiers are allowed on both the actuator and joint sides
* (depicted as timing belts in the figure).
* A transmission ratio can take any real value \e except zero. In particular:
* - If its absolute value is greater than one, it's a velocity reducer / effort amplifier, while if its absolute
* value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
* - Negative values represent a direction flip, ie. input and output move in opposite directions.
* - <b>Important:</b> Use transmission ratio signs to match this class' convention of positive actuator/joint
* directions with a given mechanical design, as they will in general not match.
*
* \note This implementation currently assumes a specific layout for location of the actuators and joint axes which is
* common in robotic mechanisms. Please file an enhancement ticket if your use case does not adhere to this layout.
*
* \ingroup transmission_types
*/
class AdapriveSinergyTransmission : public Transmission
{
public:
  /**
* \param actuatojr_reduction Reduction ratio of actuators.
* \param joint_reduction Reduction ratio of joints.
* \param joint_offset Joint position offset used in the position mappings.
* \pre Nonzero actuator and joint reduction values.
*/
  AdatptiveSinergyTransmission(const std::vector<double>& actuatojr_reduction,
                               const std::vector<double>& joint_reduction,
                               const std::vector<double>& elasic_reduction,
                               const std::vector<double>& joint_offset = std::vector<double>(20, 0.0));

  /**
* \brief Transform \e effort variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint effort vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void actuatorToJointEffort(const ActuatorData& act_data,
                                   JointData& jnt_data);

  /**
* \brief Transform \e velocity variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void actuatorToJointVelocity(const ActuatorData& act_data,
                                     JointData& jnt_data);

  /**
* \brief Transform \e position variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint position vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void actuatorToJointPosition(const ActuatorData& act_data,
                                     JointData& jnt_data);

  /**
* \brief Transform \e effort variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint effort vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void jointToActuatorEffort(const JointData& jnt_data,
                                   ActuatorData& act_data);

  /**
* \brief Transform \e velocity variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void jointToActuatorVelocity(const JointData& jnt_data,
                                     ActuatorData& act_data);

  /**
* \brief Transform \e position variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint position vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void jointToActuatorPosition(const JointData& jnt_data,
                                     ActuatorData& act_data);

  std::size_t numActuators() const {return 1;}
  std::size_t numJoints() const {return 20;}

  const std::vector<double>& getActuatorReduction() const {return act_reduction_;}
  const std::vector<double>& getJointReduction() const {return jnt_reduction_;}
  const std::vector<double>& getElasticReduction() const {return els_reduction_;}
  const std::vector<double>& getJointOffset() const {return jnt_offset_;}

protected:
  std::vector<double> act_reduction_;
  std::vector<double> els_reduction_;
  std::vector<double> jnt_reduction_;
  std::vector<double> jnt_offset_;
};

inline AdaptiveSybergyTransmission::AdaptiveSybergyTransmission(const std::vector<double>& actuatojr_reduction,
                                                                const std::vector<double>& joint_reduction,
                                                                const std::vector<double>& elastic_reduction,
                                                                const std::vector<double>& joint_offset)
  : Transmission(),
    act_reduction_(actuatojr_reduction),
    els_reduction_(elastic_reduction),
    jnt_reduction_(joint_reduction),
    jnt_offset_(joint_offset)
{
  if (numActuators() != act_reduction_.size() ||
      numJoints() != jnt_reduction_.size()    ||
      numElastics() != els_reduction_.size()   ||
      numJoints() != jnt_offset_.size())
  {
    throw AdaptiveSynergyTransmissionInterfaceException("Adaptive Sybergy transmission: Elastic and Joint reduction and offset vectors must have size 20, Actuator must have size 1.");
  }

  if (0.0 == act_reduction_[0]  ||
      0.0 == jnt_reduction_[0]  ||
      0.0 == jnt_reduction_[1]  ||
      0.0 == jnt_reduction_[2]  ||
      0.0 == jnt_reduction_[3]  ||
      0.0 == jnt_reduction_[4]  || 
      0.0 == jnt_reduction_[5]  ||
      0.0 == jnt_reduction_[6]  ||
      0.0 == jnt_reduction_[7]  ||
      0.0 == jnt_reduction_[8]  ||
      0.0 == jnt_reduction_[9]  || 
      0.0 == jnt_reduction_[10] ||
      0.0 == jnt_reduction_[11] ||
      0.0 == jnt_reduction_[12] ||
      0.0 == jnt_reduction_[13] ||
      0.0 == jnt_reduction_[14] ||
      0.0 == jnt_reduction_[15] ||
      0.0 == jnt_reduction_[16] ||
      0.0 == jnt_reduction_[17] ||
      0.0 == jnt_reduction_[18] ||
      0.0 == jnt_reduction_[19] ||
      0.0 == els_reduction_[0]  ||
      0.0 == els_reduction_[1]  ||
      0.0 == els_reduction_[2]  ||
      0.0 == els_reduction_[3]  ||
      0.0 == els_reduction_[4]  || 
      0.0 == els_reduction_[5]  ||
      0.0 == els_reduction_[6]  ||
      0.0 == els_reduction_[7]  ||
      0.0 == els_reduction_[8]  ||
      0.0 == els_reduction_[9]  || 
      0.0 == els_reduction_[10] ||
      0.0 == els_reduction_[11] ||
      0.0 == els_reduction_[12] ||
      0.0 == els_reduction_[13] ||
      0.0 == els_reduction_[14] ||
      0.0 == els_reduction_[15] ||
      0.0 == els_reduction_[16] ||
      0.0 == els_reduction_[17] ||
      0.0 == els_reduction_[18] ||
      0.0 == els_reduction_[19]
  )
  {
    throw AdaptiveSynergyTransmissionInterfaceException("Transmission parameters ratios cannot be zero.");
  }
}

inline void AdaptiveSybergyTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                                     JointData& jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.position[0]  && jnt_data.position[1]  && jnt_data.position[2]  && jnt_data.position[3]
                            && jnt_data.position[4]  && jnt_data.position[5]  && jnt_data.position[6]  && jnt_data.position[7]
                            && jnt_data.position[8]  && jnt_data.position[9]  && jnt_data.position[10] && jnt_data.position[11]
                            && jnt_data.position[12] && jnt_data.position[13] && jnt_data.position[14] && jnt_data.position[15]
                            && jnt_data.position[16] && jnt_data.position[17] && jnt_data.position[18] && jnt_data.position[19]
                            && jnt_data.effort[0]    && jnt_data.effort[1]    && jnt_data.effort[2]    && jnt_data.effort[3]
                            && jnt_data.effort[4]    && jnt_data.effort[5]    && jnt_data.effort[6]    && jnt_data.effort[7]
                            && jnt_data.effort[8]    && jnt_data.effort[9]    && jnt_data.effort[10]   && jnt_data.effort[11]
                            && jnt_data.effort[12]   && jnt_data.effort[13]   && jnt_data.effort[14]   && jnt_data.effort[15]
                            && jnt_data.effort[16]   && jnt_data.effort[17]   && jnt_data.effort[18]   && jnt_data.effort[19]);

  // Tau_q =    R'  * tau_m -    E   *  q;
  //  20x1 = ( 20x1 *  1x1) - (20x20 * 20x1)
  // R == jnt_reduction_ = jr --> raw vector 1x20
  // E == els_reduction_ = elr --> diagonal matrix 20x20 --> we can simplified by 20x1
  // Tau_q[] = jnt_data.effort[0-19]
  // tau_m = act_data_effort[0]
  

  // Since the matrix 'E' is diagonal, we can simplify the calculation by writing 
  // the system in extended form by eliminating null values
  
  std::vector<double>& ar_  = act_reduction_;
  std::vector<double>& jr_  = jnt_reduction_;
  std::vector<double>& elr_ = els_reduction_;


  *jnt_data.effort[0]  = ((jr_[0]  * (*act_data.effort[0])) + (elr_[0]  * (*jnt_data.position[0])))/ar_[0];
  *jnt_data.effort[1]  = ((jr_[1]  * (*act_data.effort[0])) + (elr_[1]  * (*jnt_data.position[1])))/ar_[0];
  *jnt_data.effort[2]  = ((jr_[2]  * (*act_data.effort[0])) + (elr_[2]  * (*jnt_data.position[2])))/ar_[0];
  *jnt_data.effort[3]  = ((jr_[3]  * (*act_data.effort[0])) + (elr_[3]  * (*jnt_data.position[3])))/ar_[0];
  *jnt_data.effort[4]  = ((jr_[4]  * (*act_data.effort[0])) + (elr_[4]  * (*jnt_data.position[4])))/ar_[0];
  *jnt_data.effort[5]  = ((jr_[5]  * (*act_data.effort[0])) + (elr_[5]  * (*jnt_data.position[5])))/ar_[0];
  *jnt_data.effort[6]  = ((jr_[6]  * (*act_data.effort[0])) + (elr_[6]  * (*jnt_data.position[6])))/ar_[0];
  *jnt_data.effort[7]  = ((jr_[7]  * (*act_data.effort[0])) + (elr_[7]  * (*jnt_data.position[7])))/ar_[0];
  *jnt_data.effort[8]  = ((jr_[8]  * (*act_data.effort[0])) + (elr_[8]  * (*jnt_data.position[8])))/ar_[0];
  *jnt_data.effort[9]  = ((jr_[9]  * (*act_data.effort[0])) + (elr_[9]  * (*jnt_data.position[9])))/ar_[0];
  *jnt_data.effort[10] = ((jr_[10] * (*act_data.effort[0])) + (elr_[10] * (*jnt_data.position[10])))/ar_[0];
  *jnt_data.effort[11] = ((jr_[11] * (*act_data.effort[0])) + (elr_[11] * (*jnt_data.position[11])))/ar_[0];
  *jnt_data.effort[12] = ((jr_[12] * (*act_data.effort[0])) + (elr_[12] * (*jnt_data.position[12])))/ar_[0];
  *jnt_data.effort[13] = ((jr_[13] * (*act_data.effort[0])) + (elr_[13] * (*jnt_data.position[13])))/ar_[0];
  *jnt_data.effort[14] = ((jr_[14] * (*act_data.effort[0])) + (elr_[14] * (*jnt_data.position[14])))/ar_[0];
  *jnt_data.effort[15] = ((jr_[15] * (*act_data.effort[0])) + (elr_[15] * (*jnt_data.position[15])))/ar_[0];
  *jnt_data.effort[16] = ((jr_[16] * (*act_data.effort[0])) + (elr_[16] * (*jnt_data.position[16])))/ar_[0];
  *jnt_data.effort[17] = ((jr_[17] * (*act_data.effort[0])) + (elr_[17] * (*jnt_data.position[17])))/ar_[0];
  *jnt_data.effort[18] = ((jr_[18] * (*act_data.effort[0])) + (elr_[18] * (*jnt_data.position[18])))/ar_[0];
  *jnt_data.effort[19] = ((jr_[19] * (*act_data.effort[0])) + (elr_[19] * (*jnt_data.position[19])))/ar_[0];
}

inline void AdaptiveSybergyTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                                       JointData& jnt_data)
{
  // assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  // assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *jnt_data.velocity[0] = (*act_data.velocity[0] / ar[0] + *act_data.velocity[1] / ar[1]) / (2.0 * jr[0]);
  // *jnt_data.velocity[1] = (*act_data.velocity[0] / ar[0] - *act_data.velocity[1] / ar[1]) / (2.0 * jr[1]);
}

inline void AdaptiveSybergyTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                                    JointData& jnt_data)
{
  // assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  // assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0] && jnt_data.position[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *jnt_data.position[0] = (*act_data.position[0] / ar[0] + *act_data.position[1] / ar[1]) / (2.0 * jr[0]) + jnt_offset_[0];
  // *jnt_data.position[1] = (*act_data.position[0] / ar[0] - *act_data.position[1] / ar[1]) / (2.0 * jr[1]) + jnt_offset_[1];
}

inline void AdaptiveSybergyTransmission::jointToActuatorEffort(const JointData& jnt_data,
                                                                  ActuatorData& act_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.position[0] && act_data.effort[0]  && jnt_data.effort[0]    && jnt_data.effort[1]    && jnt_data.effort[2]    
                              && jnt_data.effort[3]  && jnt_data.effort[4]    && jnt_data.effort[5]    && jnt_data.effort[6]    
                              && jnt_data.effort[7]  && jnt_data.effort[8]    && jnt_data.effort[9]    && jnt_data.effort[10]   
                              && jnt_data.effort[11] && jnt_data.effort[12]   && jnt_data.effort[13]   && jnt_data.effort[14]   
                              && jnt_data.effort[15] && jnt_data.effort[16]   && jnt_data.effort[17]   && jnt_data.effort[18]
                              && jnt_data.effort[19]);


  std::vector<double>& ar_  = act_reduction_;
  std::vector<double>& jr_  = jnt_reduction_;
  std::vector<double>& elr_ = els_reduction_;
  std::vector<double>& temp_(2);   // temporary vector 

  temp_[0]=0.0;   // vector of sum ri^2/ei

  for (int i = 0; i < jnt_data.effort.size(); i++)
  {
    temp_[0]= temp_[0]+((jr_[i]^2)/elr_[i]);
  }

  temp_[1]=0.0; // vector of summary ri*ti/ei+temp_[0]

  for (int i = 0; i < jnt_data.effort.size(); i++)
  {
    temp_[1]=temp_[1]+((jr_[i]+(*jnt_data.effort[i]))/(elr_[i]*temp_[0]));
  }

  *act_data.effort[0] = (*temp_[1] + (*act_data.position[0])/(*temp_[0]));  
}


inline void AdaptiveSybergyTransmission::jointToActuatorVelocity(const JointData& jnt_data,
                                                                    ActuatorData& act_data)
{
  // assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  // assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *act_data.velocity[0] = (*jnt_data.velocity[0] * jr[0] + *jnt_data.velocity[1] * jr[1]) * ar[0];
  // *act_data.velocity[1] = (*jnt_data.velocity[0] * jr[0] - *jnt_data.velocity[1] * jr[1]) * ar[1];
}

inline void DifferentialTransmission::jointToActuatorPosition(const JointData& jnt_data,
                                                                    ActuatorData& act_data)
{
  // assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  // assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0] && jnt_data.position[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // double jnt_pos_off[2] = {*jnt_data.position[0] - jnt_offset_[0], *jnt_data.position[1] - jnt_offset_[1]};

  // *act_data.position[0] = (jnt_pos_off[0] * jr[0] + jnt_pos_off[1] * jr[1]) * ar[0];
  // *act_data.position[1] = (jnt_pos_off[0] * jr[0] - jnt_pos_off[1] * jr[1]) * ar[1];
}

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_ADAPTIVESINERGY_TRANSMISSION_H