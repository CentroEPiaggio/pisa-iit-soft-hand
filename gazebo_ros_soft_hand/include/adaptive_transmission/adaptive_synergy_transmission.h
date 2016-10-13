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

// \author Carlos Rosales, based on templates of other transmissions

#ifndef TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_H

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{

/**
* \brief Implementation of an adaptive synergy transmission.
* ToDo: Write documentation
*
*
* \ingroup transmission_types
*/
class AdaptiveSynergyTransmission : public Transmission
{
public:
  /**
* \param actuator_reduction Reduction ratio of actuators.
* \param joint_reduction    Reduction ratio of joints, it is a vector 19x1.
* \param joint_elastic      Spring constants of joints, it is a diagonal matrix 19x19; we simplified by a vetor 19x1.
* \param joint_offset       Joint position offset used in the position mappings, it is a vector of 19x1.
* \pre Nonzero actuator and joint reduction values.
*/
  AdaptiveSynergyTransmission(const std::vector<double>& actuator_reduction,
                              const std::vector<double>& joint_reduction,
                              const std::vector<double>& joint_elastic,
                              const std::vector<double>& joint_offset = std::vector<double>(19, 0.0));

  /**
* \brief Transform \e effort variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint effort vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data);

  /**
* \brief Transform \e velocity variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/

 void actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data);

  /**
* \brief Transform \e position variables from actuator to joint space.
* \param[in] act_data Actuator-space variables.
* \param[out] jnt_data Joint-space variables.
* \pre Actuator and joint position vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data);

  /**
* \brief Transform \e effort variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint effort vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data);

  /**
* \brief Transform \e velocity variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  void jointToActuatorVelocity(const JointData& jnt_data, ActuatorData& act_data);

  /**
* \brief Transform \e position variables from joint to actuator space.
* \param[in] jnt_data Joint-space variables.
* \param[out] act_data Actuator-space variables.
* \pre Actuator and joint position vectors must have size 2 and point to valid data.
* To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
*/
  
  void jointToActuatorPosition(const JointData& jnt_data, ActuatorData& act_data);

  std::size_t numActuators() const {return 1;}      // we have 1 actuator
  std::size_t numJoints() const {return 19;}      // we have 19 joint distibuted over 5 fingers
  
  const std::vector<double>& getActuatorReduction() const {return act_reduction_;}      // is a vector 19x1
  const std::vector<double>& getJointReduction() const {return jnt_reduction_;}  // is a vector 1x19
  const std::vector<double>& getJointElastic() const {return jnt_elastic_;}  // is a vector 19x1
  const std::vector<double>& getJointOffset() const {return jnt_offset_;} // is a vector 19x1, but useless for now

protected:
  std::vector<double> act_reduction_;
  std::vector<double> jnt_reduction_;
  std::vector<double> jnt_elastic_;
  std::vector<double> jnt_offset_;
  
};

inline AdaptiveSynergyTransmission::AdaptiveSynergyTransmission(const std::vector<double>& actuator_reduction,
                                                                const std::vector<double>& joint_reduction,
                                                                const std::vector<double>& joint_elastic,
                                                                const std::vector<double>& joint_offset)
  : Transmission(),
    act_reduction_(actuator_reduction),
    jnt_reduction_(joint_reduction),
    jnt_elastic_(joint_elastic),
    jnt_offset_(joint_offset)
{
  if (numActuators() != act_reduction_.size() ||
      numJoints() != jnt_reduction_.size() ||
      numJoints() != jnt_elastic_.size() ||
      numJoints() != jnt_offset_.size())
  {
    throw TransmissionInterfaceException("Array of a adaptive sinergy transmission must has size 19x1 and actuator has size 1.");
  }

  if (0.0 == act_reduction_[0] ||
      0.0 == jnt_reduction_[0] ||
      0.0 == jnt_reduction_[1] ||
      0.0 == jnt_reduction_[2] ||
      0.0 == jnt_reduction_[3] ||
      0.0 == jnt_reduction_[4] ||
      0.0 == jnt_reduction_[5] ||
      0.0 == jnt_reduction_[6] ||
      0.0 == jnt_reduction_[7] ||
      0.0 == jnt_reduction_[8] ||
      0.0 == jnt_reduction_[9] ||
      0.0 == jnt_reduction_[10] ||
      0.0 == jnt_reduction_[11] ||
      0.0 == jnt_reduction_[12] ||
      0.0 == jnt_reduction_[13] ||
      0.0 == jnt_reduction_[14] ||
      0.0 == jnt_reduction_[15] ||
      0.0 == jnt_reduction_[16] ||
      0.0 == jnt_reduction_[17] ||
      0.0 == jnt_reduction_[18] ||
      0.0 == jnt_elastic_[0] ||
      0.0 == jnt_elastic_[1] ||
      0.0 == jnt_elastic_[2] ||
      0.0 == jnt_elastic_[3] ||
      0.0 == jnt_elastic_[4] ||
      0.0 == jnt_elastic_[5] ||
      0.0 == jnt_elastic_[6] ||
      0.0 == jnt_elastic_[7] ||
      0.0 == jnt_elastic_[8] ||
      0.0 == jnt_elastic_[9] ||
      0.0 == jnt_elastic_[10] ||
      0.0 == jnt_elastic_[11] ||
      0.0 == jnt_elastic_[12] ||
      0.0 == jnt_elastic_[13] ||
      0.0 == jnt_elastic_[14] ||
      0.0 == jnt_elastic_[15] ||
      0.0 == jnt_elastic_[16] ||
      0.0 == jnt_elastic_[17] ||
      0.0 == jnt_elastic_[18]
  ) 
 
  {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

inline void AdaptiveSynergyTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                                     JointData& jnt_data)
{
  assert(numActuators() == act_data.effort.size());
  assert(numJoints() == jnt_data.position.size());
  assert(numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.position[0]  && jnt_data.position[1]  && jnt_data.position[2]  && jnt_data.position[3]
                            && jnt_data.position[4]  && jnt_data.position[5]  && jnt_data.position[6]  && jnt_data.position[7]
                            && jnt_data.position[8]  && jnt_data.position[9]  && jnt_data.position[10] && jnt_data.position[11]
                            && jnt_data.position[12] && jnt_data.position[13] && jnt_data.position[14] && jnt_data.position[15]
                            && jnt_data.position[16] && jnt_data.position[17] && jnt_data.position[18]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;
  
  // tau_q =    R'  * tau_m -    E   *  q;
  // 19x1 = ( 19x1 *  1x1) - (19x19 * 19x1)
/*
  for(int i = 0; i < je.size() ; ++i)
  {
    *jnt_data.effort[i]  =  jr[i]*(ar[0] * (*act_data.effort[0])) - je[i]  * (*jnt_data.position[i]);
  }
*/
  for(int i = 0; i < je.size() ; ++i)
  {
    *jnt_data.effort[i]  =  jr[i]*(ar[0] * (*act_data.effort[0])) - je[i]  * (*jnt_data.position[i]);
  }
}

inline void AdaptiveSynergyTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                                       JointData& jnt_data)
{
  assert(numActuators() == act_data.velocity.size());
  assert(numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;

  // ToDo: this mapping is to be developed, perhaps it is not necessary if the hand is always controlled in position/effort
  *jnt_data.velocity[0] =  0.0;
  *jnt_data.velocity[1] =  0.0;
  *jnt_data.velocity[2] =  0.0;
  *jnt_data.velocity[3] =  0.0;
  *jnt_data.velocity[4] =  0.0;
  *jnt_data.velocity[5] =  0.0;
  *jnt_data.velocity[6] =  0.0;
  *jnt_data.velocity[7] =  0.0;
  *jnt_data.velocity[8] =  0.0;
  *jnt_data.velocity[9] =  0.0;
  *jnt_data.velocity[10] = 0.0;
  *jnt_data.velocity[11] = 0.0;
  *jnt_data.velocity[12] = 0.0;
  *jnt_data.velocity[13] = 0.0;
  *jnt_data.velocity[14] = 0.0;
  *jnt_data.velocity[15] = 0.0;
  *jnt_data.velocity[16] = 0.0;
  *jnt_data.velocity[17] = 0.0;
  *jnt_data.velocity[18] = 0.0;
}

inline void AdaptiveSynergyTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                                    JointData&    jnt_data)
{
  assert(numActuators() == act_data.position.size());
  assert(numJoints() == jnt_data.effort.size());
  assert(numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && jnt_data.effort[0]  && jnt_data.effort[1]  && jnt_data.effort[2]  && jnt_data.effort[3]
                              && jnt_data.effort[4]  && jnt_data.effort[5]  && jnt_data.effort[6]  && jnt_data.effort[7]
                              && jnt_data.effort[8]  && jnt_data.effort[9]  && jnt_data.effort[10] && jnt_data.effort[11]
                              && jnt_data.effort[12] && jnt_data.effort[13] && jnt_data.effort[14] && jnt_data.effort[15]
                              && jnt_data.effort[16] && jnt_data.effort[17] && jnt_data.effort[18] );

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;
  
  // E^-1 * R^t
  std::vector<double> EinvRtr( je.size() );

  // (R * E^-1 * R^t)^-1
  double REinvRtr = 0.0;
  double REinvRtr_inv = 0.0;

  // R * E^-1 * tau_q
  double REinvTau_q = 0.0;
  
  // -E^-1 * tau_q
  std::vector<double> EinvTau_q( je.size() );
  
  // build auxiliary terms  
  for( int i = 0 ; i < je.size() ; ++i )
  {
    EinvTau_q[i] = (*jnt_data.effort[i]) / je[i];
    REinvTau_q += ( jr[i]/je[i] ) * (*jnt_data.effort[i]);
    EinvRtr[i] = jr[i]/je[i];
    REinvRtr += (jr[i]*jr[i])/je[i];
  }

  REinvRtr_inv = 1/REinvRtr;

  // first and second terms
  double first_term;
  double second_term;

  for( int i = 0; i < je.size() ; ++i)
  {
    first_term = -1*EinvTau_q[i] + EinvRtr[i]*REinvRtr_inv*REinvTau_q;
    second_term = EinvRtr[i]*REinvRtr_inv* (*act_data.position[0] / ar[0]);

    *jnt_data.position[i] = first_term + second_term + (jnt_offset_[i]);
  }
}

inline void AdaptiveSynergyTransmission::jointToActuatorEffort(const JointData& jnt_data,
                                                                     ActuatorData& act_data)
{
  assert(numActuators() == act_data.position.size());
  assert(numActuators() == act_data.effort.size());
  assert(numJoints() == jnt_data.effort.size());
  assert(act_data.position[0] && jnt_data.effort[0]  && jnt_data.effort[1]  && jnt_data.effort[2]  && jnt_data.effort[3]
                              && jnt_data.effort[4]  && jnt_data.effort[5]  && jnt_data.effort[6]  && jnt_data.effort[7]
                              && jnt_data.effort[8]  && jnt_data.effort[9]  && jnt_data.effort[10] && jnt_data.effort[11]
                              && jnt_data.effort[12] && jnt_data.effort[13] && jnt_data.effort[14] && jnt_data.effort[15]
                              && jnt_data.effort[16] && jnt_data.effort[17] && jnt_data.effort[18]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;

  // (R * E^-1 * R^t)^-1
  double REinvRtr = 0.0;
  double REinvRtr_inv = 0.0;
  // R * E^-1 * tau_q
  double REinvTau_q = 0.0;
  
  for( int i = 0 ; i < je.size() ; ++i )
  {
    REinvTau_q += ( jr[i]/je[i] ) * (*jnt_data.effort[i]);
    REinvRtr += (jr[i]*jr[i])/je[i];
  }

  REinvRtr_inv = 1/REinvRtr;

  // tau_a = (...)*tau_q + (...)*pos_a
  *act_data.effort[0] = REinvRtr_inv * ( REinvTau_q + (*act_data.position[0])*ar[0]);
}

inline void AdaptiveSynergyTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                                    ActuatorData& act_data)
{
  // The actuator does not move if the fingers are moved
  *act_data.velocity[0] = 0.0;
}

inline void AdaptiveSynergyTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                                    ActuatorData& act_data)
{
  assert(numActuators() == act_data.position.size());
  assert(numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && jnt_data.position[0]  && jnt_data.position[1]  && jnt_data.position[2]  && jnt_data.position[3]
                              && jnt_data.position[4]  && jnt_data.position[5]  && jnt_data.position[6]  && jnt_data.position[7]
                              && jnt_data.position[8]  && jnt_data.position[9]  && jnt_data.position[10] && jnt_data.position[11]
                              && jnt_data.position[12] && jnt_data.position[13] && jnt_data.position[14] && jnt_data.position[15]
                              && jnt_data.position[16] && jnt_data.position[17] && jnt_data.position[18]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;
  
  *act_data.position[0] = 0.0;

  // R * q = s, for suitable matrices, s = actuator position
  // in summary, this constraint is similar to make a constant length wire

  for( int i = 0 ; i < je.size() ; ++i )
  {
    *act_data.position[0] += ( jr[i]*(*jnt_data.position[i] - jnt_offset_[i]) ) * ar[0];
  }
}

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_H
