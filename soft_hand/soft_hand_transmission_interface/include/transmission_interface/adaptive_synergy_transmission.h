// adaptive_sinergy_transmission.h

///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
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
//  \modified by Marco Bartolomei Centro Piaggio 2014

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
class AdaptiveSynergyTransmission : public Transmission
{
public:
  /**
* \param actuator_reduction Reduction ratio of actuators.
* \param joint_reduction    Reduction ratio of joints, it is a vector 1x20.
* \param joint_elastic      Spring constants of joints, it is a diagonal matrix 20x20; we simplief by a vetor 20x1.
* \param joint_offset       Joint position offset used in the position mappings, it is a vector of 20x1.
* \pre Nonzero actuator and joint reduction values.
*/
  AdaptiveSynergyTransmission(const std::vector<double>& actuator_reduction,
                              const std::vector<double>& joint_reduction,
                              const std::vector<double>& joint_elastic,
                              const std::vector<double>& joint_offset = std::vector<double>(20, 0.0));

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
  std::size_t numJoints() const {return 20;}      // we have 20 joint distibuted over 5 fingers
  
  const std::vector<double>& getActuatorReduction() const {return act_reduction_;}      // is a vector 20x1
  const std::vector<double>& getJointReduction() const {return jnt_reduction_;}  // is a vector 1x20
  const std::vector<double>& getJointElastic() const {return jnt_elastic_;}  // is a vector 20x1
  const std::vector<double>& getJointOffset() const {return jnt_offset_;} // is a vector 20x1, but useless for now

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
    throw TransmissionInterfaceException("Array of a adaptive sinergy transmission must has size 1x20 and actuator has size 1.");
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
      0.0 == jnt_reduction_[19] ||
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
      0.0 == jnt_elastic_[18] ||
      0.0 == jnt_elastic_[19]
  ) 
 
  {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

inline void AdaptiveSynergyTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                                     JointData& jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.position.size());
  assert(act_data.effort[0] && jnt_data.position[0]  && jnt_data.position[1]  && jnt_data.position[2]  && jnt_data.position[3]
                            && jnt_data.position[4]  && jnt_data.position[5]  && jnt_data.position[6]  && jnt_data.position[7]
                            && jnt_data.position[8]  && jnt_data.position[9]  && jnt_data.position[10] && jnt_data.position[11]
                            && jnt_data.position[12] && jnt_data.position[13] && jnt_data.position[14] && jnt_data.position[15]
                            && jnt_data.position[16] && jnt_data.position[17] && jnt_data.position[18] && jnt_data.position[19]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& je = jnt_elastic_;
  
  // Tau_q =    R'  * tau_m -    E   *  q;
  //  20x1 = ( 20x1 *  1x1) - (20x20 * 20x1)
  // R == jr[] * ar[0];
  // E == je[];
  // Tau_q[] = jnt_data.effort[]
  // tau_m[] = act_data_effort[]

  *jnt_data.effort[0]  =  jr[0]*(ar[0]  * (*act_data.effort[0])) - je[0]  * (*jnt_data.position[0]);
  *jnt_data.effort[1]  =  jr[1]*(ar[0]  * (*act_data.effort[0])) - je[1]  * (*jnt_data.position[1]);
  *jnt_data.effort[2]  =  jr[2]*(ar[0]  * (*act_data.effort[0])) - je[2]  * (*jnt_data.position[2]);
  *jnt_data.effort[3]  =  jr[3]*(ar[0]  * (*act_data.effort[0])) - je[3]  * (*jnt_data.position[3]);
  *jnt_data.effort[4]  =  jr[4]*(ar[0]  * (*act_data.effort[0])) - je[4]  * (*jnt_data.position[4]);
  *jnt_data.effort[5]  =  jr[5]*(ar[0]  * (*act_data.effort[0])) - je[5]  * (*jnt_data.position[5]);
  *jnt_data.effort[6]  =  jr[6]*(ar[0]  * (*act_data.effort[0])) - je[6]  * (*jnt_data.position[6]);
  *jnt_data.effort[7]  =  jr[7]*(ar[0]  * (*act_data.effort[0])) - je[7]  * (*jnt_data.position[7]);
  *jnt_data.effort[8]  =  jr[8]*(ar[0]  * (*act_data.effort[0])) - je[8]  * (*jnt_data.position[8]);
  *jnt_data.effort[9]  =  jr[9]*(ar[0]  * (*act_data.effort[0])) - je[9]  * (*jnt_data.position[9]);
  *jnt_data.effort[10] =  jr[10]*(ar[0] * (*act_data.effort[0])) - je[10] * (*jnt_data.position[10]);
  *jnt_data.effort[11] =  jr[11]*(ar[0] * (*act_data.effort[0])) - je[11] * (*jnt_data.position[11]);
  *jnt_data.effort[12] =  jr[12]*(ar[0] * (*act_data.effort[0])) - je[12] * (*jnt_data.position[12]);
  *jnt_data.effort[13] =  jr[13]*(ar[0] * (*act_data.effort[0])) - je[13] * (*jnt_data.position[13]);
  *jnt_data.effort[14] =  jr[14]*(ar[0] * (*act_data.effort[0])) - je[14] * (*jnt_data.position[14]);
  *jnt_data.effort[15] =  jr[15]*(ar[0] * (*act_data.effort[0])) - je[15] * (*jnt_data.position[15]);
  *jnt_data.effort[16] =  jr[16]*(ar[0] * (*act_data.effort[0])) - je[16] * (*jnt_data.position[16]);
  *jnt_data.effort[17] =  jr[17]*(ar[0] * (*act_data.effort[0])) - je[17] * (*jnt_data.position[17]);
  *jnt_data.effort[18] =  jr[18]*(ar[0] * (*act_data.effort[0])) - je[18] * (*jnt_data.position[18]);
  *jnt_data.effort[19] =  jr[19]*(ar[0] * (*act_data.effort[0])) - je[19] * (*jnt_data.position[19]);

}

inline void AdaptiveSynergyTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                                       JointData& jnt_data)
{
  // assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  // assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *jnt_data.velocity[0] = (*act_data.velocity[0] / ar[0] + *act_data.velocity[1] / ar[1]) / (2.0 * jr[0]);
  // *jnt_data.velocity[1] = (*act_data.velocity[0] / ar[0] - *act_data.velocity[1] / ar[1]) / (2.0 * jr[1]);
}

inline void AdaptiveSynergyTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                                    JointData&    jnt_data)
{
  // assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  // assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0] && jnt_data.position[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *jnt_data.position[0] = (*act_data.position[0] / ar[0] + *act_data.position[1] / ar[1]) / (2.0 * jr[0]) + jnt_offset_[0];
  // *jnt_data.position[1] = (*act_data.position[0] / ar[0] - *act_data.position[1] / ar[1]) / (2.0 * jr[1]) + jnt_offset_[1];
}

inline void AdaptiveSynergyTransmission::jointToActuatorEffort(const JointData& jnt_data,
                                                                     ActuatorData& act_data)
{
  /*
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.position[0] && act_data.effort[0]  && jnt_data.effort[0]    && jnt_data.effort[1]    && jnt_data.effort[2]    
                              && jnt_data.effort[3]  && jnt_data.effort[4]    && jnt_data.effort[5]    && jnt_data.effort[6]    
                              && jnt_data.effort[7]  && jnt_data.effort[8]    && jnt_data.effort[9]    && jnt_data.effort[10]   
                              && jnt_data.effort[11] && jnt_data.effort[12]   && jnt_data.effort[13]   && jnt_data.effort[14]   
                              && jnt_data.effort[15] && jnt_data.effort[16]   && jnt_data.effort[17]   && jnt_data.effort[18]
                              && jnt_data.effort[19]);


  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;
  std::vector<double>& elr  = jnt_elastic_;
  std::vector<double> temp_ = std::vector<double>(2, 0.0);   // temporary vector 

   temp_[0]= 0.0; // vector of sum ri^2/ei

  for (int indice = 0; indice < 20; indice++)
  {
    temp_[0]= temp_[0]+(((*jr[indice])^2)/(*elr[indice]));
  }

   temp_[1]= 0.0; // vector of summary ri*ti/ei+temp_[0]

  for (int indice = 0; indice < jnt_data.effort.size(); indice++)
  {
    temp_[1]=temp_[1]+((jr[indice]+(*jnt_data.effort[indice]))/(elr[indice]*temp_[0]));
  }

  *act_data.effort[0] = (temp_[1] + (*act_data.position[0])/(temp_[0]));  
  */
}

inline void AdaptiveSynergyTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                                    ActuatorData& act_data)
{
  /*
  // assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  // assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  // std::vector<double>& ar = act_reduction_;
  // std::vector<double>& jr = jnt_reduction_;

  // *act_data.velocity[0] = (*jnt_data.velocity[0] * jr[0] + *jnt_data.velocity[1] * jr[1]) * ar[0];
  // *act_data.velocity[1] = (*jnt_data.velocity[0] * jr[0] - *jnt_data.velocity[1] * jr[1]) * ar[1];
*/
}

inline void AdaptiveSynergyTransmission::jointToActuatorPosition(const JointData&    jnt_data,
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

#endif // TRANSMISSION_INTERFACE_ADAPTIVE_SYNERGY_TRANSMISSION_H
