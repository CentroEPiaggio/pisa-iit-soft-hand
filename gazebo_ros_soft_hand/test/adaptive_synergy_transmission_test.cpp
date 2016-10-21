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


#include <gtest/gtest.h>

#include <adaptive_transmission/adaptive_synergy_transmission.h>
#include "./random_generator_utils.h"

using namespace transmission_interface;
using std::vector;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  vector<double> act_reduction_good(1, 1.0);
  vector<double> act_reduction_bad1(1, 0.0);
  vector<double> act_reduction_bad2(2, 1.0);

  vector<double> joint_reduction_bad(19, 0.0);
  vector<double> joint_reduction_good(19, 1.0);
  vector<double> joint_elastic_bad(19, 0.0);
  vector<double> joint_elastic_good(19, 1.0);
  vector<double> joint_offset_good(19, 1.0);

  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_bad1, joint_reduction_good, joint_elastic_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_bad2, joint_reduction_good, joint_elastic_good), TransmissionInterfaceException);
  
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_bad, joint_elastic_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_good, joint_elastic_bad), TransmissionInterfaceException);
  
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_bad1, joint_reduction_good, joint_elastic_good, joint_offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_bad2, joint_reduction_good, joint_elastic_good, joint_offset_good), TransmissionInterfaceException);
  
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_bad, joint_elastic_good, joint_offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_good, joint_elastic_bad, joint_offset_good), TransmissionInterfaceException);

  // Valid instance creation
  EXPECT_NO_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_good, joint_elastic_good));
  EXPECT_NO_THROW(AdaptiveSynergyTransmission(act_reduction_good, joint_reduction_good, joint_elastic_good, joint_offset_good));
}
/*
#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(PreconditionsTest, AssertionTriggering)
{
  // Create input/output transmission data
  double a_val1 = 0.0, a_val2 = 0.0;
  double j_val1 = 0.0, j_val2 = 0.0;

  vector<double*> a_good_vec;
  a_good_vec.push_back(&a_val1);
  a_good_vec.push_back(&a_val2);

  vector<double*> j_good_vec;
  j_good_vec.push_back(&j_val1);
  j_good_vec.push_back(&j_val2);

  ActuatorData a_good_data;
  a_good_data.position = a_good_vec;
  a_good_data.velocity = a_good_vec;
  a_good_data.effort   = a_good_vec;

  JointData j_good_data;
  j_good_data.position = j_good_vec;
  j_good_data.velocity = j_good_vec;
  j_good_data.effort   = j_good_vec;

  ActuatorData a_bad_data;
  a_bad_data.position = vector<double*>(2);
  a_bad_data.velocity = vector<double*>(2);
  a_bad_data.effort   = vector<double*>(2);

  JointData j_bad_data;
  j_bad_data.position = vector<double*>(2);
  j_bad_data.velocity = vector<double*>(2);
  j_bad_data.effort   = vector<double*>(2);

  ActuatorData a_bad_size;
  JointData    j_bad_size;

  // Transmission instance
  AdaptiveSynergyTransmission trans(vector<double>(2, 1.0),
                                   vector<double>(2, 1.0));

  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_good_data), ".*");

  // Wrong parameter sizes should trigger an assertion
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_good_data), ".*");
}
#endif // NDEBUG
*/


TEST(PreconditionsTest, AccessorValidation)
{
  std::vector<double> act_reduction(1);
  act_reduction[0] =  10.0;

  std::vector<double> jnt_reduction(19);
  jnt_reduction[0] =  4.0;
  jnt_reduction[1] = -4.0;
  jnt_reduction[2] =  4.0;
  jnt_reduction[3] = -4.0;
  jnt_reduction[4] =  4.0;
  jnt_reduction[5] = -4.0;
  jnt_reduction[6] =  4.0;
  jnt_reduction[7] = -4.0;
  jnt_reduction[8] =  4.0;
  jnt_reduction[9] = -4.0;
  jnt_reduction[10] =  4.0;
  jnt_reduction[11] = -4.0;
  jnt_reduction[12] =  4.0;
  jnt_reduction[13] = -4.0;
  jnt_reduction[14] =  4.0;
  jnt_reduction[15] = -4.0;
  jnt_reduction[16] =  4.0;
  jnt_reduction[17] = -4.0;
  jnt_reduction[18] =  4.0;

  std::vector<double> jnt_elastic(19);
  jnt_elastic[0] =  5.0;
  jnt_elastic[1] = -5.0;
  jnt_elastic[2] =  5.0;
  jnt_elastic[3] = -5.0;
  jnt_elastic[4] =  5.0;
  jnt_elastic[5] = -5.0;
  jnt_elastic[6] =  5.0;
  jnt_elastic[7] = -5.0;
  jnt_elastic[8] =  5.0;
  jnt_elastic[9] = -5.0;
  jnt_elastic[10] =  5.0;
  jnt_elastic[11] = -5.0;
  jnt_elastic[12] =  5.0;
  jnt_elastic[13] = -5.0;
  jnt_elastic[14] =  5.0;
  jnt_elastic[15] = -5.0;
  jnt_elastic[16] =  5.0;
  jnt_elastic[17] = -5.0;
  jnt_elastic[18] =  5.0;

  std::vector<double> jnt_offset(19);
  jnt_offset[0] =  1.0;
  jnt_offset[1] = -1.0;
  jnt_offset[2] =  1.0;
  jnt_offset[3] = -1.0;
  jnt_offset[4] =  1.0;
  jnt_offset[5] = -1.0;
  jnt_offset[6] =  1.0;
  jnt_offset[7] = -1.0;
  jnt_offset[8] =  1.0;
  jnt_offset[9] = -1.0;
  jnt_offset[10] =  1.0;
  jnt_offset[11] = -1.0;
  jnt_offset[12] =  1.0;
  jnt_offset[13] = -1.0;
  jnt_offset[14] =  1.0;
  jnt_offset[15] = -1.0;
  jnt_offset[16] =  1.0;
  jnt_offset[17] = -1.0;
  jnt_offset[18] =  1.0;

  AdaptiveSynergyTransmission trans(act_reduction,
                                   jnt_reduction,
                                   jnt_elastic,
                                   jnt_offset);

  EXPECT_EQ(1, trans.numActuators());
  EXPECT_EQ(19, trans.numJoints());
  EXPECT_EQ( 10.0, trans.getActuatorReduction()[0]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[0]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[1]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[2]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[3]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[4]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[5]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[6]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[7]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[8]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[9]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[10]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[11]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[12]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[13]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[14]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[15]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[16]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[17]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[18]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[0]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[1]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[2]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[3]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[4]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[5]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[6]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[7]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[8]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[9]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[10]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[11]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[12]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[13]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[14]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[15]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[16]);
  EXPECT_EQ(-5.0, trans.getJointElastic()[17]);
  EXPECT_EQ( 5.0, trans.getJointElastic()[18]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[0]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[1]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[2]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[3]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[4]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[5]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[6]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[7]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[8]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[9]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[10]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[11]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[12]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[13]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[14]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[15]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[16]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[17]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[18]);
}

class TransmissionSetup : public ::testing::Test
{
public:
  TransmissionSetup()
    : a_val(),
      j_val(),
      a_vec(vector<double*>(1)),
      j_vec(vector<double*>(19))
   {
     a_vec[0] = &a_val[0];
     j_vec[0] = &j_val[0];
     j_vec[1] = &j_val[1];
     j_vec[2] = &j_val[2];
     j_vec[3] = &j_val[3];
     j_vec[4] = &j_val[4];
     j_vec[5] = &j_val[5];
     j_vec[6] = &j_val[6];
     j_vec[7] = &j_val[7];
     j_vec[8] = &j_val[8];
     j_vec[9] = &j_val[9];
     j_vec[10] = &j_val[10];
     j_vec[11] = &j_val[11];
     j_vec[12] = &j_val[12];
     j_vec[13] = &j_val[13];
     j_vec[14] = &j_val[14];
     j_vec[15] = &j_val[15];
     j_vec[16] = &j_val[16];
     j_vec[17] = &j_val[17];
     j_vec[18] = &j_val[18];
  }

protected:
  // Input/output transmission data
  double a_val[1];
  double j_val[19];
  vector<double*> a_vec;
  vector<double*> j_vec;
};

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup
{
protected:
  /// \param trans Transmission instance.
  /// \param ref_val Reference value (effort, velocity or position) that will be transformed with the respective forward
  /// and inverse transmission transformations.
  void testIdentityMap(AdaptiveSynergyTransmission& trans,
                       const vector<double>& ref_val)
  {
    // Effort interface
    {
      ActuatorData a_data;
      a_data.effort = a_vec;
      *a_data.effort[0] = ref_val[0];

      JointData j_data;
      double* zero_ptr = new double(0.0);
      j_data.position = vector<double*>( 19, zero_ptr ); // apply tests at home position for the joints
      j_data.effort = j_vec;

      trans.actuatorToJointEffort(a_data, j_data);

      a_data.position = a_vec;
      *a_data.position[0] = 0.0; // apply force at home position for actuator

      trans.jointToActuatorEffort(j_data, a_data);
      EXPECT_NEAR(ref_val[0], *a_data.effort[0], EPS);
    }
/*
    // Velocity interface
    {
      ActuatorData a_data;
      a_data.velocity = a_vec;
      *a_data.velocity[0] = ref_val[0];

      JointData j_data;
      j_data.velocity = j_vec;

      trans.actuatorToJointVelocity(a_data, j_data);
      trans.jointToActuatorVelocity(j_data, a_data);
      EXPECT_NEAR(ref_val[0], *a_data.velocity[0], EPS);
    }
*/
    // Position interface
    {
      ActuatorData a_data;
      a_data.position = a_vec;
      *a_data.position[0] = ref_val[0];

      JointData j_data;
      j_data.position = j_vec;
      double* zero_ptr = new double(0.0);
      j_data.effort = vector<double*>( 19, zero_ptr ); // apply tests with no effort

      trans.actuatorToJointPosition(a_data, j_data);
      trans.jointToActuatorPosition(j_data, a_data);

      EXPECT_NEAR(ref_val[0], *a_data.position[0], EPS);
    }
  }

  /// Generate a set of transmission instances with random combinations of actuator/joint reduction and joint offset.
  static vector<AdaptiveSynergyTransmission> createTestInstances(const vector<AdaptiveSynergyTransmission>::size_type size)
  {
    std::vector<AdaptiveSynergyTransmission> out;
    out.reserve(size);
    RandomDoubleGenerator rand_gen(-1.0, 1.0);                                // NOTE: Magic value

    while (out.size() < size)
    {
      try
      {
        AdaptiveSynergyTransmission trans(randomVector(1, rand_gen),
                                          randomVector(19, rand_gen),
                                          randomVector(19, rand_gen),
                                          randomVector(19, rand_gen));
        out.push_back(trans);
      }
      catch(const TransmissionInterfaceException&)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator, construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};

TEST_F(BlackBoxTest, IdentityMap)
{
  // Transmission instances
  typedef vector<AdaptiveSynergyTransmission> TransList;
  TransList trans_list = createTestInstances(100);                                  // NOTE: Magic value

  // Test different transmission configurations...
  for (TransList::iterator it = trans_list.begin(); it != trans_list.end(); ++it)
  {
    // ...and for each transmission, different input values
    RandomDoubleGenerator rand_gen(-1.0, 1.0);                                // NOTE: Magic value
    const unsigned int input_value_trials = 100;                                    // NOTE: Magic value
    for (unsigned int i = 0; i < input_value_trials; ++i)
    {
      vector<double> input_value = randomVector(1, rand_gen);
      testIdentityMap(*it, input_value);
    }
  }
}


class WhiteBoxTest : public TransmissionSetup {};

TEST_F(WhiteBoxTest, DontMoveJoints)
{
  vector<double> actuator_reduction(1, 10.0);
  vector<double> joint_reduction(19, 1.0);
  vector<double> joint_elastic(19, 1.0);
  vector<double> joint_offset(19, 0.0);

  AdaptiveSynergyTransmission trans(actuator_reduction, joint_reduction, joint_elastic, joint_offset);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 0.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;
    
    JointData j_data;
    j_data.position = j_vec;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[1], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[2], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[3], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[4], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[5], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[6], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[7], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[8], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[9], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[10], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[11], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[12], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[13], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[14], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[15], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[16], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[17], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[18], EPS);

  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[1], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[2], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[3], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[4], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[5], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[6], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[7], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[8], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[9], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[10], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[11], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[12], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[13], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[14], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[15], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[16], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[17], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[18], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;
    j_data.effort = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.position[0], EPS);
    EXPECT_NEAR(0.0, *j_data.position[1], EPS);
    EXPECT_NEAR(0.0, *j_data.position[2], EPS);
    EXPECT_NEAR(0.0, *j_data.position[3], EPS);
    EXPECT_NEAR(0.0, *j_data.position[4], EPS);
    EXPECT_NEAR(0.0, *j_data.position[5], EPS);
    EXPECT_NEAR(0.0, *j_data.position[6], EPS);
    EXPECT_NEAR(0.0, *j_data.position[7], EPS);
    EXPECT_NEAR(0.0, *j_data.position[8], EPS);
    EXPECT_NEAR(0.0, *j_data.position[9], EPS);
    EXPECT_NEAR(0.0, *j_data.position[10], EPS);
    EXPECT_NEAR(0.0, *j_data.position[11], EPS);
    EXPECT_NEAR(0.0, *j_data.position[12], EPS);
    EXPECT_NEAR(0.0, *j_data.position[13], EPS);
    EXPECT_NEAR(0.0, *j_data.position[14], EPS);
    EXPECT_NEAR(0.0, *j_data.position[15], EPS);
    EXPECT_NEAR(0.0, *j_data.position[16], EPS);
    EXPECT_NEAR(0.0, *j_data.position[17], EPS);
    EXPECT_NEAR(0.0, *j_data.position[18], EPS);
  }
}

TEST_F(WhiteBoxTest, MoveHand)
{
  vector<double> actuator_reduction(1, 10.0);
  vector<double> joint_reduction(19, 1.0);
  vector<double> joint_elastic(19, 1.0);
  vector<double> joint_offset(19, 0.0);

  AdaptiveSynergyTransmission trans(actuator_reduction, joint_reduction, joint_elastic, joint_offset);

  // Effort interface
  {
    *a_vec[0] =  10.0;

    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.position = j_vec;
    j_data.effort = j_vec;

    // If the actuator effort is 10x the reduction 10x, and the position remains the same, it means that,
    // with 1.0 as joint reduction for everyone, you will have 100 effort in the joint
    // this can be the situation for instance when in contact.
    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(100.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[1], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[2], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[3], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[4], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[5], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[6], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[7], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[8], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[9], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[10], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[11], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[12], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[13], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[14], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[15], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[16], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[17], EPS);
    EXPECT_NEAR(100.0, *j_data.effort[18], EPS);
  }

  // Velocity interface
  {
    *a_vec[0] = 10.0;

    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.position = j_vec;
    j_data.velocity = j_vec;

    // Attention, do not trust this test!!!
    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[1], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[2], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[3], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[4], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[5], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[6], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[7], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[8], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[9], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[10], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[11], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[12], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[13], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[14], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[15], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[16], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[17], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[18], EPS);

  }

  // Position interface
  {
    *a_vec[0] = 1.0;

    double* zero_ptr = new double(0.0);

    ActuatorData a_data;
    a_data.position = a_vec;
    a_data.effort = vector<double*>( 1, zero_ptr ); // a_vec;

    JointData j_data;
    j_data.position = j_vec;
    j_data.effort = vector<double*>( 19, zero_ptr ); // apply the actuation position with zero effort at joints (no contact)
    
    // if you want to play with how the synergy work, assign a value different from zero at a desired joint
    // as if it were making contact with the environment, and you will see how the other joints move
    // of course the test will fail
    double* value_ptr = new double(0.0);
    j_data.effort[5] = value_ptr;

    // if you move 1 in the actuator, you apply the reduction 10, and if all
    // joint reductions are 1, you will have the joint positions equal to 
    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[0], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[1], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[2], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[3], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[4], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[5], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[6], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[7], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[8], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[9], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[10], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[11], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[12], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[13], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[14], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[15], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[16], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[17], EPS);
    EXPECT_NEAR(0.005263157894736842, *j_data.position[18], EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
