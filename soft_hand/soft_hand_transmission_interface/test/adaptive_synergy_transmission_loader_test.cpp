// adaptive_synergy_transmission_loader.h

///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian
/// \ modified by Marco Bartolomei 2014

#include <string>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <transmission_interface/adaptive_synergy_transmission.h>
#include <transmission_interface/transmission_loader.h>
#include "read_file.h"
#include "loader_utils.h"



TEST(AdaptiveSynergyTransmissionLoaderTest, FullSpec)
{

// Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/adaptive_synergy_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  boost::shared_ptr<TransmissionLoader> transmission_loader = loader.create(infos.front().type_);
  
  ASSERT_TRUE(0 != transmission_loader);
  
  TransmissionPtr transmission;
  const TransmissionInfo& info = infos.front();
  
  transmission = transmission_loader->load(info);
  
  ASSERT_TRUE(0 != transmission);

  // Validate transmission
  AdaptiveSynergyTransmission* adaptive_synergy_transmission = dynamic_cast<AdaptiveSynergyTransmission*>(transmission.get());
  ASSERT_TRUE(0 != adaptive_synergy_transmission);

  const std::vector<double>& actuator_reduction = adaptive_synergy_transmission->getActuatorReduction();
  EXPECT_EQ( 30.0, actuator_reduction[0]);

  const std::vector<double>& joint_reduction = adaptive_synergy_transmission->getJointReduction();
  EXPECT_EQ( 2.0, joint_reduction[0]);
  EXPECT_EQ( 2.0, joint_reduction[1]);
  EXPECT_EQ( 2.0, joint_reduction[2]);
  EXPECT_EQ( 2.0, joint_reduction[3]);
  EXPECT_EQ( 2.0, joint_reduction[4]);
  EXPECT_EQ( 2.0, joint_reduction[5]);
  EXPECT_EQ( 2.0, joint_reduction[6]);
  EXPECT_EQ( 2.0, joint_reduction[7]);
  EXPECT_EQ( 2.0, joint_reduction[8]);
  EXPECT_EQ( 2.0, joint_reduction[9]);
  EXPECT_EQ( 2.0, joint_reduction[10]);
  EXPECT_EQ( 2.0, joint_reduction[11]);
  EXPECT_EQ( 2.0, joint_reduction[12]);
  EXPECT_EQ( 2.0, joint_reduction[13]);
  EXPECT_EQ( 2.0, joint_reduction[14]);
  EXPECT_EQ( 2.0, joint_reduction[15]);
  EXPECT_EQ( 2.0, joint_reduction[16]);
  EXPECT_EQ( 2.0, joint_reduction[17]);
  EXPECT_EQ( 2.0, joint_reduction[18]);
  EXPECT_EQ( 2.0, joint_reduction[19]);

  const std::vector<double>& joint_elastic = adaptive_synergy_transmission->getJointElastic();
  EXPECT_EQ( 3.0, joint_elastic[0]);
  EXPECT_EQ( 3.0, joint_elastic[1]);
  EXPECT_EQ( 3.0, joint_elastic[2]);
  EXPECT_EQ( 3.0, joint_elastic[3]);
  EXPECT_EQ( 3.0, joint_elastic[4]);
  EXPECT_EQ( 3.0, joint_elastic[5]);
  EXPECT_EQ( 3.0, joint_elastic[6]);
  EXPECT_EQ( 3.0, joint_elastic[7]);
  EXPECT_EQ( 3.0, joint_elastic[8]);
  EXPECT_EQ( 3.0, joint_elastic[9]);
  EXPECT_EQ( 3.0, joint_elastic[10]);
  EXPECT_EQ( 3.0, joint_elastic[11]);
  EXPECT_EQ( 3.0, joint_elastic[12]);
  EXPECT_EQ( 3.0, joint_elastic[13]);
  EXPECT_EQ( 3.0, joint_elastic[14]);
  EXPECT_EQ( 3.0, joint_elastic[15]);
  EXPECT_EQ( 3.0, joint_elastic[16]);
  EXPECT_EQ( 3.0, joint_elastic[17]);
  EXPECT_EQ( 3.0, joint_elastic[18]);
  EXPECT_EQ( 3.0, joint_elastic[19]);


  const std::vector<double>& joint_offset = adaptive_synergy_transmission->getJointOffset();
  EXPECT_EQ( -0.5, joint_offset[0]);
  EXPECT_EQ( -0.5, joint_offset[1]);
  EXPECT_EQ( -0.5, joint_offset[2]);
  EXPECT_EQ( -0.5, joint_offset[3]);
  EXPECT_EQ( -0.5, joint_offset[4]);
  EXPECT_EQ( -0.5, joint_offset[5]);
  EXPECT_EQ( -0.5, joint_offset[6]);
  EXPECT_EQ( -0.5, joint_offset[7]);
  EXPECT_EQ( -0.5, joint_offset[8]);
  EXPECT_EQ( -0.5, joint_offset[9]);
  EXPECT_EQ( -0.5, joint_offset[10]);
  EXPECT_EQ( -0.5, joint_offset[11]);
  EXPECT_EQ( -0.5, joint_offset[12]);
  EXPECT_EQ( -0.5, joint_offset[13]);
  EXPECT_EQ( -0.5, joint_offset[14]);
  EXPECT_EQ( -0.5, joint_offset[15]);
  EXPECT_EQ( -0.5, joint_offset[16]);
  EXPECT_EQ( -0.5, joint_offset[17]);
  EXPECT_EQ( -0.5, joint_offset[18]);
  EXPECT_EQ( -0.5, joint_offset[19]);
}


TEST(AdaptiveSynergyTransmissionLoaderTest, MinimalSpec)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/adaptive_synergy_transmission_loader_minimal.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  boost::shared_ptr<TransmissionLoader> transmission_loader = loader.create(infos.front().type_);
  ASSERT_TRUE(0 != transmission_loader);

  TransmissionPtr transmission;
  const TransmissionInfo& info = infos.front();
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(0 != transmission);

  // Validate transmission
  AdaptiveSynergyTransmission* adaptive_synergy_transmission = dynamic_cast<AdaptiveSynergyTransmission*>(transmission.get());

  const std::vector<double>& actuator_reduction = adaptive_synergy_transmission->getActuatorReduction();
  EXPECT_EQ( 30.0, actuator_reduction[0]);
  
  const std::vector<double>& joint_reduction = adaptive_synergy_transmission->getJointReduction();
  EXPECT_EQ( 1.0, joint_reduction[0]);
  EXPECT_EQ( 1.0, joint_reduction[1]);
  EXPECT_EQ( 1.0, joint_reduction[2]);
  EXPECT_EQ( 1.0, joint_reduction[3]);
  EXPECT_EQ( 1.0, joint_reduction[4]);
  EXPECT_EQ( 1.0, joint_reduction[5]);
  EXPECT_EQ( 1.0, joint_reduction[6]);
  EXPECT_EQ( 1.0, joint_reduction[7]);
  EXPECT_EQ( 1.0, joint_reduction[8]);
  EXPECT_EQ( 1.0, joint_reduction[9]);
  EXPECT_EQ( 1.0, joint_reduction[10]);
  EXPECT_EQ( 1.0, joint_reduction[11]);
  EXPECT_EQ( 1.0, joint_reduction[12]);
  EXPECT_EQ( 1.0, joint_reduction[13]);
  EXPECT_EQ( 1.0, joint_reduction[14]);
  EXPECT_EQ( 1.0, joint_reduction[15]);
  EXPECT_EQ( 1.0, joint_reduction[16]);
  EXPECT_EQ( 1.0, joint_reduction[17]);
  EXPECT_EQ( 1.0, joint_reduction[18]);
  EXPECT_EQ( 1.0, joint_reduction[19]);


 const std::vector<double>& joint_elastic = adaptive_synergy_transmission->getJointElastic();
  EXPECT_EQ( 1.0, joint_elastic[0]);
  EXPECT_EQ( 1.0, joint_elastic[1]);
  EXPECT_EQ( 1.0, joint_elastic[2]);
  EXPECT_EQ( 1.0, joint_elastic[3]);
  EXPECT_EQ( 1.0, joint_elastic[4]);
  EXPECT_EQ( 1.0, joint_elastic[5]);
  EXPECT_EQ( 1.0, joint_elastic[6]);
  EXPECT_EQ( 1.0, joint_elastic[7]);
  EXPECT_EQ( 1.0, joint_elastic[8]);
  EXPECT_EQ( 1.0, joint_elastic[9]);
  EXPECT_EQ( 1.0, joint_elastic[10]);
  EXPECT_EQ( 1.0, joint_elastic[11]);
  EXPECT_EQ( 1.0, joint_elastic[12]);
  EXPECT_EQ( 1.0, joint_elastic[13]);
  EXPECT_EQ( 1.0, joint_elastic[14]);
  EXPECT_EQ( 1.0, joint_elastic[15]);
  EXPECT_EQ( 1.0, joint_elastic[16]);
  EXPECT_EQ( 1.0, joint_elastic[17]);
  EXPECT_EQ( 1.0, joint_elastic[18]);
  EXPECT_EQ( 1.0, joint_elastic[19]);

  const std::vector<double>& joint_offset = adaptive_synergy_transmission->getJointOffset();
  EXPECT_EQ( 0.0, joint_offset[0]);
  EXPECT_EQ( 0.0, joint_offset[1]);
  EXPECT_EQ( 0.0, joint_offset[2]);
  EXPECT_EQ( 0.0, joint_offset[3]);
  EXPECT_EQ( 0.0, joint_offset[4]);
  EXPECT_EQ( 0.0, joint_offset[5]);
  EXPECT_EQ( 0.0, joint_offset[6]);
  EXPECT_EQ( 0.0, joint_offset[7]);
  EXPECT_EQ( 0.0, joint_offset[8]);
  EXPECT_EQ( 0.0, joint_offset[9]);
  EXPECT_EQ( 0.0, joint_offset[10]);
  EXPECT_EQ( 0.0, joint_offset[11]);
  EXPECT_EQ( 0.0, joint_offset[12]);
  EXPECT_EQ( 0.0, joint_offset[13]);
  EXPECT_EQ( 0.0, joint_offset[14]);
  EXPECT_EQ( 0.0, joint_offset[15]);
  EXPECT_EQ( 0.0, joint_offset[16]);
  EXPECT_EQ( 0.0, joint_offset[17]);
  EXPECT_EQ( 0.0, joint_offset[18]);
  EXPECT_EQ( 0.0, joint_offset[19]);
}

TEST(AdaptiveSynergyTransmissionLoaderTest, InvalidSpec)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/adaptive_synergy_transmission_loader_invalid.urdf");
  ASSERT_EQ(12, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  boost::shared_ptr<TransmissionLoader> transmission_loader = loader.create(infos.front().type_);
  ASSERT_TRUE(0 != transmission_loader);

  BOOST_FOREACH(const TransmissionInfo& info, infos)
  {
    TransmissionPtr transmission;
    transmission = transmission_loader->load(info);
    ASSERT_TRUE(0 == transmission);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
