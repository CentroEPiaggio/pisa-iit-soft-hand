# Adaptive Synergy Transmission Interface for The Pisa/IIT Soft Hand #

## Overview ##

This is not really a package. It contains the files and modifications you need to copy and apply to the transmission_interface package within ros_control. Next, it is described how to do it.

## How to include it to the rest of transmission types ##

1. `git clone https://github.com/ros-controls/ros_control.git` within your catkin workspace

2. `cd ros_control` && `git checkout indigo-devel` to ensure you are in the indigo version

3. Copy the following files within the ros_control/transmission_interface package using the corresponding folder:

  * `include/transmission_interface/adaptive_synergy_transmission.h`
  * `include/transmission_interface/adaptive_synergy_transmission_loader.h`
  * `src/adaptive_synergy_transmission_loader.cpp`
  * `test/adaptive_synergy_transmission_loader_test.cpp`
  * `test/urdf/adaptive_synergy_transmission_loader_full.urdf`
  * `test/urdf/adaptive_synergy_transmission_loader_invalid.urdf`
  * `test/urdf/adaptive_synergy_transmission_loader_minimal.urdf`

The remaining steps assumes that you work in the `transmission_interface` package.

4. Edit the transmission loader class with the `getJointElasticFunction()` function as a protected member. Add the declaration in `transmission_loader.h`:

```
static ParseStatus getJointElastic(const TiXmlElement& parent_el,
                                      const std::string&  joint_name,
                                      const std::string&  transmission_name,
                                      bool                required,
                                      double&             elastic);
```

And add the defintion in `transmission_loader.cpp`:

```
TransmissionLoader::ParseStatus
TransmissionLoader::getJointElastic(const TiXmlElement& parent_el,
                                      const std::string&  joint_name,
                                      const std::string&  transmission_name,
                                      bool                required,
                                      double&             elastic)
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
```

5. Edit the `ros_control_plugins.xml` by adding the new transmission type:

```
  <class name="transmission_interface/AdaptiveSynergyTransmission"
         type="transmission_interface::AdaptiveSynergyTransmissionLoader"
         base_class_type="transmission_interface::TransmissionLoader">
    <description>
      Load from a URDF description the configuration of an adaptive synergy transmission.
    </description>
  </class>
```

6. Add the new transmission to the plugins and tests to the `CMakeLists.txt`. In the section `add_library(${PROJECT_NAME}_loader_plugins`, add the following line:

```
  src/adaptive_synergy_transmission_loader.cpp include/transmission_interface/adaptive_synergy_transmission_loader.h
```

And add the tests within the `if(CATKIN_ENABLE_TESTING)` statement:

```
  catkin_add_gtest(adaptive_synergy_transmission_loader_test test/adaptive_synergy_transmission_loader_test.cpp)
  target_link_libraries(adaptive_synergy_transmission_loader_test ${PROJECT_NAME}_parser)
```

7. Compile and test: a) `catkin_make` and `catkin_make test`, or b) `catkin_make tests` and then `rosrun transmission_interface `

8. Use
