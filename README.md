# pisa-iit-soft-hand (ROS packages) [DEPRECATED]

This repository contains the model of the Pisa/IIT hand as described in:

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014

[Free version of the paper](http://www.centropiaggio.unipi.it/sites/default/files/PisaIIT_SoftHand_0.pdf) and
[IJRR version (access required)](http://ijr.sagepub.com/content/33/5/768.abstract)

Unless stated otherwise, all files within the repository are released under the BSD 3-Clause License, see the [LICENSE](https://github.com/CentroEPiaggio/pisa-iit-soft-hand/blob/master/LICENSE) file for the details.

## Cloning the repository
```
git clone --recursive https://github.com/CentroEPiaggio/pisa-iit-soft-hand.git
```

## Dependencies

The hw interface makes use of the ros package `qb_interface` in [IMU](https://github.com/CentroEPiaggio/IMU)

ToDo: Create a travis.yml file for this.

## Examples
There are several [examples](https://github.com/CentroEPiaggio/pisa-iit-soft-hand/tree/master/examples) that show how the hand can be used in different configurations.
