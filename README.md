# CarND_Project08ParticleFilter

Solution for Udacity Self driving car Nano degree eighth project: Particle Filter Project.

---

Abbreviations:

PF:  Particle Filter.

EKF: Extended Kalman Filter.

UKF: Unscented Kalman Filter.

[//]: # (Image Referensers)

---

## Objective

Implement a Particle filter localizing a car position by associating observed landmarks with landmarks on a predefined map.

---

### Reflection

This project aims to use the observed data obtained from previous projects ,as Object detectiona and result of diffferent tracking filters as EKF and UKF, to localize the car within the map, i.e. know the car position with high accuracy.

Udacity provided a simulator that allows visualize of the different landmarks and the output PF data points.

---

### Theory

Particle filters, as the name imply use particles to localize the car position, each particle consist of:
    - X Coordinate.
    - Y Coordinate.
    - Heading.
    - Weight

1) Initially We use an initial estimate to assign the particles position, ex:
We use GPS data as a mean point and assign the particle positions and headings randomly but within the variance of the GPS measurments.

2) For each particle the observed landmarks around us are assosiated with the predefined map landmarks using the nearest neigher algorithm.

3) The new particle weight is calculated based on a multivariable gaussian distribution, i.e. the x and y coordinates of both the observed landmark and it's associated map landmark.

4) The particles are resampled to remove the least fitting particles in localizing the particles.

5) The process is repeated with new observation.

---

### Implementation

Udacity have provided most of the control code as:
  * Communication between PF and Udacity simulator.
  * Parsing of the map points and feeding the values to the PF.
  * Measuring the error for each particle.
  
I had to implement the following parts:
  * PF: Complete implementation of the PF project consisting of:
    - PF initialization
    - PF particles position update based on bicycle motion model to accomodate for car motion.
    - PF particles weight update based on the difference between the observed landmarks and the associated map landmarks using nearest neighbour.
    - PF resample to eliminate the particles not performing well in initializing the car position.

---

### Results

Unlike previous projects succesful project submission is automatically done in the Udacity simulator.

However once the filter was succesful I tried searching different particle count numbers to find the optimum particle count results found were:

| Index | Part Cnt | X Error | Y Error | Yaw Error | Time  |
|-------|----------|---------|---------|-----------|-------|
|   1   |     1    |  0.578  |  1.526  |  0.035    | 02.46 |
|   2   |     2    |  0.394  |  2.268  |  0.063    | 02.66 |
|   3   |     3    |  0.273  |  0.264  |  0.01     | 49.34 |
|   4   |     5    |  0.205  |  0.195  |  0.007    | 51.06 |
|   5   |    10    |  0.159  |  0.151  |  0.005    | 52.36 |
|   6   |    15    |  0.139  |  0.133  |  0.004    | 52.74 |
|   7   |    20    |  0.136  |  0.125  |  0.004    | 52.58 |
|   8   |    25    |  0.13   |  0.122  |  0.004    | 49.56 |
|   9   |    50    |  0.122  |  0.112  |  0.004    | 54.94 |
|  10   |    75    |  0.118  |  0.111  |  0.004    | 54.98 |
|  11   |   100    |  0.117  |  0.108  |  0.004    | 55.68 |
|  12   |  1000    |  0.106  |  0.1    |  0.004    | NA    |

The law of diminishing returns is very apparent in the particle filter, perhaps because of using the nearest neighbour we need to match each particle with each map landmark. I decided to go with a 10 particle count because it showed good accuracy within acceptable time.

Please find the output video for checking.

---

### Conclusion

I think I have reached quite a good understanding of the theory and application of the PF and how it works to achieve the goal of a high confidence in localizing objects.
