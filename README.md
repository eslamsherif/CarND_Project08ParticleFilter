# CarND_Project08ParticleFilter

Solution for Udacity Self driving car Nano degree eighth project: Particle Filter Project.

---

Abbreviations:

PF:  Particle Filter.

EKF: Extended Kalman Filter.

UKF: Unscented Kalman Filter.

[//]: # (Image Referensers)

[LRImg]: ./images/Output/L_R_DS1.png
[LR2Img]: ./images/Output/L_R_DS2.png
[LImg]: ./images/Output/L_DS1.png
[L2Img]: ./images/Output/L_DS2.png
[RImg]: ./images/Output/R_DS1.png
[R2Img]: ./images/Output/R_DS2.png

[NISImg]: ./Tuning/03_L0.3_P0.4/Figure_1.png

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

