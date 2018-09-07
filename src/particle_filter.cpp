/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

/* Local defines */
/* Number of particles used in the particle filter */
#define PARTICLE_NUM (1000U)

#define EPSI (0.001)

#define X_INDEX (0U)
#define Y_INDEX (1U)
#define T_INDEX (2U)

/* Local Variables */
static default_random_engine gen;

/* local macros */

//#define GET_GAUSSIAN_SAMPLE(mean, std, id) ((normal_distribution<double> dist_##id(mean, std))(gen))

/* local functions */
double get_gaussian_sample(double mean, double std)
{
  normal_distribution<double> dist(mean, std);
  return dist(gen);
}

/* implementation code */

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  if(false == is_initialized)
  {
    num_particles = PARTICLE_NUM;

    for(int i=0; i < num_particles; i++)
    {
      Particle temp;

      temp.id     = i;
      temp.x      = get_gaussian_sample(x, std[X_INDEX]);
      temp.y      = get_gaussian_sample(y, std[Y_INDEX]);
      temp.theta  = get_gaussian_sample(theta, std[T_INDEX]);
      temp.weight = 1.0;

      particles.push_back(temp);
    }

    is_initialized = true;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
  for(int i=0; i < num_particles; i++)
  {
    double theta       = particles[i].theta;
    double delta_theta = yaw_rate * delta_t;
    double new_theta   = theta + delta_theta;

    /* check yaw rate to avoid division by zero */
    if(EPSI > fabs(yaw_rate)) /* yaw rate is zero, simple motion equations can be used */
    {
      particles[i].x     += velocity * cos(theta) * delta_t;
      particles[i].y     += velocity * sin(theta) * delta_t;
      /* No need to update theta since yaw rate is zero */
    }
    else /* yaw rate is not zero, complete model needs to be applied */
    {
      double factor = velocity / yaw_rate;

      particles[i].x     += (factor * ( sin(new_theta) - sin(theta) ));
      particles[i].y     += (factor * ( cos(theta) - cos(new_theta) ));
      particles[i].theta = new_theta;
    }

    particles[i].x     += get_gaussian_sample(0, std_pos[X_INDEX]);
    particles[i].y     += get_gaussian_sample(0, std_pos[Y_INDEX]);
    particles[i].theta += get_gaussian_sample(0, std_pos[T_INDEX]);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
  for(unsigned int i=0; i < observations.size(); i++)
  {
    /* http://www.cplusplus.com/reference/limits/numeric_limits/ */
    double min_dist = numeric_limits<double>::max();
    int idx = -1;

    for(unsigned int j=0; j < predicted.size(); j++)
    {
      double distance = dist(observations[i].x, observations[i].y, predicted[i].x, predicted[i].y);

      if(min_dist > distance)
      {
        min_dist = distance;
        idx = predicted[i].id;
      }
    }

    observations[i].id = idx;
  }
}

static double calc_wght_mult_var_gaussian(const LandmarkObs &o, const LandmarkObs &m, double std[])
{
  double x_std = std[X_INDEX];
  double y_std = std[Y_INDEX];

  double factor  = (2 * M_PI * x_std * y_std);
  double dx = o.x - m.x;
  double dy = o.y - m.y;
  double exp_val = ( (dx * dx) / (2 * x_std * x_std ) ) + ( (dy * dy) / (2 * y_std * y_std) );

  return exp( -exp_val ) / factor;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
  double Range_Max_Dist = sensor_range * sensor_range;

  /* 1) for each particle  */
  for(int i=0; i < num_particles; i++)
  {
    double theta = particles[i].theta;

    vector<LandmarkObs> TransObserv;
    /* 1) tranform the observation from vehicle coordinates to map coordinates */
    for (unsigned int i=0; i < observations.size(); i++)
    {
      LandmarkObs temp;

      temp.id = observations[i].id;
      temp.x  = particles[i].x + (cos(theta) * observations[i].x) - (sin(theta) * observations[i].y);
      temp.y  = particles[i].y + (sin(theta) * observations[i].x) + (cos(theta) * observations[i].y);

      TransObserv.push_back(temp);
    }
    
    /* 3) find map landmarks that the sensor can sense from this particle pos */
    /* essentially search for all landmarks in a circle with radius equal to sensor range */
    vector<LandmarkObs> landMarksInRange;
    for(unsigned int i=0; i < map_landmarks.landmark_list.size(); i++)
    {
      if( Range_Max_Dist >= dist(map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f, particles[i].x, particles[i].y) )
      {
        LandmarkObs temp;

        temp.id = map_landmarks.landmark_list[i].id_i;
        temp.x  = map_landmarks.landmark_list[i].x_f;
        temp.y  = map_landmarks.landmark_list[i].y_f;

        landMarksInRange.push_back(temp);
      }
    }
    
    /* 4) associate the observations and the landmarks around the particle */
    /*    using the nearest neighbour concept introduced in the lessons.   */
    dataAssociation(landMarksInRange, TransObserv);
    
    /* 5) update particle weight using using a mult-variate Gaussian distribution */
    particles[i].weight = 1.0;

    for(unsigned int j=0; j < TransObserv.size(); j++)
    {
      unsigned int k;

      for(k = 0; k < landMarksInRange.size(); k++)
      {
        if(landMarksInRange[k].id == TransObserv[j].id)
        {
          break; /* found the associated land mark */
        }
      }

      particles[i].weight *= calc_wght_mult_var_gaussian(TransObserv[j], landMarksInRange[k], std_landmark);
    }
  }
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
