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
#include <unordered_map>

#include "particle_filter.h"

/**
 * Initialize particle filter with a set number of particles with positions
 * that are randomly sampled from a Gaussian distribution with mean at the
 * GPS location with a specified uncertainty.  All particle weights are also
 * initialized to 1.0 to be updated later.
 */
void ParticleFilter::Init(double x, double y, double theta, double std[]) {

  // Set a fixed number of particles
  num_particles = 100;

  // Set up random sampling generators for x, y, and theta around mean of GPS
  // position and provided standard deviations
  std::random_device rand_dev;
  std::default_random_engine random_gen(rand_dev());
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  // Initialize all particles
  particles.clear();
  for (int i = 0; i < num_particles; ++i) {
    Particle particle;
    particle.x = dist_x(random_gen);
    particle.y = dist_y(random_gen);
    particle.theta = dist_theta(random_gen);
    // Normalize theta to [0, 2*pi] after random sampling
    while (particle.theta > 2*M_PI) { particle.theta -= 2*M_PI; }
    while (particle.theta < 0) { particle.theta += 2*M_PI; }
    particle.weight = 1.0;
    particles.push_back(particle);
    /*
     std::cout << "Init " << i << ": "<< particle.x << ", "
               << particle.y << ", "<< particle.theta << std::endl;
     */
  }
  
  // Set initialized flag to complete
  is_initialized = true;
}

/**
 * Predict where each particle would have moved to after time delta_t to the
 * current time step using CTRV motion equations and the controlled velocity
 * and yaw_rate.  Randomly sample from a Gaussian distribution around this
 * predicted mean position with the given standard deviations for x, y, and
 * theta.
 */
void ParticleFilter::Prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  
  // Parameter to set a near-zero yaw rate threshold for divide protection
  constexpr double kYawZeroThresh = 0.001;
  
  // Calculate predicted position for each particle
  for (auto &particle : particles) {
    double x_p, y_p, theta_p;
    
    // Switch motion equation to avoid division by zero from yaw_rate
    if (fabs(yaw_rate) > kYawZeroThresh) {
      // Driving in a curve
      x_p = particle.x + velocity / yaw_rate *
            (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
      
      y_p = particle.y + velocity / yaw_rate *
            (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
    }
    else {
      // Driving straight
      x_p = particle.x + velocity * delta_t * cos(yaw_rate);
      y_p = particle.y + velocity * delta_t * sin(yaw_rate);
    }
    
    theta_p = particle.theta + yaw_rate * delta_t;
    
    // Set up random sampling generators for x, y, and theta around mean of
    // predicted position and provided standard deviations
    std::random_device rand_dev;
    std::default_random_engine random_gen(rand_dev());
    std::normal_distribution<double> dist_x(x_p, std_pos[0]);
    std::normal_distribution<double> dist_y(y_p, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_p, std_pos[2]);
    
    particle.x = dist_x(random_gen);
    particle.y = dist_y(random_gen);
    double new_theta = dist_theta(random_gen);
    // Normalize theta to [0, 2*pi] after random sampling
    while (new_theta > 2*M_PI) { new_theta -= 2*M_PI; }
    while (new_theta < 0) { new_theta += 2*M_PI; }
    particle.theta = new_theta;
  }
}

/**
 * Update the weight of each particle based on the multivariate Gaussian
 * probability of the particle's predicted landmark measurement positions vs
 * the actual measured observations.  This is done with the following steps:
 *   1. Pre-filter the known map landmarks to find which ones are in the
 *      particle's sensor range with margin
 *   2. Convert vehicle's measured observations to map coordinates and associate
 *      each measured observation to the nearest pre-filtered landmark in the
 *      particle's range (using nearest neighbor matching)
 *   3. Calculate the multivariate Gaussian probability between each observed
 *      measurement and the matched landmark x,y position and set the particle's
 *      weight by multiplying them all together
 *   4. Set the particle's association information for visualization
 *   5. Check if all weights are zero to reinitialize if particles are way off
 *
 * The following references were used:
 * https://en.wikipedia.org/wiki/Multivariate_normal_distribution
 * https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
 * http://planning.cs.uiuc.edu/node99.html (equation 3.33)
 */
void ParticleFilter::UpdateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  
  // Parameter for a margin multiplier on the particle's sensor range to
  // pre-filter landmarks within range since particle may be shifted from
  // vehicle's actual position
  constexpr double kSensorRangeMargin = 1.2;
  
  // Reset list of all particle weights to be refilled in this function
  weights.clear();
  weights.resize(num_particles);
  
  // Loop through each particle to update its weight
  for (int i = 0; i < num_particles; ++i) {
    // Make a mapping of particle observations with:
    //   key = landmark ID integer
    //   value = LandmarkObs structure
    std::unordered_map<int, LandmarkObs> particle_observations;
    
    // Copy observation vector to add associated landmark IDs
    std::vector<LandmarkObs> vehicle_observations(observations);
    
    // Vectors to store the particles association info for visualization
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    
    /**
     * Step 1:
     * Find which map landmarks are in the particle's sensor range with margin.
     */
    
    for (const auto &landmark : map_landmarks.landmark_list) {
      // Recast landmark x,y position from float -> double
      const double landmark_x = static_cast<double>(landmark.x_f);
      const double landmark_y = static_cast<double>(landmark.y_f);
      
      // Calculate distance from particle to the landmark
      const double dist_to_landmark = dist(particles[i].x, particles[i].y,
                                           landmark_x, landmark_y);
      
      // If within sensor range, add landmark to particle's observations
      if (dist_to_landmark < (sensor_range * kSensorRangeMargin)) {
        LandmarkObs pred_obs;
        pred_obs.id = landmark.id_i;
        pred_obs.x = landmark_x;
        pred_obs.y = landmark_y;
        particle_observations.emplace(landmark.id_i, pred_obs);
      }
    }
    
    /**
     * Step 2:
     * Convert measured observations from vehicle coordinates to map coordinates
     * and associate the nearest landmark ID from pre-filtered particle
     * observations to be able to match up for the Gaussian weight calculation.
     */

    // Set coordinate transformation values from this particle
    const double theta = -particles[i].theta; // rotate back particle -> map
    const double x_part = particles[i].x;
    const double y_part = particles[i].y;
    
    // Loop through each measured observation
    for (auto &obs : vehicle_observations) {
      // Convert observation from vehicle coordinates to map coordinates
      const double x_map = x_part + (cos(theta) * obs.x) + (sin(theta) * obs.y);
      const double y_map = y_part - (sin(theta) * obs.x) + (cos(theta) * obs.y);
      obs.x = x_map;
      obs.y = y_map;
      
      // Compare observation position to each pre-filtered particle observation
      // to associate the closest landmark ID
      double min_dist = __DBL_MAX__;
      for (const auto &pred_obs : particle_observations) {
        const int pred_obs_id = pred_obs.first;  // landmark ID
        const double pred_obs_x = pred_obs.second.x;  // landmark x position
        const double pred_obs_y = pred_obs.second.y;  // landmark y position
        
        // Calculate distance between the observations
        const double cur_dist = dist(pred_obs_x, pred_obs_y, obs.x, obs.y);
        
        // Associate nearest landmark ID
        if (cur_dist < min_dist) {
          min_dist = cur_dist;
          obs.id = pred_obs_id;
        }
      }
      
      // Add associated observation info to visualization vectors
      associations.push_back(obs.id);
      sense_x.push_back(obs.x);
      sense_y.push_back(obs.y);
    }
    
    /**
     * Step 3:
     * Set particle weight by multiplying all of the calculated multivariate
     * Gaussian probabilities of the observations with their associated
     * landmark x,y positions.
     */
    
    double weight_ = 1.0;
    for (const auto &obs : vehicle_observations) {
      // Set landmark's standard deviations and mean x,y position
      const double sig_x = std_landmark[0];
      const double sig_y = std_landmark[1];
      const double mu_x = particle_observations[obs.id].x;
      const double mu_y = particle_observations[obs.id].y;
      
      // Multivariate Gaussian equations for each vehicle observation
      const double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));
      const double exponent = (pow(obs.x - mu_x,2))/(2 * pow(sig_x,2)) +
                              (pow(obs.y - mu_y,2))/(2 * pow(sig_y,2));
      weight_ *= gauss_norm * exp(-exponent);
    }
    
    // Set weight in the particle property and in the combined weights vector
    particles[i].weight = weight_;
    weights[i] = weight_;
    
    /**
     * Step 4:
     * Set particle's associations for visualization
     */
    SetAssociations(particles[i], associations, sense_x, sense_y);
    
  } // loop to next particle
  
  /**
   * Step 5:
   * Check if all weights are zero to reinitialize if particles are way off
   */
  double weight_sum = 0.0;
  for (auto &weight : weights) {
    weight_sum += weight;
  }
  if (weight_sum == 0.0) {
    is_initialized = false;
  }
  
  // No need to normalize particle weights because they will be resampled using
  // std::discrete_distribution which will automatically normalize them.
  /*
   // Normalize particle weights
   double weight_sum = 0.0;
   for (auto &weight : weights) { weight_sum += weight; }
   for (auto &particle : particles) { particle.weight /= weight_sum; }
   */
}

/**
 * Resample the set of particles with probability of being chosen based on the
 * particle weights using std::discrete_distribution.
 *
 * The following references were used:
 * http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
 */
void ParticleFilter::Resample() {
  // Set up new vector of resampled particles
  std::vector<Particle> new_particles(num_particles);

  // Set up random sampling generators with the combined weights vector
  std::random_device rand_dev;
  std::default_random_engine random_gen(rand_dev());
  std::discrete_distribution<int> discrete_dist(weights.begin(), weights.end());

  // Resample the particles
  for (int i = 0; i < num_particles; ++i) {
    const int index = discrete_dist(random_gen);
    new_particles[i] = particles[index];
    //std::cout << "Selected particle: " << index << std::endl;
  }
  
  // Replace the particle set with the resampled particles
  particles.clear();
  particles = new_particles;
}

/**
 * Set a particle's association info vectors for visualization by the simulator.
 */
void ParticleFilter::SetAssociations(Particle &particle,
                                     const std::vector<int> &associations,
                                     const std::vector<double> &sense_x,
                                     const std::vector<double> &sense_y) {
  // particle: The particle to assign each listed association and
  //           the associations' (x,y) world coordinates
  // associations: The landmark ID that goes along with each listed association
  // sense_x: The associations x mapping already converted to world coordinates
  // sense_y: The associations y mapping already converted to world coordinates
  
  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();
  
  // Set the particle's association info vectors
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

std::string ParticleFilter::GetAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  copy( v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

std::string ParticleFilter::GetSenseX(Particle best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

std::string ParticleFilter::GetSenseY(Particle best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
