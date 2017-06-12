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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    int num_particles = 100;

    // Generate noise
    default_random_engine gen;

    normal_distribution<double> x_init(0, std[0]);
    normal_distribution<double> y_init(0, std[1]);
    normal_distribution<double> theta_init(0, std[2]);

    vector<double> weights; // simplify resample

    vector<Particle> particles;
    for (int i = 0; i < num_particles; i++)
    {
        Particle p = {i, x + x_init(gen), y + y_init(gen), theta + theta_init(gen), 1.0};
        particles.push_back(p);
        weights.push_back(1.0);
    
    }



    // Initializer flag
    is_initialized = true;



}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // Generate noise
    default_random_engine gen;
    
    normal_distribution<double> x_init(0, std_pos[0]);   
    normal_distribution<double> y_init(0, std_pos[1]);
    normal_distribution<double> theta_init(0, std_pos[2]);   


    for (int i = 0; i < particles.size(); i++)
    {
        if (fabs(yaw_rate) < 0.00001)
        {
            particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
            particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
            particles[i].theta = particles[i].theta;

        }
        else
        {
            particles[i].x = particles[i].x + velocity*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta))/yaw_rate;
            particles[i].y = particles[i].y + velocity*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t))/yaw_rate;

            particles[i].theta = particles[i].theta + yaw_rate*delta_t;
        }

        particles[i].x += x_init(gen);
        particles[i].y += y_init(gen);
        particles[i].theta += theta_init(gen);
    }
        
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for (int i = 0; i < observations.size(); i++)
    {
        double current_dist = 1e6;
        for (int j = 0; j < predicted.size(); j++)
        {
            // Distance between two points - use dist() more convenient
            double dist_ = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            if (dist_ < current_dist)
            {
                current_dist = dist_;
                observations[i].id = j;
            }
        }

    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    // Define some constants before loop for optimization
    double stddev_x = std_landmark[0];
    double stddev_y = std_landmark[1];
    double sigma = stddev_x*stddev_y;
    double denom = 1/(2*M_PI*sigma);
    double stddev_xsqr = pow(stddev_x, 2);
    double stddev_ysqr = pow(stddev_y, 2);


    for (int i = 0; i < particles.size(); i++)
    {
        // Transform Observations to Map Coordinates
        vector<LandmarkObs> transformed_obs = observations; //check if initialization like this helps
        for (int j = 0; j < observations.size(); j++)
        {
            transformed_obs[j].x = particles[i].x + observations[j].x*cos(particles[i].theta) - observations[j].y*sin(particles[i].theta);
            transformed_obs[j].y = particles[i].y + observations[j].x*sin(particles[i].theta) + observations[j].y*cos(particles[i].theta);
            transformed_obs[j].id = j;

        }

        // Find nearest map landmark
        vector<LandmarkObs> nearest_landmark;
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
        {
            Map::single_landmark_s curr_landmark = map_landmarks.landmark_list[j];

            double curr_dist = dist(particles[i].x, particles[i].y, curr_landmark.x_f, curr_landmark.y_f);

            if (curr_dist < sensor_range)
            {
                LandmarkObs pred_landmark = {curr_landmark.id_i, curr_landmark.x_f, curr_landmark.y_f};
                nearest_landmark.push_back(pred_landmark);

            }
        }

        // Call data association function
        dataAssociation(nearest_landmark, transformed_obs);
        
        double weight_init = 1.0;
        // Update weights
        for (int k = 0; k < transformed_obs.size(); k++)
        {
            weight_init *= denom*exp(pow((transformed_obs[k].x - nearest_landmark[k].x), 2)/(2*stddev_xsqr) + pow(transformed_obs[k].y - nearest_landmark[k].y, 2)/(2*stddev_ysqr));

        }

        particles[i].weight = weight_init;
        weights[i] = weight_init; 
        
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    discrete_distribution<> discrete_dist(weights.begin(), weights.end());

    vector<Particle> resample_particles;
    for (int i = 0; i < particles.size(); i++)
    {
        resample_particles[i] = particles[discrete_dist(gen)];

    }
    particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
