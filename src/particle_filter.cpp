/*============================================================================
 Name        : Particle_filter.cpp
 Author      : Jean-Yves Bourdoncle
 Version     : v1.0
 Date		   : 01/05/2019
 Description : the implemented functionnalities in this module are :

		- Initialization Step :
		Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1

		- Prediction Step:
		Use of the CTRV Model for alle the particles

        - Data association :
        Find the predicted measurement that is closest to each observed measurement + assign the observed measurement to this
		particular landmark

 		- Update Weights :
 		Update the weights of each particle using a mult-variate Gaussian distribution

		- Resample the particle :
		Resample the particle with the replacement of news particlee
		Resampling with the resampling probability with the weight importance
		Use of the  wheel strategy

============================================================================*/

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"
using namespace std;

//using std::string;
//using std::vector;

/* normal distribution */
//using std::normal_distribution;

/* gen = random engine initializer */
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /*
   * Set the number of particles. Initialize all particles to
   * first position (based on estimates of x, y, theta and their uncertainties
   * from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.
   */
  num_particles = 1000;  /* number of particles*/

  /* creation of three gaussian distribution respective for : x,y and the heading */
  /* x,y and theta are the simulated estimation from GPS for the provided state of the car*/
  /*std[0] : standard deviation of x (meters),
    std[1] : standard deviation of y (meters),
    std[2] : standard deviation of yaw (rad)*/

  normal_distribution <double> dist_x (x,std[0]);
  normal_distribution <double> dist_y (y,std[1]);
  normal_distribution <double> dist_theta (theta,std[2]);

  /* vectors weight and particle resize with the number of particles */
  particles.resize (num_particles);
  weights.resize (num_particles);

  /* Initialize all the particles with the parameters of the Particle structur*/
  /* All the weights are initialized with 1.0 */
  for (int i=0; i <num_particles; i++){
	  particles[i].id = i;
	  particles[i].x = dist_x(gen);
	  particles[i].y = dist_y(gen);
	  particles[i].theta = dist_theta(gen);
	  particles[i].weight = 1.0;
  	  }
  /* the filter is initialized */
  is_initialized = true;
}



void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {

	/* Prediction Step : Use of the CTRV Model + Noise (normal distribution) addition for all the particles in X,Y and Theta */

	/*Creation of three gaussian NOISE distribution respective for x,y, and the heading*/
	normal_distribution <double> dist_x (0.0,std_pos[0]);
	normal_distribution <double> dist_y (0.0,std_pos[1]);
	normal_distribution <double> dist_theta (0.0,std_pos[2]);

	/* use of the CTRV Model (Constant Turn Rate and Velocity Magnitude Model) for the prediction */
	/* predicted state values declaration */
	for (int i=0; i <num_particles; i++) {
		// to avoid the division by 0, the yaw rate must be different from 0
		// theta = particles[i].theta
		// x = particles[i].x
		// y = particles[i].y
		if (fabs(yaw_rate)>0.0001) {
			particles[i].x =  particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta+ yaw_rate*delta_t) - sin (particles[i].theta)) + dist_x(gen);
			particles[i].y =  particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta)- cos (particles[i].theta+yaw_rate*delta_t)) + dist_y (gen);
		}
		else{ // second case :
			particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta) + dist_x(gen);
			particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta)+ dist_y(gen);
		}
	// theta
	particles[i].theta =  particles[i].theta + yaw_rate*delta_t + dist_theta(gen);
	}
}


void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   *   Data Association Step : Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   */

/*
 Between the prediction Step and the update Step, a data association Step is done between:
 - the observation measurement from the LIDAR (observation between the vehicle and the landmark),
 - the predicted measurement between every specific particle and all the landmarks observed from the sensor.
*/

for (unsigned int i=0; i<observations.size(); i++) { //loop for every observed landmark from the lidar

	//minimum distance as default for the start of the comparaison
	double closest_distance = 10000;
	//landmark MAP-ID
	int landmark_MAP_id = 0;

	for (unsigned int j=0; j<predicted.size(); j++) { //search the nearest neighbor data
		// current distance calculation
		double distance_x = predicted[j].x - observations[i].x;
		double distance_y = predicted[j].y - observations[i].y;
		double current_distance = sqrt (distance_x*distance_x + distance_y*distance_y);

/*Step 1 : Assign the MAP landmark ID with the nearest distance particle ID */
		if (current_distance < closest_distance) {
			closest_distance = current_distance;
			landmark_MAP_id = predicted [j].id;
		}
	}

/*Step 2 : Assign each sensor observation with the MAP landmark ID associated*/
	observations[i].id = landmark_MAP_id;
}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /*
   *   Update the weights of each particle using a mult-variate Gaussian
   *   distribution.
   *   The observations are given in the VEHICLE'S coordinate system.
   *   The particles are located according to the MAP'S coordinate system.
   *   A transformation between the two systems is useful (rotation + translation).
   */

/* use in the step 5 for the normalization*/
double sum_weight = 0.0;

for (int i=0; i <num_particles; i++)
	{
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

	/*Step 1: Transform each observation from vehicle local coordinates to map coordinates.*/
	// Use of the Homogeneous Transformation : rotation -90° + Translation for the conversion
    vector<LandmarkObs> trans_observations;
        for (unsigned int j = 0; j < observations.size(); j++) {
          LandmarkObs trans_obs;
          trans_obs.id = j;
          trans_obs.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
          trans_obs.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);
          trans_observations.push_back(trans_obs);
        }

    /*Step 2: predict measurement for all landmarks within sensor range for each particle*/
    /*collect only the landmark which are in the sensor_range and add the selected landmark in the landmark vector*/
      vector<LandmarkObs> predictions;
      	 // for each landmark in the MAP
         for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        	// get current landmark
            Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];
            // evaluation if the current landmark is in the sensor range
            if ((fabs((particle_x - current_landmark.x_f)) <= sensor_range) && (fabs((particle_y - current_landmark.y_f)) <= sensor_range)) {
            	// add the predictions in the vector
            	predictions.push_back(LandmarkObs {current_landmark.id_i, current_landmark.x_f, current_landmark.y_f});
              }
     }

/*Step 3: Associate the predicted landmark for every transformed observation*/
 //call the nearest neighbor fonction (Step 2)
dataAssociation(predictions,trans_observations);

/*Step 4: Update the weights of each particle with the Multivariate Gaussian probability density.*/

// initialize the weight with 1.0
particles[i].weight = 1.0;


// Assign weight to a given particle
double std_x = std_landmark[0];
double std_y = std_landmark[1];

// Normalizer part definition
double normalizer_part =  1.0/(2.0*M_PI*std_x*std_y);

for (unsigned int k = 0; k < trans_observations.size(); k++) {
            double obs_x = trans_observations[k].x;
            double obs_y = trans_observations[k].y;
            int obs_id = trans_observations[k].id;

            // get the x,y coordinates of the prediction associated with the current observation
            for (unsigned int l = 0; l < predictions.size(); l++) {
            	double pred_x = predictions[l].x;
            	double pred_y = predictions[l].y;
                int pred_id = predictions[l].id;


             // Exponential Part definition
             double exp_part = -(obs_x-pred_x)*(obs_x-pred_x)/(2*std_x*std_x) - (obs_y-pred_y)*(obs_y-pred_y)/(2*std_y*std_y);

			if (obs_id == pred_id) {
            	particles[i].weight *= normalizer_part*exp(exp_part);
            }
         }
	}

sum_weight += particles[i].weight;

}

/*Step 5: Normalize the weights of all particles since resampling using probabilistic approach.*/
for (unsigned int i = 0; i < particles.size(); i++) {
    particles[i].weight /= sum_weight;
    weights[i] = particles[i].weight;
  }


}

void ParticleFilter::resample() {
  /*
   Resampling with the resampling probability with the weight importance
   Use of the wheel strategy : every particle with his respective weight has a part of a wheel
   More the weight is important : more the part of th wheel is large

  */

// New vector for the resampled particles
vector<Particle> resampled_particles;

//Generate random particle index : U [1....N] with N= number of particles
uniform_int_distribution<int> particle_index(0, num_particles);

/*---------------Current_index and beta variables initialization-----------------------------------*/
// current index initialization : he moves for every step around the wheel
// the first current step is  the start index of the wheel
// the current_index for the start is choosen ramdomly
default_random_engine gen;
int current_index = particle_index(gen);
// beta initialization beta =uniform (0,1), the beta runs around the wheel during the resampling step
double beta = 0.0;


//  The variable 2*mw (max_weight) here calculated is useful for the beta mouvement in the wheel
double max_weight = 2.0 * *max_element(weights.begin(), weights.end());

// Resampling Loop for the new particles contruction
	for (unsigned int i = 0; i < particles.size(); i++) {
	// 2*max_weight is very big but with an random variable between [0,1], we have an uniformity between [0,2*max_weight]
		uniform_real_distribution<double> random_weight(0.0, max_weight);
		// the beta position moves around the wheel
		beta += random_weight(gen);

		// test the beta position with the current index position
		 while (beta > weights[current_index]) {
		    beta -= weights[current_index];// new beta = calculated beta - position index courant
		    current_index = (current_index + 1) % num_particles;
		  }
		 // result of the resampling with the new weight for each particles
		  resampled_particles.push_back(particles[current_index]);
		}
		particles = resampled_particles;
	}



void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
