//
//  vehicle.cpp
//  path_planning
//
//  Created by Alan Gordon on 2/25/18.
//

#include <stdio.h>
#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "coefficients.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
    
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
    this->target_speed=SPEED_LIMIT;
    this->lanes_available=3;
    
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions,bool lane_change_in_progress) {
    /*
     Here you can implement the transition_function code from the Behavior Planning Pseudocode
     classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
     to the next state.
     
     INPUT: A predictions map. This is a map of vehicle id keys with predicted
     vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
     the vehicle at the current timestep and one timestep in the future.
     OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
     
     */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;
    vector<Vehicle> kl_trajectory;
    int size;
    
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions, lane_change_in_progress);
        size=trajectory.size();
        if (size != 0) {
            cout << "*** Current lane=" << this->lane << ", state=" << *it << " ***" << endl;
            cost = calculate_cost(*this, predictions, trajectory);
            cout << "*** Cost for lane " << trajectory[1].lane << " = " << cost << " ***" << endl;
            costs.push_back(cost);
            if ((*it).compare("KL") == 0) {
                kl_trajectory=trajectory;
            }
            final_trajectories.push_back(trajectory);
        }
    }
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    
    int best_idx = distance(begin(costs), best_cost);
    if (costs[best_idx] > 10000)
        return kl_trajectory;
    else
        return final_trajectories[best_idx];
 //   if (costs[best_idx] < 0)
 //       cout << "*** Negative best cost=" << costs[best_idx] << ". No good options ***" << endl;
    
}

vector<string> Vehicle::successor_states() {
    /*
     Provides the possible next states given the current state for the FSM
     discussed in the course, with the exception that lane changes happen
     instantaneously, so LCL and LCR can only transition back to KL.
     */

    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (lane != 0) {
        states.push_back("LCL");
    }
    if (lane != lanes_available - 1) {
        states.push_back("LCR");
    }
    
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions, bool lane_change_in_progress) {
    /*
     Given a possible next state, generate the appropriate trajectory to realize the next state.
     */
    vector<Vehicle> trajectory;
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (!lane_change_in_progress && (state.compare("LCL") == 0 || state.compare("LCR") == 0)) {
        trajectory = lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
     Gets next timestep kinematics (position, velocity, acceleration)
     for a given lane. Tries to choose the maximum velocity and acceleration,
     given other vehicle positions and accel/velocity constraints.
     */

    float new_position;
    float new_velocity;
    float distance_ahead;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
 //       distance_ahead=vehicle_ahead.s - this->s;
        distance_ahead=dist_adjusted_for_track_length(vehicle_ahead.s,this->s);
        if (distance_ahead > 30 || (vehicle_ahead.v > SPEED_LIMIT)) { // if lots of space, stay in lane and go near the speed limit
            new_velocity=SPEED_LIMIT;
        } else {
            if (distance_ahead > 8)
                new_velocity = vehicle_ahead.v;
            else
                new_velocity = vehicle_ahead.v - 2.0; // hard brake if close to vehicle in front
        }
    }
    else
        //      new_velocity=this->target_speed;
        new_velocity=SPEED_LIMIT;
    new_position = this->s + new_velocity*N_PREDICTIONS*.02;
    if (new_position > TRACK_LENGTH)
        new_position-=TRACK_LENGTH;
    return{new_position, new_velocity, 0};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
     Generate a keep lane trajectory.
     */
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
     Generate a lane change trajectory.
     */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    vector<Vehicle> vehicle_list;
    double dist;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    if (this->s < 6920) {
        for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
            vehicle_list=it->second;
            next_lane_vehicle = vehicle_list[vehicle_list.size()-1];
            // disallow a lane change by not even returning a lane change in that direction
            // if a car is blocking the way
            dist=dist_adjusted_for_track_length(next_lane_vehicle.s,this->s);
            if (fabs(dist) < LANE_CHANGE_BUFFER*VEHICLE_SIZE  && next_lane_vehicle.lane == new_lane) {
                //If lane change is not possible, return empty trajectory.
                cout << "Lane change " << state << " is not possible this->s=" << this->s << " next lane vehicle.s=" << next_lane_vehicle.s << endl;
                return trajectory;
            }
        }
    }
    else {
        // disallow lane changes near the end of the track to work around bug
        cout << "Disallow lane change near end of track to work around simulator bug" << endl;
        return trajectory;
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
    this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    float retval=(this->s + this->v*t*.02);
    return retval;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
     Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
     rVehicle is updated if a vehicle is found.
     */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[1];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
     Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
     rVehicle is updated if a vehicle is found.
     */
    int min_dist = 15000;
    int lane_to_use;
    bool found_vehicle = false;
    double dist;
    Vehicle temp_vehicle;
    if (lane != this-> lane)
        lane_to_use=lane;
    else
        lane_to_use=this->lane;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        vector<Vehicle> vehicle_list;
        vehicle_list=it->second;
        temp_vehicle = vehicle_list[vehicle_list.size()-1];
        cout << "get_vehicle_ahead: ID=" << it->first << " temp_vehicle.s=" << temp_vehicle.s << " temp_vehicle.lane=" << temp_vehicle.lane << " this->s=" << this->s << " this->lane=" << this->lane << endl;
        dist=temp_vehicle.s-this->s;
        if (fabs(dist) > 6000) { // one vehicle has crossed start/finish
            if (dist > 0)  // vehicle behind
                dist-=TRACK_LENGTH;
            else  // vehicle ahead
                dist+=TRACK_LENGTH;
        }
            
 //       if (temp_vehicle.lane == lane_to_use && (temp_vehicle.s - 2*VEHICLE_SIZE) > this->s && temp_vehicle.s < min_s) {
//        if (temp_vehicle.lane == lane_to_use && dist > 2*VEHICLE_SIZE && temp_vehicle.s < min_s) {
        if (temp_vehicle.lane == lane_to_use && dist > 2*VEHICLE_SIZE && dist < min_dist) {
//            min_s = temp_vehicle.s;
            min_dist = dist;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

double Vehicle::dist_adjusted_for_track_length(double other_car_s,double ego_car_s)
{
    double orig_dist=other_car_s - ego_car_s;
    double retval;
    if (fabs(orig_dist) > (TRACK_LENGTH-2000)) {
        if (orig_dist > 0)
            retval=orig_dist - TRACK_LENGTH;
        else if (orig_dist < 0)
            retval= orig_dist + TRACK_LENGTH;
        else
            retval=orig_dist;
    }
    else
        retval=orig_dist;
    return retval;
}

bool Vehicle::get_vehicle_ahead_for_lane_change(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
     Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
     rVehicle is updated if a vehicle is found.
     */
    int min_dist = 15000;
    int lane_to_use;
    bool found_vehicle = false;
    double dist;
    Vehicle temp_vehicle;
    if (lane != this->lane)
        lane_to_use=lane;
    else
        lane_to_use=this->lane;
    
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); it++) {
        vector<Vehicle> vehicle_list;
        vehicle_list=it->second;
        temp_vehicle = vehicle_list[vehicle_list.size()-1];
  //      temp_vehicle = vehicle_list[0];
        
        cout << "get_vehicle_ahead_for_lane_change: ID=" << it->first << " temp_vehicle.s=" << temp_vehicle.s << " temp_vehicle.lane=" << temp_vehicle.lane << " this->s=" << this->s << " this->lane=" << this->lane << endl;
        
        dist=temp_vehicle.s-this->s;
        if (fabs(dist) > 6000) { // one vehicle has crossed start/finish
            if (dist > 0)  // vehicle behind
                dist-=TRACK_LENGTH;
            else  // vehicle ahead
                dist+=TRACK_LENGTH;
        }
        
//        if (temp_vehicle.lane == lane_to_use && (temp_vehicle.s) > this->s && temp_vehicle.s < min_s) {
          if (temp_vehicle.lane == lane_to_use && dist > 0 && dist < min_dist) {
//            min_s = temp_vehicle.s;
            min_dist = dist;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
     Generates predictions for non-ego vehicles to be used
     in trajectory generation for the ego vehicle.
     */
    vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
        float next_s = position_at(i);
        float next_v = this->v;
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
    return predictions;    
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
     Sets state and kinematics for ego vehicle using the last state of the trajectory.
     */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}
