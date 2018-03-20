//
//  cost.cpp
//  path_planning
//
//  Created by Alan Gordon on 2/27/18.
//

#include <stdio.h>
#include "cost.h"
#include "vehicle.h"
#include "coefficients.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

#define LARGE_NUMBER 1000000
#define VERY_SMALL_NUMBER 0.0001

double adjusted_distance(double ego_car_s,double other_car_s) {
    double adjusted_distance;
    adjusted_distance=ego_car_s-other_car_s;
    if (fabs(adjusted_distance) > 6000) { // one vehicle has crossed start/finish
        if (adjusted_distance > 0)  // ego vehicle behind
            adjusted_distance-=TRACK_LENGTH;
        else  // ego vehicle ahead
            adjusted_distance+=TRACK_LENGTH;
    }
    return adjusted_distance;
}

double logistic(double x){
    // A logistic function used to implement my cost functions.
    return 2.0/(1+exp(-x))-1.0;
}

double closest_distance(const vector<Vehicle> & trajectory, vector<Vehicle> & predictions) {
    double closest = LARGE_NUMBER;
    double closest_lane;
    double ego_start_s, ego_start_d, ego_end_s, ego_end_d;
    double this_final_s, this_final_d, other_final_s, other_final_d;
    Vehicle vehicle_start=trajectory[0];
    Vehicle vehicle_end=trajectory[1];
    Vehicle other_car_start = predictions[0];
    Vehicle other_car_end = predictions[predictions.size()-1];
    
    double other_start_d, other_end_d, other_start_s, other_end_s, other_future_s, other_future_d;
    other_start_d=other_car_start.lane*4+2;
    other_end_d=other_car_end.lane*4+2;
    other_start_s=other_car_start.s;
    other_end_s=other_car_end.s;
    
    ego_start_s=vehicle_start.s;
    ego_start_d=vehicle_start.lane*4+2;
    ego_end_s=vehicle_end.s;
    ego_end_d=vehicle_end.lane*4+2;
    
    other_future_s=other_end_s + other_car_end.v*N_PREDICTIONS*0.02;
    other_future_d=other_end_d;

//    double start_dist = sqrt(pow(ego_start_s - other_start_s, 2) + pow(ego_start_d - other_start_d, 2));
    double start_dist = sqrt(pow(adjusted_distance(ego_start_s,other_start_s), 2) + pow(ego_start_d - other_start_d, 2));
    if (start_dist < closest) {
        closest = start_dist;
        closest_lane=other_car_start.lane;
        this_final_s=ego_start_s;
        this_final_d=ego_start_d;
        
        other_final_s=other_start_s;
        other_final_d=other_start_d;
    }
 //   double end_dist = sqrt(pow(ego_end_s - other_end_s, 2) + pow(ego_end_d - other_end_d, 2));
    double end_dist = sqrt(pow(adjusted_distance(ego_end_s,other_end_s), 2) + pow(ego_end_d - other_end_d, 2));
    if (end_dist < closest) {
        closest = end_dist;
        closest_lane=other_car_end.lane;
        this_final_s=ego_end_s;
        this_final_d=ego_end_d;
        
        other_final_s=other_end_s;
        other_final_d=other_end_d;
    }
    return closest;
}

double closest_distance_to_a_vehicle(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
    double closest = LARGE_NUMBER;
    double ego_start_s, ego_start_d, ego_end_s, ego_end_d;
    int closest_id;
    vector<Vehicle> vehicle_list;
    Vehicle vehicle_start=trajectory[0];
    Vehicle vehicle_end=trajectory[1];
    Vehicle closest_vehicle_start, closest_vehicle_end;
    ego_start_s=vehicle_start.s;
    ego_start_d=vehicle_start.lane*4+2;
    ego_end_s=vehicle_end.s;
    ego_end_d=vehicle_end.lane*4+2;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it !=
          predictions.end(); it++) {
        vehicle_list=it->second;
        
        double current_dist = closest_distance(trajectory, vehicle_list);
        if (current_dist < closest) {
            closest = current_dist;
            closest_id=it->first;
            closest_vehicle_start=it->second[0];
            closest_vehicle_end=it->second[vehicle_list.size()-1];
        }
    }
    cout << "Closest distance to a vehicle=" << closest << endl;
    cout << "this car: start (s,lane)=(" << vehicle_start.s << "," << vehicle_start.lane << ")  end (" << vehicle_end.s << "," << vehicle_end.lane << ")" << endl;
    
    if (closest != LARGE_NUMBER) {
        cout << "ID=" << closest_id << " s(start)=" << closest_vehicle_start.s << " lane(start)=" << closest_vehicle_start.lane << " s(end)=" << closest_vehicle_end.s << " lane(end)=" << closest_vehicle_end.lane << endl;
    }
    else
        cout << "There are no vehicles in sensor range" << endl;
    return closest;
}

double closest_distance_to_a_vehicle_in_lane(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
    double closest = LARGE_NUMBER;
    int closest_id;
    vector<Vehicle> vehicle_list;
    Vehicle vehicle_start=trajectory[0];
    Vehicle vehicle_end=trajectory[1];
     Vehicle closest_vehicle_start, closest_vehicle_end;
    
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it !=
         predictions.end(); it++) {
        vector<Vehicle> vehicle_list;
        vehicle_list=it->second;
        Vehicle other_car_start = vehicle_list[0];
        Vehicle other_car_end = vehicle_list[vehicle_list.size()-1];
        
        if (vehicle_end.lane == other_car_end.lane) {
            double current_dist = closest_distance(trajectory, vehicle_list);
            if (current_dist < closest && current_dist < 120) {
                closest = current_dist;
                closest_id=it->first;
                closest_vehicle_start=other_car_start;
                closest_vehicle_end=other_car_end;
            }
        }
    }
    cout << "Closest distance to a vehicle in lane " << closest << endl;
    cout << "this car: start (s,lane)=(" << vehicle_start.s << "," << vehicle_start.lane << ")  end (" << vehicle_end.s << "," << vehicle_end.lane << ")" << endl;
    if (closest!=LARGE_NUMBER) {
        cout << "ID=" << closest_id << " s(start)=" << closest_vehicle_start.s << " lane(start)=" << closest_vehicle_start.lane << " s(end)=" << closest_vehicle_end.s << " lane(end)=" << closest_vehicle_end.lane << endl;
    }
    else
        cout << "There is no car within sensor range in end lane " << vehicle_end.lane << endl;
    return closest;
}

double collision_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Binary cost function which penalizes collisions.
    double closest = closest_distance_to_a_vehicle(trajectory, predictions);
    double cost;
    if (closest < 2 * VEHICLE_SIZE)
       cost=1;
    else
        cost=0;
    return cost;
}

double lane_change_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Binary cost function which penalizes lane changes.
    double cost;
    if (trajectory[0].lane != trajectory[1].lane)
        cost=1;
    else
        cost=0;
    cout << "Lane change cost=" << cost << endl;
    return cost;
}

double proximity_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Penalize getting close to other vehicles.
    double closest = closest_distance_to_a_vehicle(trajectory, predictions);
// make sure we don't divide by zero
    if (fabs(closest) < VERY_SMALL_NUMBER)
        closest=VERY_SMALL_NUMBER;
    double cost=logistic(2 * VEHICLE_SIZE / closest);
    cout << "Proximity cost = " << cost << endl;
    return cost;
}

double in_lane_proximity_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Penalize getting close to other vehicles in  your lane
    double closest = closest_distance_to_a_vehicle_in_lane(trajectory, predictions);
    double cost;
    // make sure we don't divide by zero
    if (fabs(closest) < VERY_SMALL_NUMBER)
        closest=VERY_SMALL_NUMBER;
    cost=logistic(2 * VEHICLE_SIZE / closest);
    cout << "Proximity in lane cost = " << cost << endl;
    return cost;
}

double not_middle_lane_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // penalize not being in middle lane
    double dist_from_middle_lane = (trajectory[1].lane*4-4);
    double cost;
    cost=logistic(pow(dist_from_middle_lane,2));
    cout << "Not middle lane cost = " << cost << endl;
    return cost;
}

double avg_nonzero(vector<double> vec)
{
    double sum=0;
    int nonzero_count=0;
    for (int i=0;i<vec.size();i++) {
        if (vec[i] != 0)
        {
            sum+=vec[i];
            nonzero_count++;
        }
    }
    return sum/nonzero_count;
}
double speed_for_trajectory(vector<Vehicle> trajectory) {
    // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
    // also can be used to find accelerations from velocities, jerks from accelerations, etc.
    // (i.e. discrete derivatives)

    return trajectory[1].v;
}

double lane_speed_cost(const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    // Lane with higher speed has lower cost
    double lane_speed, lane_speed_cost;
    int final_lane;
    Vehicle myCar, front_vehicle;
    myCar=trajectory[0];
    final_lane=trajectory[1].lane;
    
    cout << "Trajectory[0].lane=" << trajectory[0].lane << ", Trajectory[1].lane=" << trajectory[1].lane << endl;
    
    if (myCar.get_vehicle_ahead_for_lane_change(predictions, final_lane, front_vehicle))
    {
        if (front_vehicle.v <= SPEED_LIMIT)
            lane_speed=front_vehicle.v;
        else
            lane_speed=SPEED_LIMIT;
        cout << "vehicle ahead in lane=" << front_vehicle.lane << ", s=" << front_vehicle.s << ", myCar.s=" << myCar.s << endl;
    }
    else {
        cout << "no car ahead" << endl;
        lane_speed=SPEED_LIMIT;
    }
//    lane_speed_cost=logistic((SPEED_LIMIT - lane_speed) / SPEED_LIMIT);
    lane_speed_cost=(SPEED_LIMIT - lane_speed) / SPEED_LIMIT;
    cout << "Lane speed cost=" << lane_speed_cost << ", lane speed=" << lane_speed << endl;
    return lane_speed_cost;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
     Sum weighted cost functions to get total cost for trajectory.
     */
    float cost = 0.0;
    
    //Add additional cost functions here.
    vector< function<float(const vector<Vehicle> &, const map<int, vector<Vehicle>> &)>> cf_list = {proximity_cost,in_lane_proximity_cost,lane_speed_cost,not_middle_lane_cost,collision_cost,lane_change_cost};
    vector<float> weight_list = {PROXIMITY_COST_WEIGHT,LANE_PROXIMITY_COST_WEIGHT,LANE_SPEED_COST_WEIGHT,NOT_MIDDLE_LANE_COST_WEIGHT,COLLISION_COST_WEIGHT,LANE_CHANGE_COST_WEIGHT};
    
    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](trajectory, predictions);
        cost += new_cost;
    }
    return cost;    
}

