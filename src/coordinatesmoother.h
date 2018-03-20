//
//  coordinatesmoother.h
//  Path_Planning
//
//  Created by Alan Gordon on 3/13/18.
//

#ifndef coordinatesmoother_h
#define coordinatesmoother_h
#include <vector>
#include <string>
#include "coefficients.h"

using namespace std;
enum SmoothingStatus { INITIALIZING,SMOOTHING, NOT_SMOOTHING };

class CoordinateSmoother {
public:
    CoordinateSmoother();
    double smooth_scoord(int step,double curr_s);
private:
    double avg();
    vector<double> s_delta;
    double m_prev_s;
    double m_smoothed_start_s;
    int m_num_times_smoothed;
    SmoothingStatus m_status;
};

CoordinateSmoother::CoordinateSmoother()
{
    m_prev_s=0;
    m_num_times_smoothed=0;
    m_status=INITIALIZING;
    s_delta.resize(10);
}

double CoordinateSmoother::avg()
{
    double sum=0.0;
    for(std::size_t i = 0; i < s_delta.size(); i++)
        sum += s_delta[i];
    return sum/s_delta.size();
}

double CoordinateSmoother:: smooth_scoord(int step,double curr_s)
{
    double delta=curr_s-m_prev_s;
    double retval;
    if (fabs(delta) > 6000)
        delta=delta + TRACK_LENGTH;
    cout << "curr_s=" << curr_s << ", m_prev_s=" << m_prev_s << endl;
    cout << "Step=" << step << ", Delta=" << delta << ", Avg=" << avg() << endl;
    
    if (m_status==NOT_SMOOTHING) {
        if (delta > 15) {
            cout << "Started smoothing, curr_s=" << curr_s << " prev_s=" << m_prev_s << endl;
            m_status=SMOOTHING;
            m_num_times_smoothed=1;
            m_smoothed_start_s=m_prev_s;
            retval=m_smoothed_start_s+m_num_times_smoothed*avg();
            if (retval > TRACK_LENGTH)
                retval-=TRACK_LENGTH;
            m_prev_s=retval;
        }
        else {
            m_prev_s=curr_s;
            retval=curr_s;
        }
    }
    else if (m_status==SMOOTHING) {
        m_num_times_smoothed++;
        retval=m_smoothed_start_s+m_num_times_smoothed*avg();
        if (retval > TRACK_LENGTH)
            retval-=TRACK_LENGTH;
        m_prev_s=retval;
        if (fabs(retval-curr_s) < 2*avg() || m_num_times_smoothed > 10) {
        // return to
            cout << "Finished smoothing, num times smoothed=" << m_num_times_smoothed << " curr_s=" << curr_s << endl;
            
            retval=curr_s;
            m_prev_s=curr_s;
            m_num_times_smoothed=0;
            m_status=NOT_SMOOTHING;
        }
    }
    else if (m_status==INITIALIZING) {
        if (step==10)
            m_status=NOT_SMOOTHING;
        retval=curr_s;
        m_prev_s=retval;
    }
    else {
        // cause a failure so we know something is wrong
        retval=0;
        cout << "CoordinateSmoother:: smooth_scoord: We should never get here" << endl;
    }
    int indx=step%10;
    cout << "Step=" << step << ", Index=" << indx << ", delta=" << delta << endl;
    s_delta[indx]=delta;
    return retval;
}
#endif /* coordinatesmoother_h */
