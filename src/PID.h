#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
    /**
     * Constructor
     */
    PID();
    
    /**
     * Destructor.
     */
    virtual ~PID();
    
    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Kd_, double Ki_);
    void SetP(std::vector<double> p);
    std::vector<double> GetP();

    double get_twiddle_error();

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);
    
    /**
     * Try to twiddle
     */
    void Twiddle(double total_error, double hyperparameter);
    
    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();
    
private:
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double twiddle_error;
    
    std::vector<double> p;
    std::vector<double> dp;
};

#endif  // PID_H
