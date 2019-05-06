#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;
    bool use_twiddle = false;
    vector<double> p;
    vector<double> dp;
    
    PID pid;
    /**
     * TODO: Initialize the pid variable.
     */
    if (use_twiddle){
//        double src1[] = {1.3, 0.01, 0.};
        double src1[] = {0.2, 3, 0.};
        p.assign(std::begin(src1), std::end(src1));
//        double src2[] = {0.02, 0.001, 1e-5};
        double src2[] = {0.001, 0.0001, 1e-6};
        dp.assign(std::begin(src2), std::end(src2));
        pid.SetP(p);
    }
    else {
        pid.Init(.2, 3.0, 1e-4);
    }
    h.onMessage([&pid, &use_twiddle, &p, &dp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        static unsigned int total_step = 0;
        static const unsigned int twiddlePeriod = 500;
        static double twiddleBestError = INFINITY;
        static unsigned int twiddle_i = 0;
        static int twiddle_step = 0;
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));
            
            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value = 0;

                    /**
                     * TODO: Calculate steering value here, remember the steering value is
                     *   [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */
                    
                    // DEBUG
                    if (use_twiddle) {
                        ++total_step;
                        if (total_step % twiddlePeriod == 0) {
                            // reset simulator
                            std::string reset_msg = "42[\"reset\",{}]";
                            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                            double twiddleError = pid.get_twiddle_error()/twiddlePeriod;
                            if (dp[0] + dp[1] + dp[2] < 0.0002 && twiddleError == twiddleBestError) {
                                cout << "final p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl;
                                exit(0);
                            }
                            if (twiddle_step > 0) {
                                cout << "twiddle_i:     " << twiddle_i << endl;
                                cout << "twiddleError:     " << twiddleError << endl;
                                cout << "twiddleBestError: " << twiddleBestError << endl;
                                cout << "dp: " << dp[0] << " " << dp[1] << " " << dp[2] << " " << endl;
                                cout << "p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl;
                            }
                            if (twiddle_step == 1){
                                if (twiddleError < twiddleBestError) {
                                    twiddleBestError = twiddleError;
                                    dp[twiddle_i] *= 1.1;
                                    ++twiddle_i;
                                    twiddle_i %= 3;
                                    twiddle_step = 0;
                                    cout << "new dp: " << dp[0] << " " << dp[1] << " " << dp[2] << " " << endl;
                                    cout << "new p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl << endl;
                                }
                                else {
                                    p[twiddle_i] -= 2 * dp[twiddle_i];
                                    twiddle_step = 2;
                                    pid.SetP(p);
                                    cout << "new p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl << endl;
                                    return;
                                }
                            }
                            else {
                                twiddle_step = 0;
                                if (twiddleError < twiddleBestError) {
                                    twiddleBestError = twiddleError;
                                    dp[twiddle_i] *= 1.1;
                                }
                                else {
                                    p[twiddle_i] += dp[twiddle_i];
                                    dp[twiddle_i] *= 0.9;
                                    cout << "new p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl;
                                }
                                ++twiddle_i;
                                twiddle_i %= 3;
                                cout << "new dp: " << dp[0] << " " << dp[1] << " " << dp[2] << " " << endl << endl;
                                cout << "new p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl << endl;

                            }
                            
                            if (twiddle_step == 0) {
                                cout << "=============new round =========" << endl;
                                cout << "twiddle_i:     " << twiddle_i << endl;
                                cout << "p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl;
                                
                                p[twiddle_i] += dp[twiddle_i];
                                pid.SetP(p);
                                twiddle_step = 1;
                                cout << "new p: " << p[0] << " " << p[1] << " " << p[2] << " " << endl << endl;
                                return;
                            }
                        }
                    }
                    pid.UpdateError(cte);
                    steer_value = pid.TotalError();
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    if (!use_twiddle){
                        std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
//                        std::cout << msg << std::endl;
                    }
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }); // end h.onMessage
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    
    h.run();
}
