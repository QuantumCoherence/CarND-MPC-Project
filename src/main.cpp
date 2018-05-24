#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char* argv[]) {
// for tuning easiness, weights for each constraint can be modified at the prompt, rather than by recompiling
  static Eigen::VectorXd modelWeights(7);
  modelWeights << 7500, 100, 1, 100, 1, 100, 70;
  std::string weightStr = "";
  static double Lf = 2.67;
  static size_t N = 10;

  for(int args =1;args<argc;args=args+2){
	  weightStr = argv[args];
	  if	  (weightStr == "-cte")  {modelWeights(Wcte)       = atof(argv[args+1]);}
	  else if (weightStr == "-epsi") {modelWeights(Wepsi)      = atof(argv[args+1]);}
	  else if (weightStr == "-v")    {modelWeights(Wspeed)     = atof(argv[args+1]);}
	  else if (weightStr == "-str")  {modelWeights(Wsteer)     = atof(argv[args+1]);}
	  else if (weightStr == "-thrt") {modelWeights(Wthrtl)     = atof(argv[args+1]);}
	  else if (weightStr == "-strd") {modelWeights(Wsteer_chg) = atof(argv[args+1]);}
	  else if (weightStr == "-thrtd"){modelWeights(Wthrtl_chg) = atof(argv[args+1]);}
  }

  uWS::Hub h;
  // MPC is initialized here!
  MPC mpc;
  //cout << setw(15) << "x" << setw(15) << "y" << setw(15) << "psi" << setw(15) << "v" << setw(15) << "cte" << setw(15) << "epsi" << endl;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px =  j[1]["x"];
          double py =  j[1]["y"];
          double psi = j[1]["psi"];
          double v =   j[1]["speed"];
          double csteering = j[1]["steering_angle"];
          double cthrottle = j[1]["throttle"];

          // Conversion of waypoints of desired trajectory to Vehicle frame
          Eigen::VectorXd ptsxXd = Eigen::Map<Eigen::VectorXd>(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsyXd = Eigen::Map<Eigen::VectorXd>(ptsy.data(), ptsy.size());
          double sinpsi = sin(-psi);
          double cospsi = cos(-psi);
          for (size_t i=0;i<ptsx.size();i++){
        	  ptsxXd(i) = (ptsx[i]-px)*cospsi - (ptsy[i]-py)*sinpsi;
        	  ptsyXd(i) = (ptsx[i]-px)*sinpsi + (ptsy[i]-py)*cospsi;
          }

          // Estimation of desired trajectory by 3d degree polynomial fitting (2 degree might be enough for this track?)
          auto coeffs = polyfit(ptsxXd, ptsyXd, 3);

          // New state estimation with Latency Compensation
          /////////////////////////////////////////////////
          double latency = 0.1; // sec

          //
          // Estimate car position and orientation as well as cte and epsi using kinematic model at time latency from now
          // x = x + v * cos(psi) * latency
          //		      psi = 0 in vehicle frame >>
          px = v * latency;

          // y = y + v * sin(psi(t=0)) * latency
          // y is zero in vehicle frame, psi(t=0) = 0 >> sin(0) = 0
          // Intuitively ... a vehicle doens't move laterally  unless it lost grip with the ground, which would require Dynamic Model
          py = 0.0;

          // psi(t=0) = 0 in vehicle frame coordinates. >> according to the kinematic model
          // psi(t1) = psi(t=0) + v/Lf*current_steering*latency
          double psi_l = -csteering*v*latency/Lf; // delta ie current steering, is a corrective input, hence it takes negative sign

          // v = v + a0 * latency             //approximate a by assuming it won't change during latency time
          double v_l = v + cthrottle * latency; //cthrottle is the acceleration input

          // cte at latency  = current cte + change in error caused by vehicle motion during latency
          // current cte     = polyeval(coeffs, x0)-y0 = polyeval(coeffs, 0) -0
          // change in error =   v * / Lf  * sin(epsi(t0)) * latency;
          //
          // epsi(t0)        = psi(t0) - arctan(f'(t0)) = - atan(coeffs(1))
          // in vehicle frame at time t = 0, x = 0, y = 0
          double epsi0  = - atan(coeffs(1)); //x0 = 0 >> only one coeff remains
          double cte_l  = polyeval(coeffs,0.0) + v * Lf * latency * sin(epsi0);
          double epsi_l = epsi0 -v * csteering * latency / Lf; // again csteering is a corrective input

          // Update State vector
          Eigen::VectorXd state(6);
          state << px,py,psi_l,v_l,cte_l,epsi_l;

          // Optimized actuator estimation for this desired trajectory
          auto vars = mpc.Solve(state, coeffs, modelWeights);

          // Steer and Throttle
          double nsteering = vars[4] /(deg2rad(25)*Lf);
          double nthrottle = vars[5];

          json msgJson;
          msgJson["steering_angle"] = nsteering;
          msgJson["throttle"] = nthrottle;


          //Display the MPC predicted trajectory Green line
          // the last 3 points of the predicted trajectory are often off to the side
          // with a very strong cte weight, the car follows the trajectory well, and correctly emphasizes low error versus speed,
          // forcibly slowing down the car just before a turn , but the error of the last few points of the predicted trajectory is increased.
          // A lower cte weight reduces this error but affects negatively the tracking performance, especially in the second part of turns.
          //
          // ?? more owrk needed ;-)

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (size_t i=7;i<13;i++){  // Last three points of the predicted trajectory aren't displayed
        	  mpc_x_vals.push_back(vars[i]);
        	  mpc_y_vals.push_back(vars[i+N]);

          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (unsigned int i=0;i<ptsxXd.size();i++){
        	  next_x_vals.push_back(ptsxXd(i));
        	  next_y_vals.push_back(ptsyXd(i));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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

