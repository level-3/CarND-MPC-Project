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

//#include "matplotlibcpp.h"




//#include <boost/thread.hpp>
// for convenience
using json = nlohmann::json;


const bool show_graph = false;

//namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const double Lf = 2.67;

int iters= 1;



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
  for (unsigned int i = 0; i < coeffs.size(); i++) {
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

  for (unsigned int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (unsigned int j = 0; j < xvals.size(); j++) {
    for ( int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void peigen (Eigen::VectorXd vect ) // send a vector to console
{
          
          for (unsigned int i = 0; i < vect.size() ; i++)
          {
            cout << vect[i] << endl;
          }
          cout << endl ;

}



void pvector (vector<double> vect ) // send a vector to console
{
          
          for (unsigned int i = 0; i < vect.size() ; i++)
          {
            cout << vect[i] << endl;
          }
          cout << endl ;

}


// https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector

Eigen::VectorXd stdvector2eigen(vector<double> v1)
{
          //from v1 to an eignen vector
          //double* ptr_data = &v1[0];
          Eigen::VectorXd v2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v1.data(), v1.size());
          return v2;

}

vector<double> eigen2stdvector(Eigen::VectorXd v2)
{
    //from the eigen vector to the std vector
    std::vector<double> v3(&v2[0], v2.data()+v2.cols()*v2.rows());
    return v3;

}



size_t i = 0;

array<vector<double>, 2> World2Car(vector<double> vect_world_x, vector<double> vect_world_y , double gpx_x, double gps_y, double psi)
{
          cout << "Map\tX\t" ; pvector(vect_world_x);
          cout << "\tY\t" ; pvector(vect_world_y);
          for (unsigned int i = 0 ; i < vect_world_x.size() ; i++)
          {
            double x, y;
            x = vect_world_x[i] - gpx_x;
            y = vect_world_y[i] - gps_y;
            vect_world_x[i] =  x * cos(-psi) - y * sin(-psi);
            vect_world_y[i] =  x * sin(-psi) + y * cos(-psi);
          
          }
            cout << "Car\tX\t" ; pvector(vect_world_x);
            cout << "\tY\t" ; pvector(vect_world_y);
          array<vector<double>, 2> coord_set;
          coord_set[0] = vect_world_x;
          coord_set[1] = vect_world_y;
          return coord_set ;
}





int main() {
  uWS::Hub h;
  //std::cout << setprecision(8);
  // MPC is initialized here!
  MPC mpc;
          
          

  
  std::vector<double> x_vals ;
  std::vector<double> y_vals ;
  std::vector<double> psi_vals;
  std::vector<double> v_vals ;
  std::vector<double> cte_vals ;
  std::vector<double> epsi_vals ;
  std::vector<double> delta_vals ;
  std::vector<double> a_vals ;


  h.onMessage([&mpc, &x_vals, &y_vals, &psi_vals, &v_vals, &cte_vals, &epsi_vals, &delta_vals, &a_vals](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          //double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];

          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];



          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

   
          
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());


          for (unsigned int i = 0 ; i < ptsx.size() ; i++)
          {

            double x, y;
            x = ptsx[i] - px;
            y = ptsy[i] - py;
            ptsx_car[i] =  x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] =  x * sin(-psi) + y * cos(-psi);
          
          }


          //////////
          
          
          auto coeffs = polyfit(ptsx_car , ptsy_car ,  3);
          
            // The cross track error is calculated by evaluating at polynomial at x, f(x)
            // and subtracting y. x = 0 , y = 0

            // in the vehicle coords  
            double cte = polyeval(coeffs, 0) ;

            // Due to the sign starting at 0, the orientation error is -f'(x).
            // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]

            double epsi = -atan(coeffs[1]);

            Eigen::VectorXd state(6);
          
            double dt = 0.075;
            
            //predicted values
            double x_val=0.0 , y_val=0.0, psi_val=0.0, v_val = v, cte_val = cte, epsi_val = epsi;
            x_val    = v * cos(0) *dt ;
            y_val    = v * sin(0) *dt ;
            psi_val  = v * -delta / Lf *dt;
            v_val    = v + a *dt;
            cte_val  += v * sin(epsi) *dt;
            epsi_val += v * -delta / Lf *dt;

            //cout << x_val<< "\t" << y_val<< "\t" << psi_val<< "\t" << v_val<<"\t" << cte_val<<"\t" << epsi_val << "\t";
            //cout <<  epsi_val << "\t" << cte_val << "\t" << endl ;

            state << x_val, y_val, psi_val, v_val, cte_val, epsi_val;
            
            vector<double> vars;
            vars = mpc.Solve(state, coeffs);

            delta_vals.push_back(vars[0]);
            a_vals.push_back(vars[1]);      
            cte_vals.push_back(vars[2]);

            psi_vals.push_back(vars[3]);
            epsi_vals.push_back(vars[4]);
            v_vals.push_back(vars[5]);

            x_vals.push_back(px);
            y_vals.push_back(py);



          iters++;

          delta = - vars[0] / (deg2rad(25) * Lf);
          a  = vars[1];

          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = delta;
          msgJson["throttle"] = a;


          vector<double> mpc_x ;
          vector<double> mpc_y ;

          

          for (unsigned int i = 6 ; i < vars.size() ; i+=2 )
          {
            mpc_x.push_back(vars[i]);
            mpc_y.push_back(polyeval(coeffs , vars[i]) - vars[2]);
            //mpc_y.push_back(vars[i+1]) ;
            
            
          }

          //pvector(mpc_x);
          //pvector(mpc_y);

          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;

          //Display the waypoints/reference line

          vector<double> eig_next_x_vals = eigen2stdvector( ptsx_car );
          vector<double> eig_next_y_vals = eigen2stdvector( ptsy_car );


          vector<double> next_x_vals ;
          vector<double> next_y_vals ;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line


          double poly_incr = Lf;
          int n_pts = 25;

          for (signed int i = 1; i < n_pts ; i++)
          {
            next_x_vals.push_back(poly_incr * i);
            next_y_vals.push_back(polyeval(coeffs , poly_incr * i));
          }


          msgJson["next_x"] = next_x_vals ;
          msgJson["next_y"] = next_y_vals ;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << endl << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


          if (show_graph == true)
          {


          if (iters == 100)
            {

/*
                    
            // Plot values
            // NOTE: feel free to play around with this.
            // It's useful for debugging!
            plt::subplot(6, 1, 1);
            plt::title("CTE");
            plt::plot(cte_vals);

            plt::subplot(6, 1, 2);
            plt::title("Delta (Radians)");
            plt::plot(delta_vals);

            plt::subplot(6, 1, 3);
            plt::title("Velocity");
            plt::plot(v_vals);

            plt::subplot(6, 1, 4);
            plt::title("Epsi");
            plt::plot(epsi_vals);

            plt::subplot(6, 1, 5);
            plt::title("x");
            plt::plot(x_vals);

            plt::subplot(6, 1, 6);
            plt::title("y");
            plt::plot(y_vals);

            cout << "game over";

            //plt::ion();
            plt::show();
            //plt::save("./graph.png");
            //exit(1);

            */


            }   
          }

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    
  }
  

);

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
