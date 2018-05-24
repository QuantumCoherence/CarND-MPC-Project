#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define Wcte		0
#define Wepsi		1
#define Wspeed		2
#define Wsteer		3
#define Wthrtl		4
#define Wsteer_chg	5
#define Wthrtl_chg	6


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients and constraint model Weights .
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Eigen::VectorXd modelWeights);
};

#endif /* MPC_H */
