#ifndef FUSION_H_
#define FUSION_H_

// #include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>

using std::cout;
using std::endl;
using std::vector;

class Fusion
{
 public:
  // Constructor
  Fusion();
  Fusion(vector<double> x, vector<double> y);

  // Destructor
  virtual ~Fusion();

  /**
   * TODO: declare your public variables here
   */

  /**
   * TODO: declare your public functions here
   */
  double calcOutput();

private:
  /**
   * TODO: declare your private variables here
   */
  vector<double> x_;
  vector<double> y_;

  /**
   * TODO: declare your private functions here
   */

  double doSomething();
  double doAnotherThing();
};

#endif // FUSION_H_