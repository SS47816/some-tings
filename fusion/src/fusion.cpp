#include "../include/fusion/fusion.h"

// Constructors
Fusion::Fusion() {};

Fusion::Fusion(vector<double> x, vector<double> y)
{
  this->x_ = x;
  this->y_ = y;
};

// Destructor
Fusion::~Fusion() {};

/**
 * TODO: implement your public functions here
 */

double Fusion::calcOutput()
{
  return doSomething() / doAnotherThing();
}

/**
 * TODO: implement your private functions here
 */

double Fusion::doSomething()
{
  double sum;
  for (int i = 0; i < x_.size(); i++)
  {
    sum += x_[i] * y_[i];
  }
  return sum;
}

double Fusion::doAnotherThing()
{
  double sum;
  for (int i = 0; i < x_.size() - 1; i++)
  {
    sum += (y_[i+1] - y_[i]) / (x_[i+1] - x_[i]);
  }
  return sum / (x_.size() - 1);
}