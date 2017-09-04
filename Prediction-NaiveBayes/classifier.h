#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
public:

	vector<string> possible_labels = {"left","keep","right"};


	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  string predict(vector<double>);

private:
  double normpdf(double x, double mu, double sigma);

  // typically here 3 classes and 4 variables
  vector<double> proba_class;   // 3x1
  vector<vector<double>> mu;    // 3x4
  vector<vector<double>> sigma; // 3x4

};

#endif
