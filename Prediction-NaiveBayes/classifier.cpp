#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

#include <assert.h>

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

  assert(data.size() == labels.size());

  int num_labels = possible_labels.size();
  int num_vars = data[0].size();

  vector<int> num(num_labels, 0);

  mu.resize(num_labels, vector<double>(num_vars));
  for (auto& sub : mu) {
      std::fill(sub.begin(), sub.end(), 0);
  }

  sigma.resize(num_labels, vector<double>(num_vars));
  for (auto& sub : sigma) {
      std::fill(sub.begin(), sub.end(), 0);
  }

  proba_class.reserve(num_labels);

  for (size_t i = 0; i < data.size(); i++) {
    for (size_t label = 0; label < num_labels; label++) {
      if (possible_labels[label] == labels[i]) {
        num[label]++;
        for (size_t var = 0; var < num_vars; var++) {
          mu[label][var] += data[i][var];
          sigma[label][var] += pow(data[i][var], 2);
        }
        continue;
      }
    }
  }

  for (size_t label = 0; label < num_labels; label++) {
    for (size_t var = 0; var < num_vars; var++) {
      mu[label][var] /= num[label];
      sigma[label][var] /= num[label];
      sigma[label][var] -= pow(mu[label][var], 2);
      sigma[label][var] = sqrt(sigma[label][var]);
      cout << "label: " << label << " var: " << var << " mu=" << mu[label][var] << " sigma=" << sigma[label][var] << endl;
    }
    proba_class[label] = (double)num[label] / (double)data.size();
    cout << "proba_class=" << proba_class[label] << endl;
  }
  

}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/

  int num_labels = possible_labels.size();

  vector<double> llh(num_labels, 0); // likelihood
  double llh_max = 0;
  int llh_label = 0;

  // Apply Bayes rule
  for (size_t label = 0; label < num_labels; label++) {
    llh[label] = proba_class[label];
    for (size_t var = 0; var < sample.size(); var++) {
      llh[label] *= normpdf(sample[var], mu[label][var], sigma[label][var]);
    }
    if (llh[label] > llh_max) {
      llh_max = llh[label];
      llh_label = label;
    }
  }

	//return this->possible_labels[1];
	return this->possible_labels[llh_label];

}

double GNB::normpdf(double x, double mu, double sigma) { 
    return  (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-0.5 * pow((x - mu) / sigma, 2));
}
