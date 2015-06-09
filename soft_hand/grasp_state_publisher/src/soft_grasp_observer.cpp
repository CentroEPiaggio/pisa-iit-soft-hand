// class definition
#include <grasp_state_publisher/soft_grasp_observer.h>

namespace grasp_state_publisher
{

// constructor
SoftGraspObserver::SoftGraspObserver() 
{
	is_initialized_ = false;
}

void SoftGraspObserver::init(const std::vector<double> &mu_grasp, const std::vector<double> &var_grasp,
								const std::vector<double> &mu_no_grasp, const std::vector<double> &var_no_grasp)
{
	if(!is_initialized_)
	{
		n_vars_ = mu_grasp.size();

		mu_grasp_ = mu_grasp;
		var_grasp_ = var_grasp;
		grasp_pdf_ = gaussianDiagonalCov(mu_grasp_, var_grasp);

		mu_no_grasp_ = mu_no_grasp;
		var_no_grasp_ = var_no_grasp;
		no_grasp_pdf_ = gaussianDiagonalCov(mu_no_grasp_, var_no_grasp_);

		grasp_state_.grasp = false;
		grasp_state_.grasp_confidence = 0.0;
		grasp_state_.no_grasp_confidence = 0.0;

		is_initialized_ = true;
	}

	return;
}

// map from std vector to column vector
void SoftGraspObserver::setX(const std::vector<double> &x)
{
	x_ = MatrixWrapper::ColumnVector(x.size());

	for (unsigned int i = 0; i < x_.size(); i++) {
		x_(i + 1) = x[i];
	}

	// update after setting a new input
	SoftGraspObserver::update();

	return;
}

// update only if initialized
void SoftGraspObserver::update()
{
	if(!is_initialized_)
	{
		std::cout << "The observer hasn't been initialized." << std::endl;
		return;
	}

	// compute confidences
	grasp_state_.grasp_confidence = grasp_pdf_.ProbabilityGet(x_).getValue();
	grasp_state_.no_grasp_confidence = no_grasp_pdf_.ProbabilityGet(x_).getValue();

	// decide
	if (grasp_state_.grasp_confidence > grasp_state_.no_grasp_confidence)
		grasp_state_.grasp = true;
	else
		grasp_state_.grasp = false;

	return;
}

BFL::Gaussian SoftGraspObserver::gaussianDiagonalCov(const std::vector<double> &mu, const std::vector<double> &var) {

	MatrixWrapper::ColumnVector Mu(mu.size());

	for (unsigned int i = 0; i < mu.size(); i++) {
		Mu(i + 1) = mu[i];
	}

	MatrixWrapper::SymmetricMatrix Cov(var.size());
	Cov = 0.0;
	for (unsigned int i = 0; i < var.size(); i++) {
		for (unsigned int j = 0; j < var.size(); j++) {
			if (i == j) {
				Cov(i + 1, j + 1) = var[i];
			}

			else {
				Cov(i + 1, j + 1) = 0.0;
			}
		}
	}

	return BFL::Gaussian(Mu, Cov);
}

}