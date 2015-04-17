#ifndef SOFTGRASPOBSERVER_H_
#define SOFTGRASPOBSERVER_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/JointState.h>

// bfl
#include <bfl/pdf/gaussian.h>

namespace grasp_state_publisher
{

class SoftGraspObserver
{
private:

	bool is_initialized_;

	struct grasp
	{
		bool grasp;
		double grasp_confidence;
		double no_grasp_confidence;
	} grasp_state_;

	// grasp observer stuff
	BFL::Gaussian grasp_pdf_;
	BFL::Gaussian no_grasp_pdf_;

	MatrixWrapper::ColumnVector x_;

	int n_vars_;

	std::vector<double> mu_grasp_;
	std::vector<double> var_grasp_;

	std::vector<double> mu_no_grasp_;
	std::vector<double> var_no_grasp_;

	BFL::Gaussian gaussianDiagonalCov(const std::vector<double> &mu, const std::vector<double> &var);

	void update();

public:

	SoftGraspObserver();
	//! Empty stub
	~SoftGraspObserver() {}

	void init(const std::vector<double> &mu_grasp, const std::vector<double> &var_grasp,
				const std::vector<double> &mu_no_grasp, const std::vector<double> &var_no_grasp);

	// this automatically update the internal state
	void setX(const std::vector<double> &x);

	// just return the internal state
	bool isGrasping() { return grasp_state_.grasp; }
	double getGraspConfidence() { return grasp_state_.grasp_confidence; }
	double getNoGraspConfidence() { return grasp_state_.no_grasp_confidence; }

	int getNumberOfVars() {return n_vars_; }
};

}

#endif /* SOFTGRASPOBSERVER_H_ */