/**
 * This file implements computing policies.
 *
 * The actual parameters used in the policy can be reconfigured with setPolicy
 */

#include <messages.pb.h>   // for LogEntry, LinearPolicy, SetController

// internal functions in an anonymous namespace
namespace {
	//! pointers to the fields that our controller takes as input
	struct field_pair {
		float LogEntry::* state;
		float LinearPolicy::* policy;
		operator bool() const { return state != nullptr || policy != nullptr; }
	};

	//! List of state fields and their corresponding controller field
	//! Null-terminated
	const field_pair fields[] = {
		{&LogEntry::droll,    &LinearPolicy::k_droll},
		{&LogEntry::dyaw,     &LinearPolicy::k_dyaw},
		{&LogEntry::dAngleW,  &LinearPolicy::k_dAngleW},
		{&LogEntry::dpitch,   &LinearPolicy::k_dpitch},
		{&LogEntry::dAngleTT, &LinearPolicy::k_dAngleTT},
		{&LogEntry::xOrigin,  &LinearPolicy::k_xOrigin},
		{&LogEntry::yOrigin,  &LinearPolicy::k_yOrigin},
		{&LogEntry::roll,     &LinearPolicy::k_roll},
		{&LogEntry::yaw,      &LinearPolicy::k_yaw},
		{&LogEntry::pitch,    &LinearPolicy::k_pitch},
		{nullptr, nullptr}
	};

	//! compute the policy output for a given policy and state
	float computePolicy(const LinearPolicy& policy, const LogEntry& state) {
		float result = policy.k_bias;

		// iterate over the pairs of members defined in fields
		for(const field_pair* fp = fields; *fp; fp++) {
			result += policy.*(fp->policy) * state.*(fp->state);
		}
		return result;
	}

	//! Turntable policy
	LinearPolicy policyTTParams = {
		0.13994,
		2.56325, -0.02692, 0.02074, 0.06003, -0.00312, 1.61079, -2.93646, 11.57684, -0.05651, 0.53147
	};

	//! Wheel policy
	LinearPolicy policyWheelParams = {
		0.02970,
		0.04465, -0.01305, 0.02915, 0.43688, 0.00171, -0.56873, -0.62321, 0.15515, -0.01179, 1.00638
	};
}

//! Set the policy from an incoming message
void setPolicy(const SetController& new_controller)
{
	policyWheelParams = new_controller.wheel;
	policyTTParams = new_controller.turntable;
}

//! compute the turntable output from the current policy, given the state
float policyTurntable(const LogEntry& state)
{
	return computePolicy(policyTTParams, state);
}

//! compute the wheel output from the current policy, given the state
float policyWheel(const LogEntry& state)
{
	return computePolicy(policyWheelParams, state);
}
