/**
 * This file implements computing policies.
 *
 * The actual parameters used in the policy can be reconfigured with setPolicy
 */

#include <messages.pb.h>   // for LogEntry, LinearPolicy, Controller

// internal functions in an anonymous namespace
namespace {
	//! pointers to the fields that our controller takes as input
	struct field_pair {
		float LogEntry::* state;
		float LinearPolicy::* lin_field;
		LinearPolicy PureQuadraticPolicy::* quad_field;
		operator bool() const {
			return state != nullptr || lin_field != nullptr || quad_field != nullptr;
		}
	};

	//! List of state fields and their corresponding controller field
	//! Null-terminated
	const field_pair fields[] = {
		{&LogEntry::droll,    &LinearPolicy::k_droll,    &PureQuadraticPolicy::k_droll},
		{&LogEntry::dyaw,     &LinearPolicy::k_dyaw,     &PureQuadraticPolicy::k_dyaw},
		{&LogEntry::dAngleW,  &LinearPolicy::k_dAngleW,  &PureQuadraticPolicy::k_dAngleW},
		{&LogEntry::dpitch,   &LinearPolicy::k_dpitch,   &PureQuadraticPolicy::k_dpitch},
		{&LogEntry::dAngleTT, &LinearPolicy::k_dAngleTT, &PureQuadraticPolicy::k_dAngleTT},
		{&LogEntry::xOrigin,  &LinearPolicy::k_xOrigin,  &PureQuadraticPolicy::k_xOrigin},
		{&LogEntry::yOrigin,  &LinearPolicy::k_yOrigin,  &PureQuadraticPolicy::k_yOrigin},
		{&LogEntry::roll,     &LinearPolicy::k_roll,     &PureQuadraticPolicy::k_roll},
		{&LogEntry::yaw,      &LinearPolicy::k_yaw,      &PureQuadraticPolicy::k_yaw},
		{&LogEntry::pitch,    &LinearPolicy::k_pitch,    &PureQuadraticPolicy::k_pitch},
		{nullptr, nullptr, nullptr}
	};

	//! compute the policy output for a given policy and state
	float computePolicy(const LinearPolicy& policy, const LogEntry& state) {
		// dot product over the pairs of members defined in fields
		float result = 0;
		for(const field_pair* fp = fields; *fp; fp++) {
			result += policy.*(fp->lin_field) * state.*(fp->state);
		}
		return result;
	}
	float computePolicy(const PureQuadraticPolicy& policy, const LogEntry& state) {
		// note that we define this in terms of a sum over linear policies
		// weighted by state
		float result = 0;
		for(const field_pair* fp = fields; *fp; fp++) {
			result += computePolicy(policy.*(fp->quad_field), state) * state.*(fp->state);
		}
		return result;
	}
	float computePolicy(const AffinePolicy& policy, const LogEntry& state) {
		return policy.k_bias
			+ computePolicy(policy.k_lin, state);
	}
	float computePolicy(const QuadraticPolicy& policy, const LogEntry& state) {
		return policy.k_bias
			+ computePolicy(policy.k_lin, state)
			+ computePolicy(policy.k_quad, state);
	}

	//! Dispatch from a generic policy to a specific one
	float computePolicy(const Policy& policy, const LogEntry& state) {
		switch (policy.which_msg) {
			case Policy_lin_tag:    return computePolicy(policy.msg.lin, state);
			case Policy_affine_tag: return computePolicy(policy.msg.affine, state);
			case Policy_quad_tag:   return computePolicy(policy.msg.quad, state);
			default:                return 0;
		}
	}

	//! Turntable policy
	Policy policyTTParams = { 0 };

	//! Wheel policy
	Policy policyWheelParams = { 0 };
}

//! Set the policy from an incoming message
void setPolicy(const Controller& new_controller)
{
	policyWheelParams = new_controller.wheel;
	policyTTParams = new_controller.turntable;
}

float saturate(float p){
	// TODO: implement the same saturation as in simulation
	if (p > 1) {
		return 1;
	}
	else if (p < -1) {
		return -1;
	}
	else {
		return p;
	}
}

//! compute the turntable output from the current policy, given the state
float policyTurntable(const LogEntry& state)
{
	return saturate(computePolicy(policyTTParams, state));
}

//! compute the wheel output from the current policy, given the state
float policyWheel(const LogEntry& state)
{
	return saturate(computePolicy(policyWheelParams, state));
}
