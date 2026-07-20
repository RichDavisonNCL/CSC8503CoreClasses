#include "StateMachine.h"

using namespace NCL;
using namespace CSC8503;

StateMachine::StateMachine()	{
}

StateMachine::~StateMachine()	{
}

int StateMachine::AddState(StateUpdateFunction s) {
	allStates.emplace_back(s);
	if (activeState == -1) {
		activeState = allStates.size() - 1;
		newState = true;
	}
	return allStates.size() - 1;
}

void StateMachine::AddTransition(int sourceState, int destState, StateTransitionFunction func) {
	Transition transition;
	transition.destState	= destState;
	transition.testFunc		= func;
	stateTransitions.insert(std::make_pair(sourceState, transition));
}

void StateMachine::Update(float dt) {
	if (activeState != -1) {
		allStates[activeState](dt, newState);
		newState = false;
	}

	auto range = stateTransitions.equal_range(activeState);

	for (auto& i = range.first; i != range.second; ++i) {
		if (i->second.testFunc()) {
			activeState = i->second.destState;
			newState = true;
		}
	}
}