#pragma once

namespace NCL {
	namespace CSC8503 {
		using StateUpdateFunction		= std::function<void(float, bool)>;
		using StateTransitionFunction	= std::function<bool()>;

		class StateMachine	{
		public:
			StateMachine();
			virtual ~StateMachine(); //made it virtual!

			int  AddState(StateUpdateFunction s);

			void AddTransition(int sourceState, int destState, StateTransitionFunction func);

			virtual void Update(float dt); //made it virtual!

		protected:
			int activeState = -1;
			bool newState = false;

			std::vector<StateUpdateFunction> allStates;

			struct Transition {
				int destState;
				StateTransitionFunction testFunc;
			};

			std::multimap<int, Transition> stateTransitions;
		};
	}
}