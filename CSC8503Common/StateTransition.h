#pragma once
#include "../../Common/Vector3.h"
#include "GameObject.h"
namespace NCL {
	using namespace Maths;
	namespace CSC8503 {
		class GameObject;
		class State;

		class StateTransition
		{
		public:
			virtual bool CanTransition() const = 0;

			State* GetDestinationState()  const {
				return destinationState;
			}

			State* GetSourceState() const {
				return sourceState;
			}

		protected:
			State* sourceState;
			State* destinationState;
		};

		template <class T, class U>
		class GenericTransition : public StateTransition
		{
		public:
			typedef bool(*GenericTransitionFunc)(T, U);
			GenericTransition(GenericTransitionFunc f, T testData, U otherData, State* srcState, State* destState) :
				dataA(testData), dataB(otherData)
			{
				func = f;
				sourceState = srcState;		//
				destinationState = destState;
			}
			~GenericTransition() {}

			virtual bool CanTransition() const override {
				if (func) {
					return func(dataA, dataB);
				}
				return false;
			}

			static bool GreaterThanTransition(T dataA, U dataB) {
				return dataA > dataB;
			}

			static bool LessThanTransition(T dataA, U dataB) {
				return dataA < dataB;
			}

			static bool EqualsTransition(T dataA, U dataB) {
				return dataA == dataB;
			}

			static bool NotEqualsTransition(T dataA, U dataB) {
				return dataA != dataB;
			}

		protected:
			GenericTransitionFunc  func;
			T dataA;
			U dataB;
		};

		class RobotTransition : public StateTransition
		{
		public:
			RobotTransition(GameObject* robotOb, vector<GameObject*> ballObs, State* srcState, State* destState, int* currBall, float hyst)
			{
				robotObject = robotOb;
				ballObjects = ballObs;
				sourceState = srcState;
				destinationState = destState;
				currentBall = currBall;
				hysteresis = hyst;
				counter = 0;
			}
			~RobotTransition() {}

			virtual bool CanTransition() const override
			{
				Vector3 distanceToGoal = ballObjects[*currentBall]->GetTransform().GetWorldPosition() - robotObject->GetTransform().GetWorldPosition();
				if (hysteresis < 10.0f) {
					if (distanceToGoal.Length() < hysteresis) {
//						std::cout << "Trans to state B!\n";
					}
					return (distanceToGoal.Length() < hysteresis);
				}
				else {
					if (distanceToGoal.Length() > hysteresis) {
//						std::cout << "Trans to state A!\n";
					}
					return (distanceToGoal.Length() > hysteresis);
				}
			}

		protected:
			GameObject* robotObject;
			vector<GameObject*> ballObjects;
			int* currentBall;
			float hysteresis;
			int counter;
		};

		class BlockTransition : public StateTransition
		{
		public:
			BlockTransition(GameObject* blockOb, Vector3 goalPos, State* srcState, State* destState)
			{
				blockObject = blockOb;
				goal = goalPos;
				sourceState = srcState;
				destinationState = destState;
			}
			~BlockTransition() {}

			virtual bool CanTransition() const override
			{
				Vector3 distanceToGoal = goal - blockObject->GetTransform().GetWorldPosition();
				return (distanceToGoal.Length() < 30.0f);
			}

		protected:
			GameObject* blockObject;
			Vector3 goal;
		};
	}
}

