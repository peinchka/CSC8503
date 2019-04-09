#pragma once
#include "../../Common/Vector3.h"
#include "GameObject.h"
namespace NCL {
	using namespace Maths;
	namespace CSC8503 {
		class State {
		public:
			State();
			virtual ~State();
			virtual void Update() = 0; //Pure virtual base class
		};

		typedef void(*StateFunc)(void*);
		class GenericState : public State {
		public:
			GenericState(StateFunc someFunc, void* someData) {
				func = someFunc;
				funcData = someData;
			}
			void Update() override
			{
				if (funcData != nullptr)
				{
					func(funcData);
				}
			}
		protected:
			StateFunc func;
			void* funcData;
		};

		typedef void(*RobotFunc)(GameObject* robot, vector<GameObject*>* balls, Vector3 goalPos, float force, vector<Vector3>* testNodes, int* currentBall);
		class RobotState : public State {
		public:
			RobotState(RobotFunc robotFunc, GameObject* robot, vector<GameObject*>* balls, Vector3 goal, float force, vector<Vector3>* testN, int* currBall)
			{
				robotOb = robot;
				forceMag = force;
				ballObs = balls;
				goalPos = goal;
				func = robotFunc;
				testNodes = testN;
				currentBall = currBall;
			}
			void Update() override
			{
				func(robotOb, ballObs, goalPos, forceMag, testNodes, currentBall);
			}
		protected:
			RobotFunc func;
			GameObject* robotOb;
			vector<GameObject*>* ballObs;
			Vector3 goalPos;
			vector<Vector3>* testNodes;
			int* currentBall;
			float forceMag;
		};

		typedef void(*BlockFunc)(GameObject* block, Vector3 goal, float force);
		class BlockState : public State {
		public:
			BlockState(BlockFunc blockFunc, GameObject* block, Vector3 goal, float force)
			{
				blockOb = block;
				forceMag = force;
				goalPos = goal;
				func = blockFunc;
			}
			void Update() override
			{
				func(blockOb, goalPos, forceMag);
			}
		protected:
			BlockFunc func;
			GameObject* blockOb;
			Vector3 goalPos;
			float forceMag;
		};
	}
}