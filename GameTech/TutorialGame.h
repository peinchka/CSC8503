#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

namespace NCL {
	namespace CSC8503 {
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);
			
		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();

			/*
			These are some of the world/object creation functions I created when testing the functionality
			in the module. Feel free to mess around with them to see different objects being created in different
			test scenarios (constraints, collision types, and so on). 
			*/
			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitGolfWorld1(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitGolfWorldFromFile();
			void InitSphereCollisionTorqueTest();
			void InitCubeCollisionTorqueTest();
			void InitSphereAABBTest();
			void InitGJKWorld();
			void BridgeConstraintTest();
			void SimpleGJKTest();
			void SimpleAABBTest();
			void SimpleAABBTest2();
			void RobotStateMachine();
			void BlockStateMachine1();
			void BlockStateMachine2();
			void PerformPathfinding(string Filename, Vector3 startPos, Vector3 endPos);
			void DisplayPathfinding();
			void RespawnRobot();
			void HighScores();
			void Menu();

			bool SelectObject();
			void MoveSelectedObject();

			GameObject* AddFloorToWorld(const Vector3& position);
			GameObject* AddFloorToGolfWorld(const Vector3& position, const Vector3& floorsize);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f, bool hollow = false, string name = "");
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f, string name = "");
			GameObject* AddRobotToWorld(const Vector3& position, Vector3 dimensions, float inverseMass);
			GameObject* AddGoalToWorld(const Vector3& position, Vector3 dimensions, float inverseMass);

			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool applyGravity;
			bool useBroadPhase;
			bool inSelectionMode;
			bool ballRespawned;
			bool robotRespawned;
			bool robotDestroyed;
			bool robotToBeRespawned;
			bool goalHit;
			bool alreadyUpdated;
			bool pause;

			float forceMagnitude;
			float time;
			float timer;
			float timeDestroyed;

			GameObject* selectionObject = nullptr;

			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* floorTex	= nullptr;
			OGLTexture* robotTex    = nullptr;
			OGLTexture* ballTex     = nullptr;
			OGLTexture* blockTex    = nullptr;
			OGLTexture* goalTex     = nullptr;
			OGLShader*	basicShader = nullptr;

			StateMachine* robotFSM = nullptr;
			StateMachine* blockFSM1 = nullptr;
			StateMachine* blockFSM2 = nullptr;

			
			string Filename;
			string timerstring;
			vector<Vector3> testNodes;
			vector<GameObject*> balls;
			vector<Vector3> ballOriginalPosition;
			vector<int> ballObjectNumber;

			int robotObjectNumber;
			int goalObjectNumber;
			int block1Number;
			int block2Number;
			int updateNum;
			int currentBall;
			int destroyerBall;
			int strikes;
			int level;
			int menuControl;

			Vector3 robotPosition;
			Vector3 ballPosition;
			Vector3 robotOriginalPosition;
			Vector3 goalPos;
		};
	}
}

