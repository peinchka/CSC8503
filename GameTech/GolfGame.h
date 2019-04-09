#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

#include "../CSC8503Common/GameServer.h"
#include "../CSC8503Common/GameClient.h"

namespace NCL {
	namespace CSC8503 {
		class GolfGame {
		public:
			GolfGame();
			~GolfGame();

			virtual void UpdateGame(float dt);

		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();

			void InitGolfWorldFromFile();
			void InitGJKWorld();
			void DanglingRope();
			void SimpleGJKTest();
			void RobotStateMachine();
			void BlockStateMachine1();
			void BlockStateMachine2();
			void PerformPathfinding(string Filename, Vector3 startPos, Vector3 endPos);
			void DisplayPathfinding();
			void RespawnRobot();
			void HighScores();
			void Menu();
			void ReadHighScores();
			void ServeHighScores();
			void BeServer();
			void SubmitScore(string InitialsScoreString);

			bool SelectObject();
			void MoveSelectedObject();

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
			bool getScores;
			bool sendScore;
			bool noSkip;

			float forceMagnitude;
			float time;
			float timer;
			float timeDestroyed;

			GameObject* selectionObject = nullptr;

			OGLMesh*	cubeMesh = nullptr;
			OGLMesh*	sphereMesh = nullptr;
			OGLTexture* floorTex = nullptr;
			OGLTexture* robotTex = nullptr;
			OGLTexture* ballTex = nullptr;
			OGLTexture* blockTex = nullptr;
			OGLTexture* goalTex = nullptr;
			OGLShader*	basicShader = nullptr;

			StateMachine* robotFSM = nullptr;
			StateMachine* blockFSM1 = nullptr;
			StateMachine* blockFSM2 = nullptr;

			GameServer* server = nullptr;
			GameClient* client = nullptr;

			string Filename;
			string timerstring;
			string allScores;
			string entry1, entry2, entry3, entry4, entry5, entry6, entry7, entry8, entry9, entry10;
			string recentestScore;
			vector<Vector3> testNodes;
			vector<GameObject*> balls;
			vector<Vector3> ballOriginalPosition;
			vector<int> ballObjectNumber;
			vector<string> bestScores;
			vector<string> allScoresVec;

			int robotObjectNumber;
			int goalObjectNumber;
			int block1Number;
			int block2Number;
			int updateNum;
			int currentBall;
			int destroyerBall;
			int strikes;
			int finalStrikes1;
			int finalStrikes2;
			int level;
			int menuControl;
			int objectNumber;

			int nodeSize;
			int gridWidth;
			int gridHeight;

			Vector3 robotPosition;
			Vector3 ballPosition;
			Vector3 robotOriginalPosition;
			Vector3 goalPos;
			Vector3 cubeDims;
		};
	}
}

