#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../../Common/Matrix4.h"
#include "../../Common/Assets.h"
#include "../CSC8503Common/PositionConstraint.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include <fstream>
#include <ctime>

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame() {
	srand((unsigned int)std::time(NULL));
	world = new GameWorld();
	renderer = new GameTechRenderer(*world);
	physics = new PhysicsSystem(*world);

	forceMagnitude = 10000.0f;
	inSelectionMode = false;
	applyGravity = false; //Ball and robot fall through the floor, unless onset onset of gravity is delayed by a small number of physics updates
	useBroadPhase = true;
	pause = true;

	Debug::SetRenderer(renderer);
	level = 0;
	menuControl = 0;
	InitialiseAssets();
}

TutorialGame::~TutorialGame() {
	delete cubeMesh;
	delete sphereMesh;
	delete floorTex;
	delete robotTex;
	delete ballTex;
	delete blockTex;
	delete goalTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;

	delete robotFSM;
	delete blockFSM1;
	delete blockFSM2;
}

void TutorialGame::InitialiseAssets() {
	cubeMesh = new OGLMesh("cube.msh");
	cubeMesh->SetPrimitiveType(GeometryPrimitive::Triangles);
	cubeMesh->UploadToGPU();

	sphereMesh = new OGLMesh("sphere.msh");
	sphereMesh->SetPrimitiveType(GeometryPrimitive::Triangles);
	sphereMesh->UploadToGPU();

	floorTex = (OGLTexture*)TextureLoader::LoadAPITexture("grass.jpg");
	robotTex = (OGLTexture*)TextureLoader::LoadAPITexture("smileyface.png");
	ballTex = (OGLTexture*)TextureLoader::LoadAPITexture("golf.jpg");
	blockTex = (OGLTexture*)TextureLoader::LoadAPITexture("nyan.png");
	goalTex = (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");

	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitWorld();
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(3.0f);
	world->GetMainCamera()->SetFarPlane(4200.0f);
	world->GetMainCamera()->SetPitch(-35.0f);
	world->GetMainCamera()->SetYaw(320.0f);
	world->GetMainCamera()->SetPosition(Vector3(-50, 120, 200));
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();
	InitCamera();

	testNodes.clear();
	balls.clear();
	ballOriginalPosition.clear();
	ballObjectNumber.clear();

	time = 0.0f;
	timer = 0.0f;
	timeDestroyed = 0.0f;
	updateNum = 0;
	ballRespawned = false;
	robotRespawned = false;
	robotDestroyed = false;
	robotToBeRespawned = false;
	goalHit = false;
	alreadyUpdated = false;
	currentBall = 0;
	strikes = 0;
	goalPos = Vector3(0, 0, 0);

	if (level == 0) {
		Filename = "GolfGame1.txt";
	}
	if (level == 1) {
		Filename = "GolfGame2.txt";
	}
	if (level == 2) {
		Filename = "GolfGame5.txt";
	}
	InitGolfWorldFromFile();

	robotFSM = new StateMachine();
	blockFSM1 = new StateMachine();
	blockFSM2 = new StateMachine();
	RobotStateMachine();
	BlockStateMachine1();
	BlockStateMachine2();

	BridgeConstraintTest();
}

void TutorialGame::HighScores() {
	Debug::Print("HIGH SCORES!", Vector2(400, 600), Vector4(1, 0, 0, 1));
}

void TutorialGame::Menu() {
	Debug::Print("FIENDISH GOLF!", Vector2(400, 600), Vector4(1, 0, 0, 1));
	Debug::Print("1 - Play Level 1", Vector2(370, 570), Vector4(0, 0, 1, 1));
	Debug::Print("2 - Play Level 2", Vector2(370, 540), Vector4(0, 0, 1, 1));
	Debug::Print("3 - Read high scores from server", Vector2(150, 510), Vector4(0, 0, 1, 1));
}

void TutorialGame::UpdateGame(float dt) {
	if (menuControl == 0) {
		Menu();
	}
	if (menuControl == 3) {
		HighScores();
	}
	if (level == 0) {
		world->UpdateWorld(dt);
		renderer->Update(dt);
	}
	UpdateKeys();
	if (!pause) {
		if (!inSelectionMode) {
			world->GetMainCamera()->UpdateCamera(dt);
		}
		/*if (applyGravity) {
			Debug::Print("(G)ravity on", Vector2(10, 40));
		}
		else {
			Debug::Print("(G)ravity off", Vector2(10, 40));
		}

		if (useBroadPhase) {
			Debug::Print("Spacia(L) Acc on", Vector2(10, 60));
		}
		else {
			Debug::Print("Spacia(L) Acc off", Vector2(10, 60));
		}*/

		SelectObject();
		MoveSelectedObject();

		world->UpdateWorld(dt);
		renderer->Update(dt);
		physics->Update(dt);

		Debug::Print("Strikes:" + std::to_string(strikes), Vector2(900, 650), Vector4(0.75, 0.75, 0.75, 1));

		timer = timer + dt;
		timerstring = std::to_string((int)timer);
		Debug::Print("Time:" + timerstring, Vector2(10, 650), Vector4(0.75, 0.75, 0.75, 1));
		if ((int)timer % 10 == 0 && !alreadyUpdated) {
			currentBall = rand() % ((int)ballObjectNumber.size());
			alreadyUpdated = true;
		}

		if ((int)timer % 10 == 2) {
			alreadyUpdated = false;
		}

		time = time + dt;
		if (time > 0.2f) {
			testNodes.clear();
			robotPosition = world->GetGameObject(robotObjectNumber)->GetTransform().GetWorldPosition();
			ballPosition = world->GetGameObject(ballObjectNumber[currentBall])->GetTransform().GetWorldPosition();
			robotPosition.y = 0;
			ballPosition.y = 0;
			PerformPathfinding(Filename, robotPosition, ballPosition);
			time = time - 0.2;
		}
		//	Debug::DrawLine(robotPosition, ballPosition, Vector4(1, 1, 1, 1));
		//	DisplayPathfinding();

		for (int i = 0; i < (int)ballObjectNumber.size(); ++i) {
			if (CollisionDetection::BallGoalIntersection(world->GetGameObject(ballObjectNumber[i]), world->GetGameObject(goalObjectNumber))) {
				goalHit = true;
				world->GetGameObject(goalObjectNumber)->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				updateNum = 0;
			}

			if (CollisionDetection::BallRobotIntersection(world->GetGameObject(ballObjectNumber[i]), world->GetGameObject(robotObjectNumber))) {
				float ballVelocity = (world->GetGameObject(ballObjectNumber[i])->GetPhysicsObject()->GetLinearVelocity()).Length();
				if ((ballVelocity > 400.0f) && (robotDestroyed == false)) {
					robotDestroyed = true;
					robotToBeRespawned = true;
					world->GetGameObject(robotObjectNumber)->GetTransform().SetWorldPosition(Vector3(0, 10000, 0));
					timeDestroyed = timer;
					destroyerBall = ballObjectNumber[i];
					strikes -= 3;
					updateNum = 0;
				}
			}

			ballPosition = world->GetGameObject(ballObjectNumber[i])->GetTransform().GetWorldPosition();
			if (ballPosition.y < -400) {
				ballRespawned = true;
				applyGravity = false;
				physics->UseGravity(applyGravity);
				updateNum = 0;
				world->GetGameObject(ballObjectNumber[i])->GetTransform().SetWorldPosition(ballOriginalPosition[i]);
			}
		}

		if (robotToBeRespawned == true && (timer - timeDestroyed) > 15.0f) {
			robotRespawned = true;
			applyGravity = false;
			physics->UseGravity(applyGravity);
			updateNum = 0;
			RespawnRobot();
			robotToBeRespawned = false;
		}

		robotPosition = world->GetGameObject(robotObjectNumber)->GetTransform().GetWorldPosition();
		if (robotPosition.y < -400) {
			robotRespawned = true;
			applyGravity = false;
			physics->UseGravity(applyGravity);
			updateNum = 0;
			world->GetGameObject(robotObjectNumber)->GetTransform().SetWorldPosition(robotOriginalPosition);
		}

		robotFSM->Update();
		blockFSM1->Update();
		blockFSM2->Update();

		updateNum++;
		if (updateNum == 10) {  // Used to prevent ball and robot from falling through the floor, which occurs when gravity is enabled from the outset
			applyGravity = true;
			physics->UseGravity(applyGravity);
		}

		if (goalHit == true && updateNum < 200) {
			Debug::Print("Goal hit! Proceed to next level!", Vector2(10, 120), Vector4(0.75, 0.75, 0.75, 1));
			if (updateNum == 199) {
				pause = false;
				level = 2;
				menuControl = 10;
				delete robotFSM;
				delete blockFSM1;
				delete blockFSM2;
				InitWorld();
				
			}
		}

		if (ballRespawned == true && updateNum < 200) {
			Debug::Print("Ball respawned!", Vector2(10, 80), Vector4(0.75, 0.75, 0.75, 1));
			if (updateNum == 199) {
				ballRespawned = false;
			}
		}

		if (robotRespawned == true && updateNum < 200) {
			Debug::Print("Robot respawned!", Vector2(10, 100), Vector4(0.75, 0.75, 0.75, 1));
			if (updateNum == 199) {
				robotRespawned = false;
			}
		}

		if (robotDestroyed == true && updateNum < 200) {
			Debug::Print("Robot destroyed by ball " + std::to_string(destroyerBall), Vector2(10, 160), Vector4(0.75, 0.75, 0.75, 1));
			Debug::Print("Reward = strikes reduced by 3!", Vector2(10, 140), Vector4(0.75, 0.75, 0.75, 1));
			if (updateNum == 199) {
				robotDestroyed = false;
			}
		}
	}
	else {
		if (level != 0) {
			Debug::Print("Press P to unpause!", Vector2(350, 400), Vector4(0.75, 0.75, 0.75, 1));
			Debug::Print("Strikes:" + std::to_string(strikes), Vector2(900, 650), Vector4(0.75, 0.75, 0.75, 1));
			Debug::Print("Time:" + timerstring, Vector2(10, 650), Vector4(0.75, 0.75, 0.75, 1));
		}
	}

	Debug::FlushRenderables();
	renderer->Render();
}

void TutorialGame::RobotStateMachine() {
	float force = 1.0f;

	GameObject* robot = world->GetGameObject(robotObjectNumber);
	
	for (int i = 0; i < (int)ballObjectNumber.size(); ++i) {
		balls.push_back(world->GetGameObject(ballObjectNumber[i]));
	}

	RobotFunc AFunc = [](GameObject* robot, vector<GameObject*>* balls, Vector3 goalPos, float force, vector<Vector3>* testNodes, int* currentBall) {
		Vector3 robotDirection;
		if ((int)(*testNodes).size() == 0) {
			robotDirection = ((*balls)[*currentBall])->GetTransform().GetWorldPosition() - robot->GetTransform().GetWorldPosition();
		}
		else {
			if ((int)(*testNodes).size() > 1) {
				robotDirection = (*testNodes)[1] - robot->GetTransform().GetWorldPosition();
			}
			else {
				robotDirection = (*testNodes)[0] - robot->GetTransform().GetWorldPosition();
			}
		}
		robotDirection.y = 0; 
		robot->GetPhysicsObject()->AddForceAtPosition(robotDirection.Normalised() * 350.0f * force, robot->GetTransform().GetWorldPosition());
	};
	RobotFunc BFunc = [](GameObject* robot, vector<GameObject*>* balls, Vector3 goalPos, float force, vector<Vector3>* testNodes, int* currentBall) {
		Vector3 ranVec = Vector3(rand() % 200 - 100, 0, rand() % 200 - 100);							//Random vector in the y = 0 plane
		Vector3 kickDirection = ((*balls)[*currentBall])->GetTransform().GetWorldPosition() - goalPos;	//Direction vector pointing from goal to ball (ideal kickDirection)
		float testDir = Vector3::Dot(ranVec, kickDirection);											//testDir is positive if random vector doesn't point towards goal
		while (testDir < 0) {															
			ranVec = Vector3(rand() % 200 - 100, 0, rand() % 200 - 100);								//Generate new random vector until one is found that points away from goal
			testDir = Vector3::Dot(ranVec, kickDirection); 
		}
		kickDirection = (ranVec).Normalised();
		(*balls)[*currentBall]->GetPhysicsObject()->AddForceAtPosition(kickDirection * 1000.0f * force, (*balls)[*currentBall]->GetTransform().GetWorldPosition());
	};

	RobotState* stateA = new RobotState(AFunc, robot, &balls, goalPos, force, &testNodes, &currentBall);
	RobotState* stateB = new RobotState(BFunc, robot, &balls, goalPos, force, &testNodes, &currentBall);

	robotFSM->AddState(stateA);
	robotFSM->AddState(stateB);

	RobotTransition* transitionA = new RobotTransition(robot, balls, stateA, stateB, &currentBall, 8.0f);
	RobotTransition* transitionB = new RobotTransition(robot, balls, stateB, stateA, &currentBall, 12.0f);

	robotFSM->AddTransition(transitionA);
	robotFSM->AddTransition(transitionB);
}

void TutorialGame::BlockStateMachine1() {

	Vector3 goalPos1 = Vector3(200, 100, 300);
	Vector3 goalPos2 = Vector3(200, 100, 30);
	Vector3 goalPos3 = Vector3(400, 30, 30);

	float force = 25.0f;

	GameObject* block = world->GetGameObject(block1Number);

	BlockFunc AFunc = [](GameObject* block, Vector3 goalPos1, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos1 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), Vector3(0, 0, 0));
	};
	BlockFunc BFunc = [](GameObject* block, Vector3 goalPos2, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos2 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), block->GetTransform().GetWorldPosition());
	};
	BlockFunc CFunc = [](GameObject* block, Vector3 goalPos3, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos3 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), block->GetTransform().GetWorldPosition());
	};

	BlockState* stateA = new BlockState(AFunc, block, goalPos1, force);
	BlockState* stateB = new BlockState(BFunc, block, goalPos2, force);
	BlockState* stateC = new BlockState(CFunc, block, goalPos3, force);
	blockFSM1->AddState(stateA);
	blockFSM1->AddState(stateB);
	blockFSM1->AddState(stateC);
	BlockTransition* transitionA = new BlockTransition(block, goalPos1, stateA, stateB);
	BlockTransition* transitionB = new BlockTransition(block, goalPos2, stateB, stateC);
	BlockTransition* transitionC = new BlockTransition(block, goalPos3, stateC, stateA);

	blockFSM1->AddTransition(transitionA);
	blockFSM1->AddTransition(transitionB);
	blockFSM1->AddTransition(transitionC);
}

void TutorialGame::BlockStateMachine2() {

	Vector3 goalPos1 = Vector3(200, 100, -300);
	Vector3 goalPos2 = Vector3(200, 100, -30);
	Vector3 goalPos3 = Vector3(400, 30, -30);

	float force = 25.0f;

	GameObject* block = world->GetGameObject(block2Number);

	BlockFunc AFunc = [](GameObject* block, Vector3 goalPos1, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos1 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), Vector3(0, 0, 0));
	};
	BlockFunc BFunc = [](GameObject* block, Vector3 goalPos2, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos2 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), block->GetTransform().GetWorldPosition());
	};
	BlockFunc CFunc = [](GameObject* block, Vector3 goalPos3, float force) {
		block->GetPhysicsObject()->AddForceAtPosition((goalPos3 - block->GetTransform().GetWorldPosition()) * force + Vector3(0, 981, 0), block->GetTransform().GetWorldPosition());
	};

	BlockState* stateA = new BlockState(AFunc, block, goalPos1, force);
	BlockState* stateB = new BlockState(BFunc, block, goalPos2, force);
	BlockState* stateC = new BlockState(CFunc, block, goalPos3, force);
	blockFSM2->AddState(stateA);
	blockFSM2->AddState(stateB);
	blockFSM2->AddState(stateC);
	BlockTransition* transitionA = new BlockTransition(block, goalPos1, stateA, stateB);
	BlockTransition* transitionB = new BlockTransition(block, goalPos2, stateB, stateC);
	BlockTransition* transitionC = new BlockTransition(block, goalPos3, stateC, stateA);

	blockFSM2->AddTransition(transitionA);
	blockFSM2->AddTransition(transitionB);
	blockFSM2->AddTransition(transitionC);
}

void TutorialGame::PerformPathfinding(string Filename, Vector3 startPos, Vector3 endPos) {
	NavigationGrid grid(Filename);
	NavigationPath outPath;
	bool found = grid.FindPath(startPos, endPos, outPath);
	Vector3 pos;
	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
	}
}

void TutorialGame::DisplayPathfinding() {
	for (int i = 1; i < (int)testNodes.size(); ++i) {
		Vector3 a = testNodes[i - 1];
		Vector3 b = testNodes[i];
		Debug::DrawLine(a, b, Vector4(0, 1, 0, 1));
	}
	
	/*Debug::DrawLine(Vector3(0, 0, 0), Vector3(500, 0, 0), Vector4(1, 0, 0, 1));
	Debug::DrawLine(Vector3(0, 0, 0), Vector3(0, 500, 0), Vector4(0, 1, 0, 1));
	Debug::DrawLine(Vector3(0, 0, 0), Vector3(0, 0, 500), Vector4(0, 0, 1, 1));*/
}

void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_R)) {
		menuControl = 10;
		delete robotFSM;
		delete blockFSM1;
		delete blockFSM2;
		InitWorld();
		selectionObject = nullptr;
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_P)) {
		if (level != 0) {
			pause = !pause;
			if (pause == true) {
				menuControl = 0;
			}
			else{
				menuControl = 10;
			}
		}
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_RETURN)) {
		if (pause == true) {
			menuControl = 0;
		}
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_1) || Window::GetKeyboard()->KeyPressed(KEYBOARD_NUMPAD1)) {
		if (pause == true) {
			pause = false;
			level = 1;
			menuControl = 10;
			delete robotFSM;
			delete blockFSM1;
			delete blockFSM2;
			InitWorld();
			selectionObject = nullptr;
		}
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_2) || Window::GetKeyboard()->KeyPressed(KEYBOARD_NUMPAD2)) {
		if (pause == true) {
			pause = false;
			level = 2;
			menuControl = 10;
			delete robotFSM;
			delete blockFSM1;
			delete blockFSM2;
			InitWorld();
			selectionObject = nullptr;
		}
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_3) || Window::GetKeyboard()->KeyPressed(KEYBOARD_NUMPAD3)) {
		if (pause == true) {
			menuControl = 3;
		}
	}



	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	//if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::KEYBOARD_G)) {
	//	applyGravity = !applyGravity; //Toggle gravity!
	//	physics->UseGravity(applyGravity);
	//}

	//if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::KEYBOARD_L)) {
	//	useBroadPhase = !useBroadPhase; //Toggle spacial acceleration!
	//	physics->UseBroadPhase(useBroadPhase);
	//}

	if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::KEYBOARD_B)) {
		Vector3 camPos = world->GetGameObject(ballObjectNumber[0])->GetTransform().GetWorldPosition();
		camPos.y = 100;
		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(-90.0f);
	}

	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	/*if (Window::GetKeyboard()->KeyPressed(KEYBOARD_F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_F8)) {
		world->ShuffleObjects(false);
	}*/
	//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KEYBOARD_LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-100, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(100, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 100, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -100, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -100));
		}

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 100));
		}
	}
}

void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(8, 8, 8);
	
	float invCubeMass = 5.0f; // how heavy the middle pieces are
	int numLinks = 10;
	float maxDistance = 30; // constraint distance
	float cubeDistance = 20; // distance between links
	
	Vector3 startPos = Vector3(0, 200, -500);
	
	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), cubeSize, 0);
	
	GameObject* previous = start;
	
	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint * constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

GameObject* TutorialGame::AddFloorToGolfWorld(const Vector3& position, const Vector3& floorSize) {
	GameObject* floor = new GameObject();

	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetWorldScale(floorSize);
	floor->GetTransform().SetWorldPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, floorTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	floor->GetPhysicsObject()->SetElasticity(0.1f);

	world->AddGameObject(floor);

	return floor;
}

GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(1000, 10, 1000);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetWorldScale(floorSize);
	floor->GetTransform().SetWorldPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, floorTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	floor->GetPhysicsObject()->SetElasticity(0.1f);

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass, bool hollow, string name) {
	GameObject* sphere = new GameObject(name);

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);
	sphere->GetTransform().SetWorldScale(sphereSize);
	sphere->GetTransform().SetWorldPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, ballTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	
	sphere->GetPhysicsObject()->InitSphereInertia(hollow);

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass, string name) {
	GameObject* cube = new GameObject(name);

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, blockTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();
	cube->GetPhysicsObject()->SetElasticity(1);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddRobotToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, robotTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);

	cube->GetPhysicsObject()->InitCubeInertia(false);
	cube->GetPhysicsObject()->SetElasticity(0.8f);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddGoalToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform().SetWorldPosition(position);
	cube->GetTransform().SetWorldScale(dimensions);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, goalTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);

	cube->GetPhysicsObject()->InitCubeInertia(false);
	cube->GetPhysicsObject()->SetElasticity(0.8f);

	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, radius, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f, false, "Row" + std::to_string(z) + ", Column" + std::to_string(x));
		}
	}
}

void TutorialGame::RespawnRobot() {
	std::ifstream infile(Assets::DATADIR + Filename);

	int nodeSize;
	int gridWidth;
	int gridHeight;

	infile >> nodeSize;
	infile >> gridWidth;
	infile >> gridHeight;

	Vector3 cubeDims = Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2);
	Vector3 position;

	vector<int> worldmap;
	char element;
	for (int z = 0; z < gridHeight; ++z) {
		for (int x = 0; x < gridWidth; ++x) {
			infile >> element;
			worldmap.push_back(element);
		}
	}
	bool validSpawnPosition = false;
	while (validSpawnPosition == false) {
		int z = rand() % gridHeight;
		int x = rand() % gridWidth;
		if (worldmap[x + z * gridWidth] == '.') {
			position = Vector3(x * 2 * cubeDims.x, 4, z * 2 * cubeDims.z);
			validSpawnPosition = true;
			for (int i = 0; i < (int)ballObjectNumber.size(); ++i) {
				ballPosition = world->GetGameObject(i)->GetTransform().GetWorldPosition();
				if (ballPosition.x == position.x && ballPosition.y == position.y) {
					validSpawnPosition = false;
				}	
			}
		}
	}
	world->GetGameObject(robotObjectNumber)->GetTransform().SetWorldPosition(position);
}

void TutorialGame::InitGolfWorldFromFile() {
	std::ifstream infile(Assets::DATADIR + Filename);

	int nodeSize;
	int gridWidth;
	int gridHeight;

	infile >> nodeSize;
	infile >> gridWidth;
	infile >> gridHeight;

	int objectNumber = 0;

	AddFloorToGolfWorld(Vector3(gridWidth * 10 - 10, -20, gridHeight * 10 - 10)/2, Vector3(gridWidth * 10, 20, gridHeight * 10)/2);

	Vector3 cubeDims = Vector3(nodeSize/2, nodeSize/2, nodeSize/2);
	Vector3 position;

	vector<int> worldmap;
	char element;
	for (int z = 0; z < gridHeight; ++z) {
		for (int x = 0; x < gridWidth; ++x) {
			infile >> element;
			worldmap.push_back(element);
		}
	}
	for (int z = 0; z < gridHeight; ++z) {
		for (int x = 0; x < gridWidth; ++x) {
			if (worldmap[x + z * gridWidth] == '.') {
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'r') { //Check for robot, r
				position = Vector3(x * 2 * cubeDims.x, 4, z * 2 * cubeDims.z);
				AddRobotToWorld(position, Vector3(3, 5, 3), 0.2f);
				objectNumber++;
				robotObjectNumber = objectNumber;
				robotOriginalPosition = position;
				robotOriginalPosition.y = 20;
				worldmap[x + z * gridWidth] = '.';
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'b') { //Check for ball, b
				position = Vector3(x * 2 * cubeDims.x, 3, z * 2 * cubeDims.z);
				AddSphereToWorld(position, 3.0f, 1.0f, false);
				objectNumber++;
				ballObjectNumber.push_back(objectNumber);
				position.y = 20;
				ballOriginalPosition.push_back(position);
				worldmap[x + z * gridWidth] = '.';
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'g') { //Check for goal, g
				position = Vector3(x * 2 * cubeDims.x, 3, z * 2 * cubeDims.z);
				AddGoalToWorld(position, Vector3(3, 5, 3), 0.0f);
				objectNumber++;
				goalObjectNumber = objectNumber;
				goalPos = position;
				worldmap[x + z * gridWidth] = '.';
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'p') { //Advanced physics object, to be drawn manually
				continue;
			}
			if ((worldmap[x + z * gridWidth] != 'x') && (worldmap[x + z * gridWidth] != 'y')) {
				char test = worldmap[x + z * gridWidth];
				int blockHeight = (int)test - 48;
				int i = z;
				int znumOfBlocks = 1;
				while (i / (gridHeight - 1) < 1) {
					i++;
					if (test == worldmap[x + i * gridWidth]) {
						znumOfBlocks++;
						worldmap[x + i * gridWidth] = '.';
					}
					else {
						break;
					}
				}
				if (znumOfBlocks > 1) {
					position = Vector3(x * 2 * cubeDims.x, blockHeight * cubeDims.y, (z * 2 + znumOfBlocks - 1) * cubeDims.z);
					AddCubeToWorld(position, Vector3(cubeDims.x, blockHeight * cubeDims.y, znumOfBlocks * cubeDims.z), 0.0f);
					objectNumber++;
					worldmap[x + z * gridWidth] = '.';
				}
			}
		}
	}

	for (int z = 0; z < gridHeight; ++z) {
		for (int x = 0; x < gridWidth; ++x) {
			if (worldmap[x + z * gridWidth] == '.') {
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'p') { //Advanced physics object, to be drawn manually
				continue;
			}
			if (worldmap[x + z * gridWidth] == 'x') {
				char test = worldmap[x + z * gridWidth];
				position = Vector3(x * 2 * cubeDims.x, cubeDims.y, z * 2 * cubeDims.z);
				AddCubeToWorld(position, cubeDims, 0.0f);
				objectNumber++;
			}
			if (worldmap[x + z * gridWidth] == 'y') {
				char test = worldmap[x + z * gridWidth];
				position = Vector3(x * 2 * cubeDims.x, cubeDims.y, z * 2 * cubeDims.z);
				AddCubeToWorld(position, cubeDims, 1.0f);
				objectNumber++;
			}
			if ((worldmap[x + z * gridWidth] != 'x') && (worldmap[x + z * gridWidth] != 'y')) {
				char test = worldmap[x + z * gridWidth];
				int blockHeight = (int)test - 48;
				int i = x;
				int xnumOfBlocks = 1;
				while (i / (gridWidth - 1) < 1) {
					i++;
					if (test == worldmap[i + z * gridWidth]) {
						xnumOfBlocks++;
					}
					else {

						break;
					}
				}
				if (xnumOfBlocks > 1) {
					position = Vector3((x * 2 + xnumOfBlocks - 1) * cubeDims.x, blockHeight * cubeDims.y, z * 2 * cubeDims.z);
					AddCubeToWorld(position, Vector3(xnumOfBlocks * cubeDims.x, blockHeight * cubeDims.y, cubeDims.z), 0.0f);
					objectNumber++;
					x += xnumOfBlocks - 1;
				}
				else {
					position = Vector3(x * 2 * cubeDims.x, blockHeight * cubeDims.y, z * 2 * cubeDims.z);
					AddCubeToWorld(position, Vector3(cubeDims.x, blockHeight * cubeDims.y, cubeDims.z), 0.0f);
					objectNumber++;
				}
			}
		}
	}
	AddCubeToWorld(Vector3(200, 100, 300), Vector3(cubeDims.x, cubeDims.y, cubeDims.z), 1.0f);
	objectNumber++;
	block1Number = objectNumber;
	AddCubeToWorld(Vector3(400, 30, -30), Vector3(cubeDims.x, cubeDims.y, cubeDims.z), 1.0f);
	objectNumber++;
	block2Number = objectNumber;

}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 10.0f;
	Vector3 cubeDims = Vector3(10, 10, 10);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, cubeDims.y, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims, 1.0f, "Row" + std::to_string(z) + ", Column" + std::to_string(x));
			}
			else {
				AddSphereToWorld(position, sphereRadius, 1.0f, false, "Row" + std::to_string(z) + ", Column" + std::to_string(x));
			}
		}
	}
	AddFloorToWorld(Vector3(0, -100, 0));
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, cubeDims.y, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f, "Row" + std::to_string(z) + ", Column" + std::to_string(x));
		}
	}
	AddFloorToWorld(Vector3(10, -100, 1));
}

void TutorialGame::InitGolfWorld1(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	AddFloorToWorld(Vector3(0, -10, 0));
	AddSphereToWorld(Vector3(100, 15, -30), 6.0f, 1.0f, false);
	AddRobotToWorld(Vector3(-100, 15, -30), Vector3(8, 8, 8), 0.2f);
	ballObjectNumber[1] = 1;
	robotObjectNumber = 2;

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, cubeDims.y, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f, "Row" + std::to_string(z) + ", Column" + std::to_string(x));
		}
	}
}

void TutorialGame::InitSphereCollisionTorqueTest() {
	AddSphereToWorld(Vector3(15, 0, 0), 10.0f);
	AddSphereToWorld(Vector3(-25, 0, 0), 10.0f);
	AddSphereToWorld(Vector3(-50, 0, 0), 10.0f);

	AddCubeToWorld(Vector3(-50, 0, -50), Vector3(60, 10, 10), 10.0f);

	AddFloorToWorld(Vector3(0, -100, 0));
}

void TutorialGame::InitCubeCollisionTorqueTest() {
	Vector3 cubeSize(10, 10, 10);
	AddCubeToWorld(Vector3(15, 0, 0), cubeSize, 10.0f);
	AddCubeToWorld(Vector3(-25, 0, 0), cubeSize, 10.0f);
	AddCubeToWorld(Vector3(-50, 0, 0), cubeSize, 10.0f);

	AddCubeToWorld(Vector3(-50, 0, -50), Vector3(60, 10, 10), 10.0f);

	AddFloorToWorld(Vector3(0, -100, 0));
}

void TutorialGame::InitSphereAABBTest() {
	Vector3 cubeSize(10, 10, 10);

	AddCubeToWorld(Vector3(0, 0, 0), cubeSize, 10.0f);
	AddSphereToWorld(Vector3(2, 0, 0), 5.0f, 10.0f);
}

void TutorialGame::InitGJKWorld() {
	Vector3 dimensions(20, 2, 10);
	float inverseMass = 10.0f;

	for (int i = 0; i < 2; ++i) {
		GameObject* cube = new GameObject();

		OBBVolume* volume = new OBBVolume(dimensions);

		cube->SetBoundingVolume((CollisionVolume*)volume);

		cube->GetTransform().SetWorldPosition(Vector3(0, 0, 0));
		cube->GetTransform().SetWorldScale(dimensions);

		if (i == 1) {
			cube->GetTransform().SetLocalOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), 90.0f));
		}

		cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, blockTex, basicShader));
		cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

		cube->GetPhysicsObject()->SetInverseMass(inverseMass);
		cube->GetPhysicsObject()->InitCubeInertia();

		world->AddGameObject(cube);
	}
}

//void TutorialGame::BridgeConstraintTest() {
//	float sizeMultiplier = 1.0f;
//
//	Vector3 cubeSize = Vector3(8, 8, 8) * sizeMultiplier;
//
//	int numLinks = 5;
//
//	GameObject* start = AddCubeToWorld(Vector3(0, 0, 0), cubeSize, 0);
//
//	GameObject* end = AddCubeToWorld(Vector3((numLinks + 2) * 20 * sizeMultiplier, 0, 0), cubeSize, 0);
//
//	GameObject* previous = start;
//
//	for (int i = 0; i < numLinks; ++i) {
//		GameObject* block = AddCubeToWorld(Vector3((i + 1) * 20 * sizeMultiplier, 0, 0), cubeSize, 10.0f);
//		PositionConstraint* constraint = new PositionConstraint(previous, block, 30.0f);
//		world->AddConstraint(constraint);
//		previous = block;
//	}
//
//	PositionConstraint* constraint = new PositionConstraint(previous, end, 30.0f);
//	world->AddConstraint(constraint);
//}

void TutorialGame::SimpleGJKTest() {
	Vector3 dimensions = Vector3(5, 5, 5);
	Vector3 floorDimensions = Vector3(100, 2, 100);

	GameObject* fallingCube = AddCubeToWorld(Vector3(0, 20, 0), dimensions, 10.0f);
	GameObject* newFloor = AddCubeToWorld(Vector3(0, 0, 0), floorDimensions, 0.0f);

	delete fallingCube->GetBoundingVolume();
	delete newFloor->GetBoundingVolume();

	fallingCube->SetBoundingVolume((CollisionVolume*)new OBBVolume(dimensions));
	newFloor->SetBoundingVolume((CollisionVolume*)new OBBVolume(floorDimensions));

}

void TutorialGame::SimpleAABBTest() {
	Vector3 dimensions = Vector3(5, 5, 5);
	Vector3 floorDimensions = Vector3(100, 2, 100);

	GameObject* newFloor = AddCubeToWorld(Vector3(0, 0, 0), floorDimensions, 0.0f);
	GameObject* fallingCube = AddCubeToWorld(Vector3(10, 20, 0), dimensions, 10.0f);
}

void TutorialGame::SimpleAABBTest2() {
	Vector3 dimensions = Vector3(5, 5, 5);
	Vector3 floorDimensions = Vector3(8, 2, 8);

	GameObject* newFloor = AddCubeToWorld(Vector3(0, 0, 0), floorDimensions, 0.0f);
	GameObject* fallingCube = AddCubeToWorld(Vector3(8, 20, 0), dimensions, 10.0f);
}

/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around.

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KEYBOARD_Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		renderer->DrawString("Press Q to change to camera mode!", Vector2(10, 0));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::MOUSE_LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(10, 0));
	}
	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void TutorialGame::MoveSelectedObject() {
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 5000.0f;
	if (forceMagnitude > 50000) {
		forceMagnitude = 50000;
	}
	if (forceMagnitude < -50000) {
		forceMagnitude = -50000;
	}
	renderer->DrawString("Click Force:" + std::to_string((int)(forceMagnitude / 1000)), Vector2(10, 20));

	if (!selectionObject) {
		return;//we haven't selected anything!
	}

	/*if (Window::GetKeyboard()->KeyDown(KEYBOARD_5)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 1, 0) * forceMagnitude, Vector3(0, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KEYBOARD_6)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, -1, 0) * forceMagnitude, Vector3(0, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KEYBOARD_3)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(-1, 0, 0) * forceMagnitude, Vector3(0, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KEYBOARD_4)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(1, 0, 0) * forceMagnitude, Vector3(0, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KEYBOARD_1)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 0, -1) * forceMagnitude, Vector3(0, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KEYBOARD_2)) {
		selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 0, 1) * forceMagnitude, Vector3(0, 0, 0));
	}*/

	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::MOUSE_RIGHT)) {
		//Vector3 ballPos = world->GetGameObject(ballObjectNumber[0])->GetTransform().GetWorldPosition();
		//Vector3 camPos = world->GetMainCamera()->GetPosition();
		//Vector3 forceDir = (ballPos - camPos);
		//forceDir.y = 0.0f;
		//forceDir = forceDir.Normalised();
		//world->GetGameObject(ballObjectNumber[0])->GetPhysicsObject()->AddForceAtPosition(forceDir * forceMagnitude, ballPos);

		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			/*if (Window::GetKeyboard()->KeyDown(KEYBOARD_J)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(-1, 0, 0) * forceMagnitude, closestCollision.collidedAt);
			}

			if (Window::GetKeyboard()->KeyDown(KEYBOARD_L)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(1, 0, 0) * forceMagnitude, closestCollision.collidedAt);
			}

			if (Window::GetKeyboard()->KeyDown(KEYBOARD_I)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 1, 0) * forceMagnitude, closestCollision.collidedAt);
			}

			if (Window::GetKeyboard()->KeyDown(KEYBOARD_K)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, -1, 0) * forceMagnitude, closestCollision.collidedAt);
			}

			if (Window::GetKeyboard()->KeyDown(KEYBOARD_Y)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 0, -1) * forceMagnitude, closestCollision.collidedAt);
			}

			if (Window::GetKeyboard()->KeyDown(KEYBOARD_H)) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(Vector3(0, 0, 1) * forceMagnitude, closestCollision.collidedAt);
			}*/

			if (closestCollision.node == selectionObject) {
				if (world->GetGameObject(ballObjectNumber[0]) == selectionObject) {
					if (forceMagnitude != 0) {
						Vector3 rayDirection = ray.GetDirection();
						rayDirection.y = 0.0f;
						selectionObject->GetPhysicsObject()->AddForceAtPosition(rayDirection * forceMagnitude, closestCollision.collidedAt);
						strikes++;
					}
				}
			}
		}
	}
}