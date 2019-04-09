#pragma once
#include "TutorialGame.h"

namespace NCL {
	namespace CSC8503 {
		class NetworkedGame : public TutorialGame {
		public:
			NetworkedGame() {};
			~NetworkedGame() {};

			void UpdateAsClient(float dt);

			void BroadcastSnapshot(bool deltaFrame);

		protected:
			NetworkedGame* game;
			int playerNum;
		};
	}
}