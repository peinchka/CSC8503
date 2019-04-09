#pragma once

namespace NCL { // keep track of yournamespaces !
	enum class VolumeType {
		AABB = 1, OBB = 2, Sphere = 4, Mesh = 8,
		Compound = 16, Invalid = 256
	};
	class CollisionVolume {
		public:
			CollisionVolume() {
				type = VolumeType::Invalid;
			}
			~CollisionVolume() {}
			VolumeType type;
	};
} // end of namespace !