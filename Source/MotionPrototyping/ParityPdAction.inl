// Joint target = offset + scale * action, derived from the training rig's joint ranges
// (humanoid_from_ue.xml) by the training environment's rule. Must match training exactly.

inline constexpr int32 ParityPdDofCount = 28;

const double GPdActionOffset[ParityPdDofCount] = {
	    0.00000000,  // abdomen_x
	    0.00000000,  // abdomen_y
	    0.00000000,  // abdomen_z
	    0.00000000,  // neck_x
	    0.00000000,  // neck_y
	    0.00000000,  // neck_z
	    0.00000000,  // right_shoulder_x
	    0.00000000,  // right_shoulder_y
	    0.00000000,  // right_shoulder_z
	   -1.39626340,  // right_elbow
	    0.00000000,  // left_shoulder_x
	    0.00000000,  // left_shoulder_y
	    0.00000000,  // left_shoulder_z
	   -1.39626340,  // left_elbow
	    0.00000000,  // right_hip_x
	    0.00000000,  // right_hip_y
	    0.00000000,  // right_hip_z
	    1.39626340,  // right_knee
	    0.00000000,  // right_ankle_x
	    0.00000000,  // right_ankle_y
	    0.00000000,  // right_ankle_z
	    0.00000000,  // left_hip_x
	    0.00000000,  // left_hip_y
	    0.00000000,  // left_hip_z
	    1.39626340,  // left_knee
	    0.00000000,  // left_ankle_x
	    0.00000000,  // left_ankle_y
	    0.00000000,  // left_ankle_z
};

const double GPdActionScale[ParityPdDofCount] = {
	    1.88495559,  // abdomen_x
	    1.88495559,  // abdomen_y
	    1.88495559,  // abdomen_z
	    1.25663706,  // neck_x
	    1.25663706,  // neck_y
	    1.25663706,  // neck_z
	    3.14159265,  // right_shoulder_x
	    3.14159265,  // right_shoulder_y
	    3.14159265,  // right_shoulder_z
	    1.95476876,  // right_elbow
	    3.14159265,  // left_shoulder_x
	    3.14159265,  // left_shoulder_y
	    3.14159265,  // left_shoulder_z
	    1.95476876,  // left_elbow
	    2.93215314,  // right_hip_x
	    2.93215314,  // right_hip_y
	    2.93215314,  // right_hip_z
	    1.95476876,  // right_knee
	    1.15191731,  // right_ankle_x
	    1.15191731,  // right_ankle_y
	    1.15191731,  // right_ankle_z
	    2.93215314,  // left_hip_x
	    2.93215314,  // left_hip_y
	    2.93215314,  // left_hip_z
	    1.95476876,  // left_knee
	    1.15191731,  // left_ankle_x
	    1.15191731,  // left_ankle_y
	    1.15191731,  // left_ankle_z
};

