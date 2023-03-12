typedef struct {
	uint16_t port;
	char ipv4[20]; // xxx.xxx.xxx.xxx
} PARAMETER_t;


typedef struct {
	float quatx;
	float quaty;
	float quatz;
	float quatw;
	float roll;
	float pitch;
	float yaw;
} POSE_t;

