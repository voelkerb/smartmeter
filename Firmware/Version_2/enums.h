
// Internal state machine for sampling
enum class SampleState{IDLE, SAMPLE, SAMPLE_LATCHED};

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or RMS
enum class Measures{VI, VI_L1, VI_L2, VI_L3, VI_RMS, PQ};
enum class StreamType{USB, TCP, UDP, TCP_RAW, MQTT};