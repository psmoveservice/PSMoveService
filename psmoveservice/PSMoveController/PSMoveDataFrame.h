/// Position and orientation together.
typedef struct psmovePosef_
{
    float qx, qy, qz, qw, px, py, pz;
    // TODO: Quat and Vectors
    
} psmovePosef;

class PSMoveDataFrame {
public:
    psmovePosef pose;
    // TODO: Button states
    double timeInSeconds;
};