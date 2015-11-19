/// Position and orientation together.
typedef struct psmovePosef_
{
    float qx, qy, qz, qw, px, py, pz;
    // TODO: Quat and Vectors

    void clear()
    {
        px= py= pz= qx= qy= qz= 0.f;
        qw= 1.f;
    }
} psmovePosef;

class PSMoveDataFrame {
public:
    psmovePosef pose;
    // TODO: Button states
    double timeInSeconds;
};