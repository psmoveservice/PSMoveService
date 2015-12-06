#ifndef PSMOVE_DATA_FRAME_H
#define PSMOVE_DATA_FRAME_H

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

#endif // PSMOVE_DATA_FRAME_H