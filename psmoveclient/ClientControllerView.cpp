//-- includes -----
#include "ClientControllerView.h"
#include "PSMoveDataFrame.pb.h"

//-- pre-declarations -----

//-- constants -----
const PSMoveVector3 g_psmove_vector3_zero= {0.f, 0.f, 0.f};
const PSMoveVector3 *k_psmove_vector3_zero= &g_psmove_vector3_zero;

const PSMoveQuaternion g_psmove_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMoveQuaternion *k_psmove_quaternion_identity= &g_psmove_quaternion_identity;

//-- implementation -----
ClientControllerView::ClientControllerView(int PSMoveID)
{
    Clear();
    this->PSMoveID= PSMoveID;
}

void ClientControllerView::Clear()
{
    Pose.Clear();
    PSMoveID = -1;
    ListenerCount= 0;

    IsConnected= false;
    IsTrackingEnabled= false;
    IsCurrentlyTracking= false;

    TriangleButton= PSMoveButton_UP;
    CircleButton= PSMoveButton_UP;
    CrossButton= PSMoveButton_UP;
    SquareButton= PSMoveButton_UP;
    SelectButton= PSMoveButton_UP;
    StartButton= PSMoveButton_UP;
    PSButton= PSMoveButton_UP;
    MoveButton= PSMoveButton_UP;
    TriggerButton= PSMoveButton_UP;

    PreviousTriggerValue= 0;
    TriggerValue= 0;
}

void ClientControllerView::ApplyControllerDataFrame(
    const ControllerDataFrame *data_frame)
{
    //TODO
}