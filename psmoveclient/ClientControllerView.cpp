//-- includes -----
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include <chrono>

//-- pre-declarations -----

//-- constants -----
const PSMoveVector3 g_psmove_vector3_zero= {0.f, 0.f, 0.f};
const PSMoveVector3 *k_psmove_vector3_zero= &g_psmove_vector3_zero;

const PSMoveQuaternion g_psmove_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMoveQuaternion *k_psmove_quaternion_identity= &g_psmove_quaternion_identity;

//-- prototypes ----
static void update_button_state(PSMoveButtonState &button, unsigned int button_bitmask, unsigned int button_bit);

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
    SequenceNum= -1;
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

    data_frame_last_received_time= 
        std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
    data_frame_average_fps= 0.f;
}

void ClientControllerView::ApplyControllerDataFrame(
    const PSMoveProtocol::ControllerDataFrame *data_frame)
{
    assert(data_frame->psmove_id() == PSMoveID);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        float seconds= static_cast<float>(now - data_frame_last_received_time) / 1000.f;
        float fps= 1.f / seconds;

        data_frame_average_fps= (0.9f)*data_frame_average_fps + (0.1f)*fps;
        data_frame_last_received_time= now;
    }

    if (data_frame->sequence_num() > this->SequenceNum)
    {
        this->SequenceNum= data_frame->sequence_num();
        
        this->IsConnected= data_frame->isconnected();
        this->IsTrackingEnabled= data_frame->istrackingenabled();
        this->IsCurrentlyTracking= data_frame->iscurrentlytracking();

        this->Pose.Orientation.w= data_frame->orientation().w();
        this->Pose.Orientation.x= data_frame->orientation().x();
        this->Pose.Orientation.y= data_frame->orientation().y();
        this->Pose.Orientation.z= data_frame->orientation().z();

        this->Pose.Position.x= data_frame->position().x();
        this->Pose.Position.y= data_frame->position().y();
        this->Pose.Position.z= data_frame->position().z();

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(TriangleButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_TRIANGLE);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_CROSS);
        update_button_state(SquareButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_SQUARE);
        update_button_state(SelectButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_SELECT);
        update_button_state(StartButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_START);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_PS);
        update_button_state(MoveButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_MOVE);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_TRIGGER);

        this->PreviousTriggerValue= this->TriggerValue;

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(data_frame->trigger_value());
    }
}

//-- helper functions -----
static void update_button_state(
    PSMoveButtonState &button,
    unsigned int button_bitmask,
    unsigned int button_bit)
{
    const bool is_down= (button_bitmask & (1 << button_bit)) > 0;

    switch (button)
    {
    case PSMoveButton_UP:
        button= is_down ? PSMoveButton_PRESSED : PSMoveButton_UP;
        break;
    case PSMoveButton_PRESSED:
        button= is_down ? PSMoveButton_DOWN : PSMoveButton_RELEASED;
        break;
    case PSMoveButton_DOWN:
        button= is_down ? PSMoveButton_DOWN : PSMoveButton_RELEASED;
        break;
    case PSMoveButton_RELEASED:
        button= is_down ? PSMoveButton_PRESSED : PSMoveButton_UP;
        break;
    };
}