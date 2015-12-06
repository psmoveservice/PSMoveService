//-- includes -----
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include <chrono>
#include <assert.h>

//-- pre-declarations -----

//-- constants -----
const PSMoveVector3 g_psmove_vector3_zero= {0.f, 0.f, 0.f};
const PSMoveVector3 *k_psmove_vector3_zero= &g_psmove_vector3_zero;

const PSMoveQuaternion g_psmove_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMoveQuaternion *k_psmove_quaternion_identity= &g_psmove_quaternion_identity;

//-- prototypes ----
static void update_button_state(PSMoveButtonState &button, unsigned int button_bitmask, unsigned int button_bit);

//-- implementation -----

//-- ClientPSMoveView -----
void ClientPSMoveView::Clear()
{
    bValid= false;
    bIsTrackingEnabled= false;
    bIsCurrentlyTracking= false;

    Pose.Clear();

    TriangleButton= PSMoveButton_UP;
    CircleButton= PSMoveButton_UP;
    CrossButton= PSMoveButton_UP;
    SquareButton= PSMoveButton_UP;
    SelectButton= PSMoveButton_UP;
    StartButton= PSMoveButton_UP;
    PSButton= PSMoveButton_UP;
    MoveButton= PSMoveButton_UP;
    TriggerButton= PSMoveButton_UP;

    TriggerValue= 0;
}

void ClientPSMoveView::ApplyControllerDataFrame(const PSMoveProtocol::ControllerDataFrame *data_frame)
{
    if (data_frame->isconnected())
    {
        const PSMoveProtocol::ControllerDataFrame_PSMoveState &psmove_data_frame= data_frame->psmove_state();

        this->bIsTrackingEnabled= psmove_data_frame.istrackingenabled();
        this->bIsCurrentlyTracking= psmove_data_frame.iscurrentlytracking();

        this->Pose.Orientation.w= psmove_data_frame.orientation().w();
        this->Pose.Orientation.x= psmove_data_frame.orientation().x();
        this->Pose.Orientation.y= psmove_data_frame.orientation().y();
        this->Pose.Orientation.z= psmove_data_frame.orientation().z();

        this->Pose.Position.x= psmove_data_frame.position().x();
        this->Pose.Position.y= psmove_data_frame.position().y();
        this->Pose.Position.z= psmove_data_frame.position().z();

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

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(psmove_data_frame.trigger_value());

        this->bValid= true;
    }
    else
    {
        Clear();
    }
}

//-- ClientPSNaviView -----
void ClientPSNaviView::Clear()
{
    bValid= false;

    L1Button= PSMoveButton_UP;
    L2Button= PSMoveButton_UP;
    L3Button= PSMoveButton_UP;
    CircleButton= PSMoveButton_UP;
    CrossButton= PSMoveButton_UP;
    PSButton= PSMoveButton_UP;
    TriggerButton= PSMoveButton_UP;
    DPadUpButton= PSMoveButton_UP;
    DPadRightButton= PSMoveButton_UP;
    DPadDownButton= PSMoveButton_UP;
    DPadLeftButton= PSMoveButton_UP;

    TriggerValue= 0;
    Stick_XAxis= 0x80;
    Stick_YAxis= 0x80;
}

void ClientPSNaviView::ApplyControllerDataFrame(const PSMoveProtocol::ControllerDataFrame *data_frame)
{
    if (data_frame->isconnected())
    {
        const PSMoveProtocol::ControllerDataFrame_PSNaviState &psnavi_data_frame= data_frame->psnavi_state();

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(L1Button, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_L1);
        update_button_state(L2Button, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_L2);
        update_button_state(L3Button, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_L3);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_CROSS);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_PS);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_TRIGGER);
        update_button_state(DPadUpButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_UP);
        update_button_state(DPadRightButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_RIGHT);
        update_button_state(DPadDownButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_DOWN);
        update_button_state(DPadLeftButton, button_bitmask, PSMoveProtocol::ControllerDataFrame_ButtonType_LEFT);

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(psnavi_data_frame.trigger_value());
        this->Stick_XAxis= static_cast<unsigned char>(psnavi_data_frame.stick_xaxis());
        this->Stick_YAxis= static_cast<unsigned char>(psnavi_data_frame.stick_yaxis());

        this->bValid= true;
    }
    else
    {
        Clear();
    }
}

//-- ClientControllerView -----
ClientControllerView::ClientControllerView(int PSMoveID)
{
    Clear();
    this->ControllerID= PSMoveID;
}

void ClientControllerView::Clear()
{
    ControllerID = -1;
    SequenceNum= -1;
    ListenerCount= 0;

    IsConnected= false;

    ControllerViewType= None;
    memset(&ViewState, 0, sizeof(ViewState));

    data_frame_last_received_time= 
        std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
    data_frame_average_fps= 0.f;
}

void ClientControllerView::ApplyControllerDataFrame(
    const PSMoveProtocol::ControllerDataFrame *data_frame)
{
    assert(data_frame->controller_id() == ControllerID);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff= now - data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds= static_cast<float>(diff) / 1000.f;
            float fps= 1.f / seconds;

            data_frame_average_fps= (0.9f)*data_frame_average_fps + (0.1f)*fps;
        }

        data_frame_last_received_time= now;
    }

    if (data_frame->sequence_num() > this->SequenceNum)
    {
        this->SequenceNum= data_frame->sequence_num();
        this->IsConnected= data_frame->isconnected();

        switch(data_frame->controller_type())
        {
            case PSMoveProtocol::PSMOVE:
            {
                this->ControllerViewType= PSMove;
                this->ViewState.PSMoveView.ApplyControllerDataFrame(data_frame);
            } break;

            case PSMoveProtocol::PSNAVI:
            {
                this->ControllerViewType= PSNavi;
                this->ViewState.PSNaviView.ApplyControllerDataFrame(data_frame);
            } break;

            default:
                assert(0 && "Unhandled controller type");
        }
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