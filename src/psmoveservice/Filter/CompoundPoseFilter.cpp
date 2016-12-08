// -- includes --
#include "CompoundPoseFilter.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "KalmanPositionFilter.h"
#include "KalmanOrientationFilter.h"

// -- public interface --
bool CompoundPoseFilter::init(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant)
{
	bool bSuccess = true;

	allocate_filters(deviceType, orientationFilterType, positionFilterType, constant);

	if (m_orientation_filter != nullptr)
	{
		bSuccess &= m_orientation_filter->init(constant.orientation_constants);
	}

	if (m_position_filter != nullptr)
	{
		bSuccess &= m_position_filter->init(constant.position_constants);
	}

	return bSuccess;
}

bool CompoundPoseFilter::init(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation)
{
	bool bSuccess = true;

	allocate_filters(deviceType, orientationFilterType, positionFilterType, constant);

	if (m_orientation_filter != nullptr)
	{
		bSuccess &= m_orientation_filter->init(constant.orientation_constants, initial_orientation);
	}

	if (m_position_filter != nullptr)
	{
		bSuccess &= m_position_filter->init(constant.position_constants, initial_position);
	}

	return bSuccess;
}

void CompoundPoseFilter::allocate_filters(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant)
{
	dispose_filters();

	switch(orientationFilterType)
	{
    case OrientationFilterTypeNone:
		m_orientation_filter = nullptr;
		break;
    case OrientationFilterTypePassThru:
		m_orientation_filter = new OrientationFilterPassThru();
		break;
    case OrientationFilterTypeMadgwickARG:
		m_orientation_filter = new OrientationFilterMadgwickARG;
		break;
    case OrientationFilterTypeMadgwickMARG:
		m_orientation_filter = new OrientationFilterMadgwickMARG;
		break;
    case OrientationFilterTypeComplementaryOpticalARG:
		m_orientation_filter = new OrientationFilterComplementaryOpticalARG;
		break;
    case OrientationFilterTypeComplementaryMARG:
		m_orientation_filter = new OrientationFilterComplementaryMARG;
		break;
	case OrientationFilterTypeKalman:
		{
			switch (deviceType)
			{
			case CommonDeviceState::PSDualShock4:
				m_orientation_filter = new KalmanOrientationFilterDS4;
				break;
			case CommonDeviceState::PSMove:
				m_orientation_filter = new KalmanOrientationFilterPSMove;
				break;
			default:
				assert(0 && "unreachable");
			}
		}
		break;
	default:
		assert(0 && "unreachable");
	}

	switch(positionFilterType)
	{
    case PositionFilterTypeNone:
		m_position_filter = nullptr;
		break;
    case PositionFilterTypePassThru:
		m_position_filter = new PositionFilterPassThru;
		break;
    case PositionFilterTypeLowPassOptical:
		m_position_filter = new PositionFilterLowPassOptical;
		break;
    case PositionFilterTypeLowPassIMU:
		m_position_filter = new PositionFilterLowPassIMU;
		break;
    case PositionFilterTypeComplimentaryOpticalIMU:
		m_position_filter = new PositionFilterComplimentaryOpticalIMU;
		break;
	case PositionFilterTypeLowPassExponential:
		m_position_filter = new PositionFilterLowPassExponential;
		break;
	case PositionFilterTypeKalman:
		m_position_filter = new KalmanPositionFilter;
		break;
	default:
		assert(0 && "unreachable");
	}
}

// -- IStateFilter --
bool CompoundPoseFilter::getIsStateValid() const
{
	return 
		m_orientation_filter != nullptr && m_orientation_filter->getIsStateValid() &&
		m_position_filter != nullptr && m_position_filter->getIsStateValid();
}

void CompoundPoseFilter::update(
	const float delta_time,
	const PoseFilterPacket &orientation_filter_packet)
{
	if (m_orientation_filter != nullptr && m_position_filter != nullptr)
	{
		// Update the orientation filter first
		m_orientation_filter->update(delta_time, orientation_filter_packet);

		// Update the position filter using the latest orientation
		PoseFilterPacket position_filter_packet= orientation_filter_packet;
		position_filter_packet.current_orientation= m_orientation_filter->getOrientation();

		m_position_filter->update(delta_time, position_filter_packet);
	}
}

void CompoundPoseFilter::resetState()
{
	if (m_orientation_filter != nullptr && m_position_filter != nullptr)
	{
		m_orientation_filter->resetState();
		m_position_filter->resetState();
	}
}

void CompoundPoseFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
	if (m_orientation_filter != nullptr)
	{
		m_orientation_filter->recenterOrientation(q_pose);
	}
}

Eigen::Quaternionf CompoundPoseFilter::getOrientation(float time) const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getOrientation(time) : Eigen::Quaternionf::Identity();
}

Eigen::Vector3f CompoundPoseFilter::getAngularVelocity() const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getAngularVelocity() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getAngularAcceleration() const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getAngularAcceleration() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getPosition(float time) const
{
	return (m_position_filter != nullptr) ? m_position_filter->getPosition(time) : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getVelocity() const
{
	return (m_position_filter != nullptr) ? m_position_filter->getVelocity() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getAcceleration() const
{
	return (m_position_filter != nullptr) ? m_position_filter->getAcceleration() : Eigen::Vector3f::Zero();
}

void CompoundPoseFilter::dispose_filters()
{
	if (m_orientation_filter != nullptr)
	{
		delete m_orientation_filter;
		m_orientation_filter= nullptr;
	}

	if (m_position_filter != nullptr)
	{
		delete m_position_filter;
		m_position_filter= nullptr;
	}
}