#ifndef POSITION_FILTER_H
#define POSITION_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"
#include <chrono>

//-- definitions -----
/// Abstract base class for all orientation only filters
class PositionFilter : public IPositionFilter
{
public:
    PositionFilter();
    virtual ~PositionFilter();

    //-- IStateFilter --
    bool getIsStateValid() const override;
    void resetState() override;
    void recenterState() override;

    // -- IOrientationFilter --
    bool init(const PositionFilterConstants &constant) override;
	bool init(const PositionFilterConstants &constant, const Eigen::Vector3f &initial_position) override;
    Eigen::Vector3f getPosition(float time = 0.f) const override;
    Eigen::Vector3f getVelocity() const override;
    Eigen::Vector3f getAcceleration() const override;

protected:
    PositionFilterConstants m_constants;
    struct PositionFilterState *m_state;
};

/// Just use the optical orientation passed in unfiltered
class PositionFilterPassThru : public PositionFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

class PositionFilterLowPassOptical : public PositionFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

class PositionFilterLowPassIMU : public PositionFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

class PositionFilterComplimentaryOpticalIMU : public PositionFilter
{
public:
	PositionFilterComplimentaryOpticalIMU()
		: PositionFilter()
		, bLast_visible_position_timestamp_valid(false)
	{}

	void resetState() override;
    void update(const float delta_time, const PoseFilterPacket &packet) override;

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_position_timestamp;
    bool bLast_visible_position_timestamp_valid;
};

#endif // POSITION_FILTER_H