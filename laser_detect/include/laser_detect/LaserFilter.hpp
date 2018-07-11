#ifndef _LASER_FILTER_HPP
#define _LASER_FILTER_HPP

#include <deque>

#include "LaserDataType.hpp"

class LaserFilter {
public:

typedef std::deque<LaserData> LaserFilterT;
typedef std::pair<const double, double> LaserPointT;
typedef std::vector<double> FilterCoeffT;
typedef std::vector<double> RangeSeqT;
typedef std::vector<double> AngSeqT;

public:

LaserFilter()
{
	m_filter_data.erase(m_filter_data.begin(), m_filter_data.end());
}

/**
 * Receive new LaserData, and put it into deque
 * @param  laser_data New LaserData
 * @return            if deque's size is equal to m_deq_num, return true, otherwise, return false
 */
bool UpdateDeque(const LaserData& laser_data)
{
	m_filter_data.push_back(laser_data);

	if (m_deq_num > m_filter_data.size())
		return false;

	while (m_deq_num < m_filter_data.size())
		m_filter_data.pop_front();

	return true;
}

void AddSeq(RangeSeqT &dest_range, const RangeSeqT &src_range)
{
	RangeSeqT::iterator dest_iter = dest_range.begin();
	RangeSeqT::const_iterator src_iter = src_range.begin();

	for (; dest_iter != dest_range.end(); ++dest_iter, ++src_iter)
		*dest_iter += *src_iter;
}

virtual void Filtering(LaserData&	laser_data_in,
		       LaserData &	laser_data_out)
{
}


public:

LaserFilterT m_filter_data;
int m_deq_num;
};

class LaserMAFilter : public LaserFilter {
public:

LaserMAFilter(int ave_num)
{
	m_deq_num = ave_num;
	m_filter_data.erase(m_filter_data.begin(), m_filter_data.end());
}

virtual void Filtering(LaserData&	laser_data_in,
		       LaserData &	laser_data_out) override
{
	laser_data_out.EmptyData();

	if (UpdateDeque(laser_data_in)) {
		RangeSeqT sum_range;
		sum_range.resize(laser_data_in.DataSize());
		AngSeqT angle_seq = laser_data_in.GetAngSeq();

		LaserFilterT::iterator iter = m_filter_data.begin();
		for (; iter != m_filter_data.end(); ++iter)
			AddSeq(sum_range, iter->GetRangeData());

		RangeSeqT::iterator range_iter = sum_range.begin();
		AngSeqT::iterator angle_iter = angle_seq.begin();
		for (; range_iter != sum_range.end(); ++range_iter, ++angle_iter) {
			double angle = *angle_iter;
			double range = (*range_iter) / m_deq_num;
			laser_data_out.InsertData(LaserPointT(angle, range));
		}
	} else {
		laser_data_out = laser_data_in;
	}
}
};

class LaserBWFilter : public LaserFilter {
private:

FilterCoeffT m_az;
FilterCoeffT m_bz;

public:

LaserBWFilter(FilterCoeffT az, FilterCoeffT bz) : m_az(az), m_bz(bz)
{
	m_filter_data.erase(m_filter_data.begin(), m_filter_data.end());

	m_deq_num = m_az.size();
}

virtual void Filtering(LaserData&	laser_data_in,
		       LaserData &	laser_data_out) override
{
	laser_data_out.EmptyData();

	if (UpdateDeque(laser_data_in)) {
	} else {
		laser_data_out = laser_data_in;
	}
}
};

#endif // ifndef _LASER_FILTER_HPP
