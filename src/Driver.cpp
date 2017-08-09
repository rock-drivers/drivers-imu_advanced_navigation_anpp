#include <advanced_navigation_anpp/Driver.hpp>
#include <advanced_navigation_anpp/Protocol.hpp>
#include <base/Timeout.hpp>

#include <iostream>
#include <iomanip>

using namespace std;
using namespace advanced_navigation_anpp;
using advanced_navigation_anpp::protocol::Header;

using Eigen::Map;
using Eigen::Unaligned;

Driver::Driver()
    : iodrivers_base::Driver(256 * 10)
{
    // Set some sensible default read timeout
    setReadTimeout(base::Time::fromSeconds(1));
}

void Driver::setUseDeviceTime(bool enable)
{
    int period = enable;
    Header header =
        protocol::writePacketPeriod<protocol::UnixTime>(*this, period, false);
    validateAck(*this, header, getReadTimeout());
    mUseDeviceTime = enable;
}

bool Driver::getUseDeviceTime() const
{
    return mUseDeviceTime;
}

void Driver::clearPeriodicPackets()
{
    int period = mUseDeviceTime;
    Header header =
        protocol::writePacketPeriod<protocol::UnixTime>(*this, period, true);
    validateAck(*this, header, getReadTimeout());
}

void Driver::reset(RESET_MODE mode)
{
    if (mode == RESET_COLD)
        protocol::writePacket(*this, protocol::ColdStartReset());
    else if (mode == RESET_HOT)
        protocol::writePacket(*this, protocol::HotStartReset());
    else if (mode == RESET_FACTORY)
    {
        protocol::writePacket(*this, protocol::RestoreFactorySettings());
        protocol::writePacket(*this, protocol::ColdStartReset());
    }
}

DeviceInformation Driver::readDeviceInformation()
{
    return protocol::query<protocol::DeviceInformation>(*this);
}

base::Time Driver::readTime()
{
    auto raw_time = protocol::query<protocol::UnixTime>(*this);
    return base::Time::fromMicroseconds(
            static_cast<uint64_t>(raw_time.seconds) * base::Time::UsecPerSec +
            static_cast<uint64_t>(raw_time.microseconds));
}

Status Driver::readStatus()
{
    auto raw_status = protocol::query<protocol::Status>(*this);
    Status status;
    status.system_status = raw_status.system_status;
    status.orientation_initialized =
        (raw_status.filter_status & FILTER_ORIENTATION_INITIALIZED) != 0;
    status.navigation_initialized =
        (raw_status.filter_status & FILTER_NAVIGATION_INITIALIZED) != 0;
    status.heading_initialized =
        (raw_status.filter_status & FILTER_HEADING_INITIALIZED) != 0;
    status.utc_initialized =
        (raw_status.filter_status & FILTER_UTC_INITIALIZED) != 0;
    status.gnss_status =
        static_cast<GNSS_STATUS>(raw_status.filter_status & FILTER_GNSS_FIX_STATUS_MASK);
    return status;
}

CurrentConfiguration Driver::readConfiguration()
{
    protocol::PacketTimerPeriod packet_timer_period =
        protocol::query<protocol::PacketTimerPeriod>(*this);
    protocol::Alignment alignment =
        protocol::query<protocol::Alignment>(*this);
    protocol::FilterOptions filter_options =
        protocol::query<protocol::FilterOptions>(*this);
    protocol::MagneticCalibrationValues magnetic_calibration =
        protocol::query<protocol::MagneticCalibrationValues>(*this);
    protocol::MagneticCalibrationStatus magnetic_calibration_status =
        protocol::query<protocol::MagneticCalibrationStatus>(*this);

    CurrentConfiguration result;
    result.utc_synchronization = packet_timer_period.utc_synchronization != 0;
    result.packet_timer_period = base::Time::fromMicroseconds(packet_timer_period.period);
    result.gnss_antenna_offset = Map< Eigen::Vector3f, Unaligned >(alignment.gnss_antenna_offset_xyz).cast<double>();

    result.vehicle_type                 = static_cast<VEHICLE_TYPES>(filter_options.vehicle_type);
    result.enabled_internal_gnss        = filter_options.enabled_internal_gnss != 0;
    result.enabled_atmospheric_altitude = filter_options.enabled_atmospheric_altitude != 0;
    result.enabled_velocity_heading     = filter_options.enabled_velocity_heading != 0;
    result.enabled_reversing_detection  = filter_options.enabled_reversing_detection != 0;
    result.enabled_motion_analysis      = filter_options.enabled_motion_analysis != 0;

    result.magnetic_calibration_status  =
        static_cast<MAGNETIC_CALIBRATION_STATUS>(magnetic_calibration_status.status);
    result.hard_iron_bias =
        Map< Eigen::Vector3f, Unaligned >(magnetic_calibration.hard_iron_bias_xyz).cast<double>();
    result.soft_iron_transformation =
        Map< Eigen::Matrix3f, Unaligned >(magnetic_calibration.soft_iron_transformation).cast<double>();

    return result;
}

void Driver::setConfiguration(Configuration const& conf)
{
    protocol::PacketTimerPeriod packet_timer_period;
    packet_timer_period.permanent = 0;
    packet_timer_period.utc_synchronization = conf.utc_synchronization ? 1 : 0;
    packet_timer_period.period = conf.packet_timer_period.toMicroseconds();
    Header header = protocol::writePacket(*this, packet_timer_period);
    protocol::validateAck(*this, header, getReadTimeout());

    protocol::Alignment alignment;
    alignment.permanent = 0;
    std::fill_n(alignment.dcm, 9, 0);
    alignment.dcm[0] = 1;
    alignment.dcm[4] = 1;
    alignment.dcm[8] = 1;
    std::copy_n(conf.gnss_antenna_offset.data(), 3, alignment.gnss_antenna_offset_xyz);
    std::fill_n(alignment.odometer_offset_xyz, 3, 0);
    std::fill_n(alignment.external_data_offset_xyz, 3, 0);
    header = protocol::writePacket(*this, alignment);
    protocol::validateAck(*this, header, getReadTimeout());

    protocol::FilterOptions filter_options;
    filter_options.permanent = 0;
    filter_options.vehicle_type                 = static_cast<VEHICLE_TYPES>(conf.vehicle_type);
    filter_options.enabled_internal_gnss        = conf.enabled_internal_gnss ? 1 : 0;
    filter_options.enabled_atmospheric_altitude = conf.enabled_atmospheric_altitude ? 1 : 0;
    filter_options.enabled_velocity_heading     = conf.enabled_velocity_heading ? 1 : 0;
    filter_options.enabled_reversing_detection  = conf.enabled_reversing_detection ? 1 : 0;
    filter_options.enabled_motion_analysis      = conf.enabled_motion_analysis ? 1 : 0;
    header = protocol::writePacket(*this, filter_options);
    protocol::validateAck(*this, header, getReadTimeout());
}

int Driver::extractPacket(uint8_t const* buffer, size_t buffer_length) const
{
    if (buffer_length < 4)
        return 0;

    Header const& header = reinterpret_cast<Header const&>(*buffer);
    if (header.isValid())
    {
        size_t expected_packet_length = header.getPacketLength();
        if (buffer_length < expected_packet_length)
            return 0;
        else if (header.isPacketValid(buffer + Header::SIZE, buffer + expected_packet_length))
            return expected_packet_length;
        else
            return -1;
    }

    auto buffer_end = buffer + buffer_length;
    for (auto packet_start = buffer + 1; packet_start + 4 < buffer_end; packet_start++)
    {
        Header const& header = reinterpret_cast<Header const&>(*packet_start);
        if (header.isValid())
            return buffer - packet_start;
    }
    return -static_cast<int>(buffer_length - 3);
}


