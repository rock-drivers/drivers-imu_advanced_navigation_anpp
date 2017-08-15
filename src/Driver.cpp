#include <imu_advanced_navigation_anpp/Driver.hpp>
#include <imu_advanced_navigation_anpp/Protocol.hpp>
#include <base/Timeout.hpp>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace imu_advanced_navigation_anpp;
using imu_advanced_navigation_anpp::protocol::Header;

using Eigen::Map;
using Eigen::Unaligned;

Driver::Driver()
    : iodrivers_base::Driver(protocol::MAX_PACKET_SIZE * 10)
    , ned2nwu(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
{
    // Set some sensible default read timeout
    setReadTimeout(base::Time::fromSeconds(1));
    mLastPackets.resize(protocol::PACKET_ID_COUNT, 0);
    mPacketPeriods.resize(protocol::PACKET_ID_COUNT, make_pair(0, 0));
}

void Driver::openURI(std::string const& uri)
{
    iodrivers_base::Driver::openURI(uri);

    resetPollSynchronization();
    std::fill_n(mLastPackets.begin(), protocol::PACKET_ID_COUNT, 0);
    clearPeriodicPackets();
}

void Driver::setUseDeviceTime(bool enable)
{
    int period = enable;
    setPacketPeriod(protocol::UnixTime::ID, period);
    mUseDeviceTime = enable;
}

bool Driver::getUseDeviceTime() const
{
    return mUseDeviceTime;
}

void Driver::clearPeriodicPackets()
{
    int period = mUseDeviceTime;
    setPacketPeriod(protocol::UnixTime::ID, period, true);
    mWorld = base::samples::RigidBodyState();
    mBody  = base::samples::RigidBodyState();
    mAcceleration.acceleration = base::unknown<double>() * Eigen::Vector3d::Ones();
    mAcceleration.angular_acceleration = base::unknown<double>() * Eigen::Vector3d::Ones();
    mGeodeticPosition.latitude  = base::unknown<double>();
    mGeodeticPosition.longitude = base::unknown<double>();
    mGeodeticPosition.altitude  = base::unknown<double>();
    mGeodeticPosition.deviationLatitude  = base::unknown<double>();
    mGeodeticPosition.deviationLongitude = base::unknown<double>();
    mGeodeticPosition.deviationAltitude  = base::unknown<double>();
    mStatus.north_seeking = NorthSeekingInitializationStatus();
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

void protocol2public(Status& status, protocol::Status const& raw_status, base::Time const& time)
{
    status.time = time;
    status.system_status = raw_status.system_status;
    status.filter_status = raw_status.filter_status;
    status.gnss_solution_status =
        static_cast<GNSS_STATUS>((raw_status.filter_status >> 4) & 0xF);
}

Status Driver::readStatus()
{
    auto raw_status = protocol::query<protocol::Status>(*this);
    Status result;
    protocol2public(result, raw_status, base::Time::now());
    return result;
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
    result.enabled_dual_antenna_heading = filter_options.enabled_dual_antenna_heading != 0;
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
    filter_options.enabled_dual_antenna_heading = conf.enabled_dual_antenna_heading ? 1 : 0;
    filter_options.enabled_atmospheric_altitude = conf.enabled_atmospheric_altitude ? 1 : 0;
    filter_options.enabled_velocity_heading     = conf.enabled_velocity_heading ? 1 : 0;
    filter_options.enabled_reversing_detection  = conf.enabled_reversing_detection ? 1 : 0;
    filter_options.enabled_motion_analysis      = conf.enabled_motion_analysis ? 1 : 0;
    header = protocol::writePacket(*this, filter_options);
    protocol::validateAck(*this, header, getReadTimeout());
}

Status Driver::getIMUStatus() const
{
    return mStatus;
}

base::samples::RigidBodyState Driver::getWorldRigidBodyState() const
{
    return mWorld;
}

base::samples::RigidBodyState Driver::getBodyRigidBodyState() const
{
    return mBody;
}

base::samples::RigidBodyAcceleration Driver::getAcceleration() const
{
    return mAcceleration;
}

base::samples::IMUSensors Driver::getIMUSensors() const
{
    return mIMUSensors;
}

gps_base::Solution Driver::getGNSSSolution() const
{
    return mGNSSSolution;
}

gps_base::SolutionQuality Driver::getGNSSSolutionQuality() const
{
    return mGNSSSolutionQuality;
}

gps_base::SatelliteInfo Driver::getGNSSSatelliteInfo() const
{
    return mGNSSSatelliteInfo;
}

void Driver::setPacketPeriod(uint8_t packet_id, uint32_t period, bool clear_existing)
{
    Header header = protocol::writePacketPeriod(*this, packet_id, period, clear_existing);
    protocol::validateAck(*this, header, getReadTimeout());

    if (clear_existing)
        std::fill(mPacketPeriods.begin(), mPacketPeriods.end(), make_pair(0, 0));

    mPacketPeriods[packet_id] = make_pair(period, period == 0 ? 0 : packet_id);
    // UnixTime is internal, do not expose its period to the user
    mPacketPeriods[protocol::UnixTime::ID] = make_pair(0, 0);

    std::pair<int, uint8_t> sortedPeriods[protocol::PACKET_ID_COUNT];
    std::copy(mPacketPeriods.begin(), mPacketPeriods.end(), sortedPeriods);
    std::sort(sortedPeriods, sortedPeriods + protocol::PACKET_ID_COUNT);

    auto last = make_pair(0, 0);
    std::fill_n(mLastPackets.begin(), protocol::PACKET_ID_COUNT, 0);
    for (auto period_and_id : sortedPeriods)
    {
        if (last.first != period_and_id.first)
            mLastPackets[last.second] = last.first;
        last = period_and_id;
    }
    mLastPackets[last.second] = last.first;
}

void Driver::setStatusPeriod(int period)
{
    setPacketPeriod(protocol::Status::ID, period);
}

void Driver::setUTM(int zone, bool north, Eigen::Vector3d const& local_origin)
{
    mUTMConverter.setUTMZone(zone);
    mUTMConverter.setUTMNorth(north);
    mUTMConverter.setNWUOrigin(local_origin);
    updateWorldFromGeodetic();
}

void Driver::setPositionPeriod(int period, bool with_errors)
{
    setPacketPeriod(protocol::GeodeticPosition::ID, period);
    int errors_period = with_errors ? period : 0;
    setPacketPeriod(protocol::GeodeticPositionStandardDeviation::ID, errors_period);

    if (period == 0)
    {
        mGeodeticPosition.latitude  = base::unknown<float>();
        mGeodeticPosition.longitude = base::unknown<float>();
        mGeodeticPosition.altitude  = base::unknown<float>();
    }
    if (errors_period == 0)
    {
        mGeodeticPosition.deviationLatitude  = base::unknown<float>();
        mGeodeticPosition.deviationLongitude = base::unknown<float>();
        mGeodeticPosition.deviationAltitude  = base::unknown<float>();
    }
    updateWorldFromGeodetic();
}

void Driver::setOrientationPeriod(int period, bool with_errors)
{
    setPacketPeriod(protocol::QuaternionOrientation::ID, period);
    int errors_period = with_errors ? period : 0;
    setPacketPeriod(protocol::EulerOrientationStandardDeviation::ID, errors_period);

    if (period == 0)
        mWorld.invalidateOrientation();
    if (errors_period == 0)
        mWorld.invalidateOrientationCovariance();
}

void Driver::setNEDVelocityPeriod(int period, bool with_errors)
{
    setPacketPeriod(protocol::NEDVelocity::ID, period);
    int errors_period = with_errors ? period : 0;
    setPacketPeriod(protocol::NEDVelocityStandardDeviation::ID, errors_period);

    if (period == 0)
        mWorld.invalidateVelocity();
    if (errors_period == 0)
        mWorld.invalidateVelocityCovariance();
}

void Driver::setAccelerationPeriod(int period)
{
    setPacketPeriod(protocol::BodyAcceleration::ID, period);
    if (period == 0)
        mAcceleration.acceleration = base::unknown<double>() * Eigen::Vector3d::Ones();
}

void Driver::setBodyVelocityPeriod(int period)
{
    setPacketPeriod(protocol::BodyVelocity::ID, period);
    if (period == 0)
        mBody.invalidateVelocity();
}

void Driver::setAngularVelocityPeriod(int period)
{
    setPacketPeriod(protocol::AngularVelocity::ID, period);
    if (period == 0)
        mBody.invalidateAngularVelocity();
}

void Driver::setAngularAccelerationPeriod(int period)
{
    setPacketPeriod(protocol::AngularAcceleration::ID, period);
    if (period == 0)
        mAcceleration.angular_acceleration = base::unknown<double>() * Eigen::Vector3d::Ones();
}

void Driver::setRawSensorsPeriod(int period)
{
    setPacketPeriod(protocol::RawSensors::ID, period);
}

void Driver::setGNSSPeriod(int period)
{
    setPacketPeriod(protocol::RawGNSS::ID, period);
}

void Driver::setGNSSSatelliteSummaryPeriod(int period)
{
    setPacketPeriod(protocol::Satellites::ID, period);
}

void Driver::setGNSSSatelliteDetailsPeriod(int period)
{
    setPacketPeriod(protocol::DetailedSatellites::ID, period);
}

void Driver::setNorthSeekingInitializationStatusPeriod(int period)
{
    setPacketPeriod(protocol::NorthSeekingInitializationStatus::ID, period);
    if (period == 0)
        mStatus.north_seeking = NorthSeekingInitializationStatus();
}

template<typename Packet>
void Driver::dispatch(uint8_t const* packet, uint8_t const* packet_end)
{
    Packet payload = Packet::unmarshal(packet + Header::SIZE, packet_end);
    process(payload);
}

void Driver::process(protocol::UnixTime const& payload)
{
    if (mUseDeviceTime)
    {
        mCurrentTimestamp = base::Time::fromMicroseconds(
                static_cast<int64_t>(payload.seconds) * base::Time::UsecPerSec +
                static_cast<int64_t>(payload.microseconds));
    }
}

void Driver::process(protocol::Status const& payload)
{
    protocol2public(mStatus, payload, mCurrentTimestamp);
}

template<typename T>
T may_invalidate(T const& value)
{
    if (value == T::Zero())
        return T::Ones() * base::unknown<double>();
    else return value;
}

void Driver::updateWorldFromGeodetic()
{
    base::samples::RigidBodyState rbs =
        mUTMConverter.convertToNWU(mGeodeticPosition);
    mWorld.position     = rbs.position;
    mWorld.cov_position = rbs.cov_position;
}

void Driver::process(protocol::GeodeticPositionStandardDeviation const& payload)
{
    Eigen::Vector3d variance(
            payload.lat_lon_z_stddev[0] * payload.lat_lon_z_stddev[0],
            payload.lat_lon_z_stddev[1] * payload.lat_lon_z_stddev[1],
            payload.lat_lon_z_stddev[2] * payload.lat_lon_z_stddev[2]);
    variance = may_invalidate(variance);

    mGeodeticPosition.time = mCurrentTimestamp;
    mGeodeticPosition.deviationLatitude  = variance.x();
    mGeodeticPosition.deviationLongitude = variance.y();
    mGeodeticPosition.deviationAltitude  = variance.z();

    mWorld.time         = mGeodeticPosition.time;
    updateWorldFromGeodetic();
}

void Driver::process(protocol::QuaternionOrientation const& payload)
{
    mWorld.time = mCurrentTimestamp;
    Eigen::Quaterniond body2ned = Eigen::Quaterniond(
            payload.im, payload.xyz[0], payload.xyz[1], payload.xyz[2]);
    if (body2ned.w() == 0 && body2ned.x() == 0 && body2ned.y() == 0 && body2ned.z() == 0)
        mWorld.invalidateOrientation();
    else
        mWorld.orientation = ned2nwu * body2ned;
}

void Driver::process(protocol::EulerOrientationStandardDeviation const& payload)
{
    mWorld.time = mCurrentTimestamp;

    Eigen::Matrix3d ned_cov = Eigen::Matrix3d::Zero();
    ned_cov(0, 0) = payload.rpy[0] * payload.rpy[0];
    ned_cov(1, 1) = payload.rpy[1] * payload.rpy[1];
    ned_cov(2, 2) = payload.rpy[2] * payload.rpy[2];
    mWorld.cov_orientation = ned2nwu * may_invalidate(ned_cov);
}

void Driver::process(protocol::NEDVelocity const& payload)
{
    mWorld.time = mCurrentTimestamp;
    Eigen::Vector3d body2ned_velocity = Eigen::Vector3d(payload.ned[0], payload.ned[1], payload.ned[2]);
    mWorld.velocity = ned2nwu * may_invalidate(body2ned_velocity);
}

void Driver::process(protocol::NEDVelocityStandardDeviation const& payload)
{
    mWorld.time = mCurrentTimestamp;
    Eigen::Matrix3d ned = Eigen::Matrix3d::Zero();
    ned(0, 0) = payload.ned[0] * payload.ned[0];
    ned(1, 1) = payload.ned[1] * payload.ned[1];
    ned(2, 2) = payload.ned[2] * payload.ned[2];
    mWorld.cov_velocity = ned2nwu * may_invalidate(ned);
}

void Driver::process(protocol::BodyAcceleration const& payload)
{
    mAcceleration.time = mCurrentTimestamp;
    Eigen::Vector3d acceleration = Eigen::Vector3d(payload.xyz[0], payload.xyz[1], payload.xyz[2]);
    mAcceleration.acceleration = may_invalidate(acceleration);
}

void Driver::process(protocol::BodyVelocity const& payload)
{
    mBody.time = mCurrentTimestamp;
    mBody.velocity =
        may_invalidate(Eigen::Vector3d(payload.xyz[0], payload.xyz[1], payload.xyz[2]));
}

void Driver::process(protocol::AngularVelocity const& payload)
{
    mBody.time = mCurrentTimestamp;
    mBody.angular_velocity =
        may_invalidate(Eigen::Vector3d(payload.xyz[0], payload.xyz[1], payload.xyz[2]));
}

void Driver::process(protocol::AngularAcceleration const& payload)
{
    mAcceleration.time = mCurrentTimestamp;
    mAcceleration.angular_acceleration =
        may_invalidate(Eigen::Vector3d(payload.xyz[0], payload.xyz[1], payload.xyz[2]));
}

void Driver::process(protocol::RawSensors const& payload)
{
    mIMUSensors.time = mCurrentTimestamp;
    mIMUSensors.acc = may_invalidate(Eigen::Vector3d(
            payload.accelerometers_xyz[0],
            payload.accelerometers_xyz[1],
            payload.accelerometers_xyz[2]));
    mIMUSensors.gyro = may_invalidate(Eigen::Vector3d(
            payload.gyroscopes_xyz[0],
            payload.gyroscopes_xyz[1],
            payload.gyroscopes_xyz[2]));
    mIMUSensors.mag = may_invalidate(Eigen::Vector3d(
            payload.magnetometers_xyz[0],
            payload.magnetometers_xyz[1],
            payload.magnetometers_xyz[2]));
}

gps_base::GPS_SOLUTION_TYPES gnss_status_anpp2gps_base(uint16_t status)
{
    switch(status & protocol::RAW_GNSS_FIX_STATUS_MASK)
    {
        case protocol::RAW_GNSS_NO_FIX:
            return gps_base::NO_SOLUTION;
        case protocol::RAW_GNSS_2D:
            return gps_base::AUTONOMOUS_2D;
        case protocol::RAW_GNSS_3D:
            return gps_base::AUTONOMOUS;
        case protocol::RAW_GNSS_SBAS:
        case protocol::RAW_GNSS_DGPS:
        case protocol::RAW_GNSS_OMNISTAR:
            return gps_base::DIFFERENTIAL;
        case protocol::RAW_GNSS_RTK_FLOAT:
            return gps_base::RTK_FLOAT;
        case protocol::RAW_GNSS_RTK_FIXED:
            return gps_base::RTK_FIXED;
        default:
            throw std::invalid_argument("got unexpected status value");
    }
}

void Driver::process(protocol::RawGNSS const& payload)
{
    mGNSSSolution.time = base::Time::fromMicroseconds(
            static_cast<uint64_t>(payload.unix_time_seconds) * base::Time::UsecPerSec +
            static_cast<uint64_t>(payload.unix_time_microseconds));
    mGNSSSolution.positionType = gnss_status_anpp2gps_base(payload.status);
    mGNSSSolution.latitude  = payload.lat_lon_z[0];
    mGNSSSolution.longitude = payload.lat_lon_z[1];
    mGNSSSolution.altitude = payload.lat_lon_z[2];
    mGNSSSolution.deviationLatitude  = payload.lat_lon_z_stddev[0];
    mGNSSSolution.deviationLongitude = payload.lat_lon_z_stddev[1];
    mGNSSSolution.deviationAltitude  = payload.lat_lon_z_stddev[2];
}

void Driver::process(protocol::Satellites const& payload)
{
    mGNSSSolution.noOfSatellites =
        payload.gps_satellite_count +
        payload.glonass_satellite_count +
        payload.beidou_satellite_count +
        payload.galileo_satellite_count +
        payload.sbas_satellite_count;

    mGNSSSolutionQuality.time = mGNSSSolution.time;
    mGNSSSolutionQuality.hdop = payload.hdop;
    mGNSSSolutionQuality.vdop = payload.vdop;
}

void Driver::process(protocol::GeodeticPosition const& payload)
{
    Eigen::Vector3d pos(payload.lat_lon_z[0], payload.lat_lon_z[1], payload.lat_lon_z[2]);
    pos = may_invalidate(pos);

    mGeodeticPosition.time = mCurrentTimestamp;
    mGeodeticPosition.latitude  = pos.x();
    mGeodeticPosition.longitude = pos.y();
    mGeodeticPosition.altitude  = pos.z();

    mWorld.time         = mGeodeticPosition.time;
    updateWorldFromGeodetic();
}

void Driver::process(protocol::NorthSeekingInitializationStatus const& payload)
{
    NorthSeekingInitializationStatus& status = mStatus.north_seeking;
    status.time = mCurrentTimestamp;
    status.flags = payload.flags;
    for (int i = 0; i < 4; ++i)
        status.progress[i] =
            static_cast<float>(payload.progress[i]) / 255;

    status.current_rotation_angle =
        base::Angle::fromRad(payload.current_rotation_angle);
    status.gyroscope_bias =
        Eigen::Vector3d(
                payload.gyroscope_bias_solution_xyz[0],
                payload.gyroscope_bias_solution_xyz[1],
                payload.gyroscope_bias_solution_xyz[2]);
    status.gyroscope_bias_solution_error =
        payload.gyroscope_bias_solution_error;
}

void Driver::processDetailedSatellites(uint8_t const* packet, uint8_t const* packet_end)
{
    mGNSSSatelliteInfo.time = mCurrentTimestamp;
    mGNSSSatelliteInfo.knownSatellites.clear();

    std::vector<protocol::SatelliteInfo> satellite_info;
    protocol::DetailedSatellites::unmarshal(packet + Header::SIZE, packet_end, satellite_info);

    for (auto satellite : satellite_info)
    {
        gps_base::Satellite info;
        info.PRN       = satellite.prn;
        info.elevation = satellite.elevation;
        info.azimuth   = satellite.azimuth;
        info.SNR       = satellite.snr;
        mGNSSSatelliteInfo.knownSatellites.push_back(info);
    }
}

void Driver::setCurrentTimestamp(base::Time const& time)
{
    mCurrentTimestamp = time;
}

base::Time Driver::getCurrentTimestamp() const
{
    return mCurrentTimestamp;
}

void Driver::resetPollSynchronization()
{
    mCurrentTimestamp = base::Time();
    mLastPacketID = 0;
}

int Driver::poll()
{
    uint8_t packet[MAX_PACKET_SIZE];
    size_t packet_size = readPacket(packet, MAX_PACKET_SIZE);

    Header const& header(reinterpret_cast<Header const&>(*packet));
    if (mLastPacketID >= header.packet_id)
    {
        if (!mUseDeviceTime)
            mCurrentTimestamp = base::Time::now();
    }
    mLastPacketID = header.packet_id;

    if (header.packet_id == protocol::UnixTime::ID)
    {
        dispatch<protocol::UnixTime>(packet, packet + packet_size);
        return mUseDeviceTime ? 0 : -1;
    }

    // Wait for a new packet train to initialize the packet timestamp
    if (mCurrentTimestamp.isNull())
        return -1;

#define POLL_DISPATCH_CASE(packet_name) \
        case packet_name::ID: \
            dispatch<packet_name>(packet, packet + packet_size); \
            break;
    switch(header.packet_id)
    {
        POLL_DISPATCH_CASE(protocol::Status);
        POLL_DISPATCH_CASE(protocol::QuaternionOrientation);
        POLL_DISPATCH_CASE(protocol::EulerOrientationStandardDeviation);
        POLL_DISPATCH_CASE(protocol::NEDVelocity);
        POLL_DISPATCH_CASE(protocol::NEDVelocityStandardDeviation);
        POLL_DISPATCH_CASE(protocol::BodyAcceleration);
        POLL_DISPATCH_CASE(protocol::BodyVelocity);
        POLL_DISPATCH_CASE(protocol::AngularVelocity);
        POLL_DISPATCH_CASE(protocol::AngularAcceleration);
        POLL_DISPATCH_CASE(protocol::RawSensors);
        POLL_DISPATCH_CASE(protocol::RawGNSS);
        POLL_DISPATCH_CASE(protocol::Satellites);
        POLL_DISPATCH_CASE(protocol::GeodeticPosition);
        POLL_DISPATCH_CASE(protocol::GeodeticPositionStandardDeviation);
        POLL_DISPATCH_CASE(protocol::NorthSeekingInitializationStatus);
        case protocol::DetailedSatellites::ID:
            processDetailedSatellites(packet, packet + packet_size);
            break;
        default:
            LOG_ERROR_S << "Ignored message of type " << header.packet_id << std::endl;
    }

    return mLastPackets[header.packet_id];
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


