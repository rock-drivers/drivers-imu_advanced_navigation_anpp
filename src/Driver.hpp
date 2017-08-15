#ifndef ADVANCED_NAVIGATION_ANPP_DRIVER_HPP
#define ADVANCED_NAVIGATION_ANPP_DRIVER_HPP

#include <imu_advanced_navigation_anpp/DeviceInformation.hpp>
#include <imu_advanced_navigation_anpp/Status.hpp>
#include <imu_advanced_navigation_anpp/Configuration.hpp>
#include <imu_advanced_navigation_anpp/CurrentConfiguration.hpp>
#include <iodrivers_base/Driver.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/samples/IMUSensors.hpp>
#include <gps_base/BaseTypes.hpp>
#include <gps_base/UTMConverter.hpp>

namespace imu_advanced_navigation_anpp 
{
    enum RESET_MODE
    {
        RESET_COLD,
        RESET_HOT,
        RESET_FACTORY
    };

    namespace protocol
    {
        struct Header;
    }

    namespace protocol
    {
        struct UnixTime;
        struct Status;
        struct GeodeticPositionStandardDeviation;
        struct GeodeticPosition;
        struct QuaternionOrientation;
        struct EulerOrientationStandardDeviation;
        struct NEDVelocity;
        struct NEDVelocityStandardDeviation;
        struct BodyAcceleration;
        struct BodyVelocity;
        struct AngularVelocity;
        struct AngularAcceleration;
        struct RawSensors;
        struct RawGNSS;
        struct Satellites;
        struct NorthSeekingInitializationStatus;
    }

    class Driver : public iodrivers_base::Driver
    {
    private:
        static constexpr int PACKET_ID_COUNT = 256;

        bool mUseDeviceTime = false;
        uint8_t mLastPacketID = 0;
        base::Time mCurrentTimestamp;
        Eigen::Quaterniond const ned2nwu;

        gps_base::UTMConverter mUTMConverter;
        std::vector<uint32_t> mLastPackets;
        std::vector<std::pair<uint32_t, uint8_t>> mPacketPeriods;

        base::samples::RigidBodyState mWorld;
        base::samples::RigidBodyState mBody;
        base::samples::RigidBodyAcceleration mAcceleration;
        base::samples::IMUSensors mIMUSensors;
        gps_base::Solution mGeodeticPosition;
        gps_base::Solution mGNSSSolution;
        gps_base::SolutionQuality mGNSSSolutionQuality;
        gps_base::SatelliteInfo mGNSSSatelliteInfo;
        Status mStatus;

        void updateWorldFromGeodetic();

        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
        void setPacketPeriod(uint8_t packet_id, uint32_t period, bool clear_existing = false);

        template<typename Packet>
        void dispatch(uint8_t const* packet, uint8_t const* packet_end);
        void process(protocol::UnixTime const& payload);
        void process(protocol::Status const& payload);
        void process(protocol::GeodeticPositionStandardDeviation const& payload);
        void process(protocol::QuaternionOrientation const& payload);
        void process(protocol::EulerOrientationStandardDeviation const& payload);
        void process(protocol::NEDVelocity const& payload);
        void process(protocol::NEDVelocityStandardDeviation const& payload);
        void process(protocol::BodyAcceleration const& payload);
        void process(protocol::BodyVelocity const& payload);
        void process(protocol::AngularVelocity const& payload);
        void process(protocol::AngularAcceleration const& payload);
        void process(protocol::RawSensors const& payload);
        void process(protocol::RawGNSS const& payload);
        void process(protocol::Satellites const& payload);
        void process(protocol::GeodeticPosition const& payload);
        void process(protocol::NorthSeekingInitializationStatus const& payload);
        void processDetailedSatellites(uint8_t const* packet, uint8_t const* packet_end);

    public:
        Driver();

        void openURI(std::string const& uri);

        /** Whether timestamping is using the device's time
         *
         * Use this if the device is synchronized with UTC (e.g. using a
         * GPS-based time source
         *
         * @param enable whether the driver should use the device time (true) or
         *   the local host time (false). The default is false.
         */
        bool getUseDeviceTime() const;

        /** Whether timestamping should be using the device's time
         *
         * Use this if the device is synchronized with UTC (e.g. using a
         * GPS-based time source
         *
         * @param enable whether the driver should use the device time (true) or
         *   the local host time (false). The default is false.
         */
        void setUseDeviceTime(bool enable);
        
        /** Clear all defined periodic packets */
        void clearPeriodicPackets();

        /** Reset the device
         */
        void reset(RESET_MODE mode);

        /** Read the device information
         *
         * This is mainly useful to verify that the device is connected
         */
        DeviceInformation readDeviceInformation();

        /** Read the device's current time */
        base::Time readTime();

        /** Read the system and filter status */
        Status readStatus();

        /** Read the current configuration */
        CurrentConfiguration readConfiguration();

        /** Read the current configuration */
        void setConfiguration(Configuration const& conf);

        /** The current system+filter status */
        Status getIMUStatus() const;

        /** The NWU position
         */
        base::samples::RigidBodyState getWorldRigidBodyState() const;

        /** Body-relative information
         *
         * This contains only body-relative velocity data
         */
        base::samples::RigidBodyState getBodyRigidBodyState() const;

        /** Acceleration information */
        base::samples::RigidBodyAcceleration getAcceleration() const;

        /** Raw sensor data */
        base::samples::IMUSensors getIMUSensors() const;

        /** GNSS solution data */
        gps_base::Solution getGNSSSolution() const;

        /** GNSS quality information */
        gps_base::SolutionQuality getGNSSSolutionQuality() const;

        /** GNSS satellite information */
        gps_base::SatelliteInfo getGNSSSatelliteInfo() const;

        /** Set the period at which the status should be updated
         *
         * Periodic messages are processed by poll().
         *
         * This updates the structure as returned by getStatus()
         *
         * @param period the period in multiples of the base packet period
         */
        void setStatusPeriod(int period);

        /** Set the UTM zone that is used to convert the global position into a
         * local RBS
         */
        void setUTM(int zone, bool north,
                    Eigen::Vector3d const& local_origin = Eigen::Vector3d::Zero());

        /** Set the period at which the position should be generated
         *
         * Periodic messages are processed by poll().
         *
         * This updates the world-relative RBS as returned by
         * getWorldRigidBodyState().
         *
         * You must set the UTM position and origin before calling this
         *
         * @param period the period in multiples of the base packet period
         * @param with_errors if true, generate the orientation errors at the
         *   same period. Otherwise, do not generate them
         */
        void setPositionPeriod(int period, bool with_errors = true);

        /** Set the period at which the orientation should be generated
         *
         * Periodic messages are processed by poll().
         *
         * This updates the world-relative RBS as returned by
         * getWorldRigidBodyState().
         *
         * @param period the period in multiples of the base packet period
         * @param with_errors if true, generate the orientation errors at the
         *   same period. Otherwise, do not generate them
         */
        void setOrientationPeriod(int period, bool with_errors = true);

        /** Set the period at which the velocity in the NED frame should be
         * generated
         *
         * Periodic messages are processed by poll()
         *
         * This updates the world-relative RBS as returned by
         * getWorldRigidBodyState().
         *
         * @param period the period in multiples of the base packet period
         */
        void setNEDVelocityPeriod(int period, bool with_errors = true);

        /** Set the period at which the acceleration in body frame
         * should be generated. This acceleration does not contain the
         * acceleration of gravity
         *
         * Periodic messages are processed by poll()
         *
         * This updates the acceleration structure as returned by
         * getAcceleration().
         *
         * @param period the period in multiples of the base packet period
         */
        void setAccelerationPeriod(int period);

        /** Set the period at which the velocity in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * This updates the body-relative RBS as returned by
         * getBodyRigidBodyState();
         *
         * @param period the period in multiples of the base packet period
         */
        void setBodyVelocityPeriod(int period);

        /** Set the period at which the angular velocities in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * This updates the body-relative RBS as returned by
         * getBodyRigidBodyState();
         *
         * @param period the period in multiples of the base packet period
         */
        void setAngularVelocityPeriod(int period);

        /** Set the period at which the angular accelerations in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * This updates the acceleration structure as returned by
         * getAcceleration();
         *
         * @param period the period in multiples of the base packet period
         */
        void setAngularAccelerationPeriod(int period);

        /** Set the period at which the raw sensors should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * This updates the raw sensor structure as returned by
         * getRawSensors()
         *
         * @param period the period in multiples of the base packet period
         */
        void setRawSensorsPeriod(int period);

        /** Set the period at which geodetic position should be generated
         *
         * Periodic messages are processed by poll()
         *
         * This updates the structure returned by getGNSSolution()
         *
         * @param period the period in multiples of the base packet period
         */
        void setGNSSPeriod(int period);

        /** Set the period at which GNSS satellite summary should be generated
         *
         * Periodic messages are processed by poll()
         *
         * This updates the structure returned by getGNSSSolutionQuality()
         *
         * @param period the period in multiples of the base packet period
         */
        void setGNSSSatelliteSummaryPeriod(int period);

        /** Set the period at which GNSS detailed satellite info should be generated
         *
         * Periodic messages are processed by poll()
         *
         * This updates the structure returned by getGNSSSatelliteDetails()
         *
         * @param period the period in multiples of the base packet period
         */
        void setGNSSSatelliteDetailsPeriod(int period);

        /** Set the period at which we receive the north seeking status
         *
         * It is reported in the Status structure
         */
        void setNorthSeekingInitializationStatusPeriod(int period);

        /** Set the current timestamp for packets read by poll()
         *
         * Set the timestamp for the follow-up packets
         *
         * This is meant for testing purposes
         */
        void setCurrentTimestamp(base::Time const& time);

        /** Return the current timestamp for packets read by poll()
         */
        base::Time getCurrentTimestamp() const;

        /** Poll for periodic packets
         *
         * The method determines if a "packet period" just got completed, and
         * returns the period that has been completed if it did so.
         *
         * For instance, with
         *
         * <code>
         * setOrientationPeriod(1);
         * setGNSSPeriod(10);
         * </code>
         *
         * poll() will return 1 each time a full orientation-with-error has been
         * received, and 10 when the GNSS data has been received. Note that
         * within a given sensor cycle (i.e. same timestamp), it is possible
         * that a period with a higher ID is returned before one with a lower
         * ID.
         *
         * @return the ID of the period that has just been completed. 0 means
         *   that a packet has been processed, but no period was completed. -1
         *   means that poll() is attempting to re-synchronize with the period
         *   train.
         */
        int poll();

        /** Force poll() to re-synchronize to a full period
         */
        void resetPollSynchronization();
    };
}

#endif

