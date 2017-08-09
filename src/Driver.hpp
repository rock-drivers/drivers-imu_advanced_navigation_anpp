#ifndef ADVANCED_NAVIGATION_ANPP_DRIVER_HPP
#define ADVANCED_NAVIGATION_ANPP_DRIVER_HPP

#include <advanced_navigation_anpp/Types.hpp>
#include <iodrivers_base/Driver.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace advanced_navigation_anpp 
{
    enum RESET_MODE
    {
        RESET_COLD,
        RESET_HOT,
        RESET_FACTORY
    };

    class Driver : public iodrivers_base::Driver
    {
    private:
        bool mUseDeviceTime = false;

        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

    public:
        Driver();

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

        /** Set the base packet period
         *
         * When the device generates packets periodically, it is done at a
         * multiples of the base period. This methods set the period
         */
        void setBasePacketPeriod(base::Time period);

        /** Read the base packet period currently used by the device
         *
         * @see setBasePacketPeriod
         */
        base::Time readBasePacketPeriod();
    
        /** Whether there is a GNSS unit connected to the IMU or not
         */
        bool useGNSS(bool enable);

        /** Read all the available information that fit a RigidBodyState
         * structure
         */
        base::samples::RigidBodyState readFullRigidBodyState();

        /** Set the period at which the orientation should be generated
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         * @param with_errors if true, generate the orientation errors at the
         *   same period. Otherwise, do not generate them
         *
         * @see setBasePacketPeriod
         */
        void setOrientationPeriod(int period, bool with_errors = true);

        /** Set the period at which the velocity in the NED frame should be
         * generated
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setNEDVelocityPeriod(int period);

        /** Set the period at which the acceleration in body frame
         * should be generated. This acceleration does not contain the
         * acceleration of gravity
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setBodyAccelerationPeriod(int period);

        /** Set the period at which the velocity in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setBodyVelocityPeriod(int period);

        /** Set the period at which the angular velocities in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setAngularVelocityPeriod(int period);

        /** Set the period at which the angular accelerations in body frame
         * should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setAngularAccelerationPeriod(int period);

        /** Set the period at which the raw sensors should be generated.
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setRawSensorsPeriod(int period, bool with_magnetic_field = true);

        /** Set the period at which geodetic position should be generated
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setGNSSPeriod(int period);

        /** Set the period at which GNSS satellite summary should be generated
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setGNSSSatelliteSummaryPeriod(int period);

        /** Set the period at which GNSS detailed satellite info should be generated
         *
         * Periodic messages are processed by poll()
         *
         * @param period the period in multiples of the base packet period
         *
         * @see setBasePacketPeriod
         */
        void setGNSSSatelliteDetailsPeriod(int period);
    };
}

#endif

