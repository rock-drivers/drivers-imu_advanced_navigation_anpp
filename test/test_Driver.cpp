#include "test_Helpers.hpp"
#include <advanced_navigation_anpp/Driver.hpp>
#include <advanced_navigation_anpp/Protocol.hpp>

using namespace std;
using namespace advanced_navigation_anpp;
using advanced_navigation_anpp::protocol::Header;
using ::testing::ElementsAre;
using ::testing::ContainerEq;

struct DriverTest : DriverTestBase
{
    DriverTest()
    {
        openTestURI();
    }
};

TEST_F(DriverTest, extractPacket_waits_for_a_full_header)
{
    pushDataToDriver( { 0 } );
    ASSERT_THROW(readPacket(), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, extractPacket_waits_for_the_payload_of_a_valid_header)
{
    pushDataToDriver( { 0xFF, 0, 1, 0, 0 } );
    ASSERT_THROW(readPacket(), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, extractPacket_returns_a_packet_which_validates_both_the_header_and_payload_checksums)
{
    std::vector<uint8_t> expected { 0x00, 0, 1, 0x00, 0xFF, 0xFF };
    pushDataToDriver(expected);
    auto packet = readPacket();
    EXPECT_THAT(packet, ContainerEq(expected));
}

TEST_F(DriverTest, extractPacket_ignores_starting_bytes_that_do_not_validate_on_the_LRC)
{
    pushDataToDriver( { 0x10, 0x10, 0x10, 0, 1, 0, 0 } );
    ASSERT_THROW(readPacket(), iodrivers_base::TimeoutError);
    ASSERT_EQ(3, getQueuedBytes());
}

TEST_F(DriverTest, extractPacket_successfully_realigns_on_a_packet_header_towards_the_end_of_the_buffer)
{
    std::vector<uint8_t> expected { 0x00, 0, 1, 0x00, 0xFF, 0xFF };
    pushDataToDriver( { 0x10, 0x10, 0x10, 0x00, 0, 1, 0, 0xFF, 0xFF } );
    auto packet = readPacket();
    EXPECT_THAT(packet, ContainerEq(expected));
}

TEST_F(DriverTest, UseDeviceTime_is_false_by_default)
{
    ASSERT_FALSE(driver.getUseDeviceTime());
}

TEST_F(DriverTest, enabling_UseDeviceTime_sets_the_UnixTime_packet_period_to_1)
{ IODRIVERS_BASE_MOCK();
    auto periods = makePacket<protocol::PacketPeriods>({ 0, 0, 21, 1, 0, 0, 0 });
    EXPECT_REPLY(
            periods,
            makeAcknowledge(periods, ACK_SUCCESS));
    driver.setUseDeviceTime(true);
    ASSERT_TRUE(driver.getUseDeviceTime());
}

TEST_F(DriverTest, disabling_UseDeviceTime_sets_the_UnixTime_packet_period_to_1)
{ IODRIVERS_BASE_MOCK();
    auto periods = makePacket<protocol::PacketPeriods>({ 0, 0, 21, 0, 0, 0, 0 });
    EXPECT_REPLY(
            periods,
            makeAcknowledge(periods, ACK_SUCCESS));
    driver.setUseDeviceTime(false);
    ASSERT_FALSE(driver.getUseDeviceTime());
}

TEST_F(DriverTest, a_failure_while_changing_UseDeviceTime_does_not_change_the_flag)
{ IODRIVERS_BASE_MOCK();
    auto periods = makePacket<protocol::PacketPeriods>({ 0, 0, 21, 1, 0, 0, 0 });
    EXPECT_REPLY(
            periods,
            makeAcknowledge(periods, ACK_FAILED_OUT_OF_RANGE));
    ASSERT_THROW(driver.setUseDeviceTime(true), AcknowledgeFailure);
    ASSERT_FALSE(driver.getUseDeviceTime());
}

TEST_F(DriverTest, clearPeriodicPackets_sends_a_packet_clearing_the_UnixTime_packet_period_with_clear_existing_set)
{ IODRIVERS_BASE_MOCK();
    auto periods = makePacket<protocol::PacketPeriods>({ 0, 1, 21, 0, 0, 0, 0 });
    EXPECT_REPLY(
            periods,
            makeAcknowledge(periods, ACK_SUCCESS));
    driver.clearPeriodicPackets();
}

TEST_F(DriverTest, clearPeriodicPackets_sends_a_packet_setting_the_UnixTime_packet_with_clear_existing_set)
{ IODRIVERS_BASE_MOCK();
    auto useDeviceTime_set = makePacket<protocol::PacketPeriods>({ 0, 0, 21, 1, 0, 0, 0 });
    EXPECT_REPLY(
            useDeviceTime_set,
            makeAcknowledge(useDeviceTime_set, ACK_SUCCESS));
    auto periods = makePacket<protocol::PacketPeriods>({ 0, 1, 21, 1, 0, 0, 0 });
    EXPECT_REPLY(
            periods,
            makeAcknowledge(periods, ACK_SUCCESS));
    driver.setUseDeviceTime(true);
    driver.clearPeriodicPackets();
}

TEST_F(DriverTest, readTime_reads_the_device_time)
{ IODRIVERS_BASE_MOCK();
    std::vector<uint8_t> unix_time {
        1, 2, 3, 4,
        0x49, 0x86, 0x3, 0 }; // 230 985 microseconds

    EXPECT_REPLY(makeQuery<protocol::UnixTime>(),
                 makePacket<protocol::UnixTime>(unix_time));
    base::Time time = driver.readTime();

    uint64_t expected = 67305985230985;
    ASSERT_EQ(expected, time.toMicroseconds());
}

TEST_F(DriverTest, readStatus_reads_the_system_and_filter_status)
{ IODRIVERS_BASE_MOCK();
    // 0xC13 =>
    //   FILTER_ORIENTATION_INITIALIZED |
    //   FILTER_NAVIGATION_INITIALIZED |
    //   GNSS_2D |
    //   FILTER_MAGNETIC_HEADING_ENABLED |
    //   FILTER_VELOCITY_HEADING_ENABLED
    std::vector<uint8_t> status { 1, 2, 0x13, 0xC };

    EXPECT_REPLY(makeQuery<protocol::Status>(),
                 makePacket<protocol::Status>(status));
    auto result = driver.readStatus();

    ASSERT_EQ(0x0201, result.system_status);
    ASSERT_TRUE(result.orientation_initialized);
    ASSERT_TRUE(result.navigation_initialized);
    ASSERT_FALSE(result.heading_initialized);
    ASSERT_FALSE(result.utc_initialized);
    ASSERT_EQ(GNSS_2D, result.gnss_status);
}


TEST_F(DriverTest, readDeviceInformation_queries_the_information_and_returns_the_unmarshalled_packet)
{ IODRIVERS_BASE_MOCK();

    uint8_t device_info[29];
    device_info[0] = 156;
    device_info[1] = 3;
    device_info[2] = 24;
    device_info[3] = 0x9e;
    device_info[4] = 0xAB;
    std::iota(device_info + 5, device_info + 29, 0);
    EXPECT_REPLY(
            { 0x9a, 1, 1, 0x93, 0xD1, 3 },
            std::vector<uint8_t>(device_info, device_info + 29));
    DeviceInformation info = driver.readDeviceInformation();
    ASSERT_EQ(0x03020100, info.software_version);
    ASSERT_EQ(0x07060504, info.device_id);
    ASSERT_EQ(0x0b0a0908, info.hardware_revision);
    ASSERT_EQ(0x0f0e0d0c, info.serial_number_part0);
    ASSERT_EQ(0x13121110, info.serial_number_part1);
    ASSERT_EQ(0x17161514, info.serial_number_part2);
}

void vector_concat(std::vector<uint8_t>& a, std::vector<uint8_t>& b)
{
    a.insert(a.end(), b.begin(), b.end());
}

TEST_F(DriverTest, readConfiguration_queries_the_information_and_returns_the_unmarshalled_packet)
{ IODRIVERS_BASE_MOCK();

    std::vector<uint8_t> packet_timer_period { 0, 1, 2, 3 };
    std::vector<uint8_t> alignment { 0 };
    for (int i = 0; i < 18; ++i)
        vector_concat(alignment, TEST_FP4[i % 12].binary);

    std::vector<uint8_t> filter_options(protocol::FilterOptions::SIZE, 0);
    filter_options[0] = 1;
    filter_options[1] = VEHICLE_3D_UNDERWATER;
    filter_options[2] = 1;
    filter_options[4] = 0;
    filter_options[5] = 1;
    filter_options[6] = 0;
    filter_options[7] = 1;
    std::vector<uint8_t> magnetic_calibration { MAGNETIC_CALIBRATION_2D_IN_PROGRESS };
    for (int i = 0; i < 12; ++i)
        vector_concat(magnetic_calibration, TEST_FP4[i].binary);
    std::vector<uint8_t> magnetic_calibration_status { MAGNETIC_CALIBRATION_2D_IN_PROGRESS, 1, 2 };

    EXPECT_REPLY(makeQuery<protocol::PacketTimerPeriod>(),
                 makePacket<protocol::PacketTimerPeriod>(packet_timer_period));
    EXPECT_REPLY(makeQuery<protocol::Alignment>(),
                 makePacket<protocol::Alignment>(alignment));
    EXPECT_REPLY(makeQuery<protocol::FilterOptions>(),
                 makePacket<protocol::FilterOptions>(filter_options));
    EXPECT_REPLY(makeQuery<protocol::MagneticCalibrationValues>(),
                 makePacket<protocol::MagneticCalibrationValues>(magnetic_calibration));
    EXPECT_REPLY(makeQuery<protocol::MagneticCalibrationStatus>(),
                 makePacket<protocol::MagneticCalibrationStatus>(magnetic_calibration_status));
    CurrentConfiguration conf = driver.readConfiguration();
    ASSERT_TRUE(conf.utc_synchronization);
    ASSERT_EQ(base::Time::fromMicroseconds(0x0302), conf.packet_timer_period);
    ASSERT_EQ(base::Vector3d(TEST_FP4[9].fp, TEST_FP4[10].fp, TEST_FP4[11].fp),
            conf.gnss_antenna_offset);
    ASSERT_EQ(VEHICLE_3D_UNDERWATER, conf.vehicle_type);
    ASSERT_EQ(true, conf.enabled_internal_gnss);
    ASSERT_EQ(false, conf.enabled_atmospheric_altitude);
    ASSERT_EQ(true, conf.enabled_velocity_heading);
    ASSERT_EQ(false, conf.enabled_reversing_detection);
    ASSERT_EQ(true, conf.enabled_motion_analysis);

    ASSERT_EQ(MAGNETIC_CALIBRATION_2D_IN_PROGRESS, conf.magnetic_calibration_status);
    ASSERT_EQ(base::Vector3d(TEST_FP4[0].fp, TEST_FP4[1].fp, TEST_FP4[2].fp), conf.hard_iron_bias);
    for (int i = 0; i < 9; ++i)
        ASSERT_EQ(TEST_FP4[3 + i].fp, conf.soft_iron_transformation(i % 3, i / 3));
}

TEST_F(DriverTest, setConfiguration_applies_the_configuration)
{ IODRIVERS_BASE_MOCK();

    Configuration conf;
    conf.utc_synchronization = true;
    conf.packet_timer_period = base::Time::fromMilliseconds(12);
    conf.gnss_antenna_offset = Eigen::Vector3d(TEST_FP4[0].fp, TEST_FP4[1].fp, TEST_FP4[2].fp);
    conf.vehicle_type                 = VEHICLE_3D_UNDERWATER;
    conf.enabled_internal_gnss        = true;
    conf.enabled_atmospheric_altitude = false;
    conf.enabled_velocity_heading     = true;
    conf.enabled_reversing_detection  = false;
    conf.enabled_motion_analysis      = true;

    auto raw_packet_timer_period = makePacket<protocol::PacketTimerPeriod>({ 0, 1, 0xe0, 0x2e });
    EXPECT_REPLY(raw_packet_timer_period,
            makeAcknowledge(raw_packet_timer_period, ACK_SUCCESS));

    std::vector<uint8_t> raw_alignment_payload(protocol::Alignment::SIZE, 0);
    // Fill the DCM ones
    RAW_SET(&raw_alignment_payload[1], TEST_FP4_ONE.binary);
    RAW_SET(&raw_alignment_payload[1 + 16], TEST_FP4_ONE.binary);
    RAW_SET(&raw_alignment_payload[1 + 32], TEST_FP4_ONE.binary);
    RAW_SET(&raw_alignment_payload[1 + 36], TEST_FP4[0].binary);
    RAW_SET(&raw_alignment_payload[1 + 40], TEST_FP4[1].binary);
    RAW_SET(&raw_alignment_payload[1 + 44], TEST_FP4[2].binary);
    auto raw_alignment = makePacket<protocol::Alignment>(raw_alignment_payload);
    EXPECT_REPLY(raw_alignment,
            makeAcknowledge(raw_alignment, ACK_SUCCESS));

    auto raw_filter_options = makePacket<protocol::FilterOptions>({
            0, 5, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
    EXPECT_REPLY(raw_filter_options,
            makeAcknowledge(raw_filter_options, ACK_SUCCESS));

    driver.setConfiguration(conf);
}

struct PollTest : DriverTest
{
    PollTest()
    {
    }

    int poll(bool ignore_timestamp_handling = true)
    {
        if (ignore_timestamp_handling)
            driver.setCurrentTimestamp(base::Time::now());
        return driver.poll();
    }
};

TEST_F(PollTest, poll_returns_the_period_ID_of_a_single_packet_period)
{ IODRIVERS_BASE_MOCK();
    EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 5);
    EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 0);
    driver.setOrientationPeriod(5, false);
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(5, poll());
}

TEST_F(PollTest, poll_returns_the_period_ID_only_of_the_last_packet_in_the_period)
{ IODRIVERS_BASE_MOCK();
    EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 5);
    EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 5);
    driver.setOrientationPeriod(5, true);
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(0, poll());
    ASSERT_EQ(5, poll());
}

TEST_F(PollTest, poll_handles_formerly_enabled_packets)
{ IODRIVERS_BASE_MOCK();
    EXPECT_PACKET_PERIOD(protocol::NEDVelocity::ID, 5);
    EXPECT_PACKET_PERIOD(protocol::NEDVelocityStandardDeviation::ID, 0);
    EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 5);
    EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 0);
    EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 0);
    EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 0);
    driver.setNEDVelocityPeriod(5, false);
    driver.setOrientationPeriod(5, false);
    // Last packet ID here is QuaternionOrientation
    driver.setOrientationPeriod(0, false);
    // Last packet ID here should be NEDVelocity
    pushDataToDriver(makePacket<protocol::NEDVelocity>());
    ASSERT_EQ(5, poll());
}

TEST_F(PollTest, poll_handles_interleaved_periods)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::NEDVelocity::ID, 2);
        EXPECT_PACKET_PERIOD(protocol::NEDVelocityStandardDeviation::ID, 2);
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        // ID order is
        //   NEDVelocityStandardDeviation = 25
        //   EulerOrientationStandardDeviation = 26
        //   NEDVelocity = 35
        //   QuaternionOrientation = 40
        driver.setNEDVelocityPeriod(2);
        driver.setOrientationPeriod(4);
    }

    pushDataToDriver(makePacket<protocol::NEDVelocityStandardDeviation>());
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::NEDVelocity>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(0, poll());
    ASSERT_EQ(0, poll());
    ASSERT_EQ(2, poll());
    ASSERT_EQ(4, poll());
}

TEST_F(PollTest, poll_waits_for_a_new_cycle_to_process_packets)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        driver.setOrientationPeriod(4);
    }

    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(0, poll(false));
    ASSERT_EQ(4, poll(false));
}

TEST_F(PollTest, poll_uses_Time_now_to_process_packets_if_UseDeviceTime_is_false)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        driver.setOrientationPeriod(4);
    }

    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(-1, poll(false));
    base::Time before = base::Time::now();
    ASSERT_EQ(0, poll(false));
    base::Time after = base::Time::now();
    ASSERT_EQ(4, poll(false));
    ASSERT_TRUE(before <= driver.getCurrentTimestamp());
    ASSERT_TRUE(driver.getCurrentTimestamp() <= after);
}

TEST_F(PollTest, poll_ignores_UnixTime_packets_if_UseDeviceTime_is_false)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        driver.setOrientationPeriod(4);
    }

    pushDataToDriver(makePacket<protocol::UnixTime>());
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(-1, poll(false));
    base::Time before = base::Time::now();
    ASSERT_EQ(0, poll(false));
    base::Time after = base::Time::now();
    ASSERT_EQ(4, poll(false));
    ASSERT_TRUE(before <= driver.getCurrentTimestamp());
    ASSERT_TRUE(driver.getCurrentTimestamp() <= after);
}

TEST_F(PollTest, poll_uses_UnixTime_packets_to_process_packets_if_UseDeviceTime_is_true)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::UnixTime::ID, 1);
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        driver.setUseDeviceTime(true);
        driver.setOrientationPeriod(4);
    }

    std::vector<uint8_t> unix_time {
        1, 2, 3, 4,
        0x49, 0x86, 0x3, 0 }; // 230 985 microseconds
    uint64_t expected = 67305985230985;

    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    pushDataToDriver(makePacket<protocol::UnixTime>(unix_time));
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(-1, poll(false));
    ASSERT_EQ(0, poll(false));
    ASSERT_EQ(expected, driver.getCurrentTimestamp().toMicroseconds());
    ASSERT_EQ(0, poll(false));
    ASSERT_EQ(4, poll(false));
    ASSERT_EQ(expected, driver.getCurrentTimestamp().toMicroseconds());
}

TEST_F(PollTest, poll_does_not_return_the_period_ID_of_the_UnixTime_packet)
{
    { IODRIVERS_BASE_MOCK();
        EXPECT_PACKET_PERIOD(protocol::QuaternionOrientation::ID, 4);
        EXPECT_PACKET_PERIOD(protocol::EulerOrientationStandardDeviation::ID, 4);
        driver.setOrientationPeriod(4);
    }

    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::DeviceInformation>());
    driver.readDeviceInformation();
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(-1, poll(false));
    pushDataToDriver(makePacket<protocol::EulerOrientationStandardDeviation>());
    pushDataToDriver(makePacket<protocol::QuaternionOrientation>());
    ASSERT_EQ(0, poll(false));
    ASSERT_EQ(4, poll(false));
}

TEST_F(DriverTest, synchronous_reading_in_the_middle_of_a_train_does_not_lead_to_reporting_partial_periods)
{
    // Whenever a read method is used, it will drop any unintended packages,
    // which can lead to partially updated structures.
    //
    // The driver is supposed to wait for a new full cycle before it's allowed
    // to process again
}

