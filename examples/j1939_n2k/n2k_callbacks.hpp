#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/nmea2000_message_definitions.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"
#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_stack_logger.hpp"

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_parameter_group_number_request_protocol.hpp"

#include "isobus/isobus/can_transport_protocol.hpp"
#include "isobus/isobus/can_message.hpp"

#include "isobus/isobus/isobus_diagnostic_protocol.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <future>
#include <iostream>
#include <iterator>
#include <memory>
#include <math.h>

#include <fstream>
#include <vector>
#include <iomanip>

#include <mutex>
#include <condition_variable>
#include <optional>

#define PI 3.141592653589793238463

void on_cog_sog_update(const std::shared_ptr<isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate> message, bool changed)
{
	std::cout << "COG/SOG update: (updated=" << changed << ")" << std::endl;
	std::cout << "  SID: " << static_cast<int>(message->get_sequence_id()) << std::endl;
	std::cout << "  COG reference: " << static_cast<int>(message->get_course_over_ground_reference()) << std::endl;
	std::cout << "  COG: " << message->get_course_over_ground() / (PI / 180) << " degrees" << std::endl;
	std::cout << "  SOG: " << message->get_speed_over_ground() * 3.6 << " km/h" << std::endl;
}

void on_datum_update(const std::shared_ptr<isobus::NMEA2000Messages::Datum> message, bool changed)
{
	std::cout << "Datum update: (updated=" << changed << ")" << std::endl;
	std::cout << "  Local datum: " << message->get_local_datum() << std::endl;
	std::cout << "  Delta latitude: " << message->get_delta_latitude() << " degrees" << std::endl;
	std::cout << "  Delta longitude: " << message->get_delta_longitude() << " degrees" << std::endl;
	std::cout << "  Delta altitude: " << message->get_delta_altitude() << " m" << std::endl;
	std::cout << "  Reference datum: " << message->get_reference_datum() << std::endl;
}

void on_position_update(const std::shared_ptr<isobus::NMEA2000Messages::GNSSPositionData> message, bool changed)
{
	const auto daysSinceEpoch = std::chrono::duration_cast<std::chrono::hours>(std::chrono::system_clock::now().time_since_epoch()).count() / 24;
	const auto secondsSinceMidnight = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % (24 * 60 * 60);

	std::cout << "Position update: (updated=" << changed << ")" << std::endl;
	std::cout << "  SID: " << static_cast<int>(message->get_sequence_id()) << std::endl;

	std::cout << "  Date: " << static_cast<int>(message->get_position_date()) << " days since epoch"
	          << " (today is " << static_cast<int>(daysSinceEpoch) << ")" << std::endl;
	std::cout << "  Time: " << static_cast<int>(message->get_position_time()) << " seconds since midnight"
	          << " (now is " << static_cast<int>(secondsSinceMidnight) << ")" << std::endl;
	std::cout << "  Latitude: " << message->get_latitude() << " degrees" << std::endl;
	std::cout << "  Longitude: " << message->get_longitude() << " degrees" << std::endl;
	std::cout << "  Altitude: " << message->get_altitude() << " m" << std::endl;
	std::cout << "  GNSS type: " << static_cast<int>(message->get_gnss_method()) << std::endl;
	std::cout << "  Method: " << static_cast<int>(message->get_gnss_method()) << std::endl;
	std::cout << "  Number of satellites: " << static_cast<int>(message->get_number_of_space_vehicles()) << std::endl;
	std::cout << "  HDOP: " << message->get_horizontal_dilution_of_precision() << std::endl;
	std::cout << "  PDOP: " << message->get_positional_dilution_of_precision() << std::endl;
	std::cout << "  Geoidal separation: " << message->get_geoidal_separation() << " m" << std::endl;
	std::cout << "  Number of reference stations: " << static_cast<int>(message->get_number_of_reference_stations()) << std::endl;
	for (uint8_t i = 0; i < message->get_number_of_reference_stations(); i++)
	{
		std::cout << "    Reference station " << static_cast<int>(i) << ":" << std::endl;
		std::cout << "      Station ID: " << static_cast<int>(message->get_reference_station_id(i)) << std::endl;
		std::cout << "      Type of system: " << static_cast<int>(message->get_reference_station_system_type(i)) << std::endl;
		std::cout << "      Age of correction: " << message->get_reference_station_corrections_age(i) << " sec" << std::endl;
	}
}

void on_position_rapid_update(const std::shared_ptr<isobus::NMEA2000Messages::PositionRapidUpdate> message, bool changed)
{
	std::cout << "Position rapid update: (updated=" << changed << ")" << std::endl;
	std::cout << "  Latitude: " << message->get_latitude() << " degrees" << std::endl;
	std::cout << "  Longitude: " << message->get_longitude() << " degrees" << std::endl;
}

void on_turn_rate_update(const std::shared_ptr<isobus::NMEA2000Messages::RateOfTurn> message, bool changed)
{
	std::cout << "Rate of turn update: (updated=" << changed << ")" << std::endl;
	std::cout << "  SID: " << static_cast<int>(message->get_sequence_id()) << std::endl;
	std::cout << "  Rate of turn: " << message->get_rate_of_turn() / (PI / 180) << " degrees/s" << std::endl;
}

void on_vessel_heading_update(const std::shared_ptr<isobus::NMEA2000Messages::VesselHeading> message, bool changed)
{
	std::cout << "Vessel heading update: (updated=" << changed << ")" << std::endl;
	std::cout << "  SID: " << static_cast<int>(message->get_sequence_id()) << std::endl;
	std::cout << "  Heading: " << message->get_heading() / (PI / 180) << " degrees" << std::endl;
	std::cout << "  Magnetic deviation: " << message->get_magnetic_deviation() / (PI / 180) << " degrees" << std::endl;
	std::cout << "  Magnetic variation: " << message->get_magnetic_variation() / (PI / 180) << " degrees" << std::endl;
	std::cout << "  Sensor reference: " << static_cast<int>(message->get_sensor_reference()) << std::endl;
}

/*
void on_system_time_received(const std::shared_ptr<isobus::NMEA2000Messages::SystemTime> message, 
                             bool changed) {
    if (changed) {
        std::cout << "\n╔════════════════════════════════════════╗\n";
        std::cout << "║  PGN 126992 - SYSTEM TIME RECEBIDO    ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        
        std::cout << "SID: " << static_cast<int>(message->get_sequence_id()) << "\n";
        std::cout << "Source: " << static_cast<int>(message->get_source()) << "\n";
        std::cout << "Date (days since 1970): " << message->get_system_date() << "\n";
        
        // Converter tempo (unidades de 0.0001s desde meia-noite)
        uint32_t time_raw = message->get_system_time();
        double seconds = time_raw / 10000.0;
        int hours = static_cast<int>(seconds / 3600);
        int minutes = static_cast<int>((seconds - hours * 3600) / 60);
        double secs = seconds - hours * 3600 - minutes * 60;
        
        std::cout << "Time: " << std::setfill('0')
                  << std::setw(2) << hours << ":"
                  << std::setw(2) << minutes << ":"
                  << std::fixed << std::setprecision(4) << secs << " UTC\n";
    }
}

void on_heartbeat_received(const std::shared_ptr<isobus::NMEA2000Messages::Heartbeat> message,
                           bool changed) {
    if (changed) {
        std::cout << "\n╔════════════════════════════════════════╗\n";
        std::cout << "║  PGN 126993 - HEARTBEAT RECEBIDO      ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        
        uint16_t offset = message->get_data_transmit_offset();
        std::cout << "Data Transmit Offset: ";
        if (offset == 0xFFFF) {
            std::cout << "Do not change\n";
        } else if (offset == 0) {
            std::cout << "Transmit immediately\n";
        } else {
            std::cout << (offset * 0.01) << " seconds\n";
        }
        
        std::cout << "Sequence Counter: " << static_cast<int>(message->get_sequence_counter()) << "\n";
    }
}

void on_attitude_received(const std::shared_ptr<isobus::NMEA2000Messages::Attitude> message,
                         bool changed) {
    if (changed) {
        std::cout << "\n╔════════════════════════════════════════╗\n";
        std::cout << "║  PGN 127257 - ATTITUDE RECEBIDO       ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        
        std::cout << "SID: " << static_cast<int>(message->get_sequence_id()) << "\n";
        std::cout << std::fixed << std::setprecision(2);
        
        int16_t yaw = message->get_yaw();
        if (yaw != static_cast<int16_t>(0x7FFF)) {
            double yaw_deg = (yaw * 0.0001) * 180.0 / M_PI;
            std::cout << "Yaw:   " << yaw_deg << "°\n";
        } else {
            std::cout << "Yaw:   N/A\n";
        }
        
        int16_t pitch = message->get_pitch();
        if (pitch != static_cast<int16_t>(0x7FFF)) {
            double pitch_deg = (pitch * 0.0001) * 180.0 / M_PI;
            std::cout << "Pitch: " << pitch_deg << "°\n";
        } else {
            std::cout << "Pitch: N/A\n";
        }
        
        int16_t roll = message->get_roll();
        if (roll != static_cast<int16_t>(0x7FFF)) {
            double roll_deg = (roll * 0.0001) * 180.0 / M_PI;
            std::cout << "Roll:  " << roll_deg << "°\n";
        } else {
            std::cout << "Roll:  N/A\n";
        }
    }
}

void on_gnss_dops_received(const std::shared_ptr<isobus::NMEA2000Messages::GNSSDilutionOfPrecision> message,
                          bool changed) {
    if (changed) {
        std::cout << "\n╔════════════════════════════════════════╗\n";
        std::cout << "║  PGN 129539 - GNSS DOPs RECEBIDO      ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        
        std::cout << "SID: " << static_cast<int>(message->get_sequence_id()) << "\n";
        
        auto desired = message->get_desired_mode();
        auto actual = message->get_actual_mode();
        
        const char* modes[] = {"1D", "2D", "3D", "Auto", "Reserved", "Reserved", "Error", "N/A"};
        std::cout << "Desired Mode: " << modes[static_cast<int>(desired)] << "\n";
        std::cout << "Actual Mode:  " << modes[static_cast<int>(actual)] << "\n";
        
        std::cout << std::fixed << std::setprecision(2);
        
        uint16_t hdop = message->get_horizontal_dilution_of_precision();
        if (hdop != 0xFFFF) {
            std::cout << "HDOP: " << (hdop * 0.01) << "\n";
        } else {
            std::cout << "HDOP: N/A\n";
        }
        
        uint16_t vdop = message->get_vertical_dilution_of_precision();
        if (vdop != 0xFFFF) {
            std::cout << "VDOP: " << (vdop * 0.01) << "\n";
        } else {
            std::cout << "VDOP: N/A\n";
        }
        
        uint16_t tdop = message->get_time_dilution_of_precision();
        if (tdop != 0xFFFF) {
            std::cout << "TDOP: " << (tdop * 0.01) << "\n";
        } else {
            std::cout << "TDOP: N/A\n";
        }
    }
}
*/