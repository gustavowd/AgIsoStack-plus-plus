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

#include "n2k_callbacks.hpp"
#include "j1939_callbaks.hpp"
#include "diagnostic_callbacks.hpp"

#define PI 3.141592653589793238463


static std::atomic_bool running = { true };

void signal_handler(int)
{
	running = false;
}

/*
bool reset_can_interface(const std::string& interface_name = "can0", 
                         int bitrate = 500000) {
    std::string down_cmd = "sudo ip link set " + interface_name + " down";
    std::string up_cmd = "sudo ip link set " + interface_name + 
                        " up type can bitrate " + std::to_string(bitrate);
    
    // Desliga a interface
    if (std::system(down_cmd.c_str()) != 0) {
        std::cerr << "Erro ao desligar interface CAN" << std::endl;
        return false;
    }
    
    // Pequeno delay
	std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait a bit before proceeding
    //usleep(500000); // 500ms
    
    // Liga a interface
    if (std::system(up_cmd.c_str()) != 0) {
        std::cerr << "Erro ao ligar interface CAN" << std::endl;
        return false;
    }
    
    std::cout << "Interface CAN resetada com sucesso" << std::endl;
    return true;
}
*/


class CustomLogger : public isobus::CANStackLogger
{
public:
    void sink_CAN_stack_log(CANStackLogger::LoggingLevel level, const std::string &text) override
    {
		(void)level; // Unused parameter
		// Get the current time point from the system clock
		auto now = std::chrono::system_clock::now();

		// Calculate the duration since the epoch
		auto duration = now.time_since_epoch();

		// Cast the duration to milliseconds and get the count
		long long milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    	// Print the result
    	std::cout << "Time: " << milliseconds << " ms - event:";
        std::cout << text << std::endl;
    }
};



int main()
{
	std::signal(SIGINT, signal_handler);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver = nullptr;
	canDriver = std::make_shared<isobus::SocketCANInterface>("can0");

	if (nullptr == canDriver)
	{
		std::cout << "Unable to find a CAN driver. Please make sure you have one of the above drivers installed with the library." << std::endl;
		std::cout << "If you want to use a different driver, please add it to the list above." << std::endl;
		return -1;
	}

	// Define o número de canais CAN e atribui o driver ao canal 0
	isobus::CANHardwareInterface::set_number_of_can_channels(1, 200);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		std::cout << "Failed to start hardware interface. A CAN driver might be invalid." << std::endl;
		return -2;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(250));

	// Define o NAME do ECU que é utilizado no processo de reivindicação de endereço
	isobus::NAME TestDeviceNAME(0);
	TestDeviceNAME.set_arbitrary_address_capable(true);
	TestDeviceNAME.set_industry_group(0);
	TestDeviceNAME.set_device_class(0);
	TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SteeringControl));
	TestDeviceNAME.set_identity_number(2);
	TestDeviceNAME.set_ecu_instance(0);
	TestDeviceNAME.set_function_instance(0);
	TestDeviceNAME.set_device_class_instance(0);
	TestDeviceNAME.set_manufacturer_code(1407);

	// Cria uma instância de ECU interna para este ECU
	auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);

	static CustomLogger logger;
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	// If you want to change the logging level, you can do so as followed; the default level is INFO.
	isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Debug);

	// Garante que o processo de reivindicação de endereço ocorra antes de prosseguir
	auto addressClaimedFuture = std::async(std::launch::async, [&TestInternalECU]() {
		while (!TestInternalECU->get_address_valid())
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	});
	if (addressClaimedFuture.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
	{
		std::cout << "Address claiming failed. Please make sure that your internal control function can claim a valid address." << std::endl;
		return -3;
	}

	// Inicializa o protocolo de diagnóstico do protocolo J1939
	isobus::DiagnosticProtocol diagnosticProtocol(TestInternalECU);
	//isobus::DiagnosticProtocol diagnosticProtocol(TestInternalECU, (isobus::DiagnosticProtocol::TransmitFlags::ProductIdentification | isobus::DiagnosticProtocol::TransmitFlags::ECUIdentification | isobus::DiagnosticProtocol::TransmitFlags::SoftwareIdentification));
	diagnosticProtocol.initialize();
	//diagnosticProtocol.set_j1939_mode(true); // Ativa o modo J1939 (desativa OBD2 específico para veículos leves)

	// Set a product identification string (in case someone requests it)
	diagnosticProtocol.set_product_identification_code("1234567890ABC");
	diagnosticProtocol.set_product_identification_brand("Open-Agriculture");
	diagnosticProtocol.set_product_identification_model("AgIsoStack++ CAN Stack DP Example");

	// Set a software ID string (This is what tells other ECUs what version your software is)
	diagnosticProtocol.set_software_id_field(0, "Diagnostic Protocol Example 1.0.0");
	diagnosticProtocol.set_software_id_field(1, "Another version string x.x.x.x");

	// Set an ECU ID (This is what tells other ECUs more details about your specific physical ECU)
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::HardwareID, "Hardware ID");
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::Location, "The Aether");
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::ManufacturerName, "None");
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::PartNumber, "1234");
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::SerialNumber, "1");
	diagnosticProtocol.set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::Type, "AgISOStack");

	// Important: we need to update the diagnostic protocol using the hardware interface periodic update event,
	// otherwise the diagnostic protocol will not be able to update its internal state.
	isobus::CANHardwareInterface::get_periodic_update_event_dispatcher().add_listener([&diagnosticProtocol]() {
		diagnosticProtocol.update();
	});
	std::cout << "Diagnostic Protocol initialized." << std::endl;

	// Make a few test DTCs
	isobus::DiagnosticProtocol::DiagnosticTroubleCode testDTC1(1234, isobus::DiagnosticProtocol::FailureModeIdentifier::ConditionExists, isobus::DiagnosticProtocol::LampStatus::None);
	isobus::DiagnosticProtocol::DiagnosticTroubleCode testDTC2(567, isobus::DiagnosticProtocol::FailureModeIdentifier::DataErratic, isobus::DiagnosticProtocol::LampStatus::AmberWarningLampSlowFlash);
	isobus::DiagnosticProtocol::DiagnosticTroubleCode testDTC3(8910, isobus::DiagnosticProtocol::FailureModeIdentifier::BadIntelligentDevice, isobus::DiagnosticProtocol::LampStatus::RedStopLampSolid);

	/*
	// Let's say that our ECU has the capability of a universal terminal working set (as an example) and
	// contains weak internal bus termination.
	// This info gets reported to any ECU on the bus that requests our capabilities through the
	// control function functionalities message.
	diagnosticProtocol.ControlFunctionFunctionalitiesMessageInterface.set_functionality_is_supported(isobus::ControlFunctionFunctionalities::Functionalities::MinimumControlFunction, 1, true);
	diagnosticProtocol.ControlFunctionFunctionalitiesMessageInterface.set_minimum_control_function_option_state(isobus::ControlFunctionFunctionalities::MinimumControlFunctionOptions::Type1ECUInternalWeakTermination, true);
	diagnosticProtocol.ControlFunctionFunctionalitiesMessageInterface.set_functionality_is_supported(isobus::ControlFunctionFunctionalities::Functionalities::UniversalTerminalWorkingSet, 1, true);
	*/

	// Set the DTCs active. This should put them in the DM1 message
	/*
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC1, true);
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC2, true);
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC3, true);

	std::cout << "Diagnostic Trouble Codes set active. (DM1)" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Send the DM1 for a while

	// Set the DTCs inactive. This should put them in the DM2 message
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC1, false);
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC2, false);
	diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC3, false);

	std::cout << "Diagnostic Trouble Codes set inactive. (DM2)" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Send the DM2 for a while

	diagnosticProtocol.clear_inactive_diagnostic_trouble_codes(); // All messages should now be clear!
	std::cout << "Diagnostic Trouble Codes cleared." << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Wait a bit before proceeding
	*/

	// Construct NMEA2K interface, defaulting to all messages disabled
	isobus::NMEA2000MessageInterface n2kInterface(TestInternalECU, false, false, false, false, false, false, false);
	n2kInterface.initialize();

	// The sequence identifier is set to an arbitrary value, but is in practice used to tie related messages together.
	// Example: A GNSS position message and a COG/SOG message that are not sent at the same time, but their sequence identifiers are the same,
	// then the data can be seen as taken at the same time.
	auto sequenceIdentifier = 1;

	// Enable and configure the messages we want to send
	auto &cog_sog_message = n2kInterface.get_cog_sog_transmit_message();
	cog_sog_message.set_sequence_id(sequenceIdentifier);
	cog_sog_message.set_course_over_ground_reference(isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate::CourseOverGroundReference::Error);
	cog_sog_message.set_course_over_ground(43633); // 4.3633 radians = 250 degrees
	cog_sog_message.set_speed_over_ground(200); // 2 m/s = 7.2 km/h
	n2kInterface.set_enable_sending_cog_sog_cyclically(true);

	auto &datum_message = n2kInterface.get_datum_transmit_message();
	datum_message.set_local_datum("W84");
	datum_message.set_delta_latitude(1234000); // 0.1234 degrees
	datum_message.set_delta_longitude(5678000); // 0.5678 degrees
	datum_message.set_delta_altitude(98); // 0.98 meters
	datum_message.set_reference_datum("WGS84");
	n2kInterface.set_enable_sending_datum_cyclically(true);

	auto &position_data_message = n2kInterface.get_gnss_position_data_transmit_message();
	position_data_message.set_sequence_id(sequenceIdentifier);
	auto daysSinceEpoch = std::chrono::duration_cast<std::chrono::hours>(std::chrono::system_clock::now().time_since_epoch()).count() / 24;
	position_data_message.set_position_date(static_cast<uint16_t>(daysSinceEpoch));
	auto secondsSinceMidnight = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % (24 * 60 * 60);
	position_data_message.set_position_time(static_cast<uint32_t>(secondsSinceMidnight / 0.0001));
	position_data_message.set_latitude(static_cast<int64_t>(-51.69917 / 1E-16)); // 51.69917 degrees
	position_data_message.set_longitude(static_cast<int64_t>(-5.30417 / 1E-16)); // 5.30417 degrees
	position_data_message.set_altitude(static_cast<int64_t>(-1.23 / 1E-06)); // 1.23 meters
	position_data_message.set_type_of_system(isobus::NMEA2000Messages::GNSSPositionData::TypeOfSystem::GPSPlusSBASPlusGLONASS);
	position_data_message.set_gnss_method(isobus::NMEA2000Messages::GNSSPositionData::GNSSMethod::RTKFixedInteger);
	position_data_message.set_integrity(isobus::NMEA2000Messages::GNSSPositionData::Integrity::Caution);
	position_data_message.set_number_of_space_vehicles(12); // 12 satellites
	position_data_message.set_horizontal_dilution_of_precision(-123); // -1.23
	position_data_message.set_positional_dilution_of_precision(-456); // -4.56
	position_data_message.set_geoidal_separation(-789); // -7.89 meters
	position_data_message.set_number_of_reference_stations(3);
	for (uint8_t i = 0; i < 3; i++)
	{
		position_data_message.set_reference_station(i, // Index
		                                            i + 1, // Station ID
		                                            isobus::NMEA2000Messages::GNSSPositionData::TypeOfSystem::GPSPlusGLONASS, // Type of system
		                                            i * 150 // Arbitrary age of correction (1.5s * i)
		);
	}
	n2kInterface.set_enable_sending_gnss_position_data_cyclically(true);

	n2kInterface.get_position_rapid_update_transmit_message().set_latitude(static_cast<int32_t>(51.69917 / 1E-07)); // 51.69917 degrees
	n2kInterface.get_position_rapid_update_transmit_message().set_longitude(static_cast<int32_t>(5.30417 / 1E-07)); // 5.30417 degrees
	n2kInterface.set_enable_sending_position_rapid_update_cyclically(true);

	n2kInterface.get_rate_of_turn_transmit_message().set_sequence_id(sequenceIdentifier);
	n2kInterface.get_rate_of_turn_transmit_message().set_rate_of_turn(static_cast<int32_t>(-1.234 / 3.125E-08)); // -1.234 radians/s = -70.7 degrees/s
	n2kInterface.set_enable_sending_rate_of_turn_cyclically(true);

	auto &vessel_heading_message = n2kInterface.get_vessel_heading_transmit_message();
	vessel_heading_message.set_sequence_id(sequenceIdentifier);
	vessel_heading_message.set_heading(43633); // 4.3633 radians = 250 degrees
	vessel_heading_message.set_magnetic_deviation(-4363); // -0.4363 radians = -25 degrees
	vessel_heading_message.set_magnetic_variation(-5236); // -0.5236 radians = -30 degrees
	vessel_heading_message.set_sensor_reference(isobus::NMEA2000Messages::VesselHeading::HeadingSensorReference::Error);
	n2kInterface.set_enable_sending_vessel_heading_cyclically(true);

	/*
	// PGN 126992 - System Time
    auto &system_time_message = n2kInterface.get_system_time_transmit_message();
    system_time_message.set_sequence_id(sequenceIdentifier);
    system_time_message.set_source(isobus::NMEA2000Messages::SystemTime::SystemTimeSource::GPS);
    
    time_t now = time(nullptr);
    system_time_message.set_system_date(static_cast<uint16_t>(now / 86400));
    system_time_message.set_system_time(static_cast<uint32_t>((now % 86400) * 10000));
    
    n2kInterface.set_enable_sending_system_time_cyclically(true);
    std::cout << "✓ System Time configurado (1 Hz)\n";
    
    // PGN 126993 - Heartbeat
    auto &heartbeat_message = n2kInterface.get_heartbeat_transmit_message();
    heartbeat_message.set_data_transmit_offset(0xFFFF); // Do not change
    heartbeat_message.set_sequence_counter(0);
    
    n2kInterface.set_enable_sending_heartbeat_cyclically(true);
    std::cout << "✓ Heartbeat configurado (60 Hz)\n";

	// PGN 127257 - Attitude
    auto &attitude_message = n2kInterface.get_attitude_transmit_message();
    attitude_message.set_sequence_id(sequenceIdentifier);
    attitude_message.set_yaw(static_cast<int16_t>((10.0 * M_PI / 180.0) / 0.0001));   // 10°
    attitude_message.set_pitch(static_cast<int16_t>((5.0 * M_PI / 180.0) / 0.0001));  // 5°
    attitude_message.set_roll(static_cast<int16_t>((-2.0 * M_PI / 180.0) / 0.0001));  // -2°
    
    n2kInterface.set_enable_sending_attitude_cyclically(true);
    std::cout << "✓ Attitude configurado (1 Hz) - Yaw:10° Pitch:5° Roll:-2°\n";
    
    // PGN 129539 - GNSS DOPs
    auto &gnss_dops_message = n2kInterface.get_gnss_dilution_of_precision_transmit_message();
    gnss_dops_message.set_sequence_id(sequenceIdentifier);
    gnss_dops_message.set_desired_mode(isobus::NMEA2000Messages::GNSSDilutionOfPrecision::GNSSDOPMode::ThreeDimensional);
    gnss_dops_message.set_actual_mode(isobus::NMEA2000Messages::GNSSDilutionOfPrecision::GNSSDOPMode::ThreeDimensional);
    gnss_dops_message.set_horizontal_dilution_of_precision(static_cast<uint16_t>(1.2 / 0.01));  // 1.2
    gnss_dops_message.set_vertical_dilution_of_precision(static_cast<uint16_t>(1.8 / 0.01));    // 1.8
    gnss_dops_message.set_time_dilution_of_precision(static_cast<uint16_t>(0.9 / 0.01));        // 0.9

    n2kInterface.set_enable_sending_gnss_dilution_of_precision_cyclically(true);
    std::cout << "✓ GNSS DOPs configurado (1 Hz) - HDOP:1.2 VDOP:1.8 TDOP:0.9\n";
	*/

	// Listen to incoming NMEA2K messages
	//n2kInterface.get_system_time_event_dispatcher().add_listener(on_system_time_received);
    //n2kInterface.get_heartbeat_event_dispatcher().add_listener(on_heartbeat_received);
	n2kInterface.get_course_speed_over_ground_rapid_update_event_publisher().add_listener(on_cog_sog_update);
	n2kInterface.get_datum_event_publisher().add_listener(on_datum_update);
	n2kInterface.get_gnss_position_data_event_publisher().add_listener(on_position_update);
	n2kInterface.get_position_rapid_update_event_publisher().add_listener(on_position_rapid_update);
	n2kInterface.get_rate_of_turn_event_publisher().add_listener(on_turn_rate_update);
	n2kInterface.get_vessel_heading_event_publisher().add_listener(on_vessel_heading_update);
	//n2kInterface.get_attitude_event_dispatcher().add_listener(on_attitude_received);
    //n2kInterface.get_gnss_dilution_of_precision_event_dispatcher().add_listener(on_gnss_dops_received);

	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::SoftwareIdentification), bam_software_information_callback, nullptr);
	
	// Define callbacks for receiving identification PGNs (after a request for one of these PGNs)
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::SoftwareIdentification), identification_pgn_handler, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ECUIdentificationInformation), identification_pgn_handler, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ComponentIdentification), identification_pgn_handler, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ProductIdentification), identification_pgn_handler, nullptr);

	//isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::DiagnosticMessage1), diagnostic_message_pgn_handler, nullptr);
	    // Registra callback para receber mensagens DM1
    isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(
        static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::DiagnosticMessage1),  // PGN da DM1
        diagnostic_message_pgn_handler,
        nullptr
    );

	// Register callback to handle request for ECU Information
	auto pgn_protocol = TestInternalECU->get_pgn_request_protocol().lock();
	if (pgn_protocol) {
		if (pgn_protocol->register_pgn_request_callback(
			static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ComponentIdentification),
			handle_component_information_request,
			TestInternalECU.get()
		)) {
			std::cout << "PGN request protocol callback registered successfully." << std::endl;
		} else {
			std::cout << "Failed to register PGN request protocol callback." << std::endl;
		}
	} else {
		// O weak_ptr está expirado
		std::cout << "Erro: PGN request protocol não está disponível" << std::endl;
	}
	
	std::cout << "Starting to send NMEA2K messages. Press Ctrl+C to stop." << std::endl;

	int counter = 0;
	int counter_seq = 0;
	int state = 0;
	//std::vector<uint8_t> ecuData = build_ecu_information_message();

	while (running)
	{
		// Update the NMEA2K interface periodically so that it can send messages
		n2kInterface.update();
		counter++;
		counter_seq++;
		if (counter_seq >= 20){
			counter_seq = 0;
			// Update sequence ID every second
			sequenceIdentifier++;
			n2kInterface.get_rate_of_turn_transmit_message().set_sequence_id(sequenceIdentifier);
			vessel_heading_message.set_sequence_id(sequenceIdentifier);
			cog_sog_message.set_sequence_id(sequenceIdentifier);
			position_data_message.set_sequence_id(sequenceIdentifier);
		}
		// Every 2 seconds, request the software information PGN from another device
		if (counter >= 40){
			counter = 0;
			auto targetECU = findECUByAddress(129);
			if (targetECU  == nullptr) {
				std::cerr << "ECU 129 não encontrado na rede!" << std::endl;
			}

			if (targetECU) {
				switch (state){
					case 0:
					    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
						std::cout << "║          Iniciando o ciclo de teste !                  ║\n";
						std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
						// This is how you would request a PGN from someone else. In this example, we request it from ECU with address 129.
						// Generally you'd want to replace nullptr with your partner control function as its a little nicer than just asking everyone on the bus for a PGN
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::SoftwareIdentification), TestInternalECU, targetECU);
						break;
					case 1:
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ECUIdentificationInformation), TestInternalECU, targetECU);
						break;
					case 2:
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ComponentIdentification), TestInternalECU, targetECU);
						break;
					case 3:
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ProductIdentification), TestInternalECU, targetECU);
						break;
					case 4:
						// Set the DTCs active. This should put them in the DM1 message
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC1, true);
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC2, true);
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC3, true);
						break;
					case 5:
						// 4 seconds with DTCs active
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(129026, TestInternalECU, targetECU);
						isobus::ParameterGroupNumberRequestProtocol::request_parameter_group_number(129029, TestInternalECU, targetECU);
						break;
					case 6:
						// Set the DTCs inactive. This should put them in the DM2 message
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC1, false);
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC2, false);
						diagnosticProtocol.set_diagnostic_trouble_code_active(testDTC3, false);
						break;
					/*
					case 7:
						// Alternatively, you could also just send the message yourself like this:
						isobus::CANNetworkManager::CANNetwork.send_can_message(
							static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ECUIdentificationInformation),
							ecuData.data(),
							ecuData.size(),
							TestInternalECU,
							nullptr,
							isobus::CANIdentifier::CANPriority::PriorityDefault6
						);
						break;
					*/
					default:
						break;
				}
			}
			state++;
			if (state > 6){
				state = 0;
			}
			
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	//reset_can_interface("can0", 500000);
	isobus::CANHardwareInterface::stop();
	return 0;
}
