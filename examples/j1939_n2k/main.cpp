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


static std::atomic_bool running = { true };

void signal_handler(int)
{
	running = false;
}


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

void bam_software_information_callback(const isobus::CANMessage &message, void *)
{
	auto source = message.get_source_control_function();
	auto identifier = message.get_identifier();
	auto pgn = identifier.get_parameter_group_number();
	std::cout << std::endl << "[BAM] Received PGN " << static_cast<int>(pgn) << " from " << static_cast<int>(source->get_address()) << std::endl;
  	std::cout << "Data lenght: " << message.get_data_length() << std::endl;
	auto data_vec = message.get_data();
	for (const auto& element : data_vec) {
        std::cout << element;
    }
	std::cout << std::endl << std::endl;
}

void identification_pgn_handler(const isobus::CANMessage &message, void *){
	auto source = message.get_source_control_function();
	auto identifier = message.get_identifier();
	auto pgn = identifier.get_parameter_group_number();
	if (message.is_broadcast()) {
		// endereço de broadcast
		std::cout << std::endl << "[BAM] Received PGN " << static_cast<int>(pgn) << " from " << static_cast<int>(source->get_address()) << std::endl;
	}else{
		// endereço específico
		std::cout << std::endl << "[CMTP] Received PGN " << static_cast<int>(pgn) << " from " << static_cast<int>(source->get_address()) << std::endl;
	}
  	std::cout << "Data lenght: " << message.get_data_length() << std::endl;
	auto data_vec = message.get_data();
	for (const auto& element : data_vec) {
        std::cout << element;
    }
	std::cout << std::endl << std::endl;
}


// Estrutura para DTC
struct J1939_DTC {
    uint32_t spn;
    uint8_t fmi;
    uint8_t oc;
    uint8_t conversion;
};

// Estados de lâmpada
enum class LampState {
    OFF = 0,
    ON = 1,
    ERROR = 2,
    NOT_AVAILABLE = 3
};

// Estados de flash
enum class FlashState {
    SOLID = 0,
    SLOW = 1,
    FAST = 2,
    RESERVED = 3
};

// Converte estado de lâmpada para string
const char* lamp_state_to_string(LampState state) {
    switch(state) {
        case LampState::OFF: return "OFF";
        case LampState::ON: return "ON";
        case LampState::ERROR: return "ERROR";
        case LampState::NOT_AVAILABLE: return "N/A";
        default: return "UNKNOWN";
    }
}

// Converte estado de flash para string
const char* flash_state_to_string(FlashState state) {
    switch(state) {
        case FlashState::SOLID: return "SOLID";
        case FlashState::SLOW: return "SLOW FLASH";
        case FlashState::FAST: return "FAST FLASH";
        case FlashState::RESERVED: return "RESERVED";
        default: return "UNKNOWN";
    }
}

// Converte FMI para string descritiva
const char* fmi_to_string(uint8_t fmi) {
    switch(fmi) {
        case 0: return "Data Valid Above Normal";
        case 1: return "Data Valid Below Normal";
        case 2: return "Data Erratic";
        case 3: return "Voltage Above Normal";
        case 4: return "Voltage Below Normal";
        case 5: return "Current Below Normal";
        case 6: return "Current Above Normal";
        case 7: return "Mechanical System Not Responding";
        case 8: return "Abnormal Frequency";
        case 9: return "Abnormal Update Rate";
        case 10: return "Abnormal Rate of Change";
        case 11: return "Root Cause Not Known";
        case 12: return "Bad Intelligent Device";
        case 13: return "Out of Calibration";
        case 14: return "Special Instructions";
        case 31: return "Condition Exists";
        default: return "Reserved/Unknown";
    }
}

// Extrai estado de lâmpada de um byte (2 bits por lâmpada)
LampState extract_lamp_state(uint8_t byte, uint8_t lamp_position) {
    uint8_t shift = lamp_position * 2;
    return static_cast<LampState>((byte >> shift) & 0x03);
}

// Extrai estado de flash de um byte (2 bits por lâmpada)
FlashState extract_flash_state(uint8_t byte, uint8_t lamp_position) {
    uint8_t shift = lamp_position * 2;
    return static_cast<FlashState>((byte >> shift) & 0x03);
}

// Decodifica um DTC de 4 bytes
J1939_DTC decode_dtc(const uint8_t* buffer) {
    J1939_DTC dtc;
    
    // SPN: 19 bits (bytes 0-2)
    dtc.spn = static_cast<uint32_t>(buffer[0]) |
              (static_cast<uint32_t>(buffer[1]) << 8) |
              ((static_cast<uint32_t>(buffer[2] >> 5) & 0x07) << 16);
    
    // FMI: 5 bits (byte 2, bits 3-7)
    dtc.fmi = buffer[2] & 0x1F;

    // OC: 7 bits (byte 3, bits 0-6)
    dtc.oc = buffer[3] & 0x7F;
    
    // Conversion Method: 1 bit (byte 3, bit 7)
    dtc.conversion = (buffer[3] >> 7) & 0x01;
    
    return dtc;
}

// Função principal para processar e imprimir mensagem DM1
void process_and_print_dm1(const isobus::CANMessage& message) {
    std::vector<uint8_t> data = message.get_data();
    
    // Verifica se tem pelo menos os 2 bytes de cabeçalho
    if (data.size() < 2) {
        std::cout << "Erro: Mensagem DM1 muito curta\n";
        return;
    }
    
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║          MENSAGEM DM1 - DTCs ATIVOS                    ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
    
    if ((data[0] == 0xFF) && (data[1] == 0xFF)) {
		std::cout << "DM1 em modo ISOBUS\n\r";
		std::cout << "\n\r";
	}else{
		// Byte 0: Status das Lâmpadas
		uint8_t lamp_status = data[0];
		std::cout << "┌─ STATUS DAS LÂMPADAS ─────────────────────────────────┐\n";
		std::cout << "│ Byte 0 (Lamp Status): 0x" << std::hex << std::setfill('0') 
				<< std::setw(2) << static_cast<int>(lamp_status) << std::dec << "\n";
		
		LampState protect = extract_lamp_state(lamp_status, 0);
		LampState amber = extract_lamp_state(lamp_status, 1);
		LampState red_stop = extract_lamp_state(lamp_status, 2);
		LampState mil = extract_lamp_state(lamp_status, 3);
		
		std::cout << "│   Protect Lamp:        " << std::setw(15) << std::left 
				<< lamp_state_to_string(protect) << "│\n";
		std::cout << "│   Amber Warning Lamp:  " << std::setw(15) << std::left 
				<< lamp_state_to_string(amber) << "│\n";
		std::cout << "│   Red Stop Lamp:       " << std::setw(15) << std::left 
				<< lamp_state_to_string(red_stop) << "│\n";
		std::cout << "│   MIL (Check Engine):  " << std::setw(15) << std::left 
				<< lamp_state_to_string(mil) << "│\n";
		std::cout << "└───────────────────────────────────────────────────────┘\n\n";
		
		// Byte 1: Status de Flash
		uint8_t flash_status = data[1];
		std::cout << "┌─ STATUS DE FLASH ─────────────────────────────────────┐\n";
		std::cout << "│ Byte 1 (Flash Status): 0x" << std::hex << std::setfill('0') 
				<< std::setw(2) << static_cast<int>(flash_status) << std::dec << "\n";

		FlashState protect_flash = extract_flash_state(flash_status, 0);
		FlashState amber_flash = extract_flash_state(flash_status, 1);
		FlashState red_flash = extract_flash_state(flash_status, 2);
		FlashState mil_flash = extract_flash_state(flash_status, 3);
		
		std::cout << "│   Protect Lamp:        " << std::setw(15) << std::left 
				<< flash_state_to_string(protect_flash) << "│\n";
		std::cout << "│   Amber Warning Lamp:  " << std::setw(15) << std::left 
				<< flash_state_to_string(amber_flash) << "│\n";
		std::cout << "│   Red Stop Lamp:       " << std::setw(15) << std::left 
				<< flash_state_to_string(red_flash) << "│\n";
		std::cout << "│   MIL (Check Engine):  " << std::setw(15) << std::left 
				<< flash_state_to_string(mil_flash) << "│\n";
		std::cout << "└───────────────────────────────────────────────────────┘\n\n";
	}
    
    // Calcular número de DTCs
    uint8_t dtc_count = (data.size() - 2) / 4;
	if ((data[2] == 0) && (data[3] == 0) && (data[4] == 0) && (data[5] == 0)){
    	dtc_count = 0;
    }

	if (dtc_count){    
		std::cout << "┌─ DIAGNOSTIC TROUBLE CODES (DTCs) ─────────────────────┐\n";
		std::cout << "│ Total de DTCs Ativos: " << static_cast<int>(dtc_count) << "\n";
		std::cout << "└───────────────────────────────────────────────────────┘\n\n";
		
		// Processar cada DTC
		for (uint8_t i = 0; i < dtc_count; i++) {
			size_t offset = 2 + (i * 4);
			
			if (offset + 4 > data.size()) {
				std::cout << "Erro: DTC " << static_cast<int>(i+1) << " incompleto\n";
				break;
			}
			
			J1939_DTC dtc = decode_dtc(&data[offset]);
			
			std::cout << "╔═══════════════════════════════════════════════════════╗\n";
			std::cout << "║ DTC #" << static_cast<int>(i+1) << std::setfill(' ') << std::setw(48) << " " << "║\n";
			std::cout << "╠═══════════════════════════════════════════════════════╣\n";
			std::cout << "║ SPN (Suspect Parameter Number): " << std::setw(22) << std::left 
					<< dtc.spn << "║\n";
			std::cout << "║ FMI (Failure Mode Identifier):  " << std::setw(6) << static_cast<int>(dtc.fmi) 
					<< std::setw(16) << " " << "║\n";
			std::cout << "║   Description: " << std::setw(39) << std::left 
					<< fmi_to_string(dtc.fmi) << "║\n";
			std::cout << "║ OC (Occurrence Count):           " << std::setw(21) << static_cast<int>(dtc.oc) 
					<< "║\n";
			std::cout << "║ CM (Conversion Method):          " << std::setw(21) << static_cast<int>(dtc.conversion) 
					<< "║\n";
			std::cout << "║                                                       ║\n";
			std::cout << "║ Raw bytes: " << std::hex << std::setfill('0');
			for (int j = 0; j < 4; j++) {
				std::cout << "0x" << std::setw(2) << static_cast<int>(data[offset + j]) << " ";
			}
			std::cout << std::dec << std::setfill(' ') << std::setw(23) << " " << "║\n";
			std::cout << "╚═══════════════════════════════════════════════════════╝\n\n";
		}
	}else{
		std::cout << static_cast<int>(dtc_count) << " DTCs ativos!" << "\n\r";
	}
}

void diagnostic_message_pgn_handler(const isobus::CANMessage &message, void *){
	auto source = message.get_source_control_function();
	auto identifier = message.get_identifier();
	auto pgn = identifier.get_parameter_group_number();
	if (message.is_broadcast()) {
		// endereço de broadcast
		std::cout << std::endl << "[BAM] Received Diagnostic PGN " << static_cast<int>(pgn) << " from " << static_cast<int>(source->get_address()) << std::endl;
	}else{
		// endereço específico
		std::cout << std::endl << "[CMTP] Received Diagnostic PGN " << static_cast<int>(pgn) << " from " << static_cast<int>(source->get_address()) << std::endl;
	}
  	std::cout << "Data lenght: " << message.get_data_length() << std::endl;
	process_and_print_dm1(message);
	/*
	auto data_vec = message.get_data();
	std::cout << "Status das Lâmpadas: " << static_cast<int>(data_vec[0]) << "\n\r";
	std::cout << "Flash das Lâmpadas: " << static_cast<int>(data_vec[1]) << "\n\r";
	uint8_t dtc_count = (message.get_data_length() - 2) / 4;
	if ((data_vec[2] == 0) && (data_vec[3] == 0) && (data_vec[4] == 0) && (data_vec[5] == 0)){
    	dtc_count = 0;
    }
	std::cout << static_cast<int>(dtc_count) << " DTCs ativos!" << "\n\r";
	*/

	std::cout << std::endl << std::endl;
}

bool software_information_pgn_request_handler(std::uint32_t parameterGroupNumber,
                                               std::shared_ptr<isobus::ControlFunction>,
                                               bool &acknowledge,
                                               isobus::AcknowledgementType &acknowledgeType,
                                               void *)
{
	bool retVal;

	// This function will be called whenever PGN EF00 is requested.
	// Add whatever logic you want execute to on reciept of a PROPA request.
	// One normal thing to do would be to send a CAN message with that PGN.
	std::cout << "Received PGN:" << parameterGroupNumber << std::endl;

	// In this example though, we'll simply acknowledge the request.
	if (static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::SoftwareIdentification) == parameterGroupNumber)
	{
		acknowledge = true;
		acknowledgeType = isobus::AcknowledgementType::Positive;
		retVal = true;
	}
	else
	{
		// If any other PGN is requested, since this callback doesn't handle it, return false.
		// Returning false will tell the stack to keep looking for another callback (if any exist) to handle this PGN
		retVal = false;
	}
	return retVal;
}

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

std::shared_ptr<isobus::PartneredControlFunction> findECUByAddress(int address) {
	auto controlFunctions = isobus::CANNetworkManager::CANNetwork.get_control_functions(true);
	for (auto& cf: controlFunctions){
		std::cout << "ECU Address: " << static_cast<int>(cf->get_address()) << std::endl;
        if (static_cast<int>(cf->get_address()) == address) {
            return std::static_pointer_cast<isobus::PartneredControlFunction>(cf);
        }
	}
    
    return nullptr; // ECU não encontrado na rede
}


// Função para construir mensagem Component Information
std::vector<uint8_t> build_component_information_message() {
    std::vector<uint8_t> message;
    
    // Make
    std::string make = "20254";
    message.insert(message.end(), make.begin(), make.end());
    message.push_back('*'); // Delimiter
    
    // Model Name
    std::string model = "2025-PPT";
    message.insert(message.end(), model.begin(), model.end());
    message.push_back('*'); // Delimiter
    
    // Serial Number
    std::string serial = "1243-13245-1245";
    message.insert(message.end(), serial.begin(), serial.end());
    message.push_back('*'); // Delimiter
    
    // Unit Number
    std::string unit_number = "Unit 8K";
    message.insert(message.end(), unit_number.begin(), unit_number.end());
    message.push_back('*'); // Delimiter
    
    return message;
}


// Função auxiliar para enviar resposta multipacket
bool send_multipacket_response(
	std::uint32_t pgn,
	const std::vector<uint8_t>& data,
	std::shared_ptr<isobus::ControlFunction> destination
) {
	// Usar transport protocol para enviar dados
	auto allInternalCFs = isobus::CANNetworkManager::CANNetwork.get_internal_control_functions();
	std::shared_ptr<isobus::InternalControlFunction> internalECU;
	for (auto& cf: allInternalCFs){
		if (cf->get_address() == 128){
			internalECU = cf;
			break;
		}
	}
	std::cout << "Src addr: " << static_cast<int>(internalECU->get_address());
	if (destination != nullptr){
		std::cout << " - Dst addr: " << static_cast<int>(destination->get_address()) << std::endl;
		return isobus::CANNetworkManager::CANNetwork.send_can_message(
			pgn,
			data.data(),
			data.size(),
			internalECU,
			destination,
			isobus::CANIdentifier::CANPriority::PriorityDefault6
		);
	}else{
		std::cout << std::endl;
		return false;
	}
}


// Implementação do ECU Information
bool handle_component_information_request(
								std::uint32_t parameterGroupNumber,
								std::shared_ptr<isobus::ControlFunction> requestingControlFunction,
								bool &acknowledge,
								isobus::AcknowledgementType &acknowledgeType,
								void *)
{
	std::cout << "[PGN Request] - Component Information pgn:" << parameterGroupNumber << std::endl;
	
	std::vector<uint8_t> ecuData = build_component_information_message();
	//std::cout << "Message size: " << ecuData.size() << std::endl;
	bool success = send_multipacket_response(parameterGroupNumber, ecuData, requestingControlFunction);
	
	acknowledge = true;
	acknowledgeType = success ? isobus::AcknowledgementType::Positive : isobus::AcknowledgementType::Negative;
	
	return success;
}


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
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
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

	//diagnosticProtocol.clear_inactive_diagnostic_trouble_codes(); // All messages should now be clear!
	//std::cout << "Diagnostic Trouble Codes cleared." << std::endl;
	//std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Wait a bit before proceeding

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

	// Listen to incoming NMEA2K messages
	n2kInterface.get_course_speed_over_ground_rapid_update_event_publisher().add_listener(on_cog_sog_update);
	n2kInterface.get_datum_event_publisher().add_listener(on_datum_update);
	n2kInterface.get_gnss_position_data_event_publisher().add_listener(on_position_update);
	n2kInterface.get_position_rapid_update_event_publisher().add_listener(on_position_rapid_update);
	n2kInterface.get_rate_of_turn_event_publisher().add_listener(on_turn_rate_update);
	n2kInterface.get_vessel_heading_event_publisher().add_listener(on_vessel_heading_update);

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
			nullptr
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
					/*
					case 3:
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
			if (state > 2){
				state = 0;
			}
			
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	isobus::CANHardwareInterface::stop();
	return 0;
}
