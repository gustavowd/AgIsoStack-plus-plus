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