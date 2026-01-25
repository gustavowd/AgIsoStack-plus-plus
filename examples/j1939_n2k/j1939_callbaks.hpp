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
	void *parent,
	std::shared_ptr<isobus::ControlFunction> destination
) {
	// Usar transport protocol para enviar dados
	/*
	auto allInternalCFs = isobus::CANNetworkManager::CANNetwork.get_internal_control_functions();
	std::shared_ptr<isobus::InternalControlFunction> internalECU;
	for (auto& cf: allInternalCFs){
		if (cf->get_address() == 128){
			internalECU = cf;
			break;
		}
	}
	*/
    // Obtém a Internal Control Function para enviar a resposta
    auto internalECU = static_cast<isobus::InternalControlFunction*>(parent);
	std::cout << "Src addr: " << static_cast<int>(internalECU->get_address());
	if (destination != nullptr){
		std::cout << " - Dst addr: " << static_cast<int>(destination->get_address()) << std::endl;
		return isobus::CANNetworkManager::CANNetwork.send_can_message(
			pgn,
			data.data(),
			data.size(),
			std::shared_ptr<isobus::InternalControlFunction>(internalECU, [](auto*){}),
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
								void *parent)
{
	std::cout << "[PGN Request] - Component Information pgn:" << parameterGroupNumber << std::endl;
	
	std::vector<uint8_t> ecuData = build_component_information_message();
	//std::cout << "Message size: " << ecuData.size() << std::endl;
	bool success = send_multipacket_response(parameterGroupNumber, ecuData, parent, requestingControlFunction);
	
	acknowledge = true;
	acknowledgeType = success ? isobus::AcknowledgementType::Positive : isobus::AcknowledgementType::Negative;
	
	return success;
}
