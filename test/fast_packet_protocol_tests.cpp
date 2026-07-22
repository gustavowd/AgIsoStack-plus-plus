#include <gtest/gtest.h>

#include "isobus/isobus/nmea2000_fast_packet_protocol.hpp"

#include "helpers/control_function_helpers.hpp"
#include "helpers/messaging_helpers.hpp"
#include "helpers/test_fixture.hpp"

#include <array>

using namespace isobus;

namespace
{
	constexpr std::uint32_t TEST_FAST_PACKET_PGN = 0x1F112;
	constexpr std::uint8_t TEST_FAST_PACKET_LENGTH = 20;

	std::uint8_t create_fast_packet_header(std::uint8_t sequenceNumber, std::uint8_t frameCount)
	{
		return static_cast<std::uint8_t>((sequenceNumber << 5) | frameCount);
	}

	struct FastPacketCallbackContext
	{
		std::uint32_t callbackCount = 0;
		std::vector<std::uint8_t> lastPayload;
		std::uint64_t lastTimestamp = 0;
		bool wasCalled = false;
	};

	void receive_fast_packet_callback(const CANMessage &message, void *parentPointer)
	{
		auto *context = static_cast<FastPacketCallbackContext *>(parentPointer);
		ASSERT_NE(nullptr, context);
		context->lastPayload = message.get_data();
		context->lastTimestamp = message.get_timestamp_us();
		context->callbackCount++;
		context->wasCalled = true;
	}
} // namespace

class FastPacketProtocolTest : public AgIsoStackTestFixture
{
	// Wrapper to give tests a more meaningful name - no content.
};

TEST_F(FastPacketProtocolTest, ReceivePreservesTimestamp)
{
	auto originator = test_helpers::create_mock_control_function(0x52);
	FastPacketCallbackContext context;
	const std::array<std::uint8_t, TEST_FAST_PACKET_LENGTH> expectedPayload = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };
	constexpr std::uint64_t lastFrameTimestamp = 1234567;

	FastPacketProtocol protocol([](std::uint32_t, CANDataSpan, std::shared_ptr<InternalControlFunction>, std::shared_ptr<ControlFunction>, CANIdentifier::CANPriority) {
		return true;
	});
	protocol.allow_any_control_function(true);
	protocol.register_multipacket_message_callback(TEST_FAST_PACKET_PGN, receive_fast_packet_callback, &context);

	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 0), TEST_FAST_PACKET_LENGTH, 0, 1, 2, 3, 4, 5 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 1), 6, 7, 8, 9, 10, 11, 12 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 2), 13, 14, 15, 16, 17, 18, 19 },
	                                                                lastFrameTimestamp));

	ASSERT_TRUE(context.wasCalled);
	EXPECT_EQ(1, context.callbackCount);
	EXPECT_EQ(lastFrameTimestamp, context.lastTimestamp);
	ASSERT_EQ(expectedPayload.size(), context.lastPayload.size());
	for (std::size_t i = 0; i < expectedPayload.size(); i++)
	{
		EXPECT_EQ(expectedPayload[i], context.lastPayload.at(i));
	}
}

TEST_F(FastPacketProtocolTest, ReceiveIgnoresDuplicateContinuationFrame)
{
	auto originator = test_helpers::create_mock_control_function(0x52);
	FastPacketCallbackContext context;
	const std::array<std::uint8_t, TEST_FAST_PACKET_LENGTH> expectedPayload = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };

	FastPacketProtocol protocol([](std::uint32_t, CANDataSpan, std::shared_ptr<InternalControlFunction>, std::shared_ptr<ControlFunction>, CANIdentifier::CANPriority) {
		return true;
	});
	protocol.allow_any_control_function(true);
	protocol.register_multipacket_message_callback(TEST_FAST_PACKET_PGN, receive_fast_packet_callback, &context);

	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 0), TEST_FAST_PACKET_LENGTH, 0, 1, 2, 3, 4, 5 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 1), 6, 7, 8, 9, 10, 11, 12 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 1), 6, 7, 8, 9, 10, 11, 12 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 2), 13, 14, 15, 16, 17, 18, 19 }));

	ASSERT_TRUE(context.wasCalled);
	EXPECT_EQ(1, context.callbackCount);
	ASSERT_EQ(expectedPayload.size(), context.lastPayload.size());
	for (std::size_t i = 0; i < expectedPayload.size(); i++)
	{
		EXPECT_EQ(expectedPayload[i], context.lastPayload.at(i));
	}
}

TEST_F(FastPacketProtocolTest, ReceiveReassemblesOutOfOrderContinuationFrames)
{
	auto originator = test_helpers::create_mock_control_function(0x52);
	FastPacketCallbackContext context;
	const std::array<std::uint8_t, TEST_FAST_PACKET_LENGTH> expectedPayload = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };

	FastPacketProtocol protocol([](std::uint32_t, CANDataSpan, std::shared_ptr<InternalControlFunction>, std::shared_ptr<ControlFunction>, CANIdentifier::CANPriority) {
		return true;
	});
	protocol.allow_any_control_function(true);
	protocol.register_multipacket_message_callback(TEST_FAST_PACKET_PGN, receive_fast_packet_callback, &context);

	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 0), TEST_FAST_PACKET_LENGTH, 0, 1, 2, 3, 4, 5 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 2), 13, 14, 15, 16, 17, 18, 19 }));
	EXPECT_FALSE(context.wasCalled);
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 1), 6, 7, 8, 9, 10, 11, 12 }));

	ASSERT_TRUE(context.wasCalled);
	EXPECT_EQ(1, context.callbackCount);
	ASSERT_EQ(expectedPayload.size(), context.lastPayload.size());
	for (std::size_t i = 0; i < expectedPayload.size(); i++)
	{
		EXPECT_EQ(expectedPayload[i], context.lastPayload.at(i));
	}
}

TEST_F(FastPacketProtocolTest, ReceiveAbortsOnUnexpectedSequenceNumber)
{
	auto originator = test_helpers::create_mock_control_function(0x52);
	FastPacketCallbackContext context;

	FastPacketProtocol protocol([](std::uint32_t, CANDataSpan, std::shared_ptr<InternalControlFunction>, std::shared_ptr<ControlFunction>, CANIdentifier::CANPriority) {
		return true;
	});
	protocol.allow_any_control_function(true);
	protocol.register_multipacket_message_callback(TEST_FAST_PACKET_PGN, receive_fast_packet_callback, &context);

	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 0), TEST_FAST_PACKET_LENGTH, 0, 1, 2, 3, 4, 5 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(1, 1), 6, 7, 8, 9, 10, 11, 12 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(0, 2), 13, 14, 15, 16, 17, 18, 19 }));

	EXPECT_FALSE(context.wasCalled);
	EXPECT_EQ(0, context.callbackCount);
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(1, 0), TEST_FAST_PACKET_LENGTH, 0, 1, 2, 3, 4, 5 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(1, 1), 6, 7, 8, 9, 10, 11, 12 }));
	protocol.process_message(test_helpers::create_message_broadcast(6,
	                                                                TEST_FAST_PACKET_PGN,
	                                                                originator,
	                                                                { create_fast_packet_header(1, 2), 13, 14, 15, 16, 17, 18, 19 }));

	ASSERT_TRUE(context.wasCalled);
	EXPECT_EQ(1, context.callbackCount);
}
