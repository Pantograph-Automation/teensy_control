#include <deque>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks.hpp"
#include "serial_command_handler.hpp"

using ::testing::AnyNumber;
using ::testing::NiceMock;
using ::testing::Return;

namespace
{
Status fake_inactive_control()
{
  return Status::COMPLETE;
}

Status fake_calibrate_control()
{
  return Status::ACTIVE;
}

Status fake_active_control()
{
  return Status::ACTIVE;
}
}  // namespace

class SerialCommandHandlerTest : public ::testing::Test
{
protected:
  NiceMock<MockClock> mock_clock;
  NiceMock<MockSerial> mock_serial;
  State state;
  bool commanded_gripper_state = false;
  SerialCommandHandler handler{
    &state,
    &mock_clock,
    &commanded_gripper_state,
    fake_inactive_control,
    fake_calibrate_control,
    0.005f,
    1.0f,
    2.0f};

  void SetUp() override
  {
    state.callback = fake_inactive_control;
    state.error = Error::INVALID_SERIAL;
    state.initialize_joint_trajectories(0.0f, 0.0f, 1.0f, 2.0f, 0UL);
  }

  void TearDown() override
  {
    state.reset(fake_inactive_control);
  }

  void load_serial_bytes(std::deque<char> & input)
  {
    EXPECT_CALL(mock_serial, available()).Times(AnyNumber())
      .WillRepeatedly([&input]() { return static_cast<int>(input.size()); });
    EXPECT_CALL(mock_serial, read()).Times(AnyNumber())
      .WillRepeatedly([&input]() {
        const char value = input.front();
        input.pop_front();
        return value;
      });
  }
};

// Verifies ACTIVATE resets state and switches the controller into calibration mode.
TEST_F(SerialCommandHandlerTest, ParseSerialActivatesCalibrationFlow)
{
  state.setpoint = new Setpoint(0.2f, 0.3f, 0.4f, 0.005f, 1.0f);

  EXPECT_EQ(handler.parse_serial("ACTIVATE"), Error::OK);
  EXPECT_EQ(state.callback, fake_calibrate_control);
  EXPECT_EQ(state.setpoint, nullptr);
}

// Verifies a valid setpoint retargets the joint trajectories and updates the active setpoint payload.
TEST_F(SerialCommandHandlerTest, ParseSerialAcceptsValidSetpointWhenActive)
{
  state.callback = fake_active_control;
  state.setpoint = new Setpoint(0.0f, 0.0f, 0.0f, 0.005f, 1.0f);

  EXPECT_CALL(mock_clock, microseconds()).WillOnce(Return(100000UL));

  EXPECT_EQ(handler.parse_serial("SETPOINT 1.0 -0.5 0.2"), Error::OK);
  ASSERT_NE(state.setpoint, nullptr);
  EXPECT_FLOAT_EQ(state.setpoint->q1, 1.0f);
  EXPECT_FLOAT_EQ(state.setpoint->q2, -0.5f);
  EXPECT_FLOAT_EQ(state.setpoint->z, 0.2f);
}

// Verifies malformed setpoint messages are rejected before they can alter commanded motion.
TEST_F(SerialCommandHandlerTest, ParseSerialRejectsMalformedSetpoint)
{
  state.callback = fake_active_control;

  EXPECT_EQ(handler.parse_serial("SETPOINT 1.0 2.0"), Error::INVALID_SETPOINT);
}

// Verifies gripper commands are blocked while the system is inactive or calibrating.
TEST_F(SerialCommandHandlerTest, ParseSerialRejectsInvalidGripperTransition)
{
  commanded_gripper_state = true;

  EXPECT_EQ(handler.parse_serial("GRIPPER OPEN"), Error::INVALID_TRANSITION);
  EXPECT_TRUE(commanded_gripper_state);
}

// Verifies unrecognized commands preserve the existing protocol contract by reporting an invalid serial message.
TEST_F(SerialCommandHandlerTest, ParseSerialRejectsUnknownCommands)
{
  EXPECT_EQ(handler.parse_serial("UNKNOWN"), Error::INVALID_SERIAL);
}

// Verifies buffered input survives across calls so partial serial frames can be completed later.
TEST_F(SerialCommandHandlerTest, GetSerialBuffersPartialMessagesUntilNewline)
{
  std::deque<char> first_chunk{'A', 'C', 'T'};
  load_serial_bytes(first_chunk);

  handler.get_serial(&mock_serial);
  EXPECT_EQ(state.error, Error::INVALID_SERIAL);

  ::testing::Mock::VerifyAndClearExpectations(&mock_serial);

  std::deque<char> second_chunk{'I', 'V', 'A', 'T', 'E', '\n'};
  load_serial_bytes(second_chunk);

  handler.get_serial(&mock_serial);
  EXPECT_EQ(state.error, Error::OK);
  EXPECT_EQ(state.callback, fake_calibrate_control);
}

// Verifies newline handling resets the buffer so multiple commands can be parsed back to back.
TEST_F(SerialCommandHandlerTest, GetSerialParsesMultipleMessagesSequentially)
{
  std::deque<char> input{
    'A', 'C', 'T', 'I', 'V', 'A', 'T', 'E', '\n',
    'D', 'E', 'A', 'C', 'T', 'I', 'V', 'A', 'T', 'E', '\n'};
  load_serial_bytes(input);

  handler.get_serial(&mock_serial);

  EXPECT_EQ(state.error, Error::OK);
  EXPECT_EQ(state.callback, fake_inactive_control);
}

// Verifies serial responses print the status string on success and the error string when a command fails.
TEST_F(SerialCommandHandlerTest, RespondSerialPrintsStatusOrErrorMessage)
{
  state.error = Error::OK;
  EXPECT_CALL(mock_serial, println(::testing::StrEq("OK ACTIVE")));
  handler.respond_serial(&mock_serial, Status::ACTIVE);

  state.error = Error::INVALID_SETPOINT;
  EXPECT_CALL(mock_serial, println(::testing::StrEq("ERROR Invalid setpoint received.")));
  handler.respond_serial(&mock_serial, Status::COMPLETE);
}
