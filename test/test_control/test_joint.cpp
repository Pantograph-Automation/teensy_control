#include <gtest/gtest.h>
#include <gmock/gmock.h>

// TEST(...)
// TEST_F(...)

#if defined(ARDUINO)
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);

    ::testing::InitGoogleMock();
}

void loop()
{
  if (RUN_ALL_TESTS());
  delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS());
    return 0;
}
#endif