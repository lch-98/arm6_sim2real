// C++ 프로토콜이 Python(dm_protocol.py)과 같은 바이트를 만드는지 확인
#include <gtest/gtest.h>

#include "arm_hardware/dm_protocol.hpp"

using namespace arm_hardware;

TEST(DmProtocol, MitZero)
{
  const auto lim = dm::default_limits("DM-J4340P-2EC");
  const auto d = dm::pack_mit(0, 0, 0, 0, 0, lim);
  const dm::Data expect{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  EXPECT_EQ(d, expect);
}

TEST(DmProtocol, MitKnownVector)
{
  // Python: dm.pack_mit(1.234, -3.2, 25.0, 1.1, 5.5, Limits(12.5, 8, 28)).hex() == "8ca24cc0cc384991"
  const auto d = dm::pack_mit(1.234, -3.2, 25.0, 1.1, 5.5, dm::Limits{12.5, 8.0, 28.0});
  const dm::Data expect{0x8C, 0xA2, 0x4C, 0xC0, 0xCC, 0x38, 0x49, 0x91};
  EXPECT_EQ(d, expect);
}

TEST(DmProtocol, FeedbackDecode)
{
  // Python: pack_feedback(2, 1, 0.5, -1.0, 3.0, 40, 45, Limits(12.5, 8, 28))
  const uint8_t raw[8] = {0x12, 0x85, 0x1E, 0x6F, 0xF8, 0xDA, 40, 45};
  const auto fb = dm::unpack_feedback(raw, dm::Limits{12.5, 8.0, 28.0});
  EXPECT_EQ(fb.id4, 2);
  EXPECT_EQ(fb.status, dm::ENABLED);
  EXPECT_NEAR(fb.q, 0.5, 1e-3);
  EXPECT_NEAR(fb.dq, -1.0, 0.01);
  EXPECT_NEAR(fb.tau, 3.0, 0.02);
  EXPECT_FALSE(fb.fault());
}

TEST(DmProtocol, Register)
{
  const auto r = dm::reg_read(0x02, dm::RID_TMAX);
  const dm::Data expect{0x02, 0x00, 0x33, 23, 0, 0, 0, 0};
  EXPECT_EQ(r, expect);
}
