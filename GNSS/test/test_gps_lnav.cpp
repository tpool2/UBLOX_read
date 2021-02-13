#include <gtest/gtest.h>

class UBLOX_LNAV: public ::testing::TestWithParam<uint32_t>
{
    
};

TEST_P(UBLOX_LNAV, MSBFalse)
{
    ASSERT_FALSE(GetParam()&(1UL<<31));
}

TEST_P(UBLOX_LNAV, SecondMSBFalse)
{
    ASSERT_FALSE(GetParam()&(1UL<<30));
}

INSTANTIATE_TEST_SUITE_P(Sat_22, UBLOX_LNAV,
testing::Values
(
    0x22c05d10, 0x28ccaa9c, 
    0x5ffa71a,  0x8c413efc,
    0x1a5aa601, 0xbf9d00f0,
    0x89fd611,  0x81d2287b,
    0x38ff847,  0x1ec2d090
));

TEST(LNAV, Sat_22)
{
    uint32_t words[10] = {  0x22c05d10, 0x28ccaa9c, 
                            0x5ffa71a,  0x8c413efc,
                            0x1a5aa601, 0xbf9d00f0,
                            0x89fd611,  0x81d2287b,
                            0x38ff847,  0x1ec2d090 };
}

TEST(LNAV, SAT_27)
{
    uint32_t words[10] = {  0x22c05d10, 0x28ccaa9c,
                            0x7bf552b,  0xddb7420,
                            0x26e0deac, 0x3f7440c8,
                            0x21c78ff0, 0x461684a,
                            0x833e1c26, 0x9ec31f1b };
}

TEST(LNAV, SAT_26)
{
    uint32_t words[10] = {  0x22c05d10, 0x28ccaa9c,
                            0x147f9c40, 0xbf57e8a,
                            0x8a37d001, 0xbfa6c10a,
                            0xa3b58606, 0x81c5686d,
                            0x8359f2a6, 0x9ec31f48 };
}

TEST(LNAV, SAT_3)
{
    uint32_t words[10] = {  0X22c05d10, 0X28ccaa9c,
                            0Xc82c7d6,  0X8dba513c,
                            0X3b050cf8, 0X286c084,
                            0X38d4535b, 0X38ea845,
                            0X837eca97, 0X1ec31f97 };
}

TEST(LNAV, SAT_7)
{
    uint32_t words[10] = {  0x22c05d10, 0x28ccaa9c,
                            0x7ff6203,  0xb955c46,
                            0xa1ac8191, 0xbf7f4076,
                            0xaac9c2fe, 0x8457284c,
                            0x3553e54,  0x1ec31fc4 };
}