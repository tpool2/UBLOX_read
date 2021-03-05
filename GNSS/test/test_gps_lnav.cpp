#include <gtest/gtest.h>
#include "GNSS/gps_lnav.hpp"
#include <iostream>
#include <tuple>
#include <vector>

using std::make_tuple;

namespace gnss { namespace gps { namespace lnav {

static uint32_t sat_03[]
{
    0X22c05d10, 0X28ccaa9c,
    0Xc82c7d6,  0X8dba513c,
    0X3b050cf8, 0X286c084,
    0X38d4535b, 0X38ea845,
    0X837eca97, 0X1ec31f97
};

static uint32_t sat_07[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7ff6203,  0xb955c46,
    0xa1ac8191, 0xbf7f4076,
    0xaac9c2fe, 0x8457284c,
    0x3553e54,  0x1ec31fc4
};

static uint32_t sat_22[]
{
    0x22c05d10, 0x28ccaa9c,
    0x5ffa71a,  0x8c413efc,
    0x1a5aa601, 0xbf9d00f0,
    0x89fd611,  0x81d2287b,
    0x38ff847,  0x1ec2d090
};

static uint32_t sat_26[]
{
    0x22c05d10, 0x28ccaa9c,
    0x147f9c40, 0xbf57e8a,
    0x8a37d001, 0xbfa6c10a,
    0xa3b58606, 0x81c5686d,
    0x8359f2a6, 0x9ec31f48
};

static uint32_t sat_27[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7bf552b,  0xddb7420,
    0x26e0deac, 0x3f7440c8,
    0x21c78ff0, 0x461684a,
    0x833e1c26, 0x9ec31f1b
};

static uint32_t bad_03[]
{
    0X22c05d10, 0X28ccaa9c,
    0Xc82c7d6,  0X8dba513c,
    0X3b050cf8, 0X286c084,
    0X38d4535b, 0X38ea845,
    0X837eca97, 0X1ec31f98
};

static uint32_t bad_07[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7ff6203,  0xb955c46,
    0xa1ac8191, 0xbf7f4076,
    0xaac9c2fe, 0x8457284e,
    0x3553e54,  0x1ec31fc4
};

static uint32_t bad_22[]
{   
    0x22c05d10, 0x28ccaa9c, 
    0x5ffa71a,  0x8c413efc,
    0x1a5aa601, 0xbf9d00f0,
    0x89fd611,  0x81d2287b,
    0x38ff846,  0x1ec2d090  
};

static uint32_t bad_26[]
{
    0x22c05d10, 0x28ccaa9c,
    0x147f9c40, 0xbf57e8a,
    0x8a37d001, 0xbfa6c10b,
    0xa3b58606, 0x81c5686d,
    0x8359f2a6, 0x9ec31f48
};

static uint32_t bad_27[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7bf552a,  0xddb7420,
    0x26e0deac, 0x3f7440c8,
    0x21c78ff0, 0x461684a,
    0x833e1c26, 0x9ec31f1b
};

static uint32_t sat_04[] = 
{
    // Subframe 1
    0x22c05d10, 0x28cbe9b4,
    0x5d40074,  0x39e5a75,        
    0x9242e37b, 0x2df03e9b,
    0x88d3dca,  0x9edec319,
    0x803ff9e6, 0xba462ed8,
    // Subframe 2
    0x22c05d10, 0x28cc0a00,
    0x1eff97bc, 0xd0779d5,
    0x8e54a58c, 0x3fa4400a,
    0xa30d9af8, 0x38ae873,
    0x32dbb74,  0x1ec31fc4,
    // Subframe 3
    0x22c05d10, 0x28cc2b88,
    0x309b4,    0x1a33d03,
    0x49ea,     0x87c14f87,
    0x7b660e1,  0xb89deb62,
    0xbfe9dfb9, 0x9ec00d3c,
    // Subframe 4
    0x22c05d10, 0x28cc4c34,
    0x1e4080d8, 0x3a7b78b7,
    0xa5bff76,  0xb465f071,
    0xa2380a93, 0x25698143,
    0x19242dd,  0xab655b47,
    // Subframe 5
    0x22c05d10, 0x28cc6dbc,
    0x102aaaac, 0x2aaaaabc,
    0x2aaaaabc, 0x2aaaaabc,
    0x2aaaaabc, 0x2aaaaabc,
    0x2aaaaabc, 0x2aaaaabc,
};

static double sat_04_solution[] = 
{   // G04 2021 01 22 20 00 00
    -1.747393980622E-04,-2.842170943040E-12, 0.000000000000E+00,
     1.230000000000E+02,-1.306250000000E+01, 4.764841331910E-09,-6.080965795834E-01,
    -6.835907697678E-07, 1.069737016223E-03, 6.755813956261E-06, 5.153589319229E+03,
     5.040000000000E+05, 2.235174179077E-08, 9.332883497659E-01, 1.862645149231E-09,
     9.601780626532E-01, 2.467812500000E+02,-3.046249277721E+00,-8.092479941415E-09,
     4.643050544549E-12, 1.000000000000E+00, 2.141000000000E+03, 0.000000000000E+00,
     2.000000000000E+00, 0.000000000000E+00,-4.190952000000E-09, 3.790000000000E+02,
     4.968600000000E+05, 4.000000000000E+00,
};



std::vector<std::tuple<int, int>> bit_indicies
{
    make_tuple(0, 2),   make_tuple(1, 3),   make_tuple(27, 29),     make_tuple(28, 30),     make_tuple(29, 31),
    make_tuple(30, 34), make_tuple(31, 35), make_tuple(57, 61),     make_tuple(58, 62),     make_tuple(59, 63),
    make_tuple(60, 66), make_tuple(61, 67), make_tuple(87, 93),     make_tuple(88, 94),     make_tuple(89, 95),
    make_tuple(90, 98), make_tuple(91, 99), make_tuple(117, 125),   make_tuple(118, 126),   make_tuple(119, 127)   
};

class UBLOX_LNAV: public ::testing::TestWithParam<uint32_t*>
{
    protected:
        uint32_t* words;
        void SetUp() override
        {
            words = GetParam();
        }
};

TEST_P(UBLOX_LNAV, Preamble_Check)
{
    ASSERT_EQ(bit_utils::get_msb_bits<uint8_t>(words, 2, 8), gnss::gps::kPreamble);
}

TEST_P(UBLOX_LNAV, Full_Parity_Check_True)
{
    ASSERT_TRUE(check_parity(words));
}

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_LNAV,
testing::Values
(
    sat_03, sat_07, sat_22, sat_26, sat_27
));

class UBLOX_LNAV_BAD: public ::testing::TestWithParam<uint32_t*>
{
    protected:
        uint32_t* words;
        void SetUp() override
        {
            words = GetParam();
        }
};

TEST_P(UBLOX_LNAV_BAD, Full_Parity_Check_False)
{
    ASSERT_FALSE(check_parity(words));
}

INSTANTIATE_TEST_SUITE_P(BadNavData, UBLOX_LNAV_BAD,
testing::Values
(
    bad_03, bad_07, bad_22, bad_26, bad_27
));

class UBLOX_LNAV_With_Word_Index: public ::testing::TestWithParam<std::tuple<uint32_t*, int>>
{
    protected:
        uint32_t* words;
        int word_index;
        void SetUp() override
        {
            auto params = GetParam();
            words = std::get<0>(params);
            word_index = std::get<1>(params);
        }
};

TEST_P(UBLOX_LNAV_With_Word_Index, Parity_Check_True)
{
    bool D_29, D_30;
    if(word_index > 0)
    {
        int previous_word_index = word_index-1;
        D_29 = bit_utils::get_msb_bits<bool>(&words[previous_word_index], 30, 1);
        D_30 = bit_utils::get_msb_bits<bool>(&words[previous_word_index], 31, 1);
    }
    else
    {
        D_29 = 0;
        D_30 = 0;
    }
    ASSERT_TRUE(check_parity(words[word_index], D_29, D_30));
}

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_LNAV_With_Word_Index,
testing::Combine
(
    testing::Values
    (
        sat_03, sat_07, sat_22, sat_26, sat_27
    ),
    testing::Range(0, 10)
));

class BitIndicies: public ::testing::TestWithParam<std::tuple<int,int>>
{
    protected:
        int l1_bit_index;
        int real_bit_index;
        void SetUp() override
        {
            auto params = GetParam();
            l1_bit_index = std::get<0>(params);
            real_bit_index = std::get<1>(params);
        }
};

TEST_P(BitIndicies, MatchingBitTrue)
{
    ASSERT_EQ(get_ublox_bit_index(l1_bit_index), real_bit_index);
}

INSTANTIATE_TEST_SUITE_P(BitIndiciesParams, BitIndicies,
testing::ValuesIn
(
    bit_indicies
));

class UBLOX_LNAV_With_Bit_Index: public ::testing::TestWithParam<std::tuple<std::tuple<int, int>, uint32_t*>>
{
    protected:
        int l1_bit_index;
        int real_bit_index;
        uint32_t* words;
        void SetUp() override
        {
            auto params = GetParam();
            l1_bit_index = std::get<0>(std::get<0>(params));
            real_bit_index = std::get<1>(std::get<0>(params));
            words = std::get<1>(params);
        }
};

TEST_P(UBLOX_LNAV_With_Bit_Index, MatchingBitDataTrue)
{
    ASSERT_EQ(get_bits<bool>(words, l1_bit_index, 1), bit_utils::get_msb_bits<bool>(words, real_bit_index, 1));
}

INSTANTIATE_TEST_SUITE_P(BitIndiciesParams, UBLOX_LNAV_With_Bit_Index,
testing::Combine
(
    testing::ValuesIn(bit_indicies),
    testing::Values
    (
        sat_03, sat_07, sat_22, sat_26, sat_27
    )
));

class TestLNAVParser: public ::testing::TestWithParam<int>
{
    protected:
        int subframe_id;
        uint32_t* subframe_address;
        LNAVParser parser;
        void SetUp() override
        {
            subframe_id = GetParam();
            subframe_address = &sat_04[(subframe_id-1)*10];
        }
};

TEST_P(TestLNAVParser, SubframeID)
{
    parser.read_subframe(subframe_address);
    ASSERT_EQ(subframe_id, parser.subframe_id);
}

INSTANTIATE_TEST_SUITE_P(FullGNSSMessage, TestLNAVParser,
::testing::Range(1, 6, 1)
);

class TestSubframes: public testing::TestWithParam<std::tuple<uint32_t*, double*>>
{
    protected:
        LNAVParser parser;
        double* answers;
        uint32_t* message;
        void SetUp() override
        {
            message = std::get<0>(GetParam());
            answers = std::get<1>(GetParam());
            for(int subframe_index = 0; subframe_index < kSubframesPerMessage; ++subframe_index)
            {
                parser.read_subframe(&message[subframe_index*kWordsPerSubframe]);
            }
        }
};

INSTANTIATE_TEST_SUITE_P(FullGNSSMessage, TestSubframes,
::testing::Values
(
    std::make_tuple<uint32_t*, double*>(sat_04, sat_04_solution)
));

TEST_P(TestSubframes, a_f0)
{
    ASSERT_FLOAT_EQ(parser.a_f0, answers[0]);
}

TEST_P(TestSubframes, a_f1)
{
    ASSERT_FLOAT_EQ(parser.a_f1, answers[1]);
}

TEST_P(TestSubframes, a_f2)
{
    ASSERT_FLOAT_EQ(parser.a_f2, answers[2]);
}

TEST_P(TestSubframes, IODE)
{
    ASSERT_FLOAT_EQ(parser.IODE, answers[3]);
}

TEST_P(TestSubframes, C_rs)
{
    ASSERT_FLOAT_EQ(parser.C_rs, answers[4]);
}

TEST_P(TestSubframes, DISABLED_Delta_n)
{
    ASSERT_FLOAT_EQ(parser.Delta_n, answers[5]);
}

TEST_P(TestSubframes, M_0)
{
    ASSERT_FLOAT_EQ(parser.M_0*gps::pi, answers[6]);
}

TEST_P(TestSubframes, C_uc)
{
    ASSERT_FLOAT_EQ(parser.C_uc, answers[7]);
}

TEST_P(TestSubframes, e)
{
    ASSERT_FLOAT_EQ(parser.e, answers[8]);
}

TEST_P(TestSubframes, C_us)
{
    ASSERT_FLOAT_EQ(parser.C_us, answers[9]);
}

TEST_P(TestSubframes, sqrt_A)
{
    ASSERT_FLOAT_EQ(parser.sqrt_A, answers[10]);
}

TEST_P(TestSubframes, t_oe)
{
    ASSERT_FLOAT_EQ(parser.t_oe, answers[11]);
}

TEST_P(TestSubframes, C_ic)
{
    ASSERT_FLOAT_EQ(parser.C_ic, answers[12]);
}

TEST_P(TestSubframes, Omega_0)
{
    ASSERT_FLOAT_EQ(parser.Omega_0*gps::pi, answers[13]);
}

TEST_P(TestSubframes, C_is)
{
    ASSERT_FLOAT_EQ(parser.C_is, answers[14]);
}

TEST_P(TestSubframes, i_0)
{
    ASSERT_FLOAT_EQ(parser.i_0*gps::pi, answers[15]);
}

TEST_P(TestSubframes, C_rc)
{
    ASSERT_FLOAT_EQ(parser.C_rc, answers[16]);
}

TEST_P(TestSubframes, omega)
{
    ASSERT_FLOAT_EQ(parser.omega*gps::pi, answers[17]);
}

TEST_P(TestSubframes, Omega_dot)
{
    ASSERT_FLOAT_EQ(parser.Omega_dot*gps::pi, answers[18]);
}

TEST_P(TestSubframes, IDOT)
{
    ASSERT_FLOAT_EQ(parser.IDOT*gps::pi, answers[19]);
}

TEST_P(TestSubframes, WN)
{
    ASSERT_EQ(parser.WN, int(answers[21])%256);
}

TEST_P(TestSubframes, T_GD)
{
    ASSERT_FLOAT_EQ(parser.T_GD, answers[25]);
}

TEST_P(TestSubframes, IODC)
{
    ASSERT_FLOAT_EQ(parser.IODC, answers[26]);
}

} // namespace lnav
} // namespace gps
} // namespace gnss