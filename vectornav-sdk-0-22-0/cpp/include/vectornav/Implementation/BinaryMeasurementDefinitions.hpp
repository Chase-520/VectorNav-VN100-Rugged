// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_BINARYMEASUREMENTDEFINITIONS_HPP_
#define VN_BINARYMEASUREMENTDEFINITIONS_HPP_

#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

namespace VN
{
// Binary Group Bit Offsets
#define COMMON_BIT 0x01 << 0
#define TIME_BIT 0x01 << 1
#define IMU_BIT 0x01 << 2
#define GNSS_BIT 0x01 << 3
#define ATTITUDE_BIT 0x01 << 4
#define INS_BIT 0x01 << 5
#define GNSS2_BIT 0x01 << 6
#define GNSS3_BIT 0x01 << 12

// Binary Group Output Bit Offsets
#define COMMON_TIMESTARTUP_BIT 0x01 << 0
#define COMMON_TIMEGPS_BIT 0x01 << 1
#define COMMON_TIMESYNCIN_BIT 0x01 << 2
#define COMMON_YPR_BIT 0x01 << 3
#define COMMON_QUATERNION_BIT 0x01 << 4
#define COMMON_ANGULARRATE_BIT 0x01 << 5
#define COMMON_POSLLA_BIT 0x01 << 6
#define COMMON_VELNED_BIT 0x01 << 7
#define COMMON_ACCEL_BIT 0x01 << 8
#define COMMON_IMU_BIT 0x01 << 9
#define COMMON_MAGPRES_BIT 0x01 << 10
#define COMMON_DELTAS_BIT 0x01 << 11
#define COMMON_INSSTATUS_BIT 0x01 << 12
#define COMMON_SYNCINCNT_BIT 0x01 << 13
#define COMMON_TIMEGPSPPS_BIT 0x01 << 14

#define TIME_TIMESTARTUP_BIT 0x01 << 0
#define TIME_TIMEGPS_BIT 0x01 << 1
#define TIME_TIMEGPSTOW_BIT 0x01 << 2
#define TIME_TIMEGPSWEEK_BIT 0x01 << 3
#define TIME_TIMESYNCIN_BIT 0x01 << 4
#define TIME_TIMEGPSPPS_BIT 0x01 << 5
#define TIME_TIMEUTC_BIT 0x01 << 6
#define TIME_SYNCINCNT_BIT 0x01 << 7
#define TIME_SYNCOUTCNT_BIT 0x01 << 8
#define TIME_TIMESTATUS_BIT 0x01 << 9

#define IMU_IMUSTATUS_BIT 0x01 << 0
#define IMU_UNCOMPMAG_BIT 0x01 << 1
#define IMU_UNCOMPACCEL_BIT 0x01 << 2
#define IMU_UNCOMPGYRO_BIT 0x01 << 3
#define IMU_TEMPERATURE_BIT 0x01 << 4
#define IMU_PRESSURE_BIT 0x01 << 5
#define IMU_DELTATHETA_BIT 0x01 << 6
#define IMU_DELTAVEL_BIT 0x01 << 7
#define IMU_MAG_BIT 0x01 << 8
#define IMU_ACCEL_BIT 0x01 << 9
#define IMU_ANGULARRATE_BIT 0x01 << 10
#define IMU_SENSSAT_BIT 0x01 << 11

#define GNSS_GNSS1TIMEUTC_BIT 0x01 << 0
#define GNSS_GPS1TOW_BIT 0x01 << 1
#define GNSS_GPS1WEEK_BIT 0x01 << 2
#define GNSS_GNSS1NUMSATS_BIT 0x01 << 3
#define GNSS_GNSS1FIX_BIT 0x01 << 4
#define GNSS_GNSS1POSLLA_BIT 0x01 << 5
#define GNSS_GNSS1POSECEF_BIT 0x01 << 6
#define GNSS_GNSS1VELNED_BIT 0x01 << 7
#define GNSS_GNSS1VELECEF_BIT 0x01 << 8
#define GNSS_GNSS1POSUNCERTAINTY_BIT 0x01 << 9
#define GNSS_GNSS1VELUNCERTAINTY_BIT 0x01 << 10
#define GNSS_GNSS1TIMEUNCERTAINTY_BIT 0x01 << 11
#define GNSS_GNSS1TIMEINFO_BIT 0x01 << 12
#define GNSS_GNSS1DOP_BIT 0x01 << 13
#define GNSS_GNSS1SATINFO_BIT 0x01 << 14
#define GNSS_EXTENSION_BIT 0x01 << 15
#define GNSS_GNSS1RAWMEAS_BIT 0x01 << 16
#define GNSS_GNSS1STATUS_BIT 0x01 << 17
#define GNSS_GNSS1ALTMSL_BIT 0x01 << 18

#define ATTITUDE_YPR_BIT 0x01 << 1
#define ATTITUDE_QUATERNION_BIT 0x01 << 2
#define ATTITUDE_DCM_BIT 0x01 << 3
#define ATTITUDE_MAGNED_BIT 0x01 << 4
#define ATTITUDE_ACCELNED_BIT 0x01 << 5
#define ATTITUDE_LINBODYACC_BIT 0x01 << 6
#define ATTITUDE_LINACCELNED_BIT 0x01 << 7
#define ATTITUDE_YPRU_BIT 0x01 << 8
#define ATTITUDE_HEAVE_BIT 0x01 << 12
#define ATTITUDE_ATTU_BIT 0x01 << 13

#define INS_INSSTATUS_BIT 0x01 << 0
#define INS_POSLLA_BIT 0x01 << 1
#define INS_POSECEF_BIT 0x01 << 2
#define INS_VELBODY_BIT 0x01 << 3
#define INS_VELNED_BIT 0x01 << 4
#define INS_VELECEF_BIT 0x01 << 5
#define INS_MAGECEF_BIT 0x01 << 6
#define INS_ACCELECEF_BIT 0x01 << 7
#define INS_LINACCELECEF_BIT 0x01 << 8
#define INS_POSU_BIT 0x01 << 9
#define INS_VELU_BIT 0x01 << 10

#define GNSS2_GNSS2TIMEUTC_BIT 0x01 << 0
#define GNSS2_GPS2TOW_BIT 0x01 << 1
#define GNSS2_GPS2WEEK_BIT 0x01 << 2
#define GNSS2_GNSS2NUMSATS_BIT 0x01 << 3
#define GNSS2_GNSS2FIX_BIT 0x01 << 4
#define GNSS2_GNSS2POSLLA_BIT 0x01 << 5
#define GNSS2_GNSS2POSECEF_BIT 0x01 << 6
#define GNSS2_GNSS2VELNED_BIT 0x01 << 7
#define GNSS2_GNSS2VELECEF_BIT 0x01 << 8
#define GNSS2_GNSS2POSUNCERTAINTY_BIT 0x01 << 9
#define GNSS2_GNSS2VELUNCERTAINTY_BIT 0x01 << 10
#define GNSS2_GNSS2TIMEUNCERTAINTY_BIT 0x01 << 11
#define GNSS2_GNSS2TIMEINFO_BIT 0x01 << 12
#define GNSS2_GNSS2DOP_BIT 0x01 << 13
#define GNSS2_GNSS2SATINFO_BIT 0x01 << 14
#define GNSS2_EXTENSION_BIT 0x01 << 15
#define GNSS2_GNSS2RAWMEAS_BIT 0x01 << 16
#define GNSS2_GNSS2STATUS_BIT 0x01 << 17
#define GNSS2_GNSS2ALTMSL_BIT 0x01 << 18

#define GNSS3_GNSS3TIMEUTC_BIT 0x01 << 0
#define GNSS3_GPS3TOW_BIT 0x01 << 1
#define GNSS3_GPS3WEEK_BIT 0x01 << 2
#define GNSS3_GNSS3NUMSATS_BIT 0x01 << 3
#define GNSS3_GNSS3FIX_BIT 0x01 << 4
#define GNSS3_GNSS3POSLLA_BIT 0x01 << 5
#define GNSS3_GNSS3POSECEF_BIT 0x01 << 6
#define GNSS3_GNSS3VELNED_BIT 0x01 << 7
#define GNSS3_GNSS3VELECEF_BIT 0x01 << 8
#define GNSS3_GNSS3POSUNCERTAINTY_BIT 0x01 << 9
#define GNSS3_GNSS3VELUNCERTAINTY_BIT 0x01 << 10
#define GNSS3_GNSS3TIMEUNCERTAINTY_BIT 0x01 << 11
#define GNSS3_GNSS3TIMEINFO_BIT 0x01 << 12
#define GNSS3_GNSS3DOP_BIT 0x01 << 13
#define GNSS3_GNSS3SATINFO_BIT 0x01 << 14
#define GNSS3_EXTENSION_BIT 0x01 << 15
#define GNSS3_GNSS3RAWMEAS_BIT 0x01 << 16
#define GNSS3_GNSS3STATUS_BIT 0x01 << 17
#define GNSS3_GNSS3ALTMSL_BIT 0x01 << 18

/// @brief The CommonGroupMappingStruct contains the measurement location of the . Used by the ASCII mapping.
struct CommonGroupMappingStruct
{
    CommonGroupMappingStruct() {};
    CommonGroupMappingStruct(uint8_t MeasGroupIndex, uint8_t MeasTypeIndex) : measGroupIndex(MeasGroupIndex), measTypeIndex(MeasTypeIndex) {};
    uint8_t measGroupIndex;
    uint8_t measTypeIndex;
};

/// @brief Common Group Mapping
const std::array<const Vector<CommonGroupMappingStruct, 3>, 15> CommonGroupMapping = {{
    // TimeStartup
    {
        CommonGroupMappingStruct(1, 0),  // Group(Time)      Measurement(TimeStartup)
    },

    // TimeGps
    {
        CommonGroupMappingStruct(1, 1),  // Group(Time)      Measurement(TimeGps)
    },

    // TimeSyncIn
    {
        CommonGroupMappingStruct(1, 4),  // Group(Time)      Measurement(TimeSyncIn)
    },

    // Ypr
    {
        CommonGroupMappingStruct(4, 1),  // Group(Attitude)      Measurement(Ypr)
    },

    // Quaternion
    {
        CommonGroupMappingStruct(4, 2),  // Group(Attitude)      Measurement(Quaternion)
    },

    // AngularRate
    {
        CommonGroupMappingStruct(2, 10),  // Group(Imu)      Measurement(AngularRate)
    },

    // PosLla
    {
        CommonGroupMappingStruct(5, 1),  // Group(Ins)      Measurement(PosLla)
    },

    // VelNed
    {
        CommonGroupMappingStruct(5, 4),  // Group(Ins)      Measurement(VelNed)
    },

    // Accel
    {
        CommonGroupMappingStruct(2, 9),  // Group(Imu)      Measurement(Accel)
    },

    // Imu
    {
        CommonGroupMappingStruct(2, 2),  // Group(Imu)      Measurement(UncompAccel)
        CommonGroupMappingStruct(2, 3),  // Group(Imu)      Measurement(UncompGyro)
    },

    // MagPres
    {
        CommonGroupMappingStruct(2, 8),  // Group(Imu)      Measurement(Mag)
        CommonGroupMappingStruct(2, 4),  // Group(Imu)      Measurement(Temperature)
        CommonGroupMappingStruct(2, 5),  // Group(Imu)      Measurement(Pressure)
    },

    // Deltas
    {
        CommonGroupMappingStruct(2, 6),  // Group(Imu)      Measurement(DeltaTheta)
        CommonGroupMappingStruct(2, 7),  // Group(Imu)      Measurement(DeltaVel)
    },

    // InsStatus
    {
        CommonGroupMappingStruct(5, 0),  // Group(Ins)      Measurement(InsStatus)
    },

    // SyncInCnt
    {
        CommonGroupMappingStruct(1, 7),  // Group(Time)      Measurement(SyncInCnt)
    },

    // TimeGpsPps
    {
        CommonGroupMappingStruct(1, 5),  // Group(Time)      Measurement(TimeGpsPps)
    },

}};

/// @brief Return the number of bytes associated with a binary field.
inline std::optional<uint8_t> getStaticBinaryTypeSize(const size_t binaryGroup, const size_t binaryField)
{
    switch (binaryGroup)
    {
        case 0:  // Common Group
        {
            switch (binaryField)
            {
                case 0:  // TimeStartup
                {
                    return std::make_optional(8);
                }
                case 1:  // TimeGps
                {
                    return std::make_optional(8);
                }
                case 2:  // TimeSyncIn
                {
                    return std::make_optional(8);
                }
                case 3:  // Ypr
                {
                    return std::make_optional(12);
                }
                case 4:  // Quaternion
                {
                    return std::make_optional(16);
                }
                case 5:  // AngularRate
                {
                    return std::make_optional(12);
                }
                case 6:  // PosLla
                {
                    return std::make_optional(24);
                }
                case 7:  // VelNed
                {
                    return std::make_optional(12);
                }
                case 8:  // Accel
                {
                    return std::make_optional(12);
                }
                case 9:  // Imu
                {
                    return std::make_optional(24);
                }
                case 10:  // MagPres
                {
                    return std::make_optional(20);
                }
                case 11:  // Deltas
                {
                    return std::make_optional(28);
                }
                case 12:  // InsStatus
                {
                    return std::make_optional(2);
                }
                case 13:  // SyncInCnt
                {
                    return std::make_optional(4);
                }
                case 14:  // TimeGpsPps
                {
                    return std::make_optional(8);
                }
                default:
                    return std::nullopt;
            }
        }
        case 1:  // Time Group
        {
            switch (binaryField)
            {
                case 0:  // TimeStartup
                {
                    return std::make_optional(8);
                }
                case 1:  // TimeGps
                {
                    return std::make_optional(8);
                }
                case 2:  // TimeGpsTow
                {
                    return std::make_optional(8);
                }
                case 3:  // TimeGpsWeek
                {
                    return std::make_optional(2);
                }
                case 4:  // TimeSyncIn
                {
                    return std::make_optional(8);
                }
                case 5:  // TimeGpsPps
                {
                    return std::make_optional(8);
                }
                case 6:  // TimeUtc
                {
                    return std::make_optional(8);
                }
                case 7:  // SyncInCnt
                {
                    return std::make_optional(4);
                }
                case 8:  // SyncOutCnt
                {
                    return std::make_optional(4);
                }
                case 9:  // TimeStatus
                {
                    return std::make_optional(1);
                }
                default:
                    return std::nullopt;
            }
        }
        case 2:  // Imu Group
        {
            switch (binaryField)
            {
                case 0:  // ImuStatus
                {
                    return std::make_optional(2);
                }
                case 1:  // UncompMag
                {
                    return std::make_optional(12);
                }
                case 2:  // UncompAccel
                {
                    return std::make_optional(12);
                }
                case 3:  // UncompGyro
                {
                    return std::make_optional(12);
                }
                case 4:  // Temperature
                {
                    return std::make_optional(4);
                }
                case 5:  // Pressure
                {
                    return std::make_optional(4);
                }
                case 6:  // DeltaTheta
                {
                    return std::make_optional(16);
                }
                case 7:  // DeltaVel
                {
                    return std::make_optional(12);
                }
                case 8:  // Mag
                {
                    return std::make_optional(12);
                }
                case 9:  // Accel
                {
                    return std::make_optional(12);
                }
                case 10:  // AngularRate
                {
                    return std::make_optional(12);
                }
                case 11:  // SensSat
                {
                    return std::make_optional(2);
                }
                case 12:
                {
                    return std::make_optional(40);
                }
                default:
                    return std::nullopt;
            }
        }
        case 3:  // Gnss Group
        {
            switch (binaryField)
            {
                case 0:  // Gnss1TimeUtc
                {
                    return std::make_optional(8);
                }
                case 1:  // Gps1Tow
                {
                    return std::make_optional(8);
                }
                case 2:  // Gps1Week
                {
                    return std::make_optional(2);
                }
                case 3:  // Gnss1NumSats
                {
                    return std::make_optional(1);
                }
                case 4:  // Gnss1Fix
                {
                    return std::make_optional(1);
                }
                case 5:  // Gnss1PosLla
                {
                    return std::make_optional(24);
                }
                case 6:  // Gnss1PosEcef
                {
                    return std::make_optional(24);
                }
                case 7:  // Gnss1VelNed
                {
                    return std::make_optional(12);
                }
                case 8:  // Gnss1VelEcef
                {
                    return std::make_optional(12);
                }
                case 9:  // Gnss1PosUncertainty
                {
                    return std::make_optional(12);
                }
                case 10:  // Gnss1VelUncertainty
                {
                    return std::make_optional(4);
                }
                case 11:  // Gnss1TimeUncertainty
                {
                    return std::make_optional(4);
                }
                case 12:  // Gnss1TimeInfo
                {
                    return std::make_optional(2);
                }
                case 13:  // Gnss1Dop
                {
                    return std::make_optional(28);
                }
                case 17:  // Gnss1Status
                {
                    return std::make_optional(2);
                }
                case 18:  // Gnss1AltMSL
                {
                    return std::make_optional(8);
                }
                default:
                    return std::nullopt;
            }
        }
        case 4:  // Attitude Group
        {
            switch (binaryField)
            {
                case 0:
                {
                    return std::make_optional(2);
                }
                case 1:  // Ypr
                {
                    return std::make_optional(12);
                }
                case 2:  // Quaternion
                {
                    return std::make_optional(16);
                }
                case 3:  // Dcm
                {
                    return std::make_optional(36);
                }
                case 4:  // MagNed
                {
                    return std::make_optional(12);
                }
                case 5:  // AccelNed
                {
                    return std::make_optional(12);
                }
                case 6:  // LinBodyAcc
                {
                    return std::make_optional(12);
                }
                case 7:  // LinAccelNed
                {
                    return std::make_optional(12);
                }
                case 8:  // YprU
                {
                    return std::make_optional(12);
                }
                case 9:
                {
                    return std::make_optional(12);
                }
                case 10:
                {
                    return std::make_optional(28);
                }
                case 11:
                {
                    return std::make_optional(24);
                }
                case 12:  // Heave
                {
                    return std::make_optional(12);
                }
                case 13:  // AttU
                {
                    return std::make_optional(4);
                }
                default:
                    return std::nullopt;
            }
        }
        case 5:  // Ins Group
        {
            switch (binaryField)
            {
                case 0:  // InsStatus
                {
                    return std::make_optional(2);
                }
                case 1:  // PosLla
                {
                    return std::make_optional(24);
                }
                case 2:  // PosEcef
                {
                    return std::make_optional(24);
                }
                case 3:  // VelBody
                {
                    return std::make_optional(12);
                }
                case 4:  // VelNed
                {
                    return std::make_optional(12);
                }
                case 5:  // VelEcef
                {
                    return std::make_optional(12);
                }
                case 6:  // MagEcef
                {
                    return std::make_optional(12);
                }
                case 7:  // AccelEcef
                {
                    return std::make_optional(12);
                }
                case 8:  // LinAccelEcef
                {
                    return std::make_optional(12);
                }
                case 9:  // PosU
                {
                    return std::make_optional(4);
                }
                case 10:  // VelU
                {
                    return std::make_optional(4);
                }
                case 11:
                {
                    return std::make_optional(68);
                }
                case 12:
                {
                    return std::make_optional(64);
                }
                default:
                    return std::nullopt;
            }
        }
        case 6:  // Gnss2 Group
        {
            switch (binaryField)
            {
                case 0:  // Gnss2TimeUtc
                {
                    return std::make_optional(8);
                }
                case 1:  // Gps2Tow
                {
                    return std::make_optional(8);
                }
                case 2:  // Gps2Week
                {
                    return std::make_optional(2);
                }
                case 3:  // Gnss2NumSats
                {
                    return std::make_optional(1);
                }
                case 4:  // Gnss2Fix
                {
                    return std::make_optional(1);
                }
                case 5:  // Gnss2PosLla
                {
                    return std::make_optional(24);
                }
                case 6:  // Gnss2PosEcef
                {
                    return std::make_optional(24);
                }
                case 7:  // Gnss2VelNed
                {
                    return std::make_optional(12);
                }
                case 8:  // Gnss2VelEcef
                {
                    return std::make_optional(12);
                }
                case 9:  // Gnss2PosUncertainty
                {
                    return std::make_optional(12);
                }
                case 10:  // Gnss2VelUncertainty
                {
                    return std::make_optional(4);
                }
                case 11:  // Gnss2TimeUncertainty
                {
                    return std::make_optional(4);
                }
                case 12:  // Gnss2TimeInfo
                {
                    return std::make_optional(2);
                }
                case 13:  // Gnss2Dop
                {
                    return std::make_optional(28);
                }
                case 17:  // Gnss2Status
                {
                    return std::make_optional(2);
                }
                case 18:  // Gnss2AltMSL
                {
                    return std::make_optional(8);
                }
                default:
                    return std::nullopt;
            }
        }
        case 12:  // Gnss3 Group
        {
            switch (binaryField)
            {
                case 0:  // Gnss3TimeUtc
                {
                    return std::make_optional(8);
                }
                case 1:  // Gps3Tow
                {
                    return std::make_optional(8);
                }
                case 2:  // Gps3Week
                {
                    return std::make_optional(2);
                }
                case 3:  // Gnss3NumSats
                {
                    return std::make_optional(1);
                }
                case 4:  // Gnss3Fix
                {
                    return std::make_optional(1);
                }
                case 5:  // Gnss3PosLla
                {
                    return std::make_optional(24);
                }
                case 6:  // Gnss3PosEcef
                {
                    return std::make_optional(24);
                }
                case 7:  // Gnss3VelNed
                {
                    return std::make_optional(12);
                }
                case 8:  // Gnss3VelEcef
                {
                    return std::make_optional(12);
                }
                case 9:  // Gnss3PosUncertainty
                {
                    return std::make_optional(12);
                }
                case 10:  // Gnss3VelUncertainty
                {
                    return std::make_optional(4);
                }
                case 11:  // Gnss3TimeUncertainty
                {
                    return std::make_optional(4);
                }
                case 12:  // Gnss3TimeInfo
                {
                    return std::make_optional(2);
                }
                case 13:  // Gnss3Dop
                {
                    return std::make_optional(28);
                }
                case 17:  // Gnss3Status
                {
                    return std::make_optional(2);
                }
                case 18:  // Gnss3AltMSL
                {
                    return std::make_optional(8);
                }
                default:
                    return std::nullopt;
            }
        }
        case 16:
        {
            switch (binaryField)
            {
                case 0:
                {
                    return std::make_optional(48);
                }
                case 1:
                {
                    return std::make_optional(48);
                }
                case 2:
                {
                    return std::make_optional(48);
                }
                case 3:
                {
                    return std::make_optional(92);
                }
                case 4:
                {
                    return std::make_optional(80);
                }
                case 5:
                {
                    return std::make_optional(76);
                }
                case 6:
                {
                    return std::make_optional(68);
                }
                case 7:
                {
                    return std::make_optional(20);
                }
                case 8:
                {
                    return std::make_optional(40);
                }
                case 9:
                {
                    return std::make_optional(60);
                }
                case 10:
                {
                    return std::make_optional(320);
                }
                case 11:
                {
                    return std::make_optional(192);
                }
                default:
                    return std::nullopt;
            }
        }
        case 17:
        {
            switch (binaryField)
            {
                case 0:
                {
                    return std::make_optional(8);
                }
                case 1:
                {
                    return std::make_optional(2);
                }
                case 2:
                {
                    return std::make_optional(2);
                }
                case 3:
                {
                    return std::make_optional(12);
                }
                case 4:
                {
                    return std::make_optional(36);
                }
                case 5:
                {
                    return std::make_optional(12);
                }
                case 6:
                {
                    return std::make_optional(36);
                }
                case 7:
                {
                    return std::make_optional(4);
                }
                case 8:
                {
                    return std::make_optional(4);
                }
                case 9:
                {
                    return std::make_optional(4);
                }
                case 10:
                {
                    return std::make_optional(4);
                }
                case 11:
                {
                    return std::make_optional(40);
                }
                case 12:
                {
                    return std::make_optional(144);
                }
                case 13:
                {
                    return std::make_optional(12);
                }
                case 14:
                {
                    return std::make_optional(36);
                }
                default:
                    return std::nullopt;
            }
        }
        default:
            return std::nullopt;
    }
}

}  // namespace VN

#endif  // VN_BINARYMEASUREMENTDEFINITIONS_HPP_
