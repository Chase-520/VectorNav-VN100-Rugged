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

#ifndef VN_CLI_COMPOSITEDATA_HPP_
#define VN_CLI_COMPOSITEDATA_HPP_

#include "CLI_MeasurementDatatypes.hpp"
#include "CLI_Registers.hpp"
#include <msclr/marshal_cppstd.h>

#pragma managed(push, off)
#include "vectornav/Interface/CompositeData.hpp"
#pragma managed(pop)

namespace VNSDK
{
public value class CompositeData
{
    private:
    String^ asciiHeader;
    String^ _binaryHeader;
    
    public:
    bool MatchesMessage(String^ headerToCheck)
    {
        return (asciiHeader != nullptr) && (asciiHeader == headerToCheck);
    };
    
    bool MatchesMessage(Registers::System::BinaryOutput^ registerToCheck)
    {
        if (_binaryHeader == nullptr) { return false; }
        marshal_context ^ context = gcnew marshal_context();
        const char* myStr = context->marshal_as<const char*>(_binaryHeader);
        bool retVal = (strcmp(VN::binaryHeaderToString<25>(registerToCheck->toBinaryHeader()).c_str(), myStr) == 0);
        delete context;
        return retVal;
    };
    
    #if (TIME_GROUP_ENABLE)
    value struct Time
    {
        #if (TIME_GROUP_ENABLE & TIME_TIMESTARTUP_BIT)
        Nullable<VNSDK::Time> timeStartup;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPS_BIT)
        Nullable<VNSDK::Time> timeGps;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSTOW_BIT)
        Nullable<VNSDK::Time> timeGpsTow;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSWEEK_BIT)
        Nullable<uint16_t> timeGpsWeek;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMESYNCIN_BIT)
        Nullable<VNSDK::Time> timeSyncIn;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSPPS_BIT)
        Nullable<VNSDK::Time> timeGpsPps;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEUTC_BIT)
        Nullable<VNSDK::TimeUtc> timeUtc;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_SYNCINCNT_BIT)
        Nullable<uint32_t> syncInCnt;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_SYNCOUTCNT_BIT)
        Nullable<uint32_t> syncOutCnt;
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMESTATUS_BIT)
        Nullable<VNSDK::TimeStatus> timeStatus;
        #endif
    };
    Time time;
    #endif
    #if (IMU_GROUP_ENABLE)
    value struct Imu
    {
        #if (IMU_GROUP_ENABLE & IMU_IMUSTATUS_BIT)
        Nullable<VNSDK::ImuStatus> imuStatus;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPMAG_BIT)
        Nullable<VNSDK::Vec3f> uncompMag;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPACCEL_BIT)
        Nullable<VNSDK::Vec3f> uncompAccel;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPGYRO_BIT)
        Nullable<VNSDK::Vec3f> uncompGyro;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_TEMPERATURE_BIT)
        Nullable<float> temperature;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_PRESSURE_BIT)
        Nullable<float> pressure;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_DELTATHETA_BIT)
        Nullable<DeltaTheta> deltaTheta;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_DELTAVEL_BIT)
        Nullable<VNSDK::Vec3f> deltaVel;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_MAG_BIT)
        Nullable<VNSDK::Vec3f> mag;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_ACCEL_BIT)
        Nullable<VNSDK::Vec3f> accel;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_ANGULARRATE_BIT)
        Nullable<VNSDK::Vec3f> angularRate;
        #endif
        #if (IMU_GROUP_ENABLE & IMU_SENSSAT_BIT)
        Nullable<uint16_t> sensSat;
        #endif
    };
    Imu imu;
    #endif
    #if (GNSS_GROUP_ENABLE)
    value struct Gnss
    {
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEUTC_BIT)
        Nullable<VNSDK::TimeUtc> gnss1TimeUtc;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GPS1TOW_BIT)
        Nullable<VNSDK::Time> gps1Tow;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GPS1WEEK_BIT)
        Nullable<uint16_t> gps1Week;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1NUMSATS_BIT)
        Nullable<uint8_t> gnss1NumSats;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1FIX_BIT)
        Nullable<uint8_t> gnss1Fix;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSLLA_BIT)
        Nullable<Lla> gnss1PosLla;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSECEF_BIT)
        Nullable<VNSDK::Vec3d> gnss1PosEcef;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELNED_BIT)
        Nullable<VNSDK::Vec3f> gnss1VelNed;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELECEF_BIT)
        Nullable<VNSDK::Vec3f> gnss1VelEcef;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSUNCERTAINTY_BIT)
        Nullable<VNSDK::Vec3f> gnss1PosUncertainty;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELUNCERTAINTY_BIT)
        Nullable<float> gnss1VelUncertainty;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEUNCERTAINTY_BIT)
        Nullable<float> gnss1TimeUncertainty;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEINFO_BIT)
        Nullable<VNSDK::GnssTimeInfo> gnss1TimeInfo;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1DOP_BIT)
        Nullable<VNSDK::GnssDop> gnss1Dop;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1SATINFO_BIT)
        Nullable<VNSDK::GnssSatInfo> gnss1SatInfo;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1RAWMEAS_BIT)
        Nullable<VNSDK::GnssRawMeas> gnss1RawMeas;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1STATUS_BIT)
        Nullable<VNSDK::GnssStatus> gnss1Status;
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1ALTMSL_BIT)
        Nullable<double> gnss1AltMsl;
        #endif
    };
    Gnss gnss;
    #endif
    #if (ATTITUDE_GROUP_ENABLE)
    value struct Attitude
    {
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_YPR_BIT)
        Nullable<Ypr> ypr;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_QUATERNION_BIT)
        Nullable<Quaternion> quaternion;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_DCM_BIT)
        Nullable<VNSDK::Mat3f> dcm;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_MAGNED_BIT)
        Nullable<VNSDK::Vec3f> magNed;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_ACCELNED_BIT)
        Nullable<VNSDK::Vec3f> accelNed;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_LINBODYACC_BIT)
        Nullable<VNSDK::Vec3f> linBodyAcc;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_LINACCELNED_BIT)
        Nullable<VNSDK::Vec3f> linAccelNed;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_YPRU_BIT)
        Nullable<VNSDK::Vec3f> yprU;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_HEAVE_BIT)
        Nullable<VNSDK::Vec3f> heave;
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_ATTU_BIT)
        Nullable<float> attU;
        #endif
    };
    Attitude attitude;
    #endif
    #if (INS_GROUP_ENABLE)
    value struct Ins
    {
        #if (INS_GROUP_ENABLE & INS_INSSTATUS_BIT)
        Nullable<VNSDK::InsStatus> insStatus;
        #endif
        #if (INS_GROUP_ENABLE & INS_POSLLA_BIT)
        Nullable<Lla> posLla;
        #endif
        #if (INS_GROUP_ENABLE & INS_POSECEF_BIT)
        Nullable<VNSDK::Vec3d> posEcef;
        #endif
        #if (INS_GROUP_ENABLE & INS_VELBODY_BIT)
        Nullable<VNSDK::Vec3f> velBody;
        #endif
        #if (INS_GROUP_ENABLE & INS_VELNED_BIT)
        Nullable<VNSDK::Vec3f> velNed;
        #endif
        #if (INS_GROUP_ENABLE & INS_VELECEF_BIT)
        Nullable<VNSDK::Vec3f> velEcef;
        #endif
        #if (INS_GROUP_ENABLE & INS_MAGECEF_BIT)
        Nullable<VNSDK::Vec3f> magEcef;
        #endif
        #if (INS_GROUP_ENABLE & INS_ACCELECEF_BIT)
        Nullable<VNSDK::Vec3f> accelEcef;
        #endif
        #if (INS_GROUP_ENABLE & INS_LINACCELECEF_BIT)
        Nullable<VNSDK::Vec3f> linAccelEcef;
        #endif
        #if (INS_GROUP_ENABLE & INS_POSU_BIT)
        Nullable<float> posU;
        #endif
        #if (INS_GROUP_ENABLE & INS_VELU_BIT)
        Nullable<float> velU;
        #endif
    };
    Ins ins;
    #endif
    #if (GNSS2_GROUP_ENABLE)
    value struct Gnss2
    {
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEUTC_BIT)
        Nullable<VNSDK::TimeUtc> gnss2TimeUtc;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GPS2TOW_BIT)
        Nullable<VNSDK::Time> gps2Tow;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GPS2WEEK_BIT)
        Nullable<uint16_t> gps2Week;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2NUMSATS_BIT)
        Nullable<uint8_t> gnss2NumSats;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2FIX_BIT)
        Nullable<uint8_t> gnss2Fix;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSLLA_BIT)
        Nullable<Lla> gnss2PosLla;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSECEF_BIT)
        Nullable<VNSDK::Vec3d> gnss2PosEcef;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELNED_BIT)
        Nullable<VNSDK::Vec3f> gnss2VelNed;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELECEF_BIT)
        Nullable<VNSDK::Vec3f> gnss2VelEcef;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSUNCERTAINTY_BIT)
        Nullable<VNSDK::Vec3f> gnss2PosUncertainty;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELUNCERTAINTY_BIT)
        Nullable<float> gnss2VelUncertainty;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEUNCERTAINTY_BIT)
        Nullable<float> gnss2TimeUncertainty;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEINFO_BIT)
        Nullable<VNSDK::GnssTimeInfo> gnss2TimeInfo;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2DOP_BIT)
        Nullable<VNSDK::GnssDop> gnss2Dop;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2SATINFO_BIT)
        Nullable<VNSDK::GnssSatInfo> gnss2SatInfo;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2RAWMEAS_BIT)
        Nullable<VNSDK::GnssRawMeas> gnss2RawMeas;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2STATUS_BIT)
        Nullable<VNSDK::GnssStatus> gnss2Status;
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2ALTMSL_BIT)
        Nullable<double> gnss2AltMsl;
        #endif
    };
    Gnss2 gnss2;
    #endif
    #if (GNSS3_GROUP_ENABLE)
    value struct Gnss3
    {
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEUTC_BIT)
        Nullable<VNSDK::TimeUtc> gnss3TimeUtc;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GPS3TOW_BIT)
        Nullable<VNSDK::Time> gps3Tow;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GPS3WEEK_BIT)
        Nullable<uint16_t> gps3Week;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3NUMSATS_BIT)
        Nullable<uint8_t> gnss3NumSats;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3FIX_BIT)
        Nullable<uint8_t> gnss3Fix;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSLLA_BIT)
        Nullable<Lla> gnss3PosLla;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSECEF_BIT)
        Nullable<VNSDK::Vec3d> gnss3PosEcef;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELNED_BIT)
        Nullable<VNSDK::Vec3f> gnss3VelNed;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELECEF_BIT)
        Nullable<VNSDK::Vec3f> gnss3VelEcef;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSUNCERTAINTY_BIT)
        Nullable<VNSDK::Vec3f> gnss3PosUncertainty;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELUNCERTAINTY_BIT)
        Nullable<float> gnss3VelUncertainty;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEUNCERTAINTY_BIT)
        Nullable<float> gnss3TimeUncertainty;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEINFO_BIT)
        Nullable<VNSDK::GnssTimeInfo> gnss3TimeInfo;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3DOP_BIT)
        Nullable<VNSDK::GnssDop> gnss3Dop;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3SATINFO_BIT)
        Nullable<VNSDK::GnssSatInfo> gnss3SatInfo;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3RAWMEAS_BIT)
        Nullable<VNSDK::GnssRawMeas> gnss3RawMeas;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3STATUS_BIT)
        Nullable<VNSDK::GnssStatus> gnss3Status;
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3ALTMSL_BIT)
        Nullable<double> gnss3AltMsl;
        #endif
    };
    Gnss3 gnss3;
    #endif
    
    CompositeData(VN::CompositeData &other)
    {
        if (std::holds_alternative<VN::AsciiHeader>(other.header()))
        {
            VN::AsciiHeader cppHeader = std::get<VN::AsciiHeader>(other.header());
            asciiHeader = gcnew String(msclr::interop::marshal_as<System::String^>(std::string(cppHeader)));
        }
        else if (std::holds_alternative<VN::BinaryHeader>(other.header()))
        {
            _binaryHeader = gcnew String(msclr::interop::marshal_as<System::String^>(VN::binaryHeaderToString<35>(std::get<VN::BinaryHeader>(other.header())).c_str()));
        }
        else
        {
            // Not possible
            VN_ABORT();
        }
        
        #if (TIME_GROUP_ENABLE & TIME_TIMESTARTUP_BIT)
        if(other.time.timeStartup.has_value())
        {
            time.timeStartup = VNSDK::Time(other.time.timeStartup->nanoseconds());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPS_BIT)
        if(other.time.timeGps.has_value())
        {
            time.timeGps = VNSDK::Time(other.time.timeGps->nanoseconds());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSTOW_BIT)
        if(other.time.timeGpsTow.has_value())
        {
            time.timeGpsTow = VNSDK::Time(other.time.timeGpsTow->nanoseconds());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSWEEK_BIT)
        if(other.time.timeGpsWeek.has_value())
        {
            time.timeGpsWeek = other.time.timeGpsWeek.value();
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMESYNCIN_BIT)
        if(other.time.timeSyncIn.has_value())
        {
            time.timeSyncIn = VNSDK::Time(other.time.timeSyncIn->nanoseconds());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEGPSPPS_BIT)
        if(other.time.timeGpsPps.has_value())
        {
            time.timeGpsPps = VNSDK::Time(other.time.timeGpsPps->nanoseconds());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMEUTC_BIT)
        if(other.time.timeUtc.has_value())
        {
            time.timeUtc = TimeUtc(other.time.timeUtc.value());
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_SYNCINCNT_BIT)
        if(other.time.syncInCnt.has_value())
        {
            time.syncInCnt = other.time.syncInCnt.value();
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_SYNCOUTCNT_BIT)
        if(other.time.syncOutCnt.has_value())
        {
            time.syncOutCnt = other.time.syncOutCnt.value();
        }
        #endif
        #if (TIME_GROUP_ENABLE & TIME_TIMESTATUS_BIT)
        if(other.time.timeStatus.has_value())
        {
            time.timeStatus = TimeStatus(uint8_t(other.time.timeStatus.value()));
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_IMUSTATUS_BIT)
        if(other.imu.imuStatus.has_value())
        {
            imu.imuStatus = ImuStatus(uint16_t(other.imu.imuStatus.value()));
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPMAG_BIT)
        if(other.imu.uncompMag.has_value())
        {
            imu.uncompMag = VNSDK::Vec3f(other.imu.uncompMag.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPACCEL_BIT)
        if(other.imu.uncompAccel.has_value())
        {
            imu.uncompAccel = VNSDK::Vec3f(other.imu.uncompAccel.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_UNCOMPGYRO_BIT)
        if(other.imu.uncompGyro.has_value())
        {
            imu.uncompGyro = VNSDK::Vec3f(other.imu.uncompGyro.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_TEMPERATURE_BIT)
        if(other.imu.temperature.has_value())
        {
            imu.temperature = other.imu.temperature.value();
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_PRESSURE_BIT)
        if(other.imu.pressure.has_value())
        {
            imu.pressure = other.imu.pressure.value();
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_DELTATHETA_BIT)
        if(other.imu.deltaTheta.has_value())
        {
            imu.deltaTheta = DeltaTheta(other.imu.deltaTheta.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_DELTAVEL_BIT)
        if(other.imu.deltaVel.has_value())
        {
            imu.deltaVel = VNSDK::Vec3f(other.imu.deltaVel.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_MAG_BIT)
        if(other.imu.mag.has_value())
        {
            imu.mag = VNSDK::Vec3f(other.imu.mag.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_ACCEL_BIT)
        if(other.imu.accel.has_value())
        {
            imu.accel = VNSDK::Vec3f(other.imu.accel.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_ANGULARRATE_BIT)
        if(other.imu.angularRate.has_value())
        {
            imu.angularRate = VNSDK::Vec3f(other.imu.angularRate.value());
        }
        #endif
        #if (IMU_GROUP_ENABLE & IMU_SENSSAT_BIT)
        if(other.imu.sensSat.has_value())
        {
            imu.sensSat = other.imu.sensSat.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEUTC_BIT)
        if(other.gnss.gnss1TimeUtc.has_value())
        {
            gnss.gnss1TimeUtc = TimeUtc(other.gnss.gnss1TimeUtc.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GPS1TOW_BIT)
        if(other.gnss.gps1Tow.has_value())
        {
            gnss.gps1Tow = VNSDK::Time(other.gnss.gps1Tow->nanoseconds());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GPS1WEEK_BIT)
        if(other.gnss.gps1Week.has_value())
        {
            gnss.gps1Week = other.gnss.gps1Week.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1NUMSATS_BIT)
        if(other.gnss.gnss1NumSats.has_value())
        {
            gnss.gnss1NumSats = other.gnss.gnss1NumSats.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1FIX_BIT)
        if(other.gnss.gnss1Fix.has_value())
        {
            gnss.gnss1Fix = other.gnss.gnss1Fix.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSLLA_BIT)
        if(other.gnss.gnss1PosLla.has_value())
        {
            gnss.gnss1PosLla = Lla(other.gnss.gnss1PosLla.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSECEF_BIT)
        if(other.gnss.gnss1PosEcef.has_value())
        {
            gnss.gnss1PosEcef = VNSDK::Vec3d(other.gnss.gnss1PosEcef.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELNED_BIT)
        if(other.gnss.gnss1VelNed.has_value())
        {
            gnss.gnss1VelNed = VNSDK::Vec3f(other.gnss.gnss1VelNed.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELECEF_BIT)
        if(other.gnss.gnss1VelEcef.has_value())
        {
            gnss.gnss1VelEcef = VNSDK::Vec3f(other.gnss.gnss1VelEcef.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1POSUNCERTAINTY_BIT)
        if(other.gnss.gnss1PosUncertainty.has_value())
        {
            gnss.gnss1PosUncertainty = VNSDK::Vec3f(other.gnss.gnss1PosUncertainty.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1VELUNCERTAINTY_BIT)
        if(other.gnss.gnss1VelUncertainty.has_value())
        {
            gnss.gnss1VelUncertainty = other.gnss.gnss1VelUncertainty.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEUNCERTAINTY_BIT)
        if(other.gnss.gnss1TimeUncertainty.has_value())
        {
            gnss.gnss1TimeUncertainty = other.gnss.gnss1TimeUncertainty.value();
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1TIMEINFO_BIT)
        if(other.gnss.gnss1TimeInfo.has_value())
        {
            gnss.gnss1TimeInfo = GnssTimeInfo(other.gnss.gnss1TimeInfo.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1DOP_BIT)
        if(other.gnss.gnss1Dop.has_value())
        {
            gnss.gnss1Dop = GnssDop(other.gnss.gnss1Dop.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1SATINFO_BIT)
        if(other.gnss.gnss1SatInfo.has_value())
        {
            gnss.gnss1SatInfo = GnssSatInfo(other.gnss.gnss1SatInfo.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1RAWMEAS_BIT)
        if(other.gnss.gnss1RawMeas.has_value())
        {
            gnss.gnss1RawMeas = GnssRawMeas(other.gnss.gnss1RawMeas.value());
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1STATUS_BIT)
        if(other.gnss.gnss1Status.has_value())
        {
            gnss.gnss1Status = GnssStatus(uint16_t(other.gnss.gnss1Status.value()));
        }
        #endif
        #if (GNSS_GROUP_ENABLE & GNSS_GNSS1ALTMSL_BIT)
        if(other.gnss.gnss1AltMsl.has_value())
        {
            gnss.gnss1AltMsl = other.gnss.gnss1AltMsl.value();
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_YPR_BIT)
        if(other.attitude.ypr.has_value())
        {
            attitude.ypr = Ypr(other.attitude.ypr.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_QUATERNION_BIT)
        if(other.attitude.quaternion.has_value())
        {
            attitude.quaternion = Quaternion(other.attitude.quaternion.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_DCM_BIT)
        if(other.attitude.dcm.has_value())
        {
            attitude.dcm = VNSDK::Mat3f(other.attitude.dcm.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_MAGNED_BIT)
        if(other.attitude.magNed.has_value())
        {
            attitude.magNed = VNSDK::Vec3f(other.attitude.magNed.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_ACCELNED_BIT)
        if(other.attitude.accelNed.has_value())
        {
            attitude.accelNed = VNSDK::Vec3f(other.attitude.accelNed.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_LINBODYACC_BIT)
        if(other.attitude.linBodyAcc.has_value())
        {
            attitude.linBodyAcc = VNSDK::Vec3f(other.attitude.linBodyAcc.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_LINACCELNED_BIT)
        if(other.attitude.linAccelNed.has_value())
        {
            attitude.linAccelNed = VNSDK::Vec3f(other.attitude.linAccelNed.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_YPRU_BIT)
        if(other.attitude.yprU.has_value())
        {
            attitude.yprU = VNSDK::Vec3f(other.attitude.yprU.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_HEAVE_BIT)
        if(other.attitude.heave.has_value())
        {
            attitude.heave = VNSDK::Vec3f(other.attitude.heave.value());
        }
        #endif
        #if (ATTITUDE_GROUP_ENABLE & ATTITUDE_ATTU_BIT)
        if(other.attitude.attU.has_value())
        {
            attitude.attU = other.attitude.attU.value();
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_INSSTATUS_BIT)
        if(other.ins.insStatus.has_value())
        {
            ins.insStatus = InsStatus(uint16_t(other.ins.insStatus.value()));
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_POSLLA_BIT)
        if(other.ins.posLla.has_value())
        {
            ins.posLla = Lla(other.ins.posLla.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_POSECEF_BIT)
        if(other.ins.posEcef.has_value())
        {
            ins.posEcef = VNSDK::Vec3d(other.ins.posEcef.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_VELBODY_BIT)
        if(other.ins.velBody.has_value())
        {
            ins.velBody = VNSDK::Vec3f(other.ins.velBody.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_VELNED_BIT)
        if(other.ins.velNed.has_value())
        {
            ins.velNed = VNSDK::Vec3f(other.ins.velNed.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_VELECEF_BIT)
        if(other.ins.velEcef.has_value())
        {
            ins.velEcef = VNSDK::Vec3f(other.ins.velEcef.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_MAGECEF_BIT)
        if(other.ins.magEcef.has_value())
        {
            ins.magEcef = VNSDK::Vec3f(other.ins.magEcef.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_ACCELECEF_BIT)
        if(other.ins.accelEcef.has_value())
        {
            ins.accelEcef = VNSDK::Vec3f(other.ins.accelEcef.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_LINACCELECEF_BIT)
        if(other.ins.linAccelEcef.has_value())
        {
            ins.linAccelEcef = VNSDK::Vec3f(other.ins.linAccelEcef.value());
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_POSU_BIT)
        if(other.ins.posU.has_value())
        {
            ins.posU = other.ins.posU.value();
        }
        #endif
        #if (INS_GROUP_ENABLE & INS_VELU_BIT)
        if(other.ins.velU.has_value())
        {
            ins.velU = other.ins.velU.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEUTC_BIT)
        if(other.gnss2.gnss2TimeUtc.has_value())
        {
            gnss2.gnss2TimeUtc = TimeUtc(other.gnss2.gnss2TimeUtc.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GPS2TOW_BIT)
        if(other.gnss2.gps2Tow.has_value())
        {
            gnss2.gps2Tow = VNSDK::Time(other.gnss2.gps2Tow->nanoseconds());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GPS2WEEK_BIT)
        if(other.gnss2.gps2Week.has_value())
        {
            gnss2.gps2Week = other.gnss2.gps2Week.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2NUMSATS_BIT)
        if(other.gnss2.gnss2NumSats.has_value())
        {
            gnss2.gnss2NumSats = other.gnss2.gnss2NumSats.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2FIX_BIT)
        if(other.gnss2.gnss2Fix.has_value())
        {
            gnss2.gnss2Fix = other.gnss2.gnss2Fix.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSLLA_BIT)
        if(other.gnss2.gnss2PosLla.has_value())
        {
            gnss2.gnss2PosLla = Lla(other.gnss2.gnss2PosLla.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSECEF_BIT)
        if(other.gnss2.gnss2PosEcef.has_value())
        {
            gnss2.gnss2PosEcef = VNSDK::Vec3d(other.gnss2.gnss2PosEcef.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELNED_BIT)
        if(other.gnss2.gnss2VelNed.has_value())
        {
            gnss2.gnss2VelNed = VNSDK::Vec3f(other.gnss2.gnss2VelNed.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELECEF_BIT)
        if(other.gnss2.gnss2VelEcef.has_value())
        {
            gnss2.gnss2VelEcef = VNSDK::Vec3f(other.gnss2.gnss2VelEcef.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2POSUNCERTAINTY_BIT)
        if(other.gnss2.gnss2PosUncertainty.has_value())
        {
            gnss2.gnss2PosUncertainty = VNSDK::Vec3f(other.gnss2.gnss2PosUncertainty.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2VELUNCERTAINTY_BIT)
        if(other.gnss2.gnss2VelUncertainty.has_value())
        {
            gnss2.gnss2VelUncertainty = other.gnss2.gnss2VelUncertainty.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEUNCERTAINTY_BIT)
        if(other.gnss2.gnss2TimeUncertainty.has_value())
        {
            gnss2.gnss2TimeUncertainty = other.gnss2.gnss2TimeUncertainty.value();
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2TIMEINFO_BIT)
        if(other.gnss2.gnss2TimeInfo.has_value())
        {
            gnss2.gnss2TimeInfo = GnssTimeInfo(other.gnss2.gnss2TimeInfo.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2DOP_BIT)
        if(other.gnss2.gnss2Dop.has_value())
        {
            gnss2.gnss2Dop = GnssDop(other.gnss2.gnss2Dop.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2SATINFO_BIT)
        if(other.gnss2.gnss2SatInfo.has_value())
        {
            gnss2.gnss2SatInfo = GnssSatInfo(other.gnss2.gnss2SatInfo.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2RAWMEAS_BIT)
        if(other.gnss2.gnss2RawMeas.has_value())
        {
            gnss2.gnss2RawMeas = GnssRawMeas(other.gnss2.gnss2RawMeas.value());
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2STATUS_BIT)
        if(other.gnss2.gnss2Status.has_value())
        {
            gnss2.gnss2Status = GnssStatus(uint16_t(other.gnss2.gnss2Status.value()));
        }
        #endif
        #if (GNSS2_GROUP_ENABLE & GNSS2_GNSS2ALTMSL_BIT)
        if(other.gnss2.gnss2AltMsl.has_value())
        {
            gnss2.gnss2AltMsl = other.gnss2.gnss2AltMsl.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEUTC_BIT)
        if(other.gnss3.gnss3TimeUtc.has_value())
        {
            gnss3.gnss3TimeUtc = TimeUtc(other.gnss3.gnss3TimeUtc.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GPS3TOW_BIT)
        if(other.gnss3.gps3Tow.has_value())
        {
            gnss3.gps3Tow = VNSDK::Time(other.gnss3.gps3Tow->nanoseconds());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GPS3WEEK_BIT)
        if(other.gnss3.gps3Week.has_value())
        {
            gnss3.gps3Week = other.gnss3.gps3Week.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3NUMSATS_BIT)
        if(other.gnss3.gnss3NumSats.has_value())
        {
            gnss3.gnss3NumSats = other.gnss3.gnss3NumSats.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3FIX_BIT)
        if(other.gnss3.gnss3Fix.has_value())
        {
            gnss3.gnss3Fix = other.gnss3.gnss3Fix.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSLLA_BIT)
        if(other.gnss3.gnss3PosLla.has_value())
        {
            gnss3.gnss3PosLla = Lla(other.gnss3.gnss3PosLla.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSECEF_BIT)
        if(other.gnss3.gnss3PosEcef.has_value())
        {
            gnss3.gnss3PosEcef = VNSDK::Vec3d(other.gnss3.gnss3PosEcef.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELNED_BIT)
        if(other.gnss3.gnss3VelNed.has_value())
        {
            gnss3.gnss3VelNed = VNSDK::Vec3f(other.gnss3.gnss3VelNed.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELECEF_BIT)
        if(other.gnss3.gnss3VelEcef.has_value())
        {
            gnss3.gnss3VelEcef = VNSDK::Vec3f(other.gnss3.gnss3VelEcef.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3POSUNCERTAINTY_BIT)
        if(other.gnss3.gnss3PosUncertainty.has_value())
        {
            gnss3.gnss3PosUncertainty = VNSDK::Vec3f(other.gnss3.gnss3PosUncertainty.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3VELUNCERTAINTY_BIT)
        if(other.gnss3.gnss3VelUncertainty.has_value())
        {
            gnss3.gnss3VelUncertainty = other.gnss3.gnss3VelUncertainty.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEUNCERTAINTY_BIT)
        if(other.gnss3.gnss3TimeUncertainty.has_value())
        {
            gnss3.gnss3TimeUncertainty = other.gnss3.gnss3TimeUncertainty.value();
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3TIMEINFO_BIT)
        if(other.gnss3.gnss3TimeInfo.has_value())
        {
            gnss3.gnss3TimeInfo = GnssTimeInfo(other.gnss3.gnss3TimeInfo.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3DOP_BIT)
        if(other.gnss3.gnss3Dop.has_value())
        {
            gnss3.gnss3Dop = GnssDop(other.gnss3.gnss3Dop.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3SATINFO_BIT)
        if(other.gnss3.gnss3SatInfo.has_value())
        {
            gnss3.gnss3SatInfo = GnssSatInfo(other.gnss3.gnss3SatInfo.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3RAWMEAS_BIT)
        if(other.gnss3.gnss3RawMeas.has_value())
        {
            gnss3.gnss3RawMeas = GnssRawMeas(other.gnss3.gnss3RawMeas.value());
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3STATUS_BIT)
        if(other.gnss3.gnss3Status.has_value())
        {
            gnss3.gnss3Status = GnssStatus(uint16_t(other.gnss3.gnss3Status.value()));
        }
        #endif
        #if (GNSS3_GROUP_ENABLE & GNSS3_GNSS3ALTMSL_BIT)
        if(other.gnss3.gnss3AltMsl.has_value())
        {
            gnss3.gnss3AltMsl = other.gnss3.gnss3AltMsl.value();
        }
        #endif
    }; // CompositeData(VN::CompositeData &other)
}; // public value class CompositeData
} // namespace VNSDK


#endif //VN_CLI_COMPOSITEDATA_HPP_


