/*
    AstroStep Focuser
    Copyright (C) 2013-2019 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "indi_astrostep.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

static std::unique_ptr<AstroStep> astrostep(new AstroStep());

AstroStep::AstroStep()
{
    setVersion(0, 1);

    // Can move in Absolute & Relative motions, can AbortFocuser motion, and has variable speed.
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_REVERSE |
                    FOCUSER_HAS_VARIABLE_SPEED | FOCUSER_CAN_SYNC);
    setSupportedConnections(CONNECTION_SERIAL | CONNECTION_TCP);
}

bool AstroStep::initProperties()
{
    INDI::Focuser::initProperties();

    FocusSpeedN[0].min   = 1;
    FocusSpeedN[0].max   = 4000000;
    FocusSpeedN[0].value = 200000;


    // Focuser temperature
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", -50, 70., 0., 0.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
                       MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Temperature Settings
    IUFillNumber(&TemperatureSettingN[0], "Calibration", "", "%6.2f", -100, 100, 0.5, 0);
    IUFillNumber(&TemperatureSettingN[1], "Coefficient", "", "%6.2f", -100, 100, 0.5, 0);
    IUFillNumberVector(&TemperatureSettingNP, TemperatureSettingN, 2, getDeviceName(), "T. Settings", "",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Compensate for temperature
    IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
    IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "T. Compensate",
                       "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillSwitch(&CoilPowerS[COIL_POWER_ON], "COIL_POWER_ON", "On", ISS_ON);
    IUFillSwitch(&CoilPowerS[COIL_POWER_OFF], "COIL_POWER_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&CoilPowerSP, CoilPowerS, 2, getDeviceName(), "FOCUS_COIL_POWER", "Coil Power", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&GotoHomeS[0], "GOTO_HOME", "Go", ISS_OFF);
    IUFillSwitchVector(&GotoHomeSP, GotoHomeS, 1, getDeviceName(), "FOCUS_HOME", "Home", MAIN_CONTROL_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min   = 0.;
    FocusRelPosN[0].max   = 1000000.;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step  = 100;

    FocusAbsPosN[0].min   = 0.;
    FocusAbsPosN[0].max   = 1000000.;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step  = 100;

    setDefaultPollingPeriod(500);
    addDebugControl();

    return true;
}

bool AstroStep::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(&GotoHomeSP);
        defineProperty(&TemperatureNP);
        defineProperty(&TemperatureSettingNP);
        defineProperty(&TemperatureCompensateSP);
        defineProperty(&CoilPowerSP);

        GetFocusParams();

        LOG_INFO("AstroStep parameters updated, focuser ready for use.");
    }
    else
    {
        deleteProperty(GotoHomeSP.name);
        deleteProperty(TemperatureNP.name);
        deleteProperty(TemperatureSettingNP.name);
        deleteProperty(TemperatureCompensateSP.name);
        deleteProperty(CoilPowerSP.name);
    }

    return true;
}

bool AstroStep::Handshake()
{
    if (Ack())
    {
        LOG_INFO("AstroStep is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO(
        "Error retrieving data from AstroStep, please ensure AstroStep controller is powered and the port is correct.");
    return false;
}

const char * AstroStep::getDefaultName()
{
    return "AstroStep";
}

bool AstroStep::Ack()
{
    bool success = false;

    for (int i = 0; i < 3; i++)
    {
        if (readVersion())
        {
            success = true;
            break;
        }

        sleep(1);
    }

    return success;
}

bool AstroStep::readCoilPowerState()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GE#", res) == false)
    {
        return false;
    }

    uint32_t temp = 0;

    int rc = sscanf(res, "%u#", &temp);

    if (rc > 0)
    {
        if(temp == 0)
        {
            CoilPowerS[COIL_POWER_OFF].s = ISS_ON;
        }
        else if (temp == 1)
        {
            CoilPowerS[COIL_POWER_ON].s = ISS_ON;
        }
        else
        {
            LOGF_ERROR("Invalid Response: focuser Coil Power value (%s)", res);
            return false;
        }
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser Coil Power value (%s)", res);
        return false;
    }
    return true;
}

bool AstroStep::readReverseDirection()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GR#", res) == false)
        return false;

    int temp = 0;

    int rc = sscanf(res, "%d#", &temp);

    if (rc > 0)
    {
        if(temp == 0)
        {
            FocusReverseS[INDI_DISABLED].s = ISS_ON;
        }
        else if (temp == 1)
        {
            FocusReverseS[INDI_ENABLED].s = ISS_ON;
        }
        else
        {
            LOGF_ERROR("Invalid Response: focuser Reverse direction value (%s)", res);
            return false;
        }
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser Reverse direction value (%s)", res);
        return false;
    }
    return true;
}


bool AstroStep::readVersion()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GV#", res, true) == false)
        return false;

    LOGF_INFO("Detected firmware version %s", res);

    return true;
}

bool AstroStep::readTemperature()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GT#", res) == false)
        return false;

    int wholepart = 0;
    int fractpart = 0;
    int rc = sscanf(res, "%d.%d#", &wholepart, &fractpart);
    if (rc > 0)
        // Signed hex
        TemperatureN[0].value = float(wholepart + fractpart / 10);
    else
    {
        LOGF_ERROR("Unknown error: focuser temperature value (%s)", res);
        return false;
    }

    return true;
}

bool AstroStep::readTemperatureCoefficient()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GC#", res) == false)
        return false;

    int wholepart = 0;
    int fractpart = 0;
    int rc = sscanf(res, "%d.%d#", &wholepart, &fractpart);
    if (rc > 0)
        // Signed hex
        TemperatureSettingN[1].value = float(wholepart + fractpart / 10);
    else
    {
        LOGF_ERROR("Unknown error: focuser temperature value (%s)", res);
        return false;
    }

    return true;
}

bool AstroStep::readTemperatureCalibration()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GO#", res) == false)
        return false;

    int wholepart = 0;
    int fractpart = 0;
    int rc = sscanf(res, "%d.%d#", &wholepart, &fractpart);
    if (rc > 0)
        // Signed hex
        TemperatureSettingN[0].value = float(wholepart + fractpart / 10);
    else
    {
        LOGF_ERROR("Unknown error: focuser temperature value (%s)", res);
        return false;
    }

    return true;
}

bool AstroStep::readPosition()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GP#", res) == false)
        return false;

    int pos;
    int rc = sscanf(res, "%i#", &pos);

    if (rc > 0)
        FocusAbsPosN[0].value = pos;
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool AstroStep::readSpeed()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GD#", res) == false)
    {
        return false;
    }

    int speed = 0;
    int rc = sscanf(res, "%i#", &speed);

    if (rc > 0)
    {
        FocusSpeedN[0].value = speed;
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser speed value (%s)", res);
        return false;
    }
    return true;
}

bool AstroStep::isMoving()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GI#", res) == false)
        return false;

    // JM 2020-03-13: 01# and 1# should be both accepted
    if (strstr(res, "1#"))
        return true;
    else if (strstr(res, "0#"))
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}

bool AstroStep::setTemperatureCalibration(uint32_t calibration)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SO%u#", calibration);
    return sendCommand(cmd);
}

bool AstroStep::setTemperatureCoefficient(uint32_t compensation)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SC%d#", compensation);
    return sendCommand(cmd);
}

bool AstroStep::SyncFocuser(uint32_t ticks)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SP%09i#", ticks);
    return sendCommand(cmd);
}

bool AstroStep::MoveFocuser(uint32_t position)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SN%09i#", position);
    // Set Position First
    if (sendCommand(cmd) == false)
        return false;
    // Now start motion toward position
    if (sendCommand(":FG#") == false)
        return false;

    return true;
}

bool AstroStep::setCoilPowerState(CoilPower enable)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SE%d#", enable);
    return sendCommand(cmd);
}

bool AstroStep::ReverseFocuser(bool enable)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SR%d#", static_cast<int>(enable));
    return sendCommand(cmd);
}

bool AstroStep::setGotoHome()
{
    char cmd[ML_RES] = {0};
    if(isMoving())
    {
        AbortFocuser();
    }
    snprintf(cmd, ML_RES, ":HO#");
    return sendCommand(cmd);
}

bool AstroStep::setSpeed(uint32_t speed)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SD%i#", speed);
    return sendCommand(cmd);
}

bool AstroStep::setTemperatureCompensation(bool enable)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":%c#", enable ? '+' : '-');
    return sendCommand(cmd);
}
bool AstroStep::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {

        // Temperature Compensation Mode
        if (strcmp(TemperatureCompensateSP.name, name) == 0)
        {
            int last_index = IUFindOnSwitchIndex(&TemperatureCompensateSP);
            IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

            bool rc = setTemperatureCompensation((TemperatureCompensateS[0].s == ISS_ON));

            if (!rc)
            {
                TemperatureCompensateSP.s = IPS_ALERT;
                IUResetSwitch(&TemperatureCompensateSP);
                TemperatureCompensateS[last_index].s = ISS_ON;
                IDSetSwitch(&TemperatureCompensateSP, nullptr);
                return false;
            }

            TemperatureCompensateSP.s = IPS_OK;
            IDSetSwitch(&TemperatureCompensateSP, nullptr);
            return true;
        }

        if (strcmp(GotoHomeSP.name, name) == 0)
        {
            bool rc = setGotoHome();
            if (!rc)
            {
                IUResetSwitch(&GotoHomeSP);
                CoilPowerSP.s              = IPS_ALERT;
                IDSetSwitch(&GotoHomeSP, nullptr);
                return false;
            }

            GotoHomeSP.s = IPS_OK;
            IDSetSwitch(&GotoHomeSP, nullptr);
            return true;
        }

        // Coil Power Mode
        if (strcmp(CoilPowerSP.name, name) == 0)
        {
            int current_mode = IUFindOnSwitchIndex(&CoilPowerSP);

            IUUpdateSwitch(&CoilPowerSP, states, names, n);

            int target_mode = IUFindOnSwitchIndex(&CoilPowerSP);

            if (current_mode == target_mode)
            {
                CoilPowerSP.s = IPS_OK;
                IDSetSwitch(&CoilPowerSP, nullptr);
            }

            bool rc = setCoilPowerState(static_cast<CoilPower>(target_mode));
            if (!rc)
            {
                IUResetSwitch(&CoilPowerSP);
                CoilPowerS[current_mode].s = ISS_ON;
                CoilPowerSP.s              = IPS_ALERT;
                IDSetSwitch(&CoilPowerSP, nullptr);
                return false;
            }

            CoilPowerSP.s = IPS_OK;
            IDSetSwitch(&CoilPowerSP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool AstroStep::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Temperature Settings
        if (strcmp(name, TemperatureSettingNP.name) == 0)
        {
            IUUpdateNumber(&TemperatureSettingNP, values, names, n);
            if (!setTemperatureCalibration(TemperatureSettingN[0].value) ||
                    !setTemperatureCoefficient(TemperatureSettingN[1].value))
            {
                TemperatureSettingNP.s = IPS_ALERT;
                IDSetNumber(&TemperatureSettingNP, nullptr);
                return false;
            }

            TemperatureSettingNP.s = IPS_OK;
            IDSetNumber(&TemperatureSettingNP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void AstroStep::GetFocusParams()
{
    if (readPosition())
        IDSetNumber(&FocusAbsPosNP, nullptr);

    if (readTemperature())
        IDSetNumber(&TemperatureNP, nullptr);

    if (readSpeed())
        IDSetNumber(&FocusSpeedNP, nullptr);
    
    if (readCoilPowerState())
        IDSetSwitch(&CoilPowerSP, nullptr);
    
    if (readTemperatureCalibration())
        IDSetNumber(&TemperatureSettingNP, nullptr);

    if (readTemperatureCoefficient())
        IDSetNumber(&TemperatureSettingNP, nullptr);
    
    readReverseDirection();
}

bool AstroStep::SetFocuserSpeed(int speed)
{
    return setSpeed(speed);
}

IPState AstroStep::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    if (speed != static_cast<int>(FocusSpeedN[0].value))
    {
        if (!setSpeed(speed))
            return IPS_ALERT;
    }

    // either go all the way in or all the way out
    // then use timer to stop
    if (dir == FOCUS_INWARD)
        MoveFocuser(0);
    else
        MoveFocuser(static_cast<uint32_t>(FocusMaxPosN[0].value));

    IEAddTimer(duration, &AstroStep::timedMoveHelper, this);
    return IPS_BUSY;
}

void AstroStep::timedMoveHelper(void * context)
{
    static_cast<AstroStep *>(context)->timedMoveCallback();
}

void AstroStep::timedMoveCallback()
{
    AbortFocuser();
    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    FocusTimerNP.s = IPS_IDLE;
    FocusTimerN[0].value = 0;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusTimerNP, nullptr);
}

IPState AstroStep::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    if (!MoveFocuser(targetPos))
        return IPS_ALERT;

    return IPS_BUSY;
}

IPState AstroStep::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // Clamp
    int32_t offset = ((dir == FOCUS_INWARD) ? -1 : 1) * static_cast<int32_t>(ticks);
    int32_t newPosition = FocusAbsPosN[0].value + offset;
    newPosition = std::max(static_cast<int32_t>(FocusAbsPosN[0].min), std::min(static_cast<int32_t>(FocusAbsPosN[0].max),
                           newPosition));

    if (!MoveFocuser(newPosition))
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s       = IPS_BUSY;

    return IPS_BUSY;
}

void AstroStep::TimerHit()
{
    if (!isConnected())
        return;

    bool rc = readPosition();
    if (rc)
    {
        if (fabs(lastPos - FocusAbsPosN[0].value) > 5)
        {
            IDSetNumber(&FocusAbsPosNP, nullptr);
            lastPos = static_cast<uint32_t>(FocusAbsPosN[0].value);
        }
    }

    rc = readTemperature();
    if (rc)
    {
        if (fabs(lastTemperature - TemperatureN[0].value) >= 0.5)
        {
            IDSetNumber(&TemperatureNP, nullptr);
            lastTemperature = static_cast<uint32_t>(TemperatureN[0].value);
        }
    }

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        if (!isMoving())
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastPos = static_cast<uint32_t>(FocusAbsPosN[0].value);
            LOG_INFO("Focuser reached requested position.");
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

bool AstroStep::AbortFocuser()
{
    return sendCommand(":FQ#");
}

bool AstroStep::saveConfigItems(FILE * fp)
{
    Focuser::saveConfigItems(fp);

    return true;
}

bool AstroStep::sendCommand(const char * cmd, char * res, bool silent, int nret)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        if (!silent)
            LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
    {
        tcdrain(PortFD);
        return true;
    }

    // this is to handle the GV command which doesn't return the terminator, use the number of chars expected
    if (nret == 0)
    {
        rc = tty_nread_section(PortFD, res, ML_RES, ML_DEL, ML_TIMEOUT, &nbytes_read);
    }
    else
    {
        rc = tty_read(PortFD, res, nret, ML_TIMEOUT, &nbytes_read);
    }
    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        if (!silent)
            LOGF_ERROR("%s Serial read error: %s.", cmd, errstr);
        return false;
    }

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return true;
}

int AstroStep::msleep( long duration)
{
    struct timespec ts;
    int res;

    if (duration < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = duration / 1000;
    ts.tv_nsec = (duration % 1000) * 1000000;

    do
    {
        res = nanosleep(&ts, &ts);
    }
    while (res && errno == EINTR);

    return res;
}
