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

#pragma once

#include "indifocuser.h"

#include <time.h>

#include <chrono>

class AstroStep : public INDI::Focuser
{
    public:
        AstroStep();
        virtual ~AstroStep() override = default;

        typedef enum {COIL_POWER_OFF, COIL_POWER_ON } CoilPower;

        const char * getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n) override;
        virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;

        static void timedMoveHelper(void * context);

    protected:
        /**
         * @brief Handshake Try to communicate with Focuser and see if there is a valid response.
         * @return True if communication is successful, false otherwise.
         */
        virtual bool Handshake() override;

        /**
         * @brief MoveFocuser Move focuser in a specific direction and speed for period of time.
         * @param dir Direction of motion. Inward or Outward
         * @param speed Speed of motion
         * @param duration Timeout in milliseconds.
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;

        /**
         * @brief MoveAbsFocuser Move to an absolute target position
         * @param targetTicks target position
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;

        /**
         * @brief MoveRelFocuser Move focuser for a relative amount of ticks in a specific direction
         * @param dir Direction of motion
         * @param ticks steps to move
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;

        /**
         * @brief SyncFocuser Set the supplied position as the current focuser position
         * @param ticks target position
         * @return IPS_OK if focuser position is now set to ticks. IPS_ALERT for problems.
         */
        virtual bool SyncFocuser(uint32_t ticks) override;

        virtual bool SetFocuserSpeed(int speed) override;
        virtual bool ReverseFocuser(bool enabled) override;
        virtual bool AbortFocuser() override;
        virtual void TimerHit() override;
        virtual bool saveConfigItems(FILE * fp) override;

    private:
        bool Ack();
        /**
         * @brief sendCommand Send a string command to AstroStep.
         * @param cmd Command to be sent, must already have the necessary delimeter ('#')
         * @param res If not nullptr, the function will read until it detects the default delimeter ('#') up to ML_RES length.
         *        if nullptr, no read back is done and the function returns true.
         * @param silent if true, do not print any error messages.
         * @param nret if > 0 read nret chars, otherwise read to the terminator
         * @return True if successful, false otherwise.
         */
        bool sendCommand(const char * cmd, char * res = nullptr, bool silent = false, int nret = 0);

        // Get initial focuser parameter when we first connect
        void GetFocusParams();
        // Read and update Temperature
        bool readTemperature();
        // Read and update Position
        bool readPosition();
        // Read and update speed
        bool readSpeed();
        // Read version
        bool readVersion();
        // Are we moving?
        bool isMoving();
        // Read coil power state
        bool readCoilPowerState();
        // Read reverse
        bool readReverseDirection();
        // Read temperature coefficient
        bool readTemperatureCoefficient();
        // Read temperature calibation
        bool readTemperatureCalibration();

        bool MoveFocuser(uint32_t position);
        bool setSpeed(uint32_t speed);
        bool setTemperatureCalibration(uint32_t calibration);
        bool setTemperatureCoefficient(uint32_t coefficient);
        bool setTemperatureCompensation(bool enable);
        void timedMoveCallback();
        bool setGotoHome();
        bool setCoilPowerState(CoilPower enable);

        uint32_t targetPos { 0 }, lastPos { 0 }, lastTemperature { 0 };

        // Read Only Temperature Reporting
        INumber TemperatureN[1];
        INumberVectorProperty TemperatureNP;

        // Temperature Settings
        INumber TemperatureSettingN[2];
        INumberVectorProperty TemperatureSettingNP;

        // Temperature Compensation Enable/Disable
        ISwitch TemperatureCompensateS[2];
        ISwitchVectorProperty TemperatureCompensateSP;

        //Goto Home Position
        ISwitch GotoHomeS[1];
        ISwitchVectorProperty GotoHomeSP;

        //CoilPower On Off
        ISwitch CoilPowerS[2];
        ISwitchVectorProperty CoilPowerSP;

        // AstroStep Buffer
        static const uint8_t ML_RES { 32 };
        // AstroStep Delimeter
        static const char ML_DEL { '#' };
        // AstroStep Timeout
        static const uint8_t ML_TIMEOUT { 3 };

        int msleep(long milliseconds);
};