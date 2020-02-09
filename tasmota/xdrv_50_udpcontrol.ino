/*
  xdrv_50_udpcontrol.ino - UDP control

  Copyright (C) 2020  Robot9706

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_UDPCONTROL

#define XDRV_50 50

/*********************************************************************************************\
 * UDP
\*********************************************************************************************/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "settings.h"

static WiFiUDP udp;

#define BUFFER_SIZE 255
static uint8_t buffer[BUFFER_SIZE];

#define CMD_DISCOVER 0xEF

#define CMD_POWER 0x67
#define CMD_POWER_NAME 0x68
#define CMD_POWER_BASE 0x50

#define CMD_TIMER 0x80
#define CMD_TIMER_NAME 0x81
#define CMD_TIMER_FLAG_REPEAT       (0b00000001)
#define CMD_TIMER_FLAG_ARM          (0b00000010)
#define CMD_TIMER_FLAG_PWRMODE_MASK (0b00001100)
#define CMD_TIMER_FLAG_PWRMODE_SHIFT (2)
#define CMD_TIMER_FLAG_DEVICE_MASK (0b11110000)
#define CMD_TIMER_FLAG_DEVICE_SHIFT (4)

void UDPInit(void)
{
    udp.begin(5000);
}

static void UDPSend(int numBytes)
{
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(buffer, numBytes);
    udp.endPacket();
}

static void UDPAnswer(int messageOK, uint8_t command, bool sendName)
{
    int bufferLen = 3;

    buffer[0] = 0x2;
    buffer[1] = (messageOK ? 0xAB : 0xFF);
    buffer[2] = command;

    if (sendName) 
    {
        const char* name = SettingsText(SET_FRIENDLYNAME1);
        int nameLen = strlen(name);

        buffer[0] = (uint8_t)(2 + 1 + nameLen);
        buffer[3] = (uint8_t)nameLen;
        memcpy(&buffer[4], name, nameLen);

        bufferLen = 4 + nameLen;
    }

    UDPSend(bufferLen);
}

void UDPLoop(void)
{
    int packetSize = udp.parsePacket();
    if (packetSize)
    {
        int len = udp.read(buffer, BUFFER_SIZE);
        if (len > BUFFER_SIZE) {
            return;
        }

        uint8_t dataLength = buffer[0];
        if (dataLength == 0) {
            return;
        }

        uint8_t command = buffer[1];
        switch (command)
        {
            case CMD_DISCOVER:
                UDPCMD_Discover();
            break;
            case CMD_POWER:
            {
                if (dataLength >= 2) 
                {
                    UDPCMD_Power(buffer[2], false);   
                }
            }
            break;
            case CMD_POWER_NAME:
            {
                if (dataLength >= 3) 
                {
                    UDPCMD_Power_WithName(buffer[2], buffer[3], &buffer[4]);
                }
            }
            break;
            case CMD_TIMER:
            {
                if (dataLength >= 5)
                {
                    uint16_t mins = ((uint16_t)buffer[3] | ((uint16_t)buffer[4]) << 8);
                    UDPCMD_Timer(buffer[2], mins, buffer[5], false);
                }
            }
            break;
            case CMD_TIMER_NAME:
            {
                 if (dataLength >= 7)
                {
                    uint16_t mins = ((uint16_t)buffer[3] | ((uint16_t)buffer[4]) << 8);
                    UDPCMD_Timer_WithName(buffer[2], mins, buffer[5], buffer[6], &buffer[7]);
                }
            }
            break;
        }
    }
}

static bool CheckName(uint8_t targetNameLen, uint8_t* targetName)
{
    const char* name = SettingsText(SET_FRIENDLYNAME1);
    int nameLen = strlen(name);

    if (nameLen != targetNameLen) 
    {
        return false;
    }

    for (int checkIndex = 0; checkIndex < nameLen; checkIndex++) 
    {
        if (name[checkIndex] != targetName[checkIndex])
        {
            return false;
        }
    }

    return true;
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

static void UDPCMD_Discover() 
{
    IPAddress ownIP = WiFi.localIP();

    const char* name = SettingsText(SET_FRIENDLYNAME1);
    int nameLen = strlen(name);

    int packetLen = 1 + 1 + (1 + 4 + 1 + nameLen);
    if (packetLen > 64) 
    {
        return;
    }
    else 
    {
        buffer[0] = packetLen - 1;
        buffer[1] = 0xAB;
        buffer[2] = CMD_DISCOVER;

        for (int ipIndex = 0; ipIndex < 4; ipIndex++) {
            buffer[3 + ipIndex] = ownIP[ipIndex];
        }

        buffer[7] = nameLen;

        memcpy(&buffer[8], name, nameLen);

        UDPSend(packetLen);
    }
}

static void UDPCMD_Power(uint8_t mode, bool name) 
{
    uint8_t state = mode - (uint8_t)CMD_POWER_BASE;
    if (state > POWER_TOGGLE) 
    {
        UDPAnswer(false, name ? CMD_POWER_NAME : CMD_POWER, name);
        return;
    }

    ExecuteCommandPower(1, state, SRC_IGNORE);
    UDPAnswer(true, name ? CMD_POWER_NAME : CMD_POWER, name);
}

static void UDPCMD_Power_WithName(uint8_t mode, uint8_t targetNameLen, uint8_t* targetName)
{
    if (!CheckName(targetNameLen, targetName))
    {
        return;
    }

    UDPCMD_Power(mode, true);
}

static void UDPCMD_Timer(uint8_t index, uint16_t mins, uint8_t flags, bool name)
{
    if (index >= MAX_TIMERS) 
    {
        UDPAnswer(false, name ? CMD_TIMER_NAME : CMD_TIMER, name);
        return;
    }

    Timer *t = &Settings.timer[index];
    t->time = mins;
    t->window = 0;
    t->repeat = ((flags & CMD_TIMER_FLAG_REPEAT) == CMD_TIMER_FLAG_REPEAT);
    t->days = 0b1111111;
    t->device = ((flags & CMD_TIMER_FLAG_DEVICE_MASK) >> CMD_TIMER_FLAG_DEVICE_SHIFT);
    t->power = ((flags & CMD_TIMER_FLAG_PWRMODE_MASK) >> CMD_TIMER_FLAG_PWRMODE_SHIFT);
    t->mode = 0;
    t->arm = ((flags & CMD_TIMER_FLAG_ARM) == CMD_TIMER_FLAG_ARM);

    UDPAnswer(true, name ? CMD_TIMER_NAME : CMD_TIMER, name);
}

static void UDPCMD_Timer_WithName(uint8_t index, uint16_t mins, uint8_t flags, uint8_t targetNameLen, uint8_t* targetName)
{
    if (!CheckName(targetNameLen, targetName))
    {
        return;
    }

    UDPCMD_Timer(index, mins, flags, true);
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv50(uint8_t function)
{
    bool result = false;

    switch (function)
    {
    case FUNC_PRE_INIT:
        UDPInit();
        break;
    case FUNC_LOOP:
        UDPLoop();
        break;
    }

    return result;
}

#endif // USE_UDPCONTROL
