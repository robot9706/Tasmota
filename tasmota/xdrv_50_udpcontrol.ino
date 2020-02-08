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

static WiFiUDP udp;

#define BUFFER_SIZE 255
static uint8_t buffer[BUFFER_SIZE];

#define CMD_DISCOVER 0xEF

#define CMD_POWER 0x67
#define CMD_POWER_NAME 0x68
#define CMD_POWER_BASE 0x50

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

static void UDPSendSimple(int messageOK)
{
    buffer[0] = 0x1;
    buffer[1] = (messageOK ? 0xAB : 0xFF);

    UDPSend(2);
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
                    UDPCMD_Power(buffer[2]);   
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
        }
    }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

static void UDPCMD_Discover() 
{
    IPAddress ownIP = WiFi.localIP();

    const char* name = SettingsText(SET_FRIENDLYNAME1);
    int nameLen = strlen(name);

    int packetLen = 1 + 1 + 4 + nameLen;
    if (packetLen > 64) 
    {
        UDPSendSimple(false);
    }
    else 
    {
        buffer[0] = packetLen - 1;

        for (int ipIndex = 0; ipIndex < 4; ipIndex++) {
            buffer[1 + ipIndex] = ownIP[ipIndex];
        }

        buffer[5] = nameLen;

        memcpy(&buffer[6], name, nameLen);

        UDPSend(packetLen);
    }
}

static void UDPCMD_Power(uint8_t mode) 
{
    uint8_t state = mode - (uint8_t)CMD_POWER_BASE;
    if (state > POWER_TOGGLE) 
    {
        UDPSendSimple(false);
        return;
    }

    ExecuteCommandPower(1, state, SRC_IGNORE);
    UDPSendSimple(true);
}

static void UDPCMD_Power_WithName(uint8_t mode, uint8_t targetNameLen, uint8_t* targetName)
{
    uint8_t state = mode - (uint8_t)CMD_POWER_BASE;
    if (state > POWER_TOGGLE) 
    {
        UDPSendSimple(false);
        return;
    }

    const char* name = SettingsText(SET_FRIENDLYNAME1);
    int nameLen = strlen(name);

    if (nameLen != targetNameLen) 
    {
        UDPSendSimple(false);
        return;
    }

    for (int checkIndex = 0; checkIndex < nameLen; checkIndex++) 
    {
        if (name[checkIndex] != targetName[checkIndex])
        {
            UDPSendSimple(false);
            return;
        }
    }

    ExecuteCommandPower(1, state, SRC_IGNORE);
    UDPSendSimple(true);
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
