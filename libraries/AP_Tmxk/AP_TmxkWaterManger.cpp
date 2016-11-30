/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   Please contribute your ideas! See http://dev.ardupilot.com for details

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
/*
  SerialManager allows defining the protocol and baud rates for the available
  serial ports and provides helper functions so objects (like a gimbal) can
  find which serial port they should use
 */

#include "AP_TmxkWaterManger.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void AP_TmxkWaterManager::init()
{
    //hal.rcout->enable_ch(5);
}

void AP_TmxkWaterManager::update()
{
	if (hal.uartE->available()) {
		while(hal.uartE->available()) {
			int c = hal.uartE->read();
			//hal.uartE->println();
			//hal.console->printf("test:%c\n", hal.uartE->read());
			switch(c) {
				case '#':
					m_start = true;
					for (int i=0;i<DATA_LEN;i++) {
						m_data[i] = -1.0;
					}
					break;
				case ';':
					m_data[m_dataCount] = atof(m_dataBuf);
					m_dataCount ++;
					m_dataBuf[0] = '\0';

					break;

				case '*':

					m_data[m_dataCount] = atoi(m_dataBuf);
					if (m_dataCount + 1 == DATA_LEN) {
					    m_waterUsed = m_data[0];
						m_waterStatus = m_data[1];
						hal.console->printf("angle=%d, range=%d\n",m_data[0], m_data[1]);
					}

					m_start = false;
					m_dataCount = 0;
					m_dataBuf[0] = '\0';

					break;
				case '\r':
				case '\n':
					break;

				default:
					if (m_start) {
						char s[2];//为什么要2个单位的长度，如果一个一个获取，一个长度的string还不够吗？？？可能与发送的数据类型有关
						sprintf(s, "%c", c);
						strcat(m_dataBuf, s);
					}
					break;
			}
		}
	}
//	hal.console->print(hal.uartE->read());
//	hal.console->printf("1=%d, 2=%d\n", m_waterUsed, m_waterStatus);


}


int16_t AP_TmxkWaterManager::getWaterUsed()
{
	return m_waterUsed;
}

int16_t AP_TmxkWaterManager::getWaterStatus()
{
	return (int) m_waterStatus;
}


