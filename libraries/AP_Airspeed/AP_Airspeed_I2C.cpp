/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
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
  backend driver for airspeed from a I2C MS4525D0 sensor
 */
#include "AP_Airspeed_I2C.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define I2C_ADDRESS_MS4525DO 0x28

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define I2C_BUS_MS4525DO HAL_BARO_MS4515DO_I2C_BUS
#else
#define I2C_BUS_MS4525DO 0
#endif

AP_Airspeed_I2C::AP_Airspeed_I2C(const AP_Float &psi_range) :
    _psi_range(psi_range)
{
}

// probe and initialise the sensor
bool AP_Airspeed_I2C::init()
{
    _dev = std::move(hal.i2c_mgr->get_device(I2C_BUS_MS4525DO, I2C_ADDRESS_MS4525DO));
    if (!_dev) {
        return false;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* sem = _dev->get_semaphore();

    // take i2c bus sempahore
    if (!sem->take(200)) {
        return false;
    }

    _measure();
    hal.scheduler->delay(10);
    _collect();
    sem->give();
    if (_last_sample_time_ms != 0) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Airspeed_I2C::_timer, void));
        return true;
    }
    return false;
}

// start a measurement
void AP_Airspeed_I2C::_measure()
{
    _measurement_started_ms = 0;
    _dev->write_register(0, 0);
    _measurement_started_ms = AP_HAL::millis();
}

// read the values from the sensor
void AP_Airspeed_I2C::_collect()
{
    uint8_t data[4];

    _measurement_started_ms = 0;

    if (!_dev->read_registers(0, data, sizeof(data))) {
        printf("i2c read failed\n");
        return;
    }

    uint8_t status = data[0] & 0xC0;
    if (status == 2 || status == 3) {
        return;
    }

    int16_t dp_raw, dT_raw;
    dp_raw = (data[0] << 8) + data[1];
    dp_raw = 0x3FFF & dp_raw;
    dT_raw = (data[2] << 8) + data[3];
    dT_raw = (0xFFE0 & dT_raw) >> 5;

    const float P_max = _psi_range.get();
    const float P_min = - P_max;
    const float PSI_to_Pa = 6894.757f;
    /*
      this equation is an inversion of the equation in the
      pressure transfer function figure on page 4 of the datasheet

      We negate the result so that positive differential pressures
      are generated when the bottom port is used as the static
      port on the pitot and top port is used as the dynamic port
     */
    float diff_press_PSI = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);

    _pressure = diff_press_PSI * PSI_to_Pa;
    _temperature = ((200.0f * dT_raw) / 2047) - 50;

    _last_sample_time_ms = AP_HAL::millis();
}

// 1kHz timer
void AP_Airspeed_I2C::_timer()
{
    AP_HAL::Semaphore* sem = _dev->get_semaphore();

    if (!sem->take_nonblocking()) {
        return;
    }

    if (_measurement_started_ms == 0) {
        _measure();
        sem->give();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
    sem->give();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_I2C::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_I2C::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    temperature = _temperature;
    return true;
}
