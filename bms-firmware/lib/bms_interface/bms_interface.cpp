#include <Arduino.h>
#include <Wire.h>

#include "Registers.h"
#include "Setpoints.h"
#include "bms_interface.h"


namespace bms_interface {
    struct Error_Timestamps {
        unsigned long XReady = 0;
        unsigned long ovrd_alert = 0;
        unsigned long SCD = 0;
        unsigned long OCD = 0;
        int short_circuit_counter = 0;
        int overcurrent_counter = 0;
    };

	const int OCD_THRESHOLD_SETTING_MV[16] =
		{ 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100 };

    const int SCD_THRESHOLD_SETTING_MV [8] = 
        { 44, 67, 89, 111, 133, 155, 178, 200 };

    Error_Timestamps error_timestamps_bms1;
    Error_Timestamps error_timestamps_bms2;
        
    void _write_register(byte i2c_addr, byte reg_addr, int reg_data);
    int _read_register(byte i2c_addr, byte reg_addr);
    int _adc_offset(byte i2c_addr);
    int _adc_gain(byte i2c_gain);
    int shunt_resistor_value_mOhm = 2;


    void enable_adc(byte i2c_addr) {
        _write_register(i2c_addr, SYS_CTRL1, 0x10);
    }

    /**
     * DIES RHAT DUBNWUUS
     */
    void enable_discharge_mosfet(byte i2c_addr) {
        byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
        _write_register(i2c_addr, SYS_CTRL2, sys_ctrl2_val | 0x02);
        Serial.println("Discharge MOSFET enabled");
    }

    void disable_discharge_mosfet(byte i2c_addr) {
        byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
        _write_register(i2c_addr, SYS_CTRL2, sys_ctrl2_val & 0xFD);
        Serial.println("Discharge MOSFET disabled");
    }

    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv) {
        int adc_offset = _adc_offset(i2c_addr);
        int adc_gain = _adc_gain(i2c_addr);

        byte overvoltage_trip = 
            (((voltage_mv - adc_offset) * 1000 / adc_gain) >> 4) & 0x00FF;
        
        _write_register(i2c_addr, OV_TRIP, overvoltage_trip);
    }

    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv) {
        int adc_offset = _adc_offset(i2c_addr);
        int adc_gain = _adc_gain(i2c_addr);

        byte undervoltage_trip = 
            (((voltage_mv - adc_offset) * 1000 / adc_gain) >> 4) & 0x00FF;
        undervoltage_trip += 1;

        _write_register(i2c_addr, UV_TRIP, undervoltage_trip);
    }

    void set_overcurrent_discharge_protection(byte i2c_addr, int current_ma) {
        byte ocd_protection = 0;
        int ocd_array_len = sizeof(OCD_THRESHOLD_SETTING_MV)
							/ sizeof(OCD_THRESHOLD_SETTING_MV[0]);
		for (int i = ocd_array_len -1; i >= 0; i--) {
			if (current_ma * shunt_resistor_value_mOhm/1000 >= OCD_THRESHOLD_SETTING_MV[i]){
                ocd_protection = i;
                break;
            }
		}   
        _write_register(i2c_addr, PROTECT2, ocd_protection);
    }

    void set_short_circuit_protection(byte i2c_addr, int current_ma) {
        byte scd_protection = 0;
		int scd_array_len = sizeof(SCD_THRESHOLD_SETTING_MV)
							/ sizeof(SCD_THRESHOLD_SETTING_MV[0]);
        for (int i = scd_array_len -1; i >= 0; i--) {
			if (current_ma * shunt_resistor_value_mOhm/1000 >= SCD_THRESHOLD_SETTING_MV[i]){
                scd_protection = i;
                break;
            }
		}   
        _write_register(i2c_addr, PROTECT1, scd_protection);
    }

    void read_cell_voltages(byte i2c_addr, int *cell_voltages, int array_len, int max_and_min_cell_ids[2]) {
        Serial.println("Updating Voltages...");
        long adc_val = 0;

        int adc_offset = _adc_offset(i2c_addr);
        int adc_gain = _adc_gain(i2c_addr);
        Serial.println(adc_offset);
        Serial.println(adc_gain);

        int connected_cells = 0;
        max_and_min_cell_ids[0] = 0; //max id
        max_and_min_cell_ids[1] = -1; // min id, initialized to an invalid index to start
        
        for (int i = 0; i < array_len; i++) {
            byte hi_byte = _read_register(i2c_addr, VC1_HI_BYTE + (i * 2));
            byte lo_byte = _read_register(i2c_addr, VC1_LO_BYTE + (i * 2));

            adc_val = ((hi_byte & 0b00111111) << 8) | lo_byte;  // Mask first 2 bits, shift left, append low byte
            cell_voltages[i] = (adc_val * adc_gain / 1000) + adc_offset;  // Convert to mV
            if (cell_voltages[i] > 500) {  // Only count valid cells
                connected_cells++;
    
                if (cell_voltages[i] > cell_voltages[max_and_min_cell_ids[0]]) {
                    max_and_min_cell_ids[0] = i;
                }
                if (max_and_min_cell_ids[1] == -1 || cell_voltages[i] < cell_voltages[max_and_min_cell_ids[1]]) {
                    max_and_min_cell_ids[1] = i;
                }
            }
        }
    
        long adc_val_pack = (( _read_register(i2c_addr, BAT_HI_BYTE) << 8) 
                            | _read_register(i2c_addr, BAT_LO_BYTE)) & 0b1111111111111111;
    
        int bat_voltage = (4 * adc_gain * adc_val_pack / 1000) + (connected_cells * adc_offset);
        
        Serial.println("Voltages updated");
    }

    void update_current(byte i2c_addr, byte sys_stat_val){
        if (sys_stat_val & 0x80){ //CC_Ready
            Serial.println("CC Ready");
            int16_t adc_val = (_read_register(i2c_addr, CC_HI_BYTE) << 8) | _read_register(i2c_addr, CC_LO_BYTE); //reads from CC_HI (0x32) and CC_LO (0x33)
            long bat_current = adc_val * 8.44/shunt_resistor_value_mOhm;
            _write_register(i2c_addr, sys_stat_val, 0x80); //Clears CC_Ready flag
        }
    }

    void check_and_clear_XReady(byte i2c_addr, byte sys_stat_val, Error_Timestamps& error_timestamps){
        if (sys_stat_val & 0x20){
            Serial.print("XReady");
            if (error_timestamps.XReady == 0){
                error_timestamps.XReady = millis(); //saves timestamp that the XREADY error occured at
            } else if ((millis() - error_timestamps.XReady) >= 3000) { //after 3 seconds pass (datasheet recommendation to wait a few seconds to clear)
                Serial.println("Clearing XReady Error");
                _write_register(i2c_addr, SYS_STAT, 0x20); // writes a 1 to clear it. not sure yet if this is a critical error where i should disable the mosfet
                error_timestamps.XReady = 0; //resets timestamp once cleared
            } 
        }
    }

    void check_and_clear_alert(byte i2c_addr, byte sys_stat_val, Error_Timestamps& error_timestamps){
        if (sys_stat_val & 0x10){
            Serial.println("Alert");
            if (error_timestamps.ovrd_alert == 0){
                error_timestamps.ovrd_alert = millis();
            } else if ((millis() - error_timestamps.ovrd_alert) >= 10000) { //Wait 10 seconds
                _write_register(i2c_addr, SYS_STAT, 0x10);
                error_timestamps.ovrd_alert = 0;
            }
        }
    }

    void check_and_clear_UV(byte i2c_addr, byte sys_stat_val, int *cell_voltages, int max_and_min_cell_ids[2], Error_Timestamps& error_timestamps){
        if (sys_stat_val & 0x08) {
            Serial.println("UV");
            int id_cell_min_voltage = max_and_min_cell_ids[1];
            if (cell_voltages[id_cell_min_voltage] > UV_THRESH_MV){
                Serial.println("Clearing UV Error");
                _write_register(i2c_addr, SYS_STAT, 0x08);
            }
        }
    }
    
    void check_and_clear_OV(byte i2c_addr, byte sys_stat_val, int *cell_voltages, int max_and_min_cell_ids[2], Error_Timestamps& error_timestamps){
        if (sys_stat_val & 0x04) {
            Serial.println("OV");
            int id_cell_max_voltage = max_and_min_cell_ids[0];
            if (cell_voltages[id_cell_max_voltage] < OV_THRESH_MV){
                Serial.println("Clearing OV Error");
                _write_register(i2c_addr, SYS_STAT, 0x04);
            }
        }
    }

    void check_and_clear_SCD(byte i2c_addr, byte sys_stat_val, Error_Timestamps& error_timestamps, byte i2c_addr2){
        if (sys_stat_val & 0x02){
            Serial.println("SCD");
            if (error_timestamps.SCD == 0){
                error_timestamps.SCD = millis();
            } else if ((millis() - error_timestamps.SCD) >= 30000){ //30 seconds waiting
                Serial.println("Clearing SCD Error: " + (error_timestamps.short_circuit_counter + 1));
                _write_register(i2c_addr, SYS_STAT, 0x02);
                error_timestamps.short_circuit_counter++; //increment the counter
                error_timestamps.SCD = 0; //reset timestamp
            }

            if (error_timestamps.short_circuit_counter == 3) { 
                disable_discharge_mosfet(i2c_addr);
                Serial.println("SERIOUS ERROR: SHORT CIRCUITED 3 TIMES");
                shutdown(i2c_addr); // shuts down the BMS and has the mosfet disabled
                shutdown(i2c_addr2); //shuts down the other BMS too
            }
        }
    }
     
    void check_and_clear_OCD(byte i2c_addr, byte sys_stat_val, Error_Timestamps& error_timestamps, byte i2c_addr2){
        if (sys_stat_val & 0x01){
            Serial.println("OCD");
            if (error_timestamps.OCD == 0){
                error_timestamps.OCD = millis();
            } else if ((millis() - error_timestamps.OCD) >= 10000){ //10 seconds waiting
                Serial.println("Clearing OCD Error: " + (error_timestamps.overcurrent_counter + 1));
                _write_register(i2c_addr, SYS_STAT, 0x01);
                error_timestamps.overcurrent_counter++; //increment the counter
                error_timestamps.OCD = 0; //reset timestamp
            }
            if (error_timestamps.overcurrent_counter == 7) { 
                disable_discharge_mosfet(i2c_addr);
                Serial.println("SERIOUS ERROR: OVERCURRENT 7 TIMES");
                shutdown(i2c_addr); // shuts down the BMS and has the mosfet disabled
                shutdown(i2c_addr2); //shuts down the other BMS too
            }
        }
    }

    void error_check_and_clear(byte i2c_addr1, int *cell_voltages1, int array_len, int max_and_min_cell_ids1[2], byte i2c_addr2, int *cell_voltages2, int max_and_min_cell_ids2[2]){
        byte sys_stat_val_bat1 = _read_register(i2c_addr1, SYS_STAT);
        byte sys_stat_val_bat2 = _read_register(i2c_addr2, SYS_STAT);

        update_current(i2c_addr1, sys_stat_val_bat1);
        update_current(i2c_addr2, sys_stat_val_bat2); //reads the current, we dont do anything with it yet though

        if ((!(sys_stat_val_bat1 & 0x3F)) && (!(sys_stat_val_bat2 & 0x3F))) { //checking bits 0-5 (00111111) for errors in both bms chips
            Serial.println("No Error");
            memset(&error_timestamps_bms1, 0, sizeof(Error_Timestamps));
            memset(&error_timestamps_bms2, 0, sizeof(Error_Timestamps)); //resets error timestamps for bms 1 and 2
            byte sys_ctrl2_val_bms1 = _read_register(i2c_addr1, SYS_CTRL2);
            byte sys_ctrl2_val_bms2 = _read_register(i2c_addr2, SYS_CTRL2);
            
            if (!(sys_ctrl2_val_bms1 & 0x02)){ //if either are not discharging but there are no errors, make them discharge
                enable_discharge_mosfet(i2c_addr1);
            } 
            if (!(sys_ctrl2_val_bms2 & 0x02)){
                enable_discharge_mosfet(i2c_addr2);
            }
        } else {
            Serial.println("Error in atleast one battery: Shutting off both batteries");
            disable_discharge_mosfet(i2c_addr1);
            disable_discharge_mosfet(i2c_addr2); //makes sure that either both batteries are off in case of error

            read_cell_voltages(i2c_addr1, cell_voltages1, array_len, max_and_min_cell_ids1);
            read_cell_voltages(i2c_addr2, cell_voltages2, array_len, max_and_min_cell_ids2); //read cell voltages to check UV and OV

            //checks and clears errors in bms 1
            check_and_clear_XReady(i2c_addr1, sys_stat_val_bat1, error_timestamps_bms1);
            check_and_clear_alert(i2c_addr1, sys_stat_val_bat1, error_timestamps_bms1); // should never have to be used
            check_and_clear_UV(i2c_addr1, sys_stat_val_bat1, cell_voltages1, max_and_min_cell_ids1, error_timestamps_bms1);
            check_and_clear_OV(i2c_addr1, sys_stat_val_bat1, cell_voltages1, max_and_min_cell_ids1, error_timestamps_bms1);
            check_and_clear_SCD(i2c_addr1, sys_stat_val_bat1, error_timestamps_bms1, i2c_addr2);
            check_and_clear_OCD(i2c_addr1, sys_stat_val_bat1, error_timestamps_bms1, i2c_addr2);

            //checks and clears errors in bms 2
            check_and_clear_XReady(i2c_addr2, sys_stat_val_bat2, error_timestamps_bms2);
            check_and_clear_alert(i2c_addr2, sys_stat_val_bat2, error_timestamps_bms2); // should never have to be used
            check_and_clear_UV(i2c_addr2, sys_stat_val_bat2, cell_voltages2, max_and_min_cell_ids2, error_timestamps_bms2);
            check_and_clear_OV(i2c_addr2, sys_stat_val_bat2, cell_voltages2, max_and_min_cell_ids2, error_timestamps_bms2);
            check_and_clear_SCD(i2c_addr2, sys_stat_val_bat2, error_timestamps_bms2, i2c_addr1);
            check_and_clear_OCD(i2c_addr2, sys_stat_val_bat2, error_timestamps_bms2, i2c_addr1);
        }
    }

    void error_check_and_clear_singular(byte i2c_addr, int *cell_voltages, int array_len, int max_and_min_cell_ids[2], byte i2c_addr2){
        byte sys_stat_val = _read_register(i2c_addr, SYS_STAT);
        update_current(i2c_addr, sys_stat_val);
        if(!(sys_stat_val & 0x3F)){
            memset(&error_timestamps_bms1, 0, sizeof(Error_Timestamps));
            int voltage_status = 1; //assume its normally true
            for (int i = 0; i < array_len; i++){
                if(cell_voltages[i]>= 500 && (cell_voltages[i] < UV_THRESH_MV || cell_voltages[i] > OV_THRESH_MV)){
                    voltage_status = 0; //over or under voltage, turn off
                    Serial.println("Voltage not okay though");
                    break;
                }
            }
            if (voltage_status == 1){ //if voltage is okay
                byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
                if (!(sys_ctrl2_val & 0x02)){
                    enable_discharge_mosfet(i2c_addr);
                }    
            }
        } else { //if the sys_stat byte has errors
            Serial.println("Error in the battery. Shutting off and Error checking");
            disable_discharge_mosfet(i2c_addr);
            read_cell_voltages(i2c_addr, cell_voltages, array_len, max_and_min_cell_ids);

            check_and_clear_XReady(i2c_addr, sys_stat_val, error_timestamps_bms1);
            check_and_clear_alert(i2c_addr, sys_stat_val, error_timestamps_bms1); // should never have to be used
            check_and_clear_UV(i2c_addr, sys_stat_val, cell_voltages, max_and_min_cell_ids, error_timestamps_bms1);
            check_and_clear_OV(i2c_addr, sys_stat_val, cell_voltages, max_and_min_cell_ids, error_timestamps_bms1);
            check_and_clear_SCD(i2c_addr, sys_stat_val, error_timestamps_bms1, i2c_addr2);
            check_and_clear_OCD(i2c_addr, sys_stat_val, error_timestamps_bms1, i2c_addr2);

        }
    }



    void shutdown(byte i2c_addr){
        byte sys_ctrl1_value = _read_register(i2c_addr, SYS_CTRL1);
        
        sys_ctrl1_value = sys_ctrl1_value & 0xFC; // keeps upper 6 bits and sets [SHUT_A] = 0, [SHUT_B] = 0;
        _write_register(i2c_addr, SYS_CTRL1, sys_ctrl1_value); 

        sys_ctrl1_value = sys_ctrl1_value & 0xFD; // sets [SHUT_A] = 0, [SHUT_B] = 1;
        _write_register(i2c_addr, SYS_CTRL1, sys_ctrl1_value);

        sys_ctrl1_value = sys_ctrl1_value & 0xFE; // sets [SHUT_A] = 1, [SHUT_B] = 0;
        _write_register(i2c_addr, SYS_CTRL1, sys_ctrl1_value);

        Serial.println("BMS Shutdown");
    }


    void _write_register(byte i2c_addr, byte reg_addr, int reg_data) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg_addr);
        Wire.write(reg_data);
        Wire.endTransmission();  
    }

    int _read_register(byte i2c_addr, byte reg_addr) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg_addr);
        Wire.endTransmission();
        Wire.requestFrom(i2c_addr, (uint8_t)1); //maybe supposed to be reg_addr instead

        if (Wire.available()) {  
            return Wire.read();  
        } else {
            return -1;  
        }
    }

    int _adc_offset(byte i2c_addr) {
        int adc_offset = _read_register(i2c_addr, ADCOFFSET);
        return adc_offset;
    }

    int _adc_gain(byte i2c_addr) {
        int adc_gain1 = _read_register(i2c_addr, ADCGAIN1);
        int adc_gain2 = _read_register(i2c_addr, ADCGAIN2);
        
        int total_adc = 365 + (((adc_gain1 & 0x0C) << 1) | ((adc_gain2 & 0xE0) >> 5));
            
        return total_adc;
    }

    int mosfet_status(byte i2c_addr){ //just to make sure the mosfet is enabled when its supposed to and disables when its supposed to, debugging purposes.
        byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
        return (sys_ctrl2_val >> 1) & 0x01; //Shift right 1 bit and mask with 0x01 (if 1, its on, if zero its off)
    }

}
