#include "default_rosjam_config.h"
#include "rosjam.h"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "tusb_config.h"
#include "tusb.h"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>

double string_to_float(char* string){
	return atof(string);
}

int float_to_string(double number, int precision, char* buf, int buf_len){
    int len = snprintf(NULL, 0, "%.*lf", precision, number);
	if (len +1 > buf_len){
		return -1;
	}
    snprintf(buf, len + 1, "%.*lf", precision, number);
	return len+1;
}

int int_to_string(int number, char* buf, int buf_len){
    int len = snprintf(NULL, 0, "%d", number);
	if (len +1 > buf_len){
		return -1;
	}
    snprintf(buf, len + 1, "%d", number);
	return len+1;
}

void setup_simple(){
	tusb_rhport_init_t dev_init = {
    	.role = TUSB_ROLE_DEVICE,
    	.speed = TUSB_SPEED_AUTO
  	};
  	tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

void print_to_usb(char* message){
	send_msg_raw(message,strlen(message));
}

void send_msg_raw(char *message, int message_len){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		tud_cdc_n_write(USB_CDC_ITF, message, message_len);
		tud_cdc_n_write_flush(USB_CDC_ITF);
	}
}

int has_data(){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		int available = tud_cdc_n_available(USB_CDC_ITF);
		return available > 0;
	}
	return 0;
}

char read_char(){
	if (has_data()){
		return tud_cdc_n_read_char(USB_CDC_ITF);
	}
	return -1;
}


void process_simple(){
	tud_cdc_n_write_flush(USB_CDC_ITF); //make sure small messages get immediately sent
	tud_task();
}