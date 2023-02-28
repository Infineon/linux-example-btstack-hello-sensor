/******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
******************************************************************************/
/******************************************************************************
 * File Name: hello_sensor_gatt_cfg.h
 *
 * Description: Hello Sensor GATT DB CFG.
 *
*/

#ifndef _HELLO_SENSOR_GATT_CFG_H_
#define _HELLO_SENSOR_GATT_CFG_H_

extern const uint8_t hello_sensor_gatt_database[];
extern uint16_t hello_sensor_gatt_database_size;

extern attribute_t gauAttributes[];
extern uint16_t gauAttributes_size;

extern host_info_t hello_sensor_hostinfo;
extern hello_sensor_state_t hello_sensor_state;

extern uint8_t hello_sensor_device_name[]          ;  /* GAP Service characteristic Device Name */
extern uint8_t hello_sensor_appearance_name[2]     ;
extern char    hello_sensor_char_notify_value[]    ; /* Notification Name */ 
extern char    hello_sensor_char_mfr_name_value[]  ;
extern char    hello_sensor_char_model_num_value[] ;
extern uint8_t hello_sensor_char_system_id_value[] ;
extern wiced_bt_device_address_t bt_device_address ;

extern uint8_t hello_sensor_char_notify_value_len;

#endif
