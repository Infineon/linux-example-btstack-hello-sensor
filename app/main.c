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
 * File Name: main.c
 *
 * Description: Entry file for Throughput GATT server application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "wiced_bt_trace.h"
#include "platform_linux.h"
#include "wiced_bt_cfg.h"
#include "hello_sensor.h"
#include "utils_arg_parser.h"

/******************************************************************************
 *                                EXTERNS
 *****************************************************************************/
/* Stack configuration */
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
/* BLE configuration */
extern wiced_bt_cfg_ble_t hello_sensor_cfg_ble;
/* GATT configuration */
extern wiced_bt_cfg_gatt_t hello_sensor_cfg_gatt;
/* BD_ADDR*/
extern wiced_bt_device_address_t bt_device_address;

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define MAX_PATH               256
#define LOCAL_BDA_LEN          50
#define IP_ADDR_LEN            16
//#define PARSE_ERROR            -1
#define BT_STACK_HEAP_SIZE     0xF000

/* ***************************************************************************
 *                              GLOBAL VARIABLES
 * **************************************************************************/
wiced_bt_heap_t *p_default_heap   = NULL;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *   Function to handle HCI receive
 *
 * Parameters:
 *   uint8_t* p_buffer  : rx buffer
 *  uint32_t length     : rx buffer length
 *
 * Return:
 *  status code
 *
 ******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    return 0;
}

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *   BT stack initialization function
 *
 * Parameters:
 *   None
 *
 * Return:
 *      None
 *
 ******************************************************************************/
void application_start(void)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;

#ifdef WICED_BT_TRACE_ENABLE
    /* Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints */
    /* wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE); */

    /* Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart() */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#endif /* WICED_BT_TRACE_ENABLE */

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(hello_sensor_management_callback, &wiced_bt_cfg_settings);

    WICED_BT_TRACE("AIROC CYW5557x Hello sensor Start");

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
        /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (NULL == p_default_heap)
        {
            printf("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
            return;
        }
    } else
    {
        printf("Bluetooth Stack Initialization failed!! Exiting App...\n");
        exit(0);
    }
}

/******************************************************************************
 * Function Name: APPLICATION_START()
 *******************************************************************************
 * Summary:
 *   BT stack initialization function wrapper
 *
 * Parameters:
 *   None
 *
 * Return:
 *      None
 *
 ******************************************************************************/
void APPLICATION_START(void)
{
    application_start();
}

/******************************************************************************
 * Function Name: main()
 *******************************************************************************
 * Summary:
 *   Application entry function
 *
 * Parameters:
 *   int argc           : argument count
 *  char *argv[]        : list of arguments
 *
 * Return:
 *      None
 *
 ******************************************************************************/
int main(int argc, char *argv[])
{
    int len = 0;
    char patchFile[MAX_PATH];
    char device[MAX_PATH];
    char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
    uint32_t baud = 0, patch_baud = 0;
    int spy_inst = 0;
    uint8_t is_socket_tcp = 0;
   
   
        /* Audobaud configuration GPIO bank and pin */
    cybt_controller_autobaud_config_t autobaud;
    //char uart_cts_gpio_path[50] = { 0x00 };    /* UART CTS GPIO Path */
    char bt_regon_gpio_path[50] = { 0x00 };    /* BT REG ONOFF GPIO Path */


    /* Parse the arguments */
    int iResult;
    int cccd = 0;
    int ip = 0;
    memset(patchFile, 0, MAX_PATH);
    memset(device, 0, MAX_PATH);
    if (PARSE_ERROR == arg_parser_get_args(argc,
                                           argv,
                                           device,
					   bt_device_address,
                                           &baud,
                                           &spy_inst,
                                           peer_ip_addr,
                                           &is_socket_tcp,
                                           patchFile,
                                           &patch_baud,
                                           &autobaud ) )
    {
        return -1;
    }

    len = strlen(argv[0]);
    if (len >= MAX_PATH)
        len = MAX_PATH - 1;

    cy_bt_spy_comm_init(is_socket_tcp, spy_inst, peer_ip_addr);
    cy_platform_bluetooth_init(patchFile, device, baud, patch_baud, &autobaud );
    
    do 
    {
        printf ("Hello Sensor Application\n");
        printf ("=============================================\n");
        printf (" Press <ENTER> key to send notification \n");
        printf (" Press 0, then <ENTER> to Exit \n");
        printf ("=============================================\n");
        ip = getchar();
        
        switch (ip)
        {
            case '0':
                /* App exits */
                break;
            default:
                hello_sensor_interrupt_handler();
                break;
        }
    }while (ip != '0');
    
   wiced_bt_delete_heap(p_default_heap);
   return 0;
}
