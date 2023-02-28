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
 * File Name: hello_sensor.c
 *
 * Description: Hello Sensor application source file.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

#include "hello_uuid.h"
#include "hello_sensor.h"
#include "app_bt_utils.h"
#include "hello_sensor_gatt_cfg.h"

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_SENSOR_GATTS_MAX_CONN     1

#define APP_ADV_NAME wiced_bt_cfg_settings.device_name

#ifndef DEV_NAME
#define DEV_NAME "Hello"
#endif

#define WICED_BT_TRACE printf

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/* Holds the host info saved in the NVRAM */
wiced_timer_t hello_sensor_second_timer;
wiced_timer_t hello_sensor_ms_timer;
wiced_timer_t hello_sensor_conn_idle_timer;

static conn_state_info_t conn_state_info;

wiced_bt_gatt_status_t          hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     hello_sensor_set_advertisement_data(void);
void                            hello_sensor_send_message( void );
static void                     hello_sensor_gatts_increment_notify_value( void );
static void                     hello_sensor_timeout( WICED_TIMER_PARAM_TYPE count );
static void                     hello_sensor_smp_bond_result( uint8_t result );
static void                     hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr );
static void                     hello_sensor_application_init( void );
static void                     hello_sensor_conn_idle_timeout ( WICED_TIMER_PARAM_TYPE arg );
static void                     hello_sensor_advertisement_stopped( void );
#ifdef ENABLE_HCI_TRACE
static void                     hello_sensor_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif
static void                     hello_sensor_load_keys_for_address_resolution( void );
static void                     hello_sensor_led_timeout( uint32_t count );
void                            hello_sensor_led_blink( uint8_t num_of_blinks );

/******************************************************************************
 * Function Name: hello_sensor_management_cback()
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the BLE stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t hello_sensor_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_bt_device_address_t           id_addr;
    uint8_t                             *p_keys;
    wiced_bt_device_address_t           bda = {0}; 
    wiced_result_t                      result = WICED_BT_SUCCESS;

    printf("hello_sensor_management_cback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr((uint8_t *)bt_device_address, BLE_ADDR_PUBLIC);

                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                hello_sensor_application_init();
            } else
            {
                printf("Bluetooth Disabled \n");
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA: ");
            print_bd_address(p_event_data->user_passkey_notification.bd_addr);
            WICED_BT_TRACE("PassKey Notification.  Key %d \n", p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d ", p_info->reason);
            hello_sensor_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;

            wiced_hal_write_nvram ( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );

            WICED_BT_TRACE("Keys saved to NVRAM result: %d\n ", result);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            {
                p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
                result = wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID,
                                      sizeof(wiced_bt_device_link_keys_t),
                                      p_keys,
                                      &result );

                /* Break if link key retrieval is failed or link key is not available. */
                if (result != WICED_SUCCESS)
                {
                    WICED_BT_TRACE("\n Reading keys from NVRAM failed or link key not available.");
                    break;
                }

                WICED_BT_TRACE("keys read from NVRAM ");
                WICED_BT_TRACE("  result: %d \n", result);

            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d\n ", result);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM ");
            WICED_BT_TRACE("  result: %d \n", result);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status Event:  res %d", p_status->result);
            hello_sensor_encryption_changed( p_status->result, p_status->bd_addr );
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
            if ( *p_mode == BTM_BLE_ADVERT_OFF )
            {
                hello_sensor_advertisement_stopped();
            }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_state_info.conn_interval =
                    (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                printf("New connection interval: %f ms\n",
                       conn_state_info.conn_interval);
                printf("Supervision Time Out = %d\n", (p_event_data->ble_connection_param_update.supervision_timeout * 10));
            }
            break;


        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event,
            get_bt_event_name(event));
        break;
    }

    return WICED_BT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_application_init()
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is
 *   called from the BT management callback once the BLE stack enabled
 *   event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void hello_sensor_application_init( void )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_result_t result = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "hello_sensor_application_init\n" );

    /* init gatt */
    hello_sensor_gatt_init();

#ifdef ENABLE_HCI_TRACE
    wiced_bt_dev_register_hci_trace( hello_sensor_hci_trace_cback );
#endif
    /* Starting the app timers , seconds timer and the ms timer  */
    wiced_init_timer(&hello_sensor_second_timer, hello_sensor_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer( &hello_sensor_second_timer, HELLO_SENSOR_APP_TIMEOUT_IN_SECONDS );
    /* init the idle timer (but do not start yet) */
    wiced_init_timer(&hello_sensor_conn_idle_timer, hello_sensor_conn_idle_timeout, 0, WICED_SECONDS_TIMER);

    /* Load previous paired keys for address resolution */
    hello_sensor_load_keys_for_address_resolution();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    hello_sensor_set_advertisement_data();

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE( "Start Advertisements returned %d\n", result );

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS !=result)
    {
        printf("Advertisement cannot start because of error: %d \n Exiting App... \n", result);
        exit(0);
    }
    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    hello_sensor_state.flag_stay_connected = 1;
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void hello_sensor_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t hello_service_uuid[LEN_UUID_128] = { UUID_HELLO_SERVICE };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = hello_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (const char *) APP_ADV_NAME);
    adv_elem[num_elem].p_data       = (uint8_t *)APP_ADV_NAME;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
void hello_sensor_advertisement_stopped( void )
{
    wiced_result_t result;

    if ( hello_sensor_state.flag_stay_connected && !hello_sensor_state.conn_id )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
    }
    else
    {
        WICED_BT_TRACE( "Connected to Hello Sensor Service\n");
        WICED_BT_TRACE( "ADV stop\n");
    }

    UNUSED_VARIABLE(result);
}

/**
 * Blinks the LED
 */
void hello_sensor_led_blink( uint8_t num_of_blinks )
{
    printf ("===================== \n\n");
    printf ("Number of blinks: %d \n\n", num_of_blinks);
    printf ("=====================\n\n");
}

/*
 * The function invoked on timeout of app seconds timer.
 */
void hello_sensor_timeout( WICED_TIMER_PARAM_TYPE count )
{
    hello_sensor_state.timer_count++;
    /* print for first 10 seconds, then once every 10 seconds thereafter */
    if ((hello_sensor_state.timer_count <= 10) || (hello_sensor_state.timer_count % 10 == 0))
        WICED_BT_TRACE(" Hello_sensor_timeout: %d  \n",
                hello_sensor_state.timer_count);
}

/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */

void hello_sensor_smp_bond_result( uint8_t result )
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "\n hello_sensor, bond result: %d\n", result );

    /* Bonding success */
    if ( result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( hello_sensor_hostinfo.bdaddr, hello_sensor_state.remote_addr, sizeof( wiced_bt_device_address_t ) );

        /* Write to NVRAM */
        written_byte = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &status );
        WICED_BT_TRACE("\n NVRAM write: %d\n", written_byte);
    }

    UNUSED_VARIABLE(written_byte);
}


/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
void hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE("Encryption changed for BD address: ");
    print_bd_address( hello_sensor_hostinfo.bdaddr );
    WICED_BT_TRACE(" res: %d ", result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    wiced_hal_read_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

    /* If there are outstanding messages that we could not send out because
     * connection was not up and/or encrypted, send them now.  If we are sending
     * indications, we can send only one and need to wait for ack.
     */
    while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }

    /* If configured to disconnect after delivering data, start idle timeout
      to do disconnection */
    if ( ( !hello_sensor_state.flag_stay_connected ) && !hello_sensor_state.flag_indication_sent  )
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }
}

void hello_sensor_interrupt_handler( void )
{
    WICED_BT_TRACE("Sending Notification... \n");

    /* Blink as configured  */
    hello_sensor_led_blink(hello_sensor_hostinfo.number_of_blinks);

    /* Increment the last byte of the hello sensor notify value */
    hello_sensor_gatts_increment_notify_value();

    /* Remember how many messages we need to send */
    hello_sensor_state.num_to_write++;

    /* If connection is down, start high duty advertisements, so client can connect */
    if ( hello_sensor_state.conn_id == 0 )
    {
        wiced_result_t result;

        WICED_BT_TRACE( "ADV start high\n");

        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

        WICED_BT_TRACE( "wiced_bt_start_advertisements:%d\n", result );
        UNUSED_VARIABLE(result);
        return;
    }

    /*
     * Connection up.
     * Send message if client registered to receive indication
     * or notification. After we send an indication wait for the ack
     * before we can send anything else
     */
     while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
     {
         hello_sensor_state.num_to_write--;
         hello_sensor_send_message();
     }

     /* if we sent all messages, start connection idle timer to disconnect */
     if ( !hello_sensor_state.flag_stay_connected && !hello_sensor_state.flag_indication_sent )
     {
         if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
         {
             wiced_stop_timer(&hello_sensor_conn_idle_timer);
             wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
         }
    }
}

/*
 * Function on connection idle timeout initiates the gatt disconnect
 */
void hello_sensor_conn_idle_timeout ( WICED_TIMER_PARAM_TYPE arg )
{
    WICED_BT_TRACE( "hello_sensor_conn_idle_timeout\n" );

    /* Stopping the app timers */
    wiced_stop_timer(&hello_sensor_second_timer);
    wiced_stop_timer(&hello_sensor_ms_timer);

    /* Initiating the gatt disconnect */
    wiced_bt_gatt_disconnect( hello_sensor_state.conn_id );
}


/*
 * Check if client has registered for notification/indication
 * and send message if appropriate
 */
void hello_sensor_send_message( void )
{
    WICED_BT_TRACE( "hello_sensor_send_message: CCC:%d\n", hello_sensor_hostinfo.characteristic_client_configuration );

    /* If client has not registered for indication or notification, no action */
    if ( hello_sensor_hostinfo.characteristic_client_configuration == 0 )
    {
        return;
    }
    else if ( hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        uint8_t *p_attr = (uint8_t*)hello_sensor_char_notify_value;

        wiced_bt_gatt_send_notification( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, hello_sensor_char_notify_value_len, p_attr );
    }
    else
    {
        if ( !hello_sensor_state.flag_indication_sent )
        {
            uint8_t *p_attr = (uint8_t *)hello_sensor_char_notify_value;

            hello_sensor_state.flag_indication_sent = TRUE;

            wiced_bt_gatt_send_indication( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, hello_sensor_char_notify_value_len, p_attr );
        }
    }
}

/*
 * Find attribute description by handle
 */
attribute_t * hello_sensor_get_attribute( uint16_t handle )
{
    uint32_t i;
    for ( i = 0; i < gauAttributes_size; i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "hello_sensor_conn_up  id:%d\n:", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr, sizeof(wiced_bt_device_address_t));

    /* Stop idle timer */
    wiced_stop_timer(&hello_sensor_conn_idle_timer);

    /* Saving host info in NVRAM  */
    memcpy( hello_sensor_hostinfo.bdaddr, p_status->bd_addr, sizeof(wiced_bt_device_address_t));
    hello_sensor_hostinfo.characteristic_client_configuration = 0;
    hello_sensor_hostinfo.number_of_blinks                    = 0;

    {
        uint8_t bytes_written = 0;
        bytes_written = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

        WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
        UNUSED_VARIABLE(bytes_written);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason );

    /* Resetting the device info */
    memset( hello_sensor_state.remote_addr, 0, 6 );
    hello_sensor_state.conn_id = 0;



    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if ( hello_sensor_state.flag_stay_connected )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );
    }

    UNUSED_VARIABLE(result);

    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return hello_sensor_gatts_connection_up( p_status );
    }

    return hello_sensor_gatts_connection_down( p_status );
}


/*
 * Keep number of the button pushes in the last byte of the Hello message.
 * That will guarantee that if client reads it, it will have correct data.
 */
void hello_sensor_gatts_increment_notify_value( void )
{
    /* Getting the last byte */
    int last_byte = hello_sensor_char_notify_value_len  - 1 ;
    char c = hello_sensor_char_notify_value[last_byte];

    c++;
    if ( (c < '0') || (c > '9') )
    {
        c = '0';
    }
    hello_sensor_char_notify_value[last_byte] = c;
}

static void hello_sensor_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
    }
    WICED_BT_TRACE("hello_sensor_load_keys_for_address_resolution result:%d \n", result );
}
/* [] END OF FILE */
