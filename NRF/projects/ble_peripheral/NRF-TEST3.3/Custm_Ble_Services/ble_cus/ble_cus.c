
#include "ble_cus.h"


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_cus_evt_t                 evt;
    
  
    // writing to the accel characteristic (cccd)
     if (p_evt_write->handle == p_cus->accel_level_char_handles.cccd_handle)
   {
      if (ble_srv_is_notification_enabled(p_evt_write->data))
      {
          evt.evt_type = BLE_ACCEL_LEVEL_CHAR_NOTIFICATIONS_ENABLED;
      }
      else
      {
          evt.evt_type = BLE_ACCEL_LEVEL_CHAR_NOTIFICATIONS_DISABLED;
      }

      p_cus->evt_handler(p_cus, &evt);
   }
}


/**@brief Function for handling the Custom servie ble events.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{

    uint32_t                  err_code;
    ble_uuid_t                ble_uuid;
    ble_add_char_params_t     add_char_params;

/* Adding the service */

    // Initialize service structure.
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;
    
    // Add the Custom ble Service UUID
    ble_uuid128_t base_uuid =  CUS_SERVICE_UUID_BASE;
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    //err_code =  sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUS_SERVICE_UUID;

    // Add the service to the database
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    ble_gatts_attr_md_t attr_md;
memset(&attr_md, 0, sizeof(attr_md));
attr_md.vloc        = BLE_GATTS_VLOC_STACK;

ble_gatts_attr_t    attr_char_value;
memset(&attr_char_value, 0, sizeof(attr_char_value));    
attr_char_value.p_uuid      = &ble_uuid;
attr_char_value.p_attr_md   = &attr_md;

ble_gatts_char_md_t char_md;
memset(&char_md, 0, sizeof(char_md));
char_md.char_props.read = 1;
char_md.char_props.notify = 1;
attr_char_value.max_len     =15;
attr_char_value.init_len    =15;
uint8_t value[]            = {0x00,0x00,0x00,0x00,0x00,0x04,0x60,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00};
attr_char_value.p_value     = value;
BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.notify_perm);
err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_cus->accel_level_char_handles);
APP_ERROR_CHECK(err_code);

}




uint32_t ble_cus_accel_level_update(ble_cus_t * p_cus, uint8_t *accel_level, uint16_t conn_handle)
{

    
        if (p_cus == NULL)
        {
            return NRF_ERROR_NULL;
        }

    ret_code_t                 err_code;
    ble_gatts_hvx_params_t params;
    uint16_t len = 15;
 
        memset(&params, 0, sizeof(params));

        params.handle = p_cus->accel_level_char_handles.value_handle;
        params.type   = BLE_GATT_HVX_NOTIFICATION;
        params.offset = 0;
        params.p_len  = &len;
        params.p_data =accel_level;



    return sd_ble_gatts_hvx(conn_handle, &params);
   
   


    return err_code;
}


