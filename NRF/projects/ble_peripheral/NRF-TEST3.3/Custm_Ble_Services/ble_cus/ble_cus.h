
#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "sdk_common.h"
#include "app_error.h"


#define BLE_CUS_BLE_OBSERVER_PRIO  2


#define BLE_CUS_DEF(_name)                                                            \
static ble_cus_t _name;                                                               \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                   \
                     BLE_CUS_BLE_OBSERVER_PRIO,                                       \
                     ble_cus_on_ble_evt, &_name)



#define CUS_SERVICE_UUID_BASE      { 0x22, 0x21, 0x20, 0x19, 0x18, 0x17, 0x16, 0x15, \
                                     0x14, 0x13, 0x12, 0x11, 0x00, 0x00, 0x00, 0x00 }

#define CUS_SERVICE_UUID           0x2000


#define ACCEL_LEVEL_CHAR_UUID   0x2002	
#define OPCODE_LENGTH   1     /**< Length of opcode inside Blood Pressure Measurement packet. */
#define HANDLE_LENGTH   2     /**< Length of handle inside Blood Pressure Measurement packet. */
//#define MAX_BPM_LEN     (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) 
//#define MAX_BPM_LEN     (BLE_GATT_ATT_MTU_DEFAULT + 20 ) 
/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief Custom service event types.
 *
 */
typedef enum
{   
   
    BLE_ACCEL_LEVEL_CHAR_NOTIFICATIONS_ENABLED,
    BLE_ACCEL_LEVEL_CHAR_NOTIFICATIONS_DISABLED,

   

} ble_cus_evt_type_t;



///////////////////


typedef struct
{
   ble_cus_evt_type_t evt_type;                     

} ble_cus_evt_t;


/**@brief Forward declaration of the ble_cus_t type. */
typedef struct ble_cus_s ble_cus_t;


/**@brief Custom service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);


/**@brief Custom Service init structure. 
 * This contains all options needed for the initialization of the service.
 *
 */
typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */  
   
    ble_srv_cccd_security_mode_t  accel_level_char_attr_md; 
   
    security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
    security_req_t         bl_cccd_wr_sec;   

} ble_cus_init_t;


/**@brief Custom Service structure.
 *        This contains various status information for the service.
 */

struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
   
   ble_gatts_char_handles_t      accel_level_char_handles;    
   
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type;                      /**< Holds the service uuid type. */
};



void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for initializing the Custom ble service.
 *
 * @param[in]   p_cus       Custom service structure.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);




uint32_t ble_cus_accel_level_update(ble_cus_t * p_cus,  uint8_t *accel_level, uint16_t conn_handle);


