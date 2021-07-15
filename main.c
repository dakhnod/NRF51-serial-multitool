#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "WT51822_S4AT.h"
#include "nrf_drv_spi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#ifdef USE_DFU
#include "fstorage.h"
#include "ble_dfu.h"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0 /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE GATT_MTU_SIZE_DEFAULT /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT 0    /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT 1 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME "Serial adapter"                          /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */


#define APP_TIMER_PRESCALER 0     /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(8, UNIT_1_25_MS)                         /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(15, UNIT_1_25_MS)                         /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(1000, UNIT_10_MS)                          /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL_FAST           MSEC_TO_UNITS(100, UNIT_0_625_MS)	    /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS 0 /**< The advertising timeout (in units of seconds). */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE 512 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 512 /**< UART RX buffer size. */

static ble_nus_t m_nus;                                  /**< Structure to identify the Nordic UART Service. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] = {{0xffe0, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifier. */

#ifdef USE_DFU
ble_dfu_t dfu;
#endif

#ifdef USE_SPI
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
#endif

APP_TIMER_DEF(uart_send_timer);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    // On assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#else
    error_info_t *error_info = (error_info_t*)info;
    NRF_LOG_ERROR("error: %d\n", error_info->err_code);
    for(;;){
        NRF_LOG_PROCESS();
    }
#endif
}

#ifdef USE_DFU
uint32_t dfu_init ()
{
  fs_init ();

  ble_dfu_init_t init = {
    .evt_handler = NULL
  };
  return ble_dfu_init (&dfu, &init);
}
#endif

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MAX_CONN_INTERVAL,
        .slave_latency = SLAVE_LATENCY,
        .conn_sup_timeout = CONN_SUP_TIMEOUT
    };
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t *p_nus, uint8_t *p_data, uint16_t length)
{
    #ifdef USE_UART
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS)
            ;
    }
    #endif
    #ifdef USE_SPI
    uint32_t err_code = nrf_drv_spi_transfer(
        &spi_instance,
        p_data,
        length,
        NULL,
        0
    );
    APP_ERROR_CHECK(err_code);
    #endif
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    (void)err_code;

    #ifdef USE_DFU
    err_code = dfu_init ();
    APP_ERROR_CHECK (err_code);
    #endif

    #if defined(USE_UART) || defined(USE_SPI)
    ble_nus_init_t nus_init = {
        .data_handler = nus_data_handler
    };
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
    #endif
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init = {
        .p_conn_params = NULL,
        .first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY,
        .next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY,
        .max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT,
        .start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID,
        .disconnect_on_fail = false,
        .evt_handler = on_conn_params_evt,
        .error_handler = conn_params_error_handler,
    };

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


static void
advertising_start (uint16_t adv_interval)
{
  uint32_t err_code;
  ble_gap_adv_params_t adv_params = {
      .type = BLE_GAP_ADV_TYPE_ADV_IND,
      .p_peer_addr = NULL,
      .fp = BLE_GAP_ADV_FP_ANY,
      .interval = adv_interval,
      .timeout = APP_ADV_TIMEOUT_IN_SECONDS,
  };

  sd_ble_gap_adv_stop ();

  err_code = sd_ble_gap_adv_start (&adv_params);
  APP_ERROR_CHECK (err_code);
}

/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_DEBUG("connected\n");
        
        err_code = app_timer_start(uart_send_timer, APP_TIMER_TICKS(5, APP_TIMER_PRESCALER), NULL);
        APP_ERROR_CHECK(err_code);

        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break; // BLE_GAP_EVT_CONNECTED

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("disconnected\n");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        err_code = app_timer_stop(uart_send_timer);
        APP_ERROR_CHECK(err_code);

        advertising_start(APP_ADV_INTERVAL_FAST);
        break; // BLE_GAP_EVT_DISCONNECTED

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTC_EVT_TIMEOUT

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_TIMEOUT

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_EVT_USER_MEM_REQUEST

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
        ble_gatts_evt_rw_authorize_request_t req;
        ble_gatts_rw_authorize_reply_params_t auth_reply;

        req = p_ble_evt->evt.gatts_evt.params.authorize_request;

        if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                           &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
    break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                   NRF_BLE_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    #ifdef USE_DFU
    ble_dfu_on_ble_evt (&dfu, p_ble_evt);
    #endif
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    ble_enable_params.common_enable_params.vs_uuid_count = 2;
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

void uart_send_timeout_handler()
{
    uint32_t err_code;
    uint32_t max_size = BLE_NUS_MAX_DATA_LEN;
    // max_size = 1;
    uint8_t current_byte;
    uint8_t data[max_size];
    uint32_t index = 0;
    while(true){
        uint32_t result = app_uart_get(&current_byte);
        if(result == NRF_ERROR_NOT_FOUND) break;
        data[index++] = current_byte;
        if(index == max_size) break;
    }
    if(index > 0){
        err_code = ble_nus_string_send(&m_nus, data, index);
        (void) err_code;
        // APP_ERROR_CHECK(err_code);
    }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t *p_event)
{

    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        break;

    case APP_UART_COMMUNICATION_ERROR:
        NRF_LOG_ERROR("uart comm error: %d\n", p_event->data.error_communication);
        // APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        NRF_LOG_ERROR("uart fifo error: %d\n", p_event->data.error_code);
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
#ifdef USE_UART
static void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            APP_UART_FLOW_CONTROL_DISABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_MID,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
#endif

#ifdef USE_SPI
static void spi_init(){
    uint32_t err_code;

    nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.sck_pin = SPIM0_SCK_PIN;
    spi_config.mosi_pin = SPIM0_MOSI_PIN;

    err_code = nrf_drv_spi_init(
        &spi_instance,
        &spi_config,
        NULL
    );

    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void
advertising_init ()
{
  uint32_t err_code;

  ble_advdata_t advdata = {
    .name_type = BLE_ADVDATA_FULL_NAME,
    .include_appearance = false,
    .flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
  };

  // ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};
  ble_advdata_t scanrsp = {
    .uuids_complete.uuid_cnt = 1,
    .uuids_complete.p_uuids  = m_adv_uuids
  };

  // sd_ble_gap_appearance_set (BLE_APPEARANCE_GENERIC_THERMOMETER);

  err_code = ble_advdata_set (&advdata, &scanrsp);
  APP_ERROR_CHECK (err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    (void)nus_data_handler;

    NRF_LOG_INIT(NULL);

    NRF_LOG_DEBUG("starting...\n");

    // Initialize.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = app_timer_create(
        &uart_send_timer,
        APP_TIMER_MODE_REPEATED,
        uart_send_timeout_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("timer inited\n");
    #ifdef USE_UART
    uart_init();
    NRF_LOG_DEBUG("uart inited\n");
    #endif
    #ifdef USE_SPI
    spi_init();
    NRF_LOG_DEBUG("spi inited\n");
    #endif
    ble_stack_init();
    NRF_LOG_DEBUG("ble stack inited\n");
    gap_params_init();
    NRF_LOG_DEBUG("gap inited\n");
    services_init();
    NRF_LOG_DEBUG("services inited\n");
    advertising_init();
    NRF_LOG_DEBUG("advertising inited\n");
    conn_params_init();
    NRF_LOG_DEBUG("conn params inited\n");

    advertising_start(APP_ADV_INTERVAL_FAST);
    NRF_LOG_DEBUG("advertising started\n");

    // Enter main loop.
    for (;;)
    {
        power_manage();
        app_sched_execute();
    }
}

/**
 * @}
 */
