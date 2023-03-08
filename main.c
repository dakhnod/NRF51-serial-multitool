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
#include "boards.h"
#include "nrf_drv_spi.h"
#include "fds.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
#include "nrf_uart.h"
#include "math.h"
#include "battery.h"
#include "ble_advertising.h"
#include "peer_manager.h"
#include "ble_conn_state.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

#if BLE_SECURE == 1
#include "secrets.h"
ble_opt_t passkey_opt;
#endif

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

#define DEVICE_NAME "Serial adapter"                     /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER 0     /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)                        /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)                        /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(1000, UNIT_10_MS)                          /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL_FAST MSEC_TO_UNITS(200, UNIT_0_625_MS)  /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_FAST_SECS 20                             /**< The advertising timeout (in units of seconds). */
#define APP_ADV_INTERVAL_SLOW MSEC_TO_UNITS(1000, UNIT_0_625_MS) /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_SLOW_SECS 0                              /**< The advertising timeout (in units of seconds). */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE 32 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 32 /**< UART RX buffer size. */

static ble_nus_t m_nus;                                  /**< Structure to identify the Nordic UART Service. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

#ifdef USE_DFU
ble_dfu_t dfu;
#endif

#ifdef USE_SPI
#define SPI_INSTANCE 0 /**< SPI instance index. */
static const nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
#endif

#define FILE_SETTINGS 0x0001
#define RECORD_NAME 0x0001

#define TRIGGER_PIN_IN 6
#define TRIGGER_PIN_OUT 8
#define TRIGGER_DURATION 4200
#define TRIGGER_POLARITY_IN 0
#define TRIGGER_POLARITY_OUT 0
#define TRIGGER_PULL_IN NRF_GPIO_PIN_NOPULL
#define TRIGGER_UART_DELAY 500

APP_TIMER_DEF(uart_send_timer);

#ifdef TRIGGER_PIN_IN

#ifndef TRIGGER_DURATION
#error Please define TRIGGER_DURATION
#endif
#ifndef TRIGGER_POLARITY_IN
#error Please define TRIGGER_POLARITY_IN
#endif
#ifndef TRIGGER_POLARITY_OUT
#error Please define TRIGGER_POLARITY_OUT
#endif

APP_TIMER_DEF(trigger_timer);
#endif


bool uart_received_flag = false;

static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);
bool uart_enabled = true;

void uart_enable_set(bool);

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
    unsigned int tmp = id;
    NRF_LOG_ERROR("app_error_print():\r\n");
    NRF_LOG_ERROR("Fault identifier:  0x%X\r\n", tmp);
    NRF_LOG_ERROR("Program counter:   0x%X\r\n", tmp = pc);
    NRF_LOG_ERROR("Fault information: 0x%X\r\n", tmp = info);

    switch (id)
    {
    case NRF_FAULT_ID_SDK_ASSERT:
        NRF_LOG_ERROR("Line Number: %u\r\n", tmp =
                                                 ((assert_info_t *)(info))->line_num);
        NRF_LOG_ERROR("File Name:   %s\r\n",
                      (uint32_t)((assert_info_t *)(info))->p_file_name);
        break;

    case NRF_FAULT_ID_SDK_ERROR:
        NRF_LOG_ERROR("Line Number: %u\r\n", tmp =
                                                 ((error_info_t *)(info))->line_num);
        NRF_LOG_ERROR("File Name:   %s\r\n",
                      (uint32_t)((error_info_t *)(info))->p_file_name);
        NRF_LOG_ERROR("Error Code:  0x%X\r\n", tmp =
                                                   ((error_info_t *)(info))->err_code);
        NRF_LOG_ERROR("Error description: %s\n", (uint32_t)ERR_TO_STR(((error_info_t *)(info))->err_code));
        break;
    }
#ifdef DEBUG
    for (;;)
        ;
#endif
}

#ifdef USE_DFU
uint32_t dfu_init()
{
    fs_init();

    ble_dfu_init_t init = {
        .evt_handler = NULL};
    return ble_dfu_init(&dfu, &init);
}
#endif

static void sys_evt_dispatch(uint32_t sys_evt)
{
    fs_sys_event_handler(sys_evt);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MAX_CONN_INTERVAL,
        .slave_latency = SLAVE_LATENCY,
        .conn_sup_timeout = CONN_SUP_TIMEOUT};
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    fds_flash_record_t flash_record;
    fds_record_desc_t record_desc;
    fds_find_token_t ftok = {0};

    bool read_name_success = false;

    err_code = fds_record_find(FILE_SETTINGS, RECORD_NAME, &record_desc, &ftok);

    if (err_code == FDS_SUCCESS)
    {
        NRF_LOG_DEBUG("found name record\n");
        if (fds_record_open(&record_desc, &flash_record) == FDS_SUCCESS)
        {
            // handle lookup

            err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                  (const uint8_t *)flash_record.p_data,
                                                  strlen(flash_record.p_data));
            APP_ERROR_CHECK(err_code);

            read_name_success = true;

            if (fds_record_close(&record_desc) != FDS_SUCCESS)
            {
                NRF_LOG_ERROR("could not close name record\n");
            }
        }
        else
        {
            NRF_LOG_ERROR("could not open name record\n");
        }
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_DEBUG("couldn't find name record\n");
    }
    else
    {
        NRF_LOG_DEBUG("fds error: %d\n", err_code);
    }

    if (!read_name_success)
    {
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
        APP_ERROR_CHECK(err_code);
    }

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
        0);
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
    err_code = dfu_init();
    APP_ERROR_CHECK(err_code);
#endif

    err_code = ble_bas_init();
    APP_ERROR_CHECK(err_code);

#if defined(USE_UART) || defined(USE_SPI)
    ble_nus_init_t nus_init = {
        .data_handler = nus_data_handler};
    err_code = ble_nus_init(&m_nus, &nus_init, BLE_SECURE == 1);
    APP_ERROR_CHECK(err_code);
#endif
}

// Simple event handler to handle errors during initialization.
void fds_evt_handler(fds_evt_t const *const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
    case FDS_EVT_INIT:
        if (p_fds_evt->result == FDS_SUCCESS)
        {
            NRF_LOG_DEBUG("fds init success\n");
        }
        else
        {
            NRF_LOG_ERROR("fds init error: %d\n", p_fds_evt->result);
        }
        break;
    case FDS_EVT_WRITE:
        if (p_fds_evt->result == FDS_SUCCESS)
        {
            NRF_LOG_DEBUG("fds write success\n");
        }
        else
        {
            NRF_LOG_ERROR("fds write error: %d\n", p_fds_evt->result);
        }
        break;
    default:
        break;
    }
}

void filesystem_init()
{
    ret_code_t err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_init();
    APP_ERROR_CHECK(err_code);
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
advertising_start()
{
    uint32_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void on_device_name_write(ble_gatts_evt_write_t *write_evt)
{
#define MAX_NAME_LEN 21
    static uint8_t __ALIGN(4) name_str[MAX_NAME_LEN];

    uint16_t len = 20;
    ret_code_t err_code;
    err_code = sd_ble_gap_device_name_get(name_str, &len);
    APP_ERROR_CHECK(err_code);

    name_str[len] = 0;

    uint16_t word_count = ceil((len + 1) / 4.0);

    fds_record_desc_t record_desc;
    fds_record_chunk_t record_chunk = {
        .p_data = name_str,
        .length_words = word_count};
    fds_record_t record = {
        .file_id = FILE_SETTINGS,
        .key = RECORD_NAME,
        .data = {
            .num_chunks = 1,
            .p_chunks = &record_chunk}};

    fds_find_token_t ftok = {0};
    err_code = fds_record_find(FILE_SETTINGS, RECORD_NAME, &record_desc, &ftok);
    if (err_code == FDS_SUCCESS)
    {
        NRF_LOG_DEBUG("updating existing name record\n");
        err_code = fds_record_update(&record_desc, &record);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("new record id: %d \r\n", record_desc.record_id);
    }
    else
    {
        NRF_LOG_DEBUG("creating new name record\n");
        err_code = fds_record_write(&record_desc, &record);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("new record id: %d \r\n", record_desc.record_id);
    }
    NRF_LOG_DEBUG("name written to fds\n");
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
    case BLE_GATTS_EVT_WRITE:
        goto start;
    start:;
        ble_gatts_evt_write_t write_evt = p_ble_evt->evt.gatts_evt.params.write;
        uint16_t uuid = write_evt.uuid.uuid;
        if (uuid == 0x2A00)
        { // device name uuid
            on_device_name_write(&write_evt);
        }
        break;
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_DEBUG("connected\n");

        // err_code = app_timer_start(uart_send_timer, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
        // APP_ERROR_CHECK(err_code);

        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break; // BLE_GAP_EVT_CONNECTED

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("disconnected\n");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        // err_code = app_timer_stop(uart_send_timer);
        // APP_ERROR_CHECK(err_code);

        advertising_start();
        break; // BLE_GAP_EVT_DISCONNECTED
    case BLE_GAP_EVT_PASSKEY_DISPLAY:{
        uint8_t *passkey = p_ble_evt->evt.gap_evt.params.passkey_display.passkey;
        NRF_LOG_INFO("passkey: %s\n", (uint32_t) passkey);
        break;
    }
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        // err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        // APP_ERROR_CHECK(err_code);
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
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    ble_bas_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
#ifdef USE_DFU
    ble_dfu_on_ble_evt(&dfu, p_ble_evt);
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

    // Check the ram settings against the used number of links
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

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
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
    while (true)
    {
        NRF_LOG_DEBUG("reading next byte\n");
        uint32_t result = app_uart_get(&current_byte);
        if (result == NRF_ERROR_NOT_FOUND)
            break;
        data[index++] = current_byte;
        if (index == max_size)
            break;
    }
    NRF_LOG_DEBUG("sending %i bytes\n", index);
    if (index > 0)
    {
        err_code = ble_nus_string_send(&m_nus, data, index);
        (void)err_code;
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

        app_timer_stop(uart_send_timer);
        app_timer_start(uart_send_timer, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);

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
        NRF_LOG_ERROR("what\n");
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
            7,
            5,
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

    nrf_delay_ms(100);

    // nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLDOWN);
}
#endif

#ifdef USE_SPI
static void spi_init()
{
    uint32_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.sck_pin = SPIM0_SCK_PIN;
    spi_config.mosi_pin = SPIM0_MOSI_PIN;

    err_code = nrf_drv_spi_init(
        &spi_instance,
        &spi_config,
        NULL);

    APP_ERROR_CHECK(err_code);
}
#endif

void uart_enable_set(bool enable)
{
    if (uart_enabled == enable)
    {
        return;
    }
    NRF_LOG_DEBUG("setting uart to %d\n", enable);
    UNUSED_PARAMETER(app_uart_inst);
    if (enable)
    {
        nrf_drv_uart_rx_enable(&app_uart_inst);
        nrf_uart_enable(app_uart_inst.reg.p_uart);
        uart_enabled = true;
    }
    else
    {
        nrf_drv_uart_rx_disable(&app_uart_inst);
        nrf_uart_disable(app_uart_inst.reg.p_uart);
        uart_enabled = false;
    }
}

void trigger_timeout_handler()
{
    nrf_gpio_pin_write(TRIGGER_PIN_OUT, !TRIGGER_POLARITY_OUT);

    uart_enable_set(false);
}

void gpio_trigger_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    action -= 1;
    app_timer_stop(trigger_timer);
    if (action != TRIGGER_POLARITY_IN)
    {
        uart_enable_set(true);
        nrf_gpio_pin_write(TRIGGER_PIN_OUT, TRIGGER_POLARITY_OUT);
    }
    else
    {
        app_timer_start(trigger_timer, APP_TIMER_TICKS(TRIGGER_DURATION, APP_TIMER_PRESCALER), NULL);
    }
}

void gpio_init()
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(TRIGGER_PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_write(TRIGGER_PIN_OUT, !TRIGGER_POLARITY_OUT);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

    in_config.pull = TRIGGER_PULL_IN;

    err_code = nrf_drv_gpiote_in_init(TRIGGER_PIN_IN, &in_config, gpio_trigger_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TRIGGER_PIN_IN, true);
}

void advertising_event_handler(ble_adv_evt_t event)
{
    // is_advertising = event != BLE_ADV_EVT_IDLE;
    switch (event)
    {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_DEBUG("advertising mode BLE_ADV_EVT_FAST\n");
        break;
    case BLE_ADV_EVT_SLOW:
        NRF_LOG_DEBUG("advertising mode BLE_ADV_EVT_SLOW\n");
        break;
    case BLE_ADV_EVT_IDLE:
        NRF_LOG_DEBUG("advertising mode BLE_ADV_EVT_IDLE\n");
        break;
    default:
        NRF_LOG_DEBUG("advertising mode UNKNOWN\n");
        break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void
advertising_init()
{
    uint32_t err_code;

    ble_adv_modes_config_t advertising_modes_config = {
        .ble_adv_whitelist_enabled = false,
        .ble_adv_directed_enabled = false,
        .ble_adv_directed_slow_enabled = false,
        .ble_adv_fast_enabled = true,
        .ble_adv_slow_enabled = true,

        .ble_adv_fast_interval = APP_ADV_INTERVAL_FAST,
        .ble_adv_fast_timeout = APP_ADV_TIMEOUT_FAST_SECS,
        .ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW,
        .ble_adv_slow_timeout = APP_ADV_TIMEOUT_SLOW_SECS,
    };

    ble_advdata_t advdata = {
        .name_type = BLE_ADVDATA_FULL_NAME,
        .include_appearance = false,
        .flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE};

    ble_uuid_t m_adv_uuids[] = {
        {0xffe0, BLE_UUID_TYPE_BLE},
        {0x0001, BLE_UUID_TYPE_VENDOR_BEGIN + 1},
    }; /**< Universally unique service identifier. */

    // ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};
    ble_advdata_t scanrsp = {
        .uuids_complete.uuid_cnt = 2,
        .uuids_complete.p_uuids = m_adv_uuids};

    // sd_ble_gap_appearance_set (BLE_APPEARANCE_GENERIC_THERMOMETER);

    err_code = ble_advertising_init(
        &advdata,
        &scanrsp,
        &advertising_modes_config,
        advertising_event_handler,
        NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
void peer_manager_event_handler(pm_evt_t const *p_evt)
{
    ret_code_t err_code;
    switch (p_evt->evt_id)
    {
    case PM_EVT_BONDED_PEER_CONNECTED:
        NRF_LOG_DEBUG("PM_EVT_BONDED_PEER_CONNECTED\n");
        // Update the rank of the peer.
        err_code = pm_peer_rank_highest(p_evt->peer_id);
        break;
    case PM_EVT_CONN_SEC_START:
        NRF_LOG_DEBUG("PM_EVT_CONN_SEC_START\n");
        break;
    case PM_EVT_CONN_SEC_SUCCEEDED:
        // Update the rank of the peer.
        NRF_LOG_DEBUG("PM_EVT_CONN_SEC_SUCCEEDED\n");
        err_code = pm_peer_rank_highest(p_evt->peer_id);
        break;
    case PM_EVT_CONN_SEC_FAILED:
        // In some cases, when securing fails, it can be restarted directly. Sometimes it can be
        // restarted, but only after changing some Security Parameters. Sometimes, it cannot be
        // restarted until the link is disconnected and reconnected. Sometimes it is impossible
        // to secure the link, or the peer device does not support it. How to handle this error
        // is highly application-dependent.
        NRF_LOG_DEBUG("PM_EVT_CONN_SEC_FAILED\n");
        break;
    case PM_EVT_CONN_SEC_CONFIG_REQ:
    {
        // A connected peer (central) is trying to pair, but the Peer Manager already has a bond
        // for that peer. Setting allow_repairing to false rejects the pairing request.
        // If this event is ignored (pm_conn_sec_config_reply is not called in the event
        // handler), the Peer Manager assumes allow_repairing to be false.
        NRF_LOG_DEBUG("PM_EVT_CONN_SEC_CONFIG_REQ\n");
        pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
        pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    }
    break;
    case PM_EVT_STORAGE_FULL:
        // Run garbage collection on the flash.
        NRF_LOG_DEBUG("PM_EVT_STORAGE_FULL\n");
        err_code = fds_gc();
        if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
        {
            // Retry.
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
        break;
    case PM_EVT_ERROR_UNEXPECTED:
        // Assert.
        NRF_LOG_DEBUG("PM_EVT_ERROR_UNEXPECTED\n");
        APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        break;
    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        NRF_LOG_DEBUG("PM_EVT_PEER_DATA_UPDATE_SUCCEEDED\n");
        break;
    case PM_EVT_PEER_DATA_UPDATE_FAILED:
        // Assert.
        NRF_LOG_DEBUG("PM_EVT_PEER_DATA_UPDATE_FAILED\n");
        APP_ERROR_CHECK_BOOL(false);
        break;
    case PM_EVT_PEER_DELETE_SUCCEEDED:
        NRF_LOG_DEBUG("PM_EVT_PEER_DELETE_SUCCEEDED\n");
        break;
    case PM_EVT_PEER_DELETE_FAILED:
        // Assert.
        NRF_LOG_DEBUG("PM_EVT_PEER_DELETE_FAILED\n");
        APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        break;
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        // At this point it is safe to start advertising or scanning.
        NRF_LOG_DEBUG("PM_EVT_PEERS_DELETE_SUCCEEDED\n");
        break;
    case PM_EVT_PEERS_DELETE_FAILED:
        // Assert.
        NRF_LOG_DEBUG("PM_EVT_PEERS_DELETE_FAILED\n");
        APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        break;
    case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        NRF_LOG_DEBUG("PM_EVT_LOCAL_DB_CACHE_APPLIED\n");
        break;
    case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        // The local database has likely changed, send service changed indications.
        NRF_LOG_DEBUG("PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED\n");
        pm_local_database_has_changed();
        break;
    case PM_EVT_SERVICE_CHANGED_IND_SENT:
        NRF_LOG_DEBUG("PM_EVT_SERVICE_CHANGED_IND_SENT\n");
        break;
    case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        NRF_LOG_DEBUG("PM_EVT_SERVICE_CHANGED_IND_CONFIRMED");
        break;
    }
}

void peer_manager_init()
{
    ret_code_t err_code;
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    ble_gap_sec_params_t sec_param = {
        .bond = 1,
        .mitm = 1,
        .lesc = 0,
        .io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY,
        .min_key_size = 7,
        .max_key_size = 16,
        .kdist_own.enc = 1,
        .kdist_own.id = 1,
        .kdist_peer.enc = 1,
        .kdist_peer.id = 1,
    };

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(peer_manager_event_handler);
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
        APP_TIMER_MODE_SINGLE_SHOT,
        uart_send_timeout_handler);
    APP_ERROR_CHECK(err_code);

#ifdef TRIGGER_PIN_IN
    err_code = app_timer_create(
        &trigger_timer,
        APP_TIMER_MODE_SINGLE_SHOT,
        trigger_timeout_handler);
    APP_ERROR_CHECK(err_code);

    gpio_init();
#endif

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
    filesystem_init();
    NRF_LOG_DEBUG("fds inited\n");
    gap_params_init();
    NRF_LOG_DEBUG("gap inited\n");
    #if BLE_SECURE == 1
    peer_manager_init();
    NRF_LOG_DEBUG("peer manager inited\n");
    passkey_opt.gap_opt.passkey.p_passkey = (uint8_t*) PASSKEY;
    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &passkey_opt);
    APP_ERROR_CHECK(err_code);
    #endif
    services_init();
    NRF_LOG_DEBUG("services inited\n");
    advertising_init();
    NRF_LOG_DEBUG("advertising inited\n");
    conn_params_init();
    NRF_LOG_DEBUG("conn params inited\n");

    advertising_start();
    NRF_LOG_DEBUG("advertising started\n");

#ifdef TRIGGER_PIN_IN
    uart_enable_set(false);
#endif

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
