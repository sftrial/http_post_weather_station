/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <misc/reboot.h>
#include <misc/printk.h>
#include <stdio.h>
#include <stdlib.h> // for strtol()
#include <string.h>

#include <logging/log_ctrl.h>
#include <logging/log.h>
#define LOG_DOMAIN main
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(LOG_DOMAIN);

#include <net/bsdlib.h>
#include <lte_lc.h>
#include <modem_info.h>
#include <nrf_socket.h>
#include <at_cmd.h>

#if defined (USE_NET_SOCKET)
#include <net/socket.h>
#endif

#include <uart.h>
static struct device *uart_dev;
static char uart_dev_name[] = DT_UART_1_NAME;
static char serbuf[32];
static unsigned int serbuf_cnt = 0;

#include <dk_buttons_and_leds.h>
#include <gpio.h>
static struct device *led_dev0;
static struct device *led_dev1;
static struct device *led_dev2;
static struct device *led_dev3;
static struct device *switch_dev1;

#define IMEI_LEN 15
static char imei_buf[IMEI_LEN + 1];
static char rsrp_data = 255;
static volatile bool is_attached_to_cellular_network = false;

#define WEATHER_STATION_DATA_LEN  21
#define WEATHER_STATION_DATA_START_BYTE 0x24
static volatile bool weather_data_valid = false;
static volatile int weather_data_current_count = 0;
static char weather_data[WEATHER_STATION_DATA_LEN]; // updated by uart_rx_handler()

//#define HTTP_HOST_NAME "103.60.63.174"
#define HTTP_HOST "103.60.63.174"
#define HTTP_HOST_ADDR1 103
#define HTTP_HOST_ADDR2 60
#define HTTP_HOST_ADDR3 63
#define HTTP_HOST_ADDR4 174
#define HTTP_PORT 50102

const char http_header_template[] = "POST /weathersta/iotpkthandler HTTP/1.1\r\n"
                                     "host: %s:%d\r\n"
                                     "user-agent: ACTILITY-LRCLRN-DEVICE-AGENT/1.0\r\n"
                                     "accept: /*\r\n"
                                     "content-type: application/json\r\n"
                                     "content-length: %d\r\n"
                                     "\r\n";

const char http_content_template[] = "{"
                                     "\"app\":\"vn-workshop-2019\","
                                     "\"imei\":\"%s\","
                                     "\"rssi\":\"%d\","
                                     "\"data\":\"%s\""
                                     "}";

#define SEND_BUF_SIZE 1024
#define RECV_BUF_SIZE 8192

static int send_data_len;
static int http_content_len;
static char http_content_buf[256];
static char send_buf[SEND_BUF_SIZE + 1];
static char recv_buf[RECV_BUF_SIZE + 1];

#if defined (HTTP_HOST_NAME)
//static struct addrinfo *host_address = NULL;
static struct nrf_addrinfo *host_address = NULL;
#else
//static struct sockaddr_in host_address;
static struct nrf_sockaddr_in host_address;
#endif
static int client_fd = -1;

static struct weather_data_work
{
    struct k_work work;
    volatile int count;
    char data[(WEATHER_STATION_DATA_LEN << 1) + 1];
} send_weather_data_work;


#if !defined (HTTP_HOST_NAME)
static u32_t inet_addr(u8_t a, u8_t b, u8_t c, u8_t d)
{
    u32_t value = 0;

    value |= (u32_t)((((u8_t)(d)) << 24) & 0xFF000000);
    value |= (u32_t)((((u8_t)(c)) << 16) & 0x00FF0000);
    value |= (u32_t)((((u8_t)(b)) << 8) & 0x0000FF00);
    value |= (u32_t)((((u8_t)(a)) << 0) & 0x000000FF);

    return value;
}
#endif

static int recv_with_retry(int fd, u8_t *buf, u32_t size, int retries)
{
    int err;

    do
    {
        k_sleep(500);
        errno = 0;
#if defined (USE_NET_SOCKET)
        err = recv(fd, buf, size, MSG_DONTWAIT);
#else
        err = nrf_recv(fd, buf, size, NRF_MSG_DONTWAIT);
#endif
        //printk("%s err %d retries %d\n", __func__, err, retries);
    } while ((err < 0) && (errno == EAGAIN) && (--retries > 0) && (is_attached_to_cellular_network == true));

    return err;
}

static int host_address_init(void)
{
#if defined (HTTP_HOST_NAME)
    if (host_address != NULL)
    {
#if defined (USE_NET_SOCKET)
        freeaddrinfo(host_address);
#else
        nrf_freeaddrinfo(host_address);
#endif
        host_address = NULL;
    }

#if defined (USE_NET_SOCKET)
    int err = getaddrinfo(HTTP_HOST_NAME, NULL, NULL, &host_address);
#else
    int err = nrf_getaddrinfo(HTTP_HOST_NAME, NULL, NULL, &host_address);
#endif
    if (err != 0)
    {
        // has seen returning 51
        printk("getaddrinfo err: %d\n\r", err);
        return err;
    }
    //__ASSERT_NO_MSG(err == 0);
    ((struct sockaddr_in *)host_address->ai_addr)->sin_port = htons(HTTP_PORT);
#else
#if defined (USE_NET_SOCKET)
    host_address.sin_family = AF_INET;
    host_address.sin_port = htons(HTTP_PORT);
#else
    host_address.sin_len = sizeof(struct nrf_sockaddr_in);
    host_address.sin_family = NRF_AF_INET;
    host_address.sin_port = nrf_htons(HTTP_PORT);
#endif
    host_address.sin_addr.s_addr = inet_addr(HTTP_HOST_ADDR1, HTTP_HOST_ADDR2, HTTP_HOST_ADDR3, HTTP_HOST_ADDR4);
#endif
    return 0;
}

static int connect_to_server(void)
{
    int err, err1;

    if (client_fd == -1)
    {
        errno = 0;
#if defined (USE_NET_SOCKET)
        client_fd = socket(AF_INET, SOCK_STREAM, 0);
#else
        client_fd = nrf_socket(NRF_AF_INET, NRF_SOCK_STREAM, NRF_IPPROTO_TCP);
#endif
        if (client_fd < 0)
        {
            printk("socket() fail: client_fd %d errno %d\n\r", client_fd, errno);
        }
        else
        {
            printk("client_fd: %d\n\r", client_fd);
        }
        __ASSERT_NO_MSG(client_fd != -1);
    }

    err = host_address_init();
    if (err != 0)
    {
#if defined (USE_NET_SOCKET)
        err1 = close(client_fd);
#else
        err1 = nrf_close(client_fd);
#endif
        if (err1 != 0)
        {
            printk("close() fail: err %d errno %d\n\r", err1, errno);
        }
        __ASSERT_NO_MSG(err1 == 0);
        client_fd = -1;
        return err;
    }

    errno = 0;
#if defined (HTTP_HOST_NAME)
    err = connect(client_fd, (struct sockaddr *)host_address->ai_addr, sizeof(struct sockaddr_in));
#else
#if defined (USE_NET_SOCKET)
    err = connect(client_fd, (struct sockaddr *)&host_address, sizeof(struct sockaddr_in));
#else
    err = nrf_connect(client_fd, (const struct nrf_sockaddr_in *)&host_address, sizeof(struct nrf_sockaddr_in));
#endif
#endif
    if (err != 0)
    {
        // high chance of failing when running NB-IoT
        // different errno seen... 116, 115, 114, 128, 11
        printk("connect() fail: err: %d errno %d\n\r", err, errno);
#if defined (USE_NET_SOCKET)
        err1 = close(client_fd);
#else
        err1 = nrf_close(client_fd);
#endif
        if (err1 != 0)
        {
            printk("close() fail: err %d errno %d\n\r", err1, errno);
        }
        __ASSERT_NO_MSG(err1 == 0);
        client_fd = -1;
    }
    return err;
}

static int send_data_to_server(char *data)
{
    int err, err1;

    err = connect_to_server();
    if (err != 0)
    {
        return err;
    }
        
    int rssi_n;

    // convert Nordic's rsrp to KokTong's rssi
    // rssi
    // 99 == unknown
    // n  == -109 + ((n - 2) * 2)
    if ((rsrp_data >= 0) && (rsrp_data <= 97))
    {
        rssi_n = ((-141 + rsrp_data + 109) >> 1) + 2;
    }
    else
    {
        rssi_n = 99;
    }        

    memset(http_content_buf, 0, sizeof(http_content_buf));
    sprintf(http_content_buf, http_content_template, imei_buf, rssi_n, data);
    http_content_len = strlen(http_content_buf);
    //printk("HTTP body length:%d\n\r", http_content_len);

    memset(send_buf, 0, sizeof(send_buf));
    sprintf(send_buf, http_header_template, HTTP_HOST, HTTP_PORT, http_content_len);
    send_data_len = strlen(send_buf);

    sprintf((char *)(send_buf + send_data_len), "%s", http_content_buf);
    send_data_len = strlen(send_buf);
    //printk("to send (%d bytes): %s\n\r", send_data_len, send_buf);
    printk("to send (%d bytes)\n\r", send_data_len);

    errno = 0;
#if defined (USE_NET_SOCKET)
    int num_bytes = send(client_fd, send_buf, send_data_len, 0);
#else
    int num_bytes = nrf_send(client_fd, send_buf, send_data_len, 0);
#endif
    if (num_bytes != send_data_len)
    {
        if (num_bytes == -1)
        {
            printk("send() fail: err %d errno %d\n\r", num_bytes, errno);
        }
        else
        {
            printk("send() fail: %d <> %d\n\r", num_bytes, send_data_len);
        }
        err = -1;
    }
    else
    {
       int tot_num_bytes = 0;

        do
        {
            num_bytes = recv_with_retry(client_fd, recv_buf, RECV_BUF_SIZE, (tot_num_bytes == 0) ? 10: 1);
            if (num_bytes <= 0)
            {
                break;
            }
            tot_num_bytes += num_bytes;
            //printk("%s", recv_buf);
        } while (num_bytes > 0);
        if (tot_num_bytes > 0)
        {
            gpio_pin_write(led_dev3, LED3_GPIO_PIN, 1);//received data from http server
            printk("recvd %d bytes\n\r", tot_num_bytes);
            err = 0;
        }
        else
        {
            printk("recvd %d bytes errno %d\n\r", tot_num_bytes, errno);
            err = -1;
        }
    }

#if defined (USE_NET_SOCKET)
    err1 = close(client_fd);
#else
    err1 = nrf_close(client_fd);
#endif
    if (err1 != 0)
    {
        printk("close() fail: err %d errno %d\n\r", err1, errno);
    }
    __ASSERT_NO_MSG(err1 == 0);
    client_fd = -1;

    return err;
}

static void send_weather_data_work_fn(struct k_work *item)
{
    struct weather_data_work *the_weather_data_work =
        CONTAINER_OF(item, struct weather_data_work, work);

    LOG_DBG(">>>>> %d", the_weather_data_work->count);

    while ((the_weather_data_work->count == weather_data_current_count) && \
            (is_attached_to_cellular_network == true))
    {
        if (send_data_to_server(the_weather_data_work->data) == 0)
        {
            break;
        }
        k_sleep(100);
    }

    weather_data_valid = false;
    LOG_DBG("<<<<< %d\n", the_weather_data_work->count);
    send_weather_data_work.count = -1;
    k_sleep(50);
    gpio_pin_write(led_dev2, LED2_GPIO_PIN, 0);
    gpio_pin_write(led_dev3, LED3_GPIO_PIN, 0);
}

static void uart_rx_handler(u8_t character)
{
    static u64_t timeout_time_stamp;
    static u64_t current_time_stamp;
/*
 Weather station data:
 96008N1 (every 16seconds)
 24 12 90 62 75 3A 00 00 00 00 00 00 00 00 00 F0 C7 01 87 59 E1	(21 bytes)
 24 12 90 62 75 3A 00 00 00 00 00 00 00 00 00 F0 C7 01 87 61 E9
 ....

*/
    current_time_stamp = z_tick_get();
    if (serbuf_cnt > 0)
    {
        if (current_time_stamp > timeout_time_stamp)
        {
            serbuf_cnt = 0;
            LOG_DBG("reset buffer %d %d%d > %d%d", CONFIG_SYS_CLOCK_TICKS_PER_SEC, (u32_t)(current_time_stamp>>32), \
                              (u32_t)current_time_stamp, (u32_t)(timeout_time_stamp>>32), (u32_t)timeout_time_stamp);
        }
    }

    if (serbuf_cnt == 0)
    {
        timeout_time_stamp = current_time_stamp + CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        if (character == WEATHER_STATION_DATA_START_BYTE)
        {
//            printk("[%d - %02x] @ %ld %ld\n", serbuf_cnt, character, current_time_stamp, timeout_time_stamp);
            serbuf[0] = WEATHER_STATION_DATA_START_BYTE;
            serbuf_cnt = 1;
        }
    }
    else
    {
//        printk("[%d - %02x] @ %ld %ld\n", serbuf_cnt, character, current_time_stamp, timeout_time_stamp);
        serbuf[serbuf_cnt++] = character;
        if (serbuf_cnt == WEATHER_STATION_DATA_LEN)  // received a full frame
        {
            serbuf_cnt = 0;
            if (serbuf[0] == WEATHER_STATION_DATA_START_BYTE)
            {
                weather_data_current_count++;
                if (weather_data_valid == false)
                {
                    memcpy(weather_data, serbuf, WEATHER_STATION_DATA_LEN);
                    weather_data_valid = true;
                }
                else
                {
                    LOG_DBG("weather data dropped %d", weather_data_current_count);
                }
            }
            else
            {
                LOG_DBG("ill-formatted data dropped");
            }
        }
    }
}

static void isr(struct device *dev)
{
    u8_t character;

    uart_irq_update(dev);

    if (!uart_irq_rx_ready(dev))
    {
        return;
    }

    while (uart_fifo_read(dev, &character, 1))
    {
        uart_rx_handler(character);
    }
}

static void uart_init(void)
{
    uart_dev = device_get_binding(uart_dev_name);
    if (uart_dev == NULL)
    {
        LOG_ERR("Cannot bind %s", uart_dev_name);
        return;
    }

#if 0
    int err = uart_err_check(uart_dev);
    if (err)
    {
        //Note: 
        // will get <err> main: UART check failed. err 12 if 
        // nrf52840 is configured for CONFIG_BOARD_PCA10090_UART1_ARDUINO=y 
        // instead of default CONFIG_BOARD_PCA10090_UART1_VCOM
        LOG_ERR("UART check failed. err %d", err);
        return;
    }
#endif

    uart_irq_callback_set(uart_dev, isr);
    uart_irq_rx_enable(uart_dev);
    LOG_DBG("Init %s done", uart_dev_name);
}

/**@brief Callback handler for LTE RSRP data. */
static void modem_rsrp_handler(char rsrp_value)
{
    rsrp_data = rsrp_value;

    LOG_DBG("%d", rsrp_data);
}

static void urc_cereg_handler(char *response)
{
#define AT_CMD_CEREG_RESP	"+CEREG"
#define AT_CMD_SIZE(x) (sizeof(x) - 1)
    static const char   status1[] = "+CEREG: 1";
    static const char   status2[] = "+CEREG:1";
    static const char   status3[] = "+CEREG: 5";
    static const char   status4[] = "+CEREG:5";

    LOG_DBG("recv: %s", response);

    if (strstr(response, AT_CMD_CEREG_RESP))
    {
        if (!memcmp(status1, response, AT_CMD_SIZE(status1)) ||
	    !memcmp(status2, response, AT_CMD_SIZE(status2)) ||
	    !memcmp(status3, response, AT_CMD_SIZE(status3)) ||
	    !memcmp(status4, response, AT_CMD_SIZE(status4)))
        {
            LOG_DBG("ATTACHED");
            is_attached_to_cellular_network = true;
            gpio_pin_write(led_dev1, LED1_GPIO_PIN, 1);//cIoT attached
	}
        else
        {
            LOG_DBG("NOT ATTACHED");
            is_attached_to_cellular_network = false;
            gpio_pin_write(led_dev1, LED1_GPIO_PIN, 0);//cIoT de-attached
        }
    }
}

/**brief Initialize LTE status containers. */
static void modem_data_init(void)
{
    int err;

    int at_socket_fd;
    int bytes_written;
    int bytes_read;
    int ret;

    char buffer[120];

    printk("Getting IMEI and initial RSRP...\n");

    at_socket_fd = nrf_socket(NRF_AF_LTE, 0, NRF_PROTO_AT);
    __ASSERT_NO_MSG(at_socket_fd >= 0);

    bytes_written = nrf_write(at_socket_fd, "AT+CGSN", 7);
    __ASSERT_NO_MSG(bytes_written == 7);

    bytes_read = nrf_read(at_socket_fd, imei_buf, IMEI_LEN);
    __ASSERT_NO_MSG(bytes_read == IMEI_LEN);
    imei_buf[IMEI_LEN] = 0;
    printk("IMEI: %s\n\r", imei_buf);

    // get initial rsrp
    bytes_written = nrf_write(at_socket_fd, "AT%XMONITOR", 11);
    __ASSERT_NO_MSG(bytes_written == 11);

    bytes_read = nrf_read(at_socket_fd, buffer, sizeof(buffer));
    buffer[bytes_read] = 0;
    printk("%s\n\r", buffer);

    //%XMONITOR: 1,"","","52501","0287",9,8,"0B2A3127",288,3716,40,0,"","00011110","11100000"
    char *pToken;
    char *pRsrp;
    uint8_t i = 0;

    pToken = strchr(buffer, ',');
    pRsrp = NULL;
    while (pToken != NULL)
    {
        //printk("[%d] %s\n\r", i, pToken);
        if (i == 9)
        {
            pRsrp = pToken + 1;
            break;
        }
        i++;
        pToken = strchr(pToken + 1, ',');
    }
    if (pRsrp != NULL)
    {
        rsrp_data = strtol(pRsrp, &pToken, 10);
        //printk("Rsrp %d\n\r", rsrp_data);
    }

    ret = nrf_close(at_socket_fd);
    __ASSERT_NO_MSG(ret == 0);

    err = modem_info_init();
    if (err)
    {
        LOG_ERR("modem_info_init() err: %d", err);
        return;
    }

    err = modem_info_rsrp_register(modem_rsrp_handler);
    if (err)
    {
        LOG_ERR("modem_info_rsrp_register() err: %d", err);
    }

    // subscribe to the cereg URC enabled in lte_lc.c
    at_cmd_set_notification_handler(urc_cereg_handler);
    LOG_DBG("done");
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static void modem_configure(u32_t switch1_val)
{
    if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
    {
        /* Do nothing, modem is already turned on
        * and connected.
        */
    }
    else
    {
        int err;

        // 0 if towards the GND and 1 if towards NC
        if (switch1_val == 0)
        {
            lte_lc_set_network_mode_to_LTE_M();
        }
        else
        {
            lte_lc_set_network_mode_to_NB_IoT();
        }
        printk("Connecting to LTE network. ");
        printk("This may take several minutes.\n");

        err = lte_lc_init_and_connect();
        if (err)
        {
            printk("LTE link could not be established. err %d\n", err);

        }
        __ASSERT_NO_MSG(err == 0);
        printk("Connected to LTE network\n");
        is_attached_to_cellular_network = true;
    }
}

/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
#if defined (USE_WORK_QUEUE)
    // let's use work queue for the sake of learning how to use it
    k_work_init(&send_weather_data_work.work, send_weather_data_work_fn);
#endif
    send_weather_data_work.count = -1;
}

static void led_init(void)
{
    led_dev0 = device_get_binding(LED0_GPIO_CONTROLLER);
    if (led_dev0 == 0)
    {
        printk("Nordic nRF GPIO driver for %s was not found!\n", LED0_GPIO_CONTROLLER);
        return;
    }
    gpio_pin_configure(led_dev0, LED0_GPIO_PIN, GPIO_DIR_OUT);

    led_dev1 = device_get_binding(LED1_GPIO_CONTROLLER);
    if (led_dev1 == 0)
    {
        printk("Nordic nRF GPIO driver for %s was not found!\n", LED1_GPIO_CONTROLLER);
        return;
    }
    gpio_pin_configure(led_dev1, LED1_GPIO_PIN, GPIO_DIR_OUT);

    led_dev2 = device_get_binding(LED2_GPIO_CONTROLLER);
    if (led_dev2 == 0)
    {
        printk("Nordic nRF GPIO driver for %s was not found!\n", LED2_GPIO_CONTROLLER);
        return;
    }
    gpio_pin_configure(led_dev2, LED2_GPIO_PIN, GPIO_DIR_OUT);

    led_dev3 = device_get_binding(LED3_GPIO_CONTROLLER);
    if (led_dev3 == 0)
    {
        printk("Nordic nRF GPIO driver for %s was not found!\n", LED3_GPIO_CONTROLLER);
        return;
    }
    gpio_pin_configure(led_dev3, LED3_GPIO_PIN, GPIO_DIR_OUT);
}

void main(void)
{
    u32_t switch1_val;

    printk("Application started:\n");

    // check switch 1 state
    switch_dev1 = device_get_binding(SW2_GPIO_CONTROLLER);
    if (switch_dev1 == 0)
    {
        printk("Nordic nRF GPIO driver for %s was not found!\n", SW2_GPIO_CONTROLLER);
        return;
    }
    gpio_pin_configure(switch_dev1, SW2_GPIO_PIN, GPIO_DIR_IN | GPIO_PUD_PULL_UP);
    k_sleep(50);    
    if (gpio_pin_read(switch_dev1, SW2_GPIO_PIN, &switch1_val))
    {
        printk("Cannot read gpio pin");
        return;
    }

    led_init();
    gpio_pin_write(led_dev0, LED0_GPIO_PIN, 1);//app running
    work_init();
    modem_configure(switch1_val);
    gpio_pin_write(led_dev1, LED1_GPIO_PIN, 1);//cIoT attached
    modem_data_init();
    uart_init();
    //app_http_connect();

    while (1)
    {
        //if (k_work_pending(&send_weather_data_work.work) == 0) // still returns 0 even if send_weather_data_work_fn is running
        if (send_weather_data_work.count == -1)
        {
            if (weather_data_valid == true)
            {
                unsigned int i,j;

                gpio_pin_write(led_dev2, LED2_GPIO_PIN, 1);//has data from weather station
                j = 0;
                for (i = 0; i < WEATHER_STATION_DATA_LEN; i++)
                {
                    char c;

                    c = (weather_data[i] >> 4) & 0x0F;
                    if(c < 10)
                    {
                        send_weather_data_work.data[j++] = c + 0x30;
                    }
                    else
                    {
                        send_weather_data_work.data[j++] = c - 10 + 0x41 ;
                    }

                    c = weather_data[i] & 0x0F;
                    if(c < 10)
                    {
                        send_weather_data_work.data[j++] = c + 0x30;
                    }
                    else
                    {
                        send_weather_data_work.data[j++] = c - 10 + 0x41 ;
                    }
                }
                send_weather_data_work.data[WEATHER_STATION_DATA_LEN << 1] = 0;
                LOG_DBG("[%d] %s", weather_data_current_count, send_weather_data_work.data);
                send_weather_data_work.count = weather_data_current_count;
#if defined (USE_WORK_QUEUE)
                k_work_submit(&send_weather_data_work.work);
#else
                send_weather_data_work_fn(&send_weather_data_work.work);
#endif
            }
        }
    }
}