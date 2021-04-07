// Quest 4 progress
/* 
Setup: 
a. Change each FOB ID accordingly on line 88.
b. Flash onto each FOB

Backend:
1. FOB able to host internal election to get leader, all edge cases accounted for
2. FOB able to vote using two switches, one to change vote, one to send vote using NFC
3. FOB which receives NFC will broadcast a voting packet once, only the polling leader will respond to it. 
4. FOB leader will repackage the vote and broadcast to PI, message type: string: vl%c%c, votecolor, myID

Frontend:
5. Thus begin Carmen's database/webpage/front-end integration.
*/

// Standard C library
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>

//RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

//WIFI and UDP
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//IR RXTX
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// Hardware interrupt definitions
#define BUTTON_SEND       4 //A5
#define BUTTON_COLOR      36 //A4
#define ESP_INTR_FLAG_DEFAULT 0

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// UDP definitions
#define UDP_PORT 3333
#define MULTICAST_LOOPBACK CONFIG_EXAMPLE_LOOPBACK
#define MULTICAST_TTL CONFIG_EXAMPLE_MULTICAST_TTL
#define MULTICAST_IPV4_ADDR CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR
#define LISTEN_ALL_IF   EXAMPLE_MULTICAST_LISTEN_ALL_IF
static const char *TAG = "multicast";
static const char *V4TAG = "mcast-ipv4";

// LED Output pins definitions
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15
#define COLOR 'R'

// Mutex (for resources)
SemaphoreHandle_t mux = NULL;

// System tags
static const char *TAG_SYSTEM = "system";       // For debug logs

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
int myID = 1; //change this variable for each new FOB
char myColor = (char) COLOR; //my own voting color
char rcvColor; //recived voting color from NFC comms - only filled in when received packet
int rcvID; //received voting ID from NFC comms - only filled in when received packet
int len_out = 4;
bool rcvvote = false; //only true when received a voting packet from any FOB

//Leader election definitions
int leaderID; //leader's ID, not needed right now
int FOB_number = 2; // no. of Fobs, not needed right now
bool iamleader = false; //every FOB starts off with false
bool becomingleader = false; //every FOB starts off passive
char msgStatus; //leader msgStatus = l, election = e, voting = v, leader_transfer_vote = vl, answer = a

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for buttons
static void button_init() {
    gpio_pad_select_gpio(BUTTON_COLOR);
    gpio_pad_select_gpio(BUTTON_SEND);
    
    gpio_set_direction(BUTTON_COLOR, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_SEND, GPIO_MODE_INPUT);
    
}
/* Add a socket, either IPV4-only or IPV6 dual mode, to the IPV4
   multicast group */
static int socket_add_ipv4_multicast_group(int sock, bool assign_source_if)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
#if LISTEN_ALL_IF
    imreq.imr_interface.s_addr = IPADDR_ANY;
#else
    esp_netif_ip_info_t ip_info = { 0 };
    err = esp_netif_get_ip_info(get_example_netif(), &ip_info);
    if (err != ESP_OK) {
        ESP_LOGE(V4TAG, "Failed to get IP address info. Error 0x%x", err);
        goto err;
    }
    inet_addr_from_ip4addr(&iaddr, &ip_info.ip);
#endif // LISTEN_ALL_IF
    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(V4TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    ESP_LOGI(TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(V4TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        goto err;
    }

 err:
    return err;
}

static int create_multicast_ipv4_socket(void)
{
    // In init, add in myID random #
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(V4TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }
    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to bind socket. Error %d", errno);
        goto err;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        goto err;
    }

#if MULTICAST_LOOPBACK
    // select whether multicast traffic should be received by this device, too
    // (if setsockopt() is not called, the default is no)
    uint8_t loopback_val = MULTICAST_LOOPBACK;
    err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP,
                     &loopback_val, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_LOOP. Error %d", errno);
        goto err;
    }
#endif

    // this is also a listening socket, so add it to the multicast
    // group for listening...
    err = socket_add_ipv4_multicast_group(sock, true);
    if (err < 0) {
        goto err;
    }

    // All set, socket is configured for sending and receiving
    return sock;

err:
    close(sock);
    return -1;
}
// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tasks
// Election and voting task
static void mcast_leader_election_task(void *pvParameters)
// if non-leader, timeout 10seconds, hold election after first timeout, becoming leader after next timeout
{
    while (1) {
        int sock;
        sock = create_multicast_ipv4_socket();
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create IPv4 multicast socket");
        }
        if (sock < 0) {
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }
        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(UDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        int err = 1;
        while (err > 0) {
            struct timeval tv_leader = { //leader heartbeat
                .tv_sec = 2,
                .tv_usec = 0,
            };
            struct timeval tv_non_leader = { //non-leader timeout
                .tv_sec = 4,
                .tv_usec = 0,
            };
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);
            int s;
            if (iamleader) {
                s = select(sock + 1, &rfds, NULL, NULL, &tv_leader);
            }
            else {
                s = select(sock + 1, &rfds, NULL, NULL, &tv_non_leader);
            }
            if (rcvvote && !iamleader) { //if i received a vote and i am not a leader, broadcast it
                //immediately send a voting packet through UDP, will continue sending UDP packets of vote until received ack
                const char sendfmt[] = "v%c%d\n";
                char sendbuf[10];
                char addrbuf[32] = { 0 };
                int len = snprintf(sendbuf, sizeof(sendbuf), sendfmt,rcvColor,rcvID);
                if (len > sizeof(sendbuf)) {
                    ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                    err = -1;
                    break;
                }
                struct addrinfo hints = {
                    .ai_flags = AI_PASSIVE,
                    .ai_socktype = SOCK_DGRAM,
                };
                struct addrinfo *res;
                hints.ai_family = AF_INET; // For an IPv4 socket
                int err = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                        NULL,
                                        &hints,
                                        &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                err = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                freeaddrinfo(res);
                if (err < 0) {
                    ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                    break;
                } 
                rcvvote = false; //for testing, sent once only
            }
            else if (rcvvote && iamleader)
            {
                const char sendfmt[] = "vl%c%d\n";
                char sendbuf[10];
                char addrbuf[32] = { 0 };
                int len = snprintf(sendbuf, sizeof(sendbuf), sendfmt,rcvColor,rcvID);
                if (len > sizeof(sendbuf)) {
                    ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                    err = -1;
                    break;
                }
                struct addrinfo hints = {
                    .ai_flags = AI_PASSIVE,
                    .ai_socktype = SOCK_DGRAM,
                };
                struct addrinfo *res;
                hints.ai_family = AF_INET; // For an IPv4 socket
                int err = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                        NULL,
                                        &hints,
                                        &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                err = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                freeaddrinfo(res);
                if (err < 0) {
                    ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                    break;
                } 
                rcvvote = false; //for testing, sent once only
            }
            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                err = -1;
                break;
            }
            else if (s > 0) {
                if (FD_ISSET(sock, &rfds)) {
                    // Incoming datagram received
                    char recvbuf[48];
                    char raddr_name[32] = { 0 };

                    struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
                    socklen_t socklen = sizeof(raddr);
                    int len = recvfrom(sock, recvbuf, sizeof(recvbuf)-1, 0,
                                       (struct sockaddr *)&raddr, &socklen);
                    if (len < 0) {
                        ESP_LOGE(TAG, "multicast recvfrom failed: errno %d", errno);
                        err = -1;
                        break;
                    }
                    // Get the sender's address as a string
                    if (raddr.ss_family == PF_INET) {
                        inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr,
                                    raddr_name, sizeof(raddr_name)-1);
                    }
                    ESP_LOGI(TAG, "received %d bytes from %s:", len, raddr_name);

                    recvbuf[len] = 0; // Null-terminate whatever we received and treat like a string...
                    ESP_LOGI(TAG, "%s", recvbuf);

                    if ((int)recvbuf[0] == 101 && recvbuf[1]-'0' < myID) //if election msg and incoming ID lower than mine
                    {
                        //chance to become leader
                        becomingleader = true;
                        //send answer message 
                        const char sendfmt[] = "a%d\n";
                        char sendbuf[10];
                        char addrbuf[32] = { 0 };
                        int lenout = snprintf(sendbuf, sizeof(sendbuf), sendfmt, myID);
                        if (lenout > sizeof(sendbuf)) {
                            ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                            err = -1;
                            break;
                        }

                        struct addrinfo hints = {
                            .ai_flags = AI_PASSIVE,
                            .ai_socktype = SOCK_DGRAM,
                        };
                        struct addrinfo *res;
                        hints.ai_family = AF_INET; // For an IPv4 socket
                        int errout = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                                NULL,
                                                &hints,
                                                &res);
                        if (errout < 0) {
                            ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                            break;
                        }
                        if (res == 0) {
                            ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                            break;
                        }
                        ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                        inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                        ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                        errout = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                        freeaddrinfo(res);
                        if (errout < 0) {
                            ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                            break;
                        }
                    }
                    else if (iamleader)
                    { 
                        if (recvbuf[1]-'0' > myID && (int)recvbuf[0] != 118) //new FOB joined and called for election where its ID is bigger than mine
                        {
                            iamleader = false;
                            becomingleader = false;
                        }
                        else if ((int)recvbuf[0] == 118) // a voting message sent to FOB, only the leader need to respond to this message
                        {
                            printf("voting message received by leader\n");
                            const char sendfmt[] = "vl%c%c\n";
                            char sendbuf[10];
                            char addrbuf[32] = { 0 };
                            int lenout = snprintf(sendbuf, sizeof(sendbuf), sendfmt, recvbuf[1],recvbuf[2]);
                            if (lenout > sizeof(sendbuf)) {
                                ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                                err = -1;
                                break;
                            }

                            struct addrinfo hints = {
                                .ai_flags = AI_PASSIVE,
                                .ai_socktype = SOCK_DGRAM,
                            };
                            struct addrinfo *res;
                            hints.ai_family = AF_INET; // For an IPv4 socket
                            int errout = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                                    NULL,
                                                    &hints,
                                                    &res);
                            if (errout < 0) {
                                ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                                break;
                            }
                            if (res == 0) {
                                ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                                break;
                            }
                            ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                            inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                            ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                            errout = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                            freeaddrinfo(res);
                            if (errout < 0) {
                                ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                                break;
                            }
                        }
                        else //tell everyone that I am still the leader
                        {
                            const char sendfmt[] = "l%d\n";
                            char sendbuf[10];
                            char addrbuf[32] = { 0 };
                            int lenout = snprintf(sendbuf, sizeof(sendbuf), sendfmt, myID);
                            if (lenout > sizeof(sendbuf)) {
                                ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                                err = -1;
                                break;
                            }

                            struct addrinfo hints = {
                                .ai_flags = AI_PASSIVE,
                                .ai_socktype = SOCK_DGRAM,
                            };
                            struct addrinfo *res;
                            hints.ai_family = AF_INET; // For an IPv4 socket
                            int errout = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                                    NULL,
                                                    &hints,
                                                    &res);
                            if (errout < 0) {
                                ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                                break;
                            }
                            if (res == 0) {
                                ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                                break;
                            }
                            ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                            inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                            ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                            errout = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                            freeaddrinfo(res);
                            if (errout < 0) {
                                ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                                break;
                            }
                        }
                    }
                    else if ((int)recvbuf[0] == 108 && recvbuf[1]-'0' < myID) //I joined a FOB network where leader ID is lower than mine
                    {
                        msgStatus = 'l';
                        becomingleader = true;
                        iamleader = true;
                        const char sendfmt[] = "%c%d\n";
                        char sendbuf[10];
                        char addrbuf[32] = { 0 };
                        int lenout = snprintf(sendbuf, sizeof(sendbuf), sendfmt, msgStatus,myID);
                        if (lenout > sizeof(sendbuf)) {
                            ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                            err = -1;
                            break;
                        }

                        struct addrinfo hints = {
                            .ai_flags = AI_PASSIVE,
                            .ai_socktype = SOCK_DGRAM,
                        };
                        struct addrinfo *res;
                        hints.ai_family = AF_INET; // For an IPv4 socket
                        int errout = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                                NULL,
                                                &hints,
                                                &res);
                        if (errout < 0) {
                            ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                            break;
                        }
                        if (res == 0) {
                            ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                            break;
                        }
                        ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                        inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                        ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                        errout = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                        freeaddrinfo(res);
                        if (errout < 0) {
                            ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                            break;
                        }
                    }
                    else { //recevied a message but its either election msg with higher ID, answer message from other higher ID when i hold election, or leader msg
                        becomingleader = false;
                    }
                }
            }
            else { // s == 0
                // Timeout passed with no incoming data, so send something
                if (becomingleader) //becomingleader already, now timeout again, becomes leader
                {
                    iamleader = true;
                    msgStatus = 'l';
                }
                else //quiet on channel, hold an election. 
                {
                    //hold election
                    msgStatus = 'e';
                    becomingleader = true;
                }
                //leader broadcast every 5 seconds, election + becoming leader broadcast every 10seconds
                const char sendfmt[] = "%c%d\n";
                char sendbuf[10];
                char addrbuf[32] = { 0 };
                int len = snprintf(sendbuf, sizeof(sendbuf), sendfmt,msgStatus,myID);
                if (len > sizeof(sendbuf)) {
                    ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                    err = -1;
                    break;
                }
                struct addrinfo hints = {
                    .ai_flags = AI_PASSIVE,
                    .ai_socktype = SOCK_DGRAM,
                };
                struct addrinfo *res;
                hints.ai_family = AF_INET; // For an IPv4 socket
                int err = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                        NULL,
                                        &hints,
                                        &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                err = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                freeaddrinfo(res);
                if (err < 0) {
                    ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                    break;
                }
            }
        }
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}
//NFC task
//turn on leds
void led_task(){
    switch((int)rcvColor){
      case 'R' : // Red
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 1);
        gpio_set_level(BLUEPIN, 0);
        break;
      case 'Y' : // Blue pin
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 0);
        gpio_set_level(BLUEPIN, 1);
        // printf("Current state: %c\n",status);
        break;
      case 'G' : // Green
        gpio_set_level(GREENPIN, 1);
        gpio_set_level(REDPIN, 0);
        gpio_set_level(BLUEPIN, 0);
        // printf("Current state: %c\n",status);
        break;
    }
}
// Send task -- sends payload | Start | myID | Start | myID
void send_task(){

    char *data_out = (char *) malloc(len_out);
    xSemaphoreTake(mux, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char) myColor;
    data_out[2] = (char) myID;
    data_out[3] = genCheckSum(data_out,len_out-1);

    uart_write_bytes(UART_NUM_1, data_out, len_out);
    xSemaphoreGive(mux);
}
// Buttons tasks
void button_task(){
    while(1){
        //input signal for button send signal 
        int signal_send = gpio_get_level(BUTTON_SEND);
        if(signal_send == 1){
          printf("Button to send pressed\n");
           send_task();
        }

        //button to change LED color
        int signal_color = gpio_get_level(BUTTON_COLOR);
        if(signal_color == 1){
            printf("Button to change LED color pressed\n");
           if (myColor == 'R') {
              myColor = 'G';
              printf("My vote is now Green\n");
            }
            else if (myColor == 'G') {
              myColor = 'Y';
              printf("My vote is now Blue\n");
            }
            else if (myColor == 'Y') {
              myColor = 'R';
              printf("My vote is now Red \n");
            }
        }
        vTaskDelay(100);
    }
  
}
// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      if (data_in[0] == start) {
            printf("Voting color received: %c%d\n", data_in[1],data_in[2]);
            //change fob color to received color 
            rcvColor = data_in[1];
            rcvID = data_in[2];
            led_task();
            rcvvote = true; //got a vote from FOB, now need to trasmit that to FOB poll leader
        }
        if (checkCheckSum(data_in,len_out)) {
          ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
        }   
    }
    else{
      //printf("Nothing received.\n");
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

void app_main(void)
{
    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    rmt_tx_init();
    uart_init();
    led_init();
    button_init();

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(&mcast_leader_election_task, "mcast_leader_election_task", 4096, NULL, 5, NULL);
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 1024*2, NULL, 5, NULL);
}
