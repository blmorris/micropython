/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// We can't include stdio.h because it defines _types_fd_set, but we
// need to use the CC3100 version of this type.

#include <std.h>
#include <string.h>
#include <stdarg.h>
//#include <errno.h>

#include "stm32f4xx_hal.h"
#include "py/mpconfig.h"
#include "py/nlr.h"
#include "py/misc.h"
#include "py/qstr.h"
#include "py/obj.h"
#include "py/objtuple.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/runtime.h"
#include "modnetwork.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "spi.h"
#include "pybioctl.h"


//*****************************************************************************
// Prefix exported names to avoid name clash
//*****************************************************************************
#define CC3100_EXPORT(name) cc3100_ ## name

#include "simplelink.h"
//#include "inet_ntop.h"
//#include "inet_pton.h"

#include "log.h"

// CC3100 defines (different from standard one!)
#define ENOENT  -2
#define EPIPE   -32

#define CC31K_SOCKET_MAX     SL_MAX_SOCKETS   // the maximum number of sockets that the CC31K could support
#define CC31K_MAX_RX_PACKET  (16000)
#define CC31K_MAX_TX_PACKET  (1460)

#define MAKE_SOCKADDR(addr, ip, port) \
    sockaddr addr = {0}; \
    addr.sa_family = AF_INET; \
    addr.sa_data[0] = port >> 8; \
    addr.sa_data[1] = port; \
    addr.sa_data[2] = ip[0]; \
    addr.sa_data[3] = ip[1]; \
    addr.sa_data[4] = ip[2]; \
    addr.sa_data[5] = ip[3];

#define UNPACK_SOCKADDR(addr, ip, port) \
    port = (addr.sa_data[0] << 8) | addr.sa_data[1]; \
    ip[0] = addr.sa_data[2]; \
    ip[1] = addr.sa_data[3]; \
    ip[2] = addr.sa_data[4]; \
    ip[3] = addr.sa_data[5];

STATIC int cc31k_socket_ioctl(mod_network_socket_obj_t *socket, mp_uint_t request, mp_uint_t arg, int *_errno);

int CC3100_EXPORT(errno); // for cc3100 driver

STATIC int cc31k_init(void);
STATIC int cc31k_ver(SlVersionFull *sVersion);

STATIC volatile uint32_t fd_closed_state = 0;
STATIC volatile bool wlan_connected      = false;
STATIC volatile bool ip_obtained         = false;

STATIC int cc31k_get_fd_closed_state(int fd) {
    return fd_closed_state & (1 << fd);
}

STATIC void cc31k_set_fd_closed_state(int fd) {
    fd_closed_state |= 1 << fd;
}

STATIC void cc31k_reset_fd_closed_state(int fd) {
    fd_closed_state &= ~(1 << fd);
}

STATIC int cc31k_socket_socket(mod_network_socket_obj_t *_socket, int *_errno) {

    mp_uint_t domain;
    switch (_socket->u_param.domain) {
        case MOD_NETWORK_AF_INET:  domain = AF_INET;  break;
        case MOD_NETWORK_AF_INET6: domain = AF_INET6; break;
        default: *_errno = EAFNOSUPPORT; return -1;
    }

    mp_uint_t type;
    switch (_socket->u_param.type) {
        case MOD_NETWORK_SOCK_STREAM: type = SOCK_STREAM; break;
        case MOD_NETWORK_SOCK_DGRAM: type = SOCK_DGRAM; break;
        case MOD_NETWORK_SOCK_RAW: type = SOCK_RAW; break;
        default: *_errno = EINVAL; return -1;
    }

    // open socket
    int fd = socket(domain, type, 0);
    if (fd < 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }
    // clear socket state
    cc31k_reset_fd_closed_state(fd);

    // store state of this socket
    _socket->u_state = fd;

    // make accept blocking by default
    //int optval = SOCK_OFF;
    //socklen_t optlen = sizeof(optval);
    //setsockopt(_socket->u_state, SOL_SOCKET, SOCKOPT_ACCEPT_NONBLOCK, &optval, optlen);

    return 0;
}

STATIC void cc31k_socket_close(mod_network_socket_obj_t *_socket) {
    close(_socket->u_state);
}

STATIC int cc31k_socket_bind(mod_network_socket_obj_t *_socket, byte *ip, mp_uint_t port, int *_errno) {
    MAKE_SOCKADDR(addr, ip, port)

    int ret = bind(_socket->u_state, (sockaddr*)&addr, sizeof(addr));
    if (ret != 0) {
        *_errno = ret;
        return -1;
    }
    return 0;
}

STATIC int cc31k_socket_listen(mod_network_socket_obj_t *_socket, mp_int_t backlog, int *_errno) {
    int ret = listen(_socket->u_state, backlog);
    if (ret != 0) {
        *_errno = ret;
        return -1;
    }
    return 0;
}

STATIC int cc31k_socket_accept(mod_network_socket_obj_t *_socket, mod_network_socket_obj_t *socket2, byte *ip, mp_uint_t *port, int *_errno) {
    // accept incoming connection
    int fd;
    sockaddr addr = {0};
    socklen_t addr_len = sizeof(addr);
    if ((fd = accept(_socket->u_state, &addr, &addr_len)) < 0) {
        if (fd == SL_EAGAIN) {
            *_errno = EAGAIN;
        } else {
            *_errno = -fd;
        }
        return -1;
    }

    // clear socket state
    cc31k_reset_fd_closed_state(fd);

    // store state in new socket object
    socket2->u_state = fd;

    // return ip and port
    *port = (addr.sa_data[1] << 8) | addr.sa_data[0];
    ip[3] = addr.sa_data[2];
    ip[2] = addr.sa_data[3];
    ip[1] = addr.sa_data[4];
    ip[0] = addr.sa_data[5];

    return 0;
}

STATIC int cc31k_socket_connect(mod_network_socket_obj_t *_socket, byte *ip, mp_uint_t port, int *_errno) {
    MAKE_SOCKADDR(addr, ip, port)
    int ret = connect(_socket->u_state, &addr, sizeof(addr));
    if (ret != 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }
    return 0;
}

STATIC mp_uint_t cc31k_socket_send(mod_network_socket_obj_t *_socket, const byte *buf, mp_uint_t len, int *_errno) {
    if (cc31k_get_fd_closed_state(_socket->u_state)) {
        close(_socket->u_state);
        *_errno = EPIPE;
        return -1;
    }

    // CC31K does not handle fragmentation, and will overflow,
    // split the packet into smaller ones and send them out.
    mp_int_t bytes = 0;
    while (bytes < len) {
        int n = MIN((len - bytes), CC31K_MAX_TX_PACKET);
        n = send(_socket->u_state, (uint8_t*)buf + bytes, n, 0);
        if (n <= 0) {
            *_errno = CC3100_EXPORT(errno);
            return -1;
        }
        bytes += n;
    }

    return bytes;
}

STATIC mp_uint_t cc31k_socket_recv(mod_network_socket_obj_t *_socket, byte *buf, mp_uint_t len, int *_errno) {
    // check the socket is open
    if (cc31k_get_fd_closed_state(_socket->u_state)) {
        // socket is closed, but CC3100 may have some data remaining in buffer, so check
        SlFdSet_t rfds;
        FD_ZERO(&rfds);
        SL_FD_SET(_socket->u_state, &rfds);
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        int nfds = select(_socket->u_state + 1, &rfds, NULL, NULL, &tv);
        if (nfds == -1 || !SL_FD_ISSET(_socket->u_state, &rfds)) {
            // no data waiting, so close socket and return 0 data
            close(_socket->u_state);
            return 0;
        }
    }

    // cap length 
    len = MIN(len, CC31K_MAX_RX_PACKET);

    // do the recv
    int ret = recv(_socket->u_state, buf, len, 0);
    if (ret < 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }

    return ret;
}

STATIC mp_uint_t cc31k_socket_sendto(mod_network_socket_obj_t *_socket, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno) {
    MAKE_SOCKADDR(addr, ip, port)
    int ret = sendto(_socket->u_state, (byte*)buf, len, 0, (sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }
    return ret;
}

STATIC mp_uint_t cc31k_socket_recvfrom(mod_network_socket_obj_t *_socket, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno) {
    sockaddr addr = {0};
    socklen_t addr_len = sizeof(addr);
    mp_int_t ret = recvfrom(_socket->u_state, buf, len, 0, &addr, &addr_len);
    if (ret < 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }
    UNPACK_SOCKADDR(addr, ip, *port);
    return ret;
}

STATIC int cc31k_socket_setsockopt(mod_network_socket_obj_t *_socket, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno) {

    int _opt, _level;

    switch (level) 
    {
        case MOD_NETWORK_SOL_SOCKET:    _level = SL_SOL_SOCKET;     break;
        default:                        _level = level;             break;
    }
    switch (opt) 
    {
        case MOD_NETWORK_SOCK_NONBLOCK: _opt = SL_SO_NONBLOCKING; break;
        default:                        _opt = opt;               break;
    }

    int ret = setsockopt(_socket->u_state, _level, _opt, optval, optlen);
    if (ret < 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }
    return 0;
}

STATIC int cc31k_socket_settimeout(mod_network_socket_obj_t *_socket, mp_uint_t timeout_ms, int *_errno) {
    int ret;

    struct SlTimeval_t timeval;
    timeval.tv_sec =  timeout_ms / 1000;                             // Seconds
    timeval.tv_usec = (timeout_ms - (1000 * timeval.tv_sec)) * 1000; // Microseconds

    // NOTE: documentation mentioes 10000 microseconds resolution

    // set timeout
    ret = setsockopt(_socket->u_state, SOL_SOCKET, SL_SO_RCVTIMEO, &timeval, sizeof(timeval));
    if (ret != 0) {
        *_errno = CC3100_EXPORT(errno);
        return -1;
    }

    return 0;
}

STATIC int cc31k_socket_ioctl(mod_network_socket_obj_t *_socket, mp_uint_t request, mp_uint_t arg, int *_errno) {
    mp_uint_t ret;
    if (request == MP_IOCTL_POLL) {
        mp_uint_t flags = arg;
        ret = 0;
        int fd = _socket->u_state;

        // init fds
        SlFdSet_t rfds, wfds, xfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&xfds);

        // set fds if needed
        if (flags & MP_IOCTL_POLL_RD) {
            SL_FD_SET(fd, &rfds);

            // A socked that just closed is available for reading.  A call to
            // recv() returns 0 which is consistent with BSD.
            if (cc31k_get_fd_closed_state(fd)) {
                ret |= MP_IOCTL_POLL_RD;
            }
        }
        if (flags & MP_IOCTL_POLL_WR) {
            SL_FD_SET(fd, &wfds);
        }
        if (flags & MP_IOCTL_POLL_HUP) {
            SL_FD_SET(fd, &xfds);
        }

        // call cc3100 select with minimum timeout
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        int nfds = sl_Select(fd + 1, &rfds, &wfds, &xfds, &tv);

        // check for error
        if (nfds == -1) {
            *_errno = CC3100_EXPORT(errno);
            return -1;
        }

        // check return of select
        if (SL_FD_ISSET(fd, &rfds)) {
            ret |= MP_IOCTL_POLL_RD;
        }
        if (SL_FD_ISSET(fd, &wfds)) {
            ret |= MP_IOCTL_POLL_WR;
        }
        if (SL_FD_ISSET(fd, &xfds)) {
            ret |= MP_IOCTL_POLL_HUP;
        }
    } else {
        *_errno = EINVAL;
        ret = -1;
    }
    return ret;
}

STATIC int cc31k_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *out_ip) {
    uint32_t ip;

    int rc = gethostbyname((signed char *)name, (uint16_t)len, &ip, SL_AF_INET);
    if (rc < 0) {
        return CC3100_EXPORT(errno);
    }

    if (ip == 0) {
        // unknown host
        return ENOENT;
    }

    out_ip[0] = ip >> 24;
    out_ip[1] = ip >> 16;
    out_ip[2] = ip >> 8;
    out_ip[3] = ip;

    return 0;
}

/******************************************************************************/
// Micro Python bindings; CC31K class

/// \class CC31k - driver for CC3100 Wifi modules

typedef struct _cc31k_obj_t {
    mp_obj_base_t base;
} cc31k_obj_t;

STATIC const cc31k_obj_t cc31k_obj = {{(mp_obj_type_t*)&mod_network_nic_type_cc31k}};

/// \classmethod \constructor(spi, pin_cs, pin_en, pin_irq)
/// Initialise the CC3100 using the given SPI bus and pins and return a CC31k object.
//
// Note: pins were originally hard-coded to:
//      PYBv1.0: init(pyb.SPI(2), pyb.Pin.board.Y5, pyb.Pin.board.Y4, pyb.Pin.board.Y3)
//        [SPI on Y position; Y6=B13=SCK, Y7=B14=MISO, Y8=B15=MOSI]
//
//      STM32F4DISC: init(pyb.SPI(2), pyb.Pin.cpu.A15, pyb.Pin.cpu.B10, pyb.Pin.cpu.B11)
STATIC mp_obj_t cc31k_make_new(mp_obj_t type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 4, 4, false);

    // set the pins to use
    /*SpiInit(
        spi_get_handle(args[0]),
        pin_find(args[1]),
        pin_find(args[2]),
        pin_find(args[3])
    );*/

    wlan_connected = false;
    ip_obtained    = false;

    #if 1
    int rc = cc31k_init();
    if(rc != 0)
    {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed to init cc31k module"));
    }
    #endif

    /* obj is static so there is exactly 1 instance of it
    cc31k_obj_t *cc31k = m_new_obj(cc31k_obj_t);
    cc31k->base.type = (mp_obj_type_t*)&mod_network_nic_type_cc31k;
    */

    // register with network module
    mod_network_register_nic((mp_obj_t)&cc31k_obj);

    return (mp_obj_t)&cc31k_obj;
}

/// \method connect(ssid, key=None, *, security=WPA2, bssid=None)
STATIC mp_obj_t cc31k_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ,       {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key, MP_ARG_OBJ,                          {.u_obj = mp_const_none} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = SL_SEC_TYPE_WPA} },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ,       {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    mp_uint_t ssid_len;
    const char *ssid = mp_obj_str_get_data(args[0].u_obj, &ssid_len);

    // get key and sec
    mp_uint_t key_len = 0;
    const char *key = NULL;
    mp_uint_t sec = SL_SEC_TYPE_OPEN;
    if (args[1].u_obj != mp_const_none) {
        key = mp_obj_str_get_data(args[1].u_obj, &key_len);
        sec = args[2].u_int;
    }

#if 0
    // get bssid
    const char *bssid = NULL;
    if (args[3].u_obj != mp_const_none) {
        bssid = mp_obj_str_get_str(args[3].u_obj);
    }
#endif

    SlSecParams_t secParams = {0};
    int16_t retVal = 0;

    secParams.Key       = (signed char *)key;
    secParams.KeyLen    = key_len;
    secParams.Type      = sec;

    retVal = sl_WlanConnect((signed char *)ssid, ssid_len, 0, &secParams, 0);
    if (retVal < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_WlanConnect"));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc31k_connect_obj, 1, cc31k_connect);

STATIC mp_obj_t cc31k_disconnect(mp_obj_t self_in) {
    int ret = sl_WlanDisconnect();

    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_disconnect_obj, cc31k_disconnect);

STATIC mp_obj_t cc31k_isconnected(mp_obj_t self_in) {

    if (wlan_connected && ip_obtained) {
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_isconnected_obj, cc31k_isconnected);

STATIC mp_obj_t cc31k_update(mp_obj_t self_in) {

    _SlNonOsMainLoopTask();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_update_obj, cc31k_update);

STATIC mp_obj_t cc31k_sleep(mp_obj_t self_in) {

    sl_Stop(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_sleep_obj, cc31k_sleep);

STATIC mp_obj_t decode_addr(unsigned char *ip, int n_bytes)
{
    char data[64] = "";
    if(n_bytes == 4)
    {
        snprintf(data, 64, "%u.%u.%u.%u", ip[3], ip[2], ip[1], ip[0]);
    }
    else if(n_bytes == 6)
    {
        snprintf(data, 64, "%02x:%02x:%02x:%02x:%02x:%02x", ip[5], ip[4], ip[3], ip[2], ip[1], ip[0]);
    }
    else if(n_bytes == 32)
    {
        snprintf(data, 64, "%s", ip);
    }
    return(mp_obj_new_str(data, strlen(data), false));
}

STATIC void decode_addr_and_store(mp_obj_t object, qstr q_attr, unsigned char *ip, int n_bytes)
{
    mp_store_attr(object, q_attr, decode_addr(ip, n_bytes));
}

typedef struct _netapp_ipconfig_ret_args_t
{
    unsigned char aucIP[4];
    unsigned char aucSubnetMask[4];
    unsigned char aucDefaultGateway[4];
    unsigned char aucDHCPServer[4];
    unsigned char aucDNSServer[4];
    unsigned char uaMacAddr[6];
    unsigned char uaSSID[32];
} tNetappIpconfigRetArgs;

STATIC mp_obj_t cc31k_ifconfig(mp_obj_t self_in) {

    unsigned char len = sizeof(SlNetCfgIpV4Args_t);
    unsigned char dhcpIsOn = 0;
    SlNetCfgIpV4Args_t ipV4 = {0};
    STATIC mp_obj_t net_address_type = MP_OBJ_NULL;
    tNetappIpconfigRetArgs ipconfig  = {{0}};

    // NOTE: the return code upon success is not 0 (this is a bug in the documenation)
    int rc = sl_NetCfgGet(SL_IPV4_STA_P2P_CL_GET_INFO, &dhcpIsOn, &len, (uint8_t *)&ipV4);
    if (rc < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_NetCfgGet ip"));
    }

    uint8_t macAddressVal[SL_MAC_ADDR_LEN] = {0};
    uint8_t macAddressLen = SL_MAC_ADDR_LEN;
    rc = sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL, &macAddressLen, (uint8_t *)macAddressVal);
    if (rc < 0) {
        //printf("WRN: negative return code, but data is valid (TI bug)\n");
        //nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_NetCfgGet mac"));
    }

    //memset(ifconfig, 0, sizeof(tNetappIpconfigRetArgs));
    memcpy(ipconfig.aucIP,            &ipV4.ipV4,         4);
    memcpy(ipconfig.aucSubnetMask,    &ipV4.ipV4Mask,     4);
    memcpy(ipconfig.aucDefaultGateway,&ipV4.ipV4Gateway,  4);
    memcpy(ipconfig.aucDNSServer,     &ipV4.ipV4DnsServer,4);
    memcpy(ipconfig.uaMacAddr,        macAddressVal,      SL_MAC_ADDR_LEN);

    net_address_type = mp_obj_new_type(QSTR_FROM_STR_STATIC("NetIfConfig"), mp_const_empty_tuple, mp_obj_new_dict(0));
    // make a new NetAddress object
    mp_obj_t net_addr = mp_call_function_0(net_address_type);

    // fill the NetAddress object with data
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("ip"),      ipconfig.aucIP,             4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("subnet"),  ipconfig.aucSubnetMask,     4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("gateway"), ipconfig.aucDefaultGateway, 4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("dhcp"),    ipconfig.aucDHCPServer,     4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("dns"),     ipconfig.aucDNSServer,      4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("mac"),     ipconfig.uaMacAddr,         6);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("ssid"),    ipconfig.uaSSID,            32);
    return(net_addr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_ifconfig_obj, cc31k_ifconfig);

STATIC mp_obj_t cc31k_urn(uint n_args, const mp_obj_t *args) {
    int rc;
    char urn[MAX_DEVICE_URN_LEN];
    uint8_t len = MAX_DEVICE_URN_LEN;

    if(n_args == 2) { // Set the URN
        const char *p = mp_obj_str_get_str(args[1]);
        uint8_t len = strlen(p);

        // The call to sl_NetAppSet corrupts the input string URN=args[1], so we copy into a local buffer
        if (len > MAX_DEVICE_URN_LEN) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "URN too long"));
        }
        strcpy(urn, p);

        rc = sl_NetAppSet(SL_NET_APP_DEVICE_CONFIG_ID, NETAPP_SET_GET_DEV_CONF_OPT_DEVICE_URN, len, (unsigned char *)urn);
        if (rc < 0) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "sl_NetAppSet %d", rc));
        }
    }
    else {   // Get the URN
        rc = sl_NetAppGet(SL_NET_APP_DEVICE_CONFIG_ID, NETAPP_SET_GET_DEV_CONF_OPT_DEVICE_URN, &len, (uint8_t *)urn);
        if (rc < 0) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "sl_NetAppGet %d", rc));
        }
        return mp_obj_new_str(urn, len-1, false);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cc31k_urn_obj, 1, 2, cc31k_urn);

STATIC int cc31k_ver(SlVersionFull *sVersion)
{
  int lRetVal = 0;
  _u8 ucConfigOpt = 0;
  _u8 ucConfigLen = 0;

  /* Get the device's version-information */
  ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
  ucConfigLen = sizeof(SlVersionFull);
  lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, &ucConfigLen, (_u8 *)(sVersion));

  printf("NWP %ld.%ld.%ld.%ld Chip Fw %ld.%ld.%ld.%ld Phy %d.%d.%d.%d\n",
          sVersion->NwpVersion[0],
          sVersion->NwpVersion[1],
          sVersion->NwpVersion[2],
          sVersion->NwpVersion[3],
          sVersion->ChipFwAndPhyVersion.FwVersion[0],
          sVersion->ChipFwAndPhyVersion.FwVersion[1],
          sVersion->ChipFwAndPhyVersion.FwVersion[2],
          sVersion->ChipFwAndPhyVersion.FwVersion[3],
          sVersion->ChipFwAndPhyVersion.PhyVersion[0],
          sVersion->ChipFwAndPhyVersion.PhyVersion[1],
          sVersion->ChipFwAndPhyVersion.PhyVersion[2],
          sVersion->ChipFwAndPhyVersion.PhyVersion[3]);

  return lRetVal;
}

/******************************************************************************/
// Micro Python bindings; CC31k socket class

typedef struct _cc31k_socket_obj_t {
    mp_obj_base_t base;
    int fd;
} cc31k_socket_obj_t;

STATIC const mp_obj_type_t cc31k_socket_type;

STATIC const mp_map_elem_t cc31k_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),         (mp_obj_t)&cc31k_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_disconnect),      (mp_obj_t)&cc31k_disconnect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_isconnected),     (mp_obj_t)&cc31k_isconnected_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ifconfig),        (mp_obj_t)&cc31k_ifconfig_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update),          (mp_obj_t)&cc31k_update_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sleep),           (mp_obj_t)&cc31k_sleep_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_urn),             (mp_obj_t)&cc31k_urn_obj },

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_WEP),           MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA),           MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA2),          MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_ENT) },
};


STATIC MP_DEFINE_CONST_DICT(cc31k_locals_dict, cc31k_locals_dict_table);

#undef gethostbyname
#undef socket
#undef close
#undef bind
#undef listen
#undef accept
#undef connect
#undef send
#undef recv
#undef sendto
#undef recvfrom
#undef setsockopt

const mod_network_nic_type_t mod_network_nic_type_cc31k = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_CC31K,
        .make_new = cc31k_make_new,
        .locals_dict = (mp_obj_t)&cc31k_locals_dict,
    },
    .gethostbyname = cc31k_gethostbyname,
    .socket = cc31k_socket_socket,
    .close = cc31k_socket_close,
    .bind = cc31k_socket_bind,
    .listen = cc31k_socket_listen,
    .accept = cc31k_socket_accept,
    .connect = cc31k_socket_connect,
    .send = cc31k_socket_send,
    .recv = cc31k_socket_recv,
    .sendto = cc31k_socket_sendto,
    .recvfrom = cc31k_socket_recvfrom,
    .setsockopt = cc31k_socket_setsockopt,
    .settimeout = cc31k_socket_settimeout,
    .ioctl = cc31k_socket_ioctl,
};

// --------------------------------------------------------------------------------------
// CC31K Driver Interface
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// This function configures the SimpleLink device in its default state.
// - Sets the mode to STATION
// - Configures connection policy to Auto and AutoSmartConfig
// - Deletes all the stored profiles
// - Enables DHCP
// - Disables Scan policy
// - Sets Tx power to maximum
// - Sets power policy to normal
// - Unregisters mDNS services
//
// NOTE:
// * configure the device to default state by clearing the persistent settings stored
// * in NVMEM (viz. connection profiles & policies, power policy etc)
// *
// * the applications may choose to skip this step if the developer is sure
// * that the device is in its default state at start of application
// *
// * all profiles and persistent settings will be lost
// *
// --------------------------------------------------------------------------------------
STATIC int cc31k_init(void)
{
    //SlVersionFull   ver = {{0}};

    uint8_t           val = 1;
    uint8_t           configOpt = 0;
//    uint8_t           configLen = 0;
    uint8_t           power = 0;
    int32_t           retVal = -1;
    int32_t           mode = -1;

    mode = sl_Start(0, 0, 0);
    LOG_COND_RET(mode>=0, -1);

    // Need to check for station mode
    if(ROLE_STA != mode)
    {   // Configure the device into station mode
        if(ROLE_AP == mode)
        {
            LOG_INFO("mode: ROLE_AP");
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(ip_obtained == false)
            {
                _SlNonOsMainLoopTask();
            }
        }

        // Select station mode, and restart to activate it
        retVal = sl_WlanSetMode(ROLE_STA);
        LOG_COND_RET(retVal>=0, -1);
        retVal = sl_Stop(100);
        LOG_COND_RET(retVal>=0, -1);
        mode = sl_Start(0, 0, 0);
        LOG_COND_RET(mode>=0, -1);
        if(ROLE_STA != mode)
        {
            LOG_ERR("not in station mode");
            return(-1);
        }
    }

    #if 0
    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (unsigned char *)(&ver));
    LOG_COND_RET(retVal>=0, -1);
    #endif

    SlVersionFull sVersion;
    cc31k_ver(&sVersion);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    LOG_COND_RET(retVal>=0, -1);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {   // wait for disconnection
        while(wlan_connected == true)
        {
            _SlNonOsMainLoopTask();
        }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,(uint8_t *)&val);
    LOG_COND_RET(retVal>=0, -1);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);
    LOG_COND_RET(retVal>=0, -1);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    LOG_COND_RET(retVal>=0, -1);

    retVal = sl_Stop(100);
    LOG_COND_RET(retVal>=0, -1);

    /* Initializing the CC3100 device */
    retVal = sl_Start(0, 0, 0);
    if((retVal < 0) || (ROLE_STA != retVal))
    {
        LOG_ERR("");
        return(-1);
    }

    // Disable the HTTP server
    retVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
    if(retVal < 0)
    {
        LOG_ERR("");
    }

    retVal = 0; // all is ok

    return(retVal);
}

// --------------------------------------------------------------------------------------
// ASYNCHRONOUS EVENT HANDLERS
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
//    This function handles WLAN events
//
//    pWlanEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
            {
                //LOG_INFO("[WLAN EVENT] connect");
                wlan_connected = true;
                /*
                 * Information about the connected AP (like name, MAC etc) will be
                 * available in 'sl_protocol_wlanConnectAsyncResponse_t' - Applications
                 * can use it if required
                 *
                 * sl_protocol_wlanConnectAsyncResponse_t *pEventData = NULL;
                 * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
                 *
                 */
            }
            break;

        case SL_WLAN_DISCONNECT_EVENT:
            {
                //LOG_INFO("[WLAN EVENT] disconnect");
                // link down
                wlan_connected = false;
                ip_obtained = false;
                // Get the reason code
                slWlanConnectAsyncResponse_t *pEventData = NULL;
                pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

                if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
                {   // user initiated a disconnect request
                    //LOG_INFO("Device disconnected from the AP on application's request");
                }
                else
                {
                    LOG_INFO("Device disconnected from the AP on an ERROR..!!");
                }
            }
            break;

        case SL_WLAN_STA_CONNECTED_EVENT:
            LOG_INFO("[WLAN EVENT] SL_WLAN_STA_CONNECTED_EVENT");
            break;
        case SL_WLAN_STA_DISCONNECTED_EVENT:
            LOG_INFO("[WLAN EVENT] SL_WLAN_STA_DISCONNECTED_EVENT");
            break;
        case SL_WLAN_CONNECTION_FAILED_EVENT:
            LOG_INFO("[WLAN EVENT] SL_WLAN_CONNECTION_FAILED_EVENT");
            break;
        default:
            LOG_INFO("[WLAN EVENT] Unexpected event");
            printf("pWlanEvent->Event: %d\n", (int)pWlanEvent->Event);
            break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles events for IP address acquisition via DHCP
//
//    pNetAppEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
                //LOG_INFO("[NETAPP EVENT] SL_NETAPP_IPV4_IPACQUIRED_EVENT");
                ip_obtained = true;
                /*
                 * Information about the connection (like IP, gateway address etc)
                 * will be available in 'SlIpV4AcquiredAsync_t'
                 * Applications can use it if required
                 *
                 * SlIpV4AcquiredAsync_t *pEventData = NULL;
                 * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
                 *
                 */
            break;
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
            LOG_INFO("[NETAPP EVENT] SL_NETAPP_IPV6_IPACQUIRED_EVENT");
            break;
        case SL_NETAPP_IP_LEASED_EVENT:
            LOG_INFO("[NETAPP EVENT] SL_NETAPP_IP_LEASED_EVENT");
            break;
        case SL_NETAPP_IP_RELEASED_EVENT:
            LOG_INFO("[NETAPP EVENT] SL_NETAPP_IP_RELEASED_EVENT");
            // mark socket for closure
            cc31k_set_fd_closed_state(pNetAppEvent->EventData.sd);
            break;
        default:
            LOG_INFO("[NETAPP EVENT] Unexpected event");
            printf("pWlanEvent->Event: %d\n", (int)pNetAppEvent->Event);
            break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles callback for the HTTP server events
//
//    pServerEvent - Contains the relevant event information
//    pServerResponse - Should be filled by the user with the
//    relevant response information
//
// --------------------------------------------------------------------------------------
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
    LOG_INFO("[HTTP EVENT] Unexpected event");
}

// --------------------------------------------------------------------------------------
//    This function handles general error events indication
//
//    pDevEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */

    switch(pDevEvent->Event)
    {
        case SL_DEVICE_FATAL_ERROR_EVENT:
            LOG_INFO("[GENERAL EVENT] Fatal error: reset device");
            break;
        default:
            LOG_INFO("[GENERAL EVENT]");
            printf("%d\n", (int)pDevEvent->Event);
            break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles socket events indication
//
//    pSock is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    switch(pSock->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            LOG_INFO("[SOCK EVENT] SL_NETAPP_SOCKET_TX_FAILED");
            /*
             * TX Failed
             *
             * Information about the socket descriptor and status will be
             * available in 'SlSockEventData_t' - Applications can use it if
             * required
             *
             * SlSockEventData_t *pEventData = NULL;
             * pEventData = & pSock->EventData;
             */
            switch(pSock->EventData.status)
            {
                case SL_ECLOSE:
                    LOG_INFO("[SOCK EVENT] Close socket operation failed to transmit all queued packets");
                    break;
                default:
                    printf("[SOCK EVENT] Unexpected status %d", pSock->EventData.status);
                    break;
            }
            break;

        /*case SL_SOCKET_ASYNC_EVENT:
            LOG_INFO("[SOCK EVENT] SL_SOCKET_ASYNC_EVENT");
            break;*/

        default:
            printf("[SOCK EVENT] Unexpected event %d\n", (int)pSock->Event);
            break;
    }
}


