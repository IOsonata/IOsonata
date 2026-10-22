/* SPDX-License-Identifier: MIT
 * Nordic errno values used by the socket test, independent of host errno values.
 */
#ifndef TEST_NRF91_ERRNO_H
#define TEST_NRF91_ERRNO_H
#define NRF_EBADF 9
#define NRF_EAGAIN 11
#define NRF_ENOMEM 12
#define NRF_EINVAL 22
#define NRF_EMFILE 24
#define NRF_EPIPE 32
#define NRF_ECONNRESET 104
#define NRF_ENOBUFS 105
#define NRF_EAFNOSUPPORT 106
#define NRF_ENOPROTOOPT 109
#define NRF_ESHUTDOWN 110
#define NRF_ECONNREFUSED 111
#define NRF_EADDRINUSE 112
#define NRF_ECONNABORTED 113
#define NRF_ENETUNREACH 114
#define NRF_ENETDOWN 115
#define NRF_ETIMEDOUT 116
#define NRF_EHOSTDOWN 117
#define NRF_EINPROGRESS 119
#define NRF_ENOTCONN 128
#define NRF_EWOULDBLOCK NRF_EAGAIN
#endif
