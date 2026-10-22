/* SPDX-License-Identifier: MIT
 * Narrow host substitute for the Nordic declarations used by SockIntrf.
 * Constants match Nordic's socket API; no radio or SDK implementation is used.
 */
#ifndef TEST_NRF91_SOCKET_H
#define TEST_NRF91_SOCKET_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define NRF_MODEM_MAX_SOCKET_COUNT 8
#define NRF_AF_UNSPEC 0
#define NRF_AF_INET 1
#define NRF_AF_INET6 2
#define NRF_SOCK_STREAM 1
#define NRF_SOCK_DGRAM 2
#define NRF_IPPROTO_TCP 6
#define NRF_IPPROTO_UDP 17
#define NRF_SPROTO_TLS1v2 258
#define NRF_SPROTO_DTLS1v2 273
#define NRF_SOL_SOCKET 1
#define NRF_SOL_SECURE 282
#define NRF_SO_SEC_TAG_LIST 1
#define NRF_SO_SEC_HOSTNAME 2
#define NRF_SO_SEC_PEER_VERIFY 5
#define NRF_SO_SEC_PEER_VERIFY_NONE 0
#define NRF_SO_SEC_PEER_VERIFY_REQUIRED 2
#define NRF_SO_SNDTIMEO 21
#define NRF_SO_POLLCB 60
#define NRF_SO_RAI 61
#define NRF_RAI_NO_DATA 1
#define NRF_RAI_LAST 2
#define NRF_RAI_ONE_RESP 3
#define NRF_RAI_ONGOING 4
#define NRF_RAI_WAIT_MORE 5
#define NRF_MSG_DONTWAIT 0x40
#define NRF_POLLIN 0x1
#define NRF_POLLOUT 0x4
#define NRF_POLLERR 0x8
#define NRF_POLLHUP 0x10
#define NRF_POLLNVAL 0x20

typedef uint32_t nrf_socklen_t;
typedef uint32_t nrf_nfds_t;
typedef uint32_t nrf_sec_tag_t;

struct nrf_in_addr
{
	uint32_t s_addr;
};
struct nrf_in6_addr
{
	uint8_t s6_addr[16];
};
struct nrf_sockaddr
{
	uint16_t sa_family;
	char sa_data[14];
};
struct nrf_sockaddr_in
{
	uint16_t sin_family;
	uint16_t sin_port;
	struct nrf_in_addr sin_addr;
	char sin_zero[8];
};
struct nrf_sockaddr_in6
{
	uint16_t sin6_family;
	uint16_t sin6_port;
	uint32_t sin6_flowinfo;
	struct nrf_in6_addr sin6_addr;
	uint32_t sin6_scope_id;
};
struct nrf_addrinfo
{
	int ai_flags;
	int ai_family;
	int ai_socktype;
	int ai_protocol;
	nrf_socklen_t ai_addrlen;
	struct nrf_sockaddr *ai_addr;
	char *ai_canonname;
	struct nrf_addrinfo *ai_next;
};
struct nrf_timeval
{
	uint32_t tv_sec;
	uint32_t tv_usec;
};
struct nrf_pollfd
{
	int fd;
	short events;
	short revents;
};
typedef void (*nrf_modem_pollcb_t)(struct nrf_pollfd *pFd);
struct nrf_modem_pollcb
{
	nrf_modem_pollcb_t callback;
	short events;
	bool oneshot;
};

static inline uint16_t nrf_htons(uint16_t Value)
{
	const uint16_t endian = 1;

	return *(const uint8_t *)&endian ? (uint16_t)(Value >> 8 | Value << 8) : Value;
}

#ifdef __cplusplus
extern "C"
{
#endif
int nrf_getaddrinfo(const char *pNode, const char *pService,
	const struct nrf_addrinfo *pHints, struct nrf_addrinfo **pResult);
void nrf_freeaddrinfo(struct nrf_addrinfo *pInfo);
int nrf_socket(int Family, int Type, int Protocol);
int nrf_setsockopt(int Socket, int Level, int Name, const void *pValue, nrf_socklen_t Len);
int nrf_bind(int Socket, const struct nrf_sockaddr *pAddress, nrf_socklen_t Len);
int nrf_connect(int Socket, const struct nrf_sockaddr *pAddress, nrf_socklen_t Len);
int nrf_close(int Socket);
int nrf_poll(struct nrf_pollfd *pFds, nrf_nfds_t Count, int Timeout);
ssize_t nrf_send(int Socket, const void *pBuffer, size_t Len, int Flags);
ssize_t nrf_recv(int Socket, void *pBuffer, size_t Len, int Flags);
#ifdef __cplusplus
}
#endif
#endif
