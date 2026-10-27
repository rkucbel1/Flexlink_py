#pragma once
#include <stdint.h>
#include "lwip/err.h"
typedef uint16_t u16_t; struct tcp_pcb; struct pbuf{struct pbuf*next; void*payload; u16_t tot_len; u16_t len;};
typedef struct {int x;} ip_addr_t; extern ip_addr_t ip_addr_any_type;
#define IP_ANY_TYPE (&ip_addr_any_type)
#define IPADDR_TYPE_ANY 46
#define TCP_WRITE_FLAG_COPY 1
#define TCP_WRITE_FLAG_MORE 2
struct tcp_pcb* tcp_new_ip_type(int); err_t tcp_bind(struct tcp_pcb*,const ip_addr_t*,u16_t);
struct tcp_pcb* tcp_listen(struct tcp_pcb*);
void tcp_accept(struct tcp_pcb*, err_t(*)(void*,struct tcp_pcb*,err_t));
void tcp_recv(struct tcp_pcb*, err_t(*)(void*,struct tcp_pcb*,struct pbuf*,err_t));
void tcp_sent(struct tcp_pcb*, err_t(*)(void*,struct tcp_pcb*,u16_t));
void tcp_err(struct tcp_pcb*, void(*)(void*,err_t)); void tcp_arg(struct tcp_pcb*,void*);
u16_t tcp_sndbuf(struct tcp_pcb*); err_t tcp_write(struct tcp_pcb*,const void*,u16_t,uint8_t);
err_t tcp_output(struct tcp_pcb*); void tcp_recved(struct tcp_pcb*,u16_t); err_t tcp_close(struct tcp_pcb*);
void tcp_abort(struct tcp_pcb*); void tcp_nagle_disable(struct tcp_pcb*); uint8_t pbuf_free(struct pbuf*);
u16_t pbuf_copy_partial(const struct pbuf*,void*,u16_t,u16_t);
