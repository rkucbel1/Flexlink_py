/* Co-simulation harness: pd_server.c over a real TCP socket with fake lwIP; DMA and registers in pl_model.cpp (Verilated RTL). */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "xil_types.h"
#include "xaxidma.h"
#include "lwip/tcp.h"
int start_application(void); void print_app_header(void);
ip_addr_t ip_addr_any_type;
static int fd=-1; struct tcp_pcb { int dummy; } the_pcb, listen_pcb;
static err_t (*acc)(void*,struct tcp_pcb*,err_t); static err_t (*rcv)(void*,struct tcp_pcb*,struct pbuf*,err_t);
static err_t (*snt)(void*,struct tcp_pcb*,u16_t); static void (*erf)(void*,err_t);
static unsigned char *out; static size_t out_len, out_cap; static int closed;
void xil_printf(const char*f,...){va_list a;va_start(a,f);vprintf(f,a);va_end(a);fflush(stdout);}
void Xil_DCacheFlushRange(UINTPTR a,u32 n){} void Xil_DCacheInvalidateRange(UINTPTR a,u32 n){}
struct tcp_pcb* tcp_new_ip_type(int t){return &listen_pcb;} err_t tcp_bind(struct tcp_pcb*p,const ip_addr_t*a,u16_t port){return 0;}
struct tcp_pcb* tcp_listen(struct tcp_pcb*p){return p;}
void tcp_accept(struct tcp_pcb*p, err_t(*f)(void*,struct tcp_pcb*,err_t)){acc=f;}
void tcp_recv(struct tcp_pcb*p, err_t(*f)(void*,struct tcp_pcb*,struct pbuf*,err_t)){rcv=f;}
void tcp_sent(struct tcp_pcb*p, err_t(*f)(void*,struct tcp_pcb*,u16_t)){snt=f;}
void tcp_err(struct tcp_pcb*p, void(*f)(void*,err_t)){erf=f;} void tcp_arg(struct tcp_pcb*p,void*a){}
u16_t tcp_sndbuf(struct tcp_pcb*p){ return (u16_t)(1 + rand()%8192); }
err_t tcp_write(struct tcp_pcb*p,const void*d,u16_t n,uint8_t fl){ if(rand()%7==0) return ERR_MEM;
  if(out_len+n>out_cap){out_cap=(out_len+n)*2;out=realloc(out,out_cap);} memcpy(out+out_len,d,n); out_len+=n; return ERR_OK;}
err_t tcp_output(struct tcp_pcb*p){ size_t o=0; while(o<out_len){ssize_t k=send(fd,out+o,out_len-o,0); if(k<=0) break; o+=k;} out_len=0; return 0;}
void tcp_recved(struct tcp_pcb*p,u16_t n){} err_t tcp_close(struct tcp_pcb*p){closed=1;return 0;} void tcp_abort(struct tcp_pcb*p){closed=1;}
void tcp_nagle_disable(struct tcp_pcb*p){} uint8_t pbuf_free(struct pbuf*p){return 1;}
u16_t pbuf_copy_partial(const struct pbuf*p,void*d,u16_t n,u16_t off){memcpy(d,(char*)p->payload+off,n);return n;}
int main(int argc,char**argv){
  int port=argc>1?atoi(argv[1]):5101; srand(1);
  if(mmap((void*)0x10000000,0x10000000,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0)!=(void*)0x10000000){perror("mmap");return 1;}
  print_app_header(); if(start_application()) return 1;
  int s=socket(AF_INET,SOCK_STREAM,0),one=1; setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&one,4);
  struct sockaddr_in a={0}; a.sin_family=AF_INET; a.sin_port=htons(port); bind(s,(void*)&a,sizeof a); listen(s,1);
  static unsigned char buf[65535];
  for(;;){ fd=accept(s,0,0); setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,4); closed=0; acc(0,&the_pcb,ERR_OK);
    while(!closed){ int want=1+rand()%5840; ssize_t k=recv(fd,buf,want,0);
      if(k<=0){ rcv(0,&the_pcb,NULL,ERR_OK); break; }
      struct pbuf p={0,buf,(u16_t)k,(u16_t)k}; err_t e=rcv(0,&the_pcb,&p,ERR_OK); if(e==ERR_ABRT) break;
      for(int g=0; g<100000 && snt; g++){ snt(0,&the_pcb,1); } /* pump: sent callbacks until reply done */
    }
    close(fd); }
}
