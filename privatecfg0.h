#ifndef PRIVATECFGH
#define PRIVATECFGH

#include <irsdefs.h>

#include <irsfinal.h>

#define IP_0 192
#define IP_1 168
#define IP_2 0
#define IP_3 211

#define MAKE_IP(IP_0, IP_1, IP_2, IP_3) #IP_0 "." #IP_1 "." #IP_2 "." #IP_3
#define IP_STR MAKE_IP(IP_0, IP_1, IP_2, IP_3)

#endif //PRIVATECFGH
