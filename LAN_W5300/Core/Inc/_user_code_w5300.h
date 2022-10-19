#include "socket.h"
#include "wizchip_conf.h"


#define true					1
#define false					0

#define SOCK_TCPS       0
#define SOCK_UDPS       1
#define PORT_TCPS		5000
#define PORT_UDPS       3000




uint8_t wiznet_memsize[2][8] = {{8,8,8,8,8,8,8,8}, {8,8,8,8,8,8,8,8}};

#define ETH_MAX_BUF_SIZE		2048

uint8_t ethBuf0[ETH_MAX_BUF_SIZE];


wiz_NetInfo gWIZNETINFO = {
		.mac = {0x00, 0x08, 0xdc, 0, 0, 0},
		.ip = {172, 30, 1, 104},
		.sn = {255, 255, 0, 0},
		.gw = {172, 30, 1, 254},
		.dns = {0, 0, 0, 0},
		.dhcp = NETINFO_STATIC
};



void Reset_W5300()
{
	HAL_GPIO_WritePin(RESET_W5300_GPIO_Port, RESET_W5300_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_W5300_GPIO_Port, RESET_W5300_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void W5300_write(uint32_t addr, iodata_t wd)
{
	_W5300_DATA(addr) = wd;
}

iodata_t W5300_read(uint32_t addr)
{
	return _W5300_DATA(addr);
}




void _InitW5300(void);

void _InitW5300(void)
{
	unsigned int tmpaddr[4];

	Reset_W5300();
	reg_wizchip_bus_cbfunc(W5300_read, W5300_write);

	printf("getMR() = %04X\r\n", getMR());

	if (ctlwizchip(CW_INIT_WIZCHIP, (void*)wiznet_memsize) == -1)
	{
		printf("W5300 memory initialization failed\r\n");
	}

	ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);
	print_network_information();
}

