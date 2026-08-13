#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>





static int wiz_device_init(void)
{
    extern rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin);
    rt_hw_spi_device_attach("spi2", WIZ_SPI_DEVICE, 28);
    
    return RT_EOK;
}
INIT_DEVICE_EXPORT(wiz_device_init);
    
