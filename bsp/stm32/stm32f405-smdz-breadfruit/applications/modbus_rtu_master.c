/*
 * Modbus RTU Master 示例 (RT-Thread)
 * 基于 agile_modbus-v1.1.2，使用 UART4 (PC10/PC11) 通信
 *
 * 功能演示：
 *   1. 读保持寄存器 (功能码 0x03)
 *   2. 写单个寄存器 (功能码 0x06)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "agile_modbus.h"

#define DBG_TAG "modbus.master"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MODBUS_UART_NAME       "uart4"
#define MODBUS_SLAVE_ADDR      1        /* 目标从机地址 */
#define MODBUS_BAUD_TIMEOUT_MS 50       /* 接收总超时(ms) */
#define MODBUS_FRAME_GAP_MS    5        /* 帧间隔超时(ms)，判定一帧结束 */
#define MODBUS_BAUDRATE        9600   /* 波特率，必须与从机一致 */

/* RS485 方向控制开关：
 * 0 = 自动收发方向的收发器/模块（MAX13487、带自动方向电路的模块），无需 GPIO
 * 1 = 经典收发器（MAX485/SP3485 等），需要 GPIO 控制 DE/RE */
#define RS485_MANUAL_DE        1

#if RS485_MANUAL_DE
/* DE 与 /RE 通常短接在一起由一个 GPIO 控制：高 = 发送，低 = 接收
 * 引脚号按实际原理图修改（本板 PB1/PB2 为空闲 GPIO，可考虑使用） */
#define RS485_DE_PIN           15
#endif

static rt_device_t _modbus_serial = RT_NULL;

/* ==================== 串口移植层 ==================== */

/* 打开串口 */
static int modbus_serial_open(void)
{
    _modbus_serial = rt_device_find(MODBUS_UART_NAME);
    if (_modbus_serial == RT_NULL)
    {
        LOG_E("can't find %s", MODBUS_UART_NAME);
        return -RT_ERROR;
    }

    /* 中断接收模式，底层有环形缓冲区 */
    if (rt_device_open(_modbus_serial, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("open %s failed", MODBUS_UART_NAME);
        return -RT_ERROR;
    }

    /* 显式配置波特率 8N1，保证与 MODBUS_BAUDRATE 宏一致 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = MODBUS_BAUDRATE;
    rt_device_control(_modbus_serial, RT_DEVICE_CTRL_CONFIG, &config);

#if RS485_MANUAL_DE
    /* DE 引脚初始化，默认处于接收模式 */
    rt_pin_mode(RS485_DE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485_DE_PIN, PIN_LOW);//recieve mode
#endif

    return RT_EOK;
}

/* 清空接收缓冲区 */
static void modbus_serial_flush(void)
{
    uint8_t tmp[64];
    while (rt_device_read(_modbus_serial, 0, tmp, sizeof(tmp)) > 0)
        ;
}

/* 发送数据 */
static void modbus_serial_send(const uint8_t *buf, int len)
{
#if RS485_MANUAL_DE
    rt_pin_write(RS485_DE_PIN, PIN_HIGH);   /* 切到发送模式 */
#endif

    rt_device_write(_modbus_serial, 0, buf, len);

    /* 关键陷阱：rt_device_write 返回 ≠ 发送完成！
     * 此时最后一字节可能还在 UART 发送移位寄存器里没移出去，
     * 如果立刻拉低 DE 会截断帧尾导致 CRC 错。
     * 按 8N1 每字节 10 bit 计算整帧时间，加 2ms 余量 */
    int frame_ms = len * 10 * 1000 / MODBUS_BAUDRATE + 2;
    rt_thread_mdelay(frame_ms);

#if RS485_MANUAL_DE
    rt_pin_write(RS485_DE_PIN, PIN_LOW);    /* 切回接收模式 */
#endif

    /* 若收发器/模块会把发送数据回环到 RX，清掉回环的请求帧，
     * 避免它被误当成从机响应（从机响应通常 >5ms 后才到，此处清空安全） */
    modbus_serial_flush();
}

/*
 * 等待数据接收结束
 * 原理：循环读取，如果两次读取之间出现 MODBUS_FRAME_GAP_MS 的空闲，
 *       说明一帧接收完毕。总超时 MODBUS_BAUD_TIMEOUT_MS。
 *
 * 返回：收到的字节数；0=超时；负数=错误
 */
static int modbus_serial_receive(uint8_t *buf, int bufsz)
{
    int total = 0;
    rt_tick_t start = rt_tick_get();
    rt_tick_t last_recv = start;

    while (1)
    {
        if (total < bufsz)
        {
            int rd = rt_device_read(_modbus_serial, 0, buf + total, bufsz - total);
            if (rd > 0)
            {
                total += rd;
                last_recv = rt_tick_get();
            }
        }

        /* 帧间隔超时：收到过数据且静默超过 MODBUS_FRAME_GAP_MS → 一帧结束 */
        if (total > 0 &&
            (rt_tick_get() - last_recv) * 1000 / RT_TICK_PER_SECOND >= MODBUS_FRAME_GAP_MS)
            break;

        /* 总超时 */
        if ((rt_tick_get() - start) * 1000 / RT_TICK_PER_SECOND >= MODBUS_BAUD_TIMEOUT_MS)
            break;

        rt_thread_mdelay(1);
    }

    return total;
}

/* ==================== Modbus 操作封装 ==================== */

/*
 * 读保持寄存器 (功能码 0x03)
 * addr: 寄存器起始地址; nb: 数量(1~125); dest: 存放结果的 uint16_t 数组
 * 返回: >=0 成功(读取的寄存器数); <0 失败
 */
static int modbus_read_holding_regs(int slave, int addr, int nb, uint16_t *dest)
{
    agile_modbus_rtu_t ctx_rtu;
    uint8_t ctx_send_buf[AGILE_MODBUS_MAX_ADU_LENGTH];
    uint8_t ctx_read_buf[AGILE_MODBUS_MAX_ADU_LENGTH];

    agile_modbus_t *ctx = &ctx_rtu._ctx;
    agile_modbus_rtu_init(&ctx_rtu, ctx_send_buf, sizeof(ctx_send_buf),
                          ctx_read_buf, sizeof(ctx_read_buf));
    agile_modbus_set_slave(ctx, slave);

    /* 1. 清空接收缓冲 → 2. 打包请求 → 3. 发送 → 4. 等接收 → 5. 解包 */
    modbus_serial_flush();

    int send_len = agile_modbus_serialize_read_registers(ctx, addr, nb);
    if (send_len < 0)
    {
        LOG_E("serialize read registers failed");
        return -1;
    }

    modbus_serial_send(ctx->send_buf, send_len);

    int read_len = modbus_serial_receive(ctx->read_buf, ctx->read_bufsz);
    if (read_len <= 0)
    {
        LOG_W("receive timeout (slave:%d addr:%d)", slave, addr);
        return -1;
    }

    int rc = agile_modbus_deserialize_read_registers(ctx, read_len, dest);
    if (rc < 0)
    {
        if (rc != -1)
            LOG_W("exception code: %d", -128 - rc);
        else
            LOG_W("deserialize failed");
        return -1;
    }

    return rc;
}

/*
 * 写单个寄存器 (功能码 0x06)
 * addr: 寄存器地址; value: 要写入的值
 * 返回: 0 成功; <0 失败
 */
static int modbus_write_single_reg(int slave, int addr, uint16_t value)
{
    agile_modbus_rtu_t ctx_rtu;
    uint8_t ctx_send_buf[AGILE_MODBUS_MAX_ADU_LENGTH];
    uint8_t ctx_read_buf[AGILE_MODBUS_MAX_ADU_LENGTH];

    agile_modbus_t *ctx = &ctx_rtu._ctx;
    agile_modbus_rtu_init(&ctx_rtu, ctx_send_buf, sizeof(ctx_send_buf),
                          ctx_read_buf, sizeof(ctx_read_buf));
    agile_modbus_set_slave(ctx, slave);

    modbus_serial_flush();

    int send_len = agile_modbus_serialize_write_register(ctx, addr, value);
    if (send_len < 0)
        return -1;

    modbus_serial_send(ctx->send_buf, send_len);

    int read_len = modbus_serial_receive(ctx->read_buf, ctx->read_bufsz);
    if (read_len <= 0)
        return -1;

    int rc = agile_modbus_deserialize_write_register(ctx, read_len);
    if (rc < 0)
    {
        if (rc != -1)
            LOG_W("exception code: %d", -128 - rc);
        return -1;
    }

    return 0;
}

/* ==================== 主线程 ==================== */

static void modbus_master_entry(void *param)
{
    uint16_t regs[10] = {0};

    if (modbus_serial_open() != RT_EOK)
        return;

    LOG_I("Modbus RTU Master started on %s", MODBUS_UART_NAME);
    rt_thread_mdelay(500);

    while (1)
    {
        /* 读保持寄存器：从地址 0 开始读 5 个 */
        int rc = modbus_read_holding_regs(MODBUS_SLAVE_ADDR, 0, 5, regs);
        if (rc >= 0)
        {
            for (int i = 0; i < rc; i++)
                LOG_I("reg[%d] = 0x%04X (%d)", i, regs[i], regs[i]);
        }

        /* 写单个寄存器：向地址 0 写入一个值 */
        static uint16_t wval = 100;
        if (modbus_write_single_reg(MODBUS_SLAVE_ADDR, 0, wval) == 0)
        {
            LOG_I("write reg[0] = %d ok", wval);
            wval += 10;
        }

        rt_thread_mdelay(2000);
    }
}

/* 自动初始化线程 */
static int modbus_master_init(void)
{
    rt_thread_t tid = rt_thread_create("mb_master", modbus_master_entry,
                                       RT_NULL, 2048, 15, 20);
    if (tid)
        rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(modbus_master_init);
