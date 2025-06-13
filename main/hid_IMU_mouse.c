// // hid_imu_mouse.c – BTstack HID mouse controlado por MPU-6050
// // Compilar: adicione este arquivo ao CMakeLists.txt exatamente como fazia com hid_buttons.c

// #include <stdint.h>
// #include <stdio.h>
// #include <string.h>
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"
// #include "hardware/watchdog.h"
// #include "btstack.h"

// /* -----------------------------------------------------------
//  *  Hardware            – ajuste se necessário
//  * ----------------------------------------------------------- */
// #define LED_PIN            11                // LED debug
// #define I2C_PORT           i2c_default
// #define I2C_SDA_PIN         4
// #define I2C_SCL_PIN         5
// #define MPU_ADDR           0x68              // endereço I²C padrão

// /* -----------------------------------------------------------
//  *  MPU-6050 – registradores essenciais
//  * ----------------------------------------------------------- */
// #define MPU_REG_PWR_MGMT_1  0x6B
// #define MPU_REG_GYRO_CFG    0x1B
// #define MPU_REG_ACCEL_CFG   0x1C
// #define MPU_REG_DATA_START  0x3B             // Acelerômetro X MSB

// /* -----------------------------------------------------------
//  *  HID – descritor de mouse (Report ID = 1, 4 bytes)
//  * ----------------------------------------------------------- */
// #define REPORT_ID           0x01
// const uint8_t HID_DESC[] = {
//     0x05,0x01,                    // Usage Page (Generic Desktop)
//     0x09,0x02,                    // Usage (Mouse)
//     0xA1,0x01,                    // Collection (Application)
//         0x85,REPORT_ID,           //   Report ID
//         0x09,0x01,                //   Usage (Pointer)
//         0xA1,0x00,                //   Collection (Physical)
//             0x05,0x09,            //     Usage Page (Buttons)
//             0x19,0x01,            //     Usage Minimum (1)
//             0x29,0x03,            //     Usage Maximum (3)
//             0x15,0x00,            //     Logical Min (0)
//             0x25,0x01,            //     Logical Max (1)
//             0x95,0x03,            //     Report Count (3)
//             0x75,0x01,            //     Report Size (1)
//             0x81,0x02,            //     Input (Data,Var,Abs)
//             0x95,0x01,            //     Report Count (1)
//             0x75,0x05,            //     Report Size (5)
//             0x81,0x01,            //     Input (Cnst,Ary,Abs) – padding
//             0x05,0x01,            //     Usage Page (Generic Desktop)
//             0x09,0x30,            //     Usage (X)
//             0x09,0x31,            //     Usage (Y)
//             0x09,0x38,            //     Usage (Wheel) – opcional
//             0x15,0x81,            //     Logical Min (-127)
//             0x25,0x7F,            //     Logical Max (127)
//             0x75,0x08,            //     Report Size (8)
//             0x95,0x03,            //     Report Count (3)
//             0x81,0x06,            //     Input (Data,Var,Rel)
//         0xC0,                     //   End Collection
//     0xC0                          // End Collection
// };

// /* -----------------------------------------------------------
//  *  Estado da aplicação
//  * ----------------------------------------------------------- */
// static btstack_timer_source_t led_tmr, imu_tmr;
// static enum { APP_BOOT, APP_IDLE, APP_CONN } app_state = APP_BOOT;
// static bool led_state = false;

// static uint16_t hid_cid      = 0;
// static bool     pending_rep  = false;
// static int8_t   dx_accum     = 0;
// static int8_t   dy_accum     = 0;

// /* -----------------------------------------------------------
//  *  Utilidades de GPIO
//  * ----------------------------------------------------------- */
// static inline void led_set(bool on){ gpio_put(LED_PIN, on); }

// /* -----------------------------------------------------------
//  *  I²C util
//  * ----------------------------------------------------------- */
// static inline void mpu_write(uint8_t reg, uint8_t val){
//     uint8_t buf[2] = { reg, val };
//     i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
// }
// static inline void mpu_read(uint8_t reg, uint8_t *dst, size_t len){
//     i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
//     i2c_read_blocking (I2C_PORT, MPU_ADDR, dst, len, false);
// }

// /* -----------------------------------------------------------
//  *  Inicialização do MPU-6050
//  * ----------------------------------------------------------- */
// static void mpu_init(void){
//     // Acorda o sensor
//     mpu_write(MPU_REG_PWR_MGMT_1, 0x00);      // clear sleep bit
//     sleep_ms(100);
//     // Gyr ±250 °/s (FS_SEL=0) – sens 131 LSB/°/s
//     mpu_write(MPU_REG_GYRO_CFG, 0x00);
//     // Acc ±2 g        (AFS_SEL=0) – sens 16384 LSB/g
//     mpu_write(MPU_REG_ACCEL_CFG, 0x00);
// }

// /* -----------------------------------------------------------
//  *  Conversão simples gyro → Δ pixel
//  *  (ajuste GAIN para controlar sensibilidade)
//  * ----------------------------------------------------------- */
// #define GYRO_GAIN   (1.0f / 500.0f)  // empiricamente ≈ pix por LSB*Δt (com Δt~10 ms)

// static void imu_poll(btstack_timer_source_t *ts){
//     watchdog_update();

//     uint8_t raw[14];
//     mpu_read(MPU_REG_DATA_START, raw, 14);

//     int16_t gx = (raw[ 8]<<8)|raw[ 9];   // gyro X
//     int16_t gy = (raw[10]<<8)|raw[11];   // gyro Y

//     /* integra (aprox) velocidade angular * Δt ≈ deslocamento
//        Δt fixo de 10 ms (timer abaixo)  */
//     float fx =  gx * GYRO_GAIN;          // rad/deg → pixels
//     float fy = -gy * GYRO_GAIN;          // inverter eixo para UX padrão

//     dx_accum += (int8_t)fx;
//     dy_accum += (int8_t)fy;

//     dx_accum = (dx_accum> 127)? 127: (dx_accum<-127)?-127:dx_accum;
//     dy_accum = (dy_accum> 127)? 127: (dy_accum<-127)?-127:dy_accum;

//     if((dx_accum || dy_accum) && !pending_rep && app_state==APP_CONN){
//         pending_rep = true;
//         hid_device_request_can_send_now_event(hid_cid);
//     }

//     btstack_run_loop_set_timer(ts, 10);      // 100 Hz
//     btstack_run_loop_add_timer(ts);
// }

// /* -----------------------------------------------------------
//  *  LED heartbeat
//  * ----------------------------------------------------------- */
// static void led_tick(btstack_timer_source_t *ts){
//     watchdog_update();
//     led_state = !led_state;
//     led_set(led_state);
//     btstack_run_loop_set_timer(ts, 500);
//     btstack_run_loop_add_timer(ts);
// }
// static void led_blink_start(void){
//     btstack_run_loop_remove_timer(&led_tmr);
//     led_tmr.process = led_tick;
//     btstack_run_loop_set_timer(&led_tmr, 0);
//     btstack_run_loop_add_timer(&led_tmr);
// }
// static void led_on(void){
//     btstack_run_loop_remove_timer(&led_tmr);
//     led_set(true);
// }

// /* -----------------------------------------------------------
//  *  Envio de relatório HID
//  * ----------------------------------------------------------- */
// static void send_mouse_report(int8_t btn, int8_t dx, int8_t dy, int8_t wheel){
//     uint8_t msg[] = { 0xA1, REPORT_ID, btn, dx, dy, wheel };
//     hid_device_send_interrupt_message(hid_cid, msg, sizeof msg);
// }

// /* -----------------------------------------------------------
//  *  Handler de pacotes BTstack
//  * ----------------------------------------------------------- */
// static void pk_handler(uint8_t type, uint16_t ch, uint8_t *pkt, uint16_t sz){
//     (void)ch; (void)sz;
//     if(type != HCI_EVENT_PACKET) return;

//     switch(hci_event_packet_get_type(pkt)){
//     case BTSTACK_EVENT_STATE:
//         if(btstack_event_state_get_state(pkt)==HCI_STATE_WORKING){
//             app_state = APP_IDLE;
//             led_blink_start();
//         }
//         break;

//     case HCI_EVENT_HID_META:
//         switch(hci_event_hid_meta_get_subevent_code(pkt)){
//         case HID_SUBEVENT_CONNECTION_OPENED:
//             if(hid_subevent_connection_opened_get_status(pkt)){
//                 app_state = APP_IDLE; hid_cid = 0; led_blink_start();
//             } else {
//                 app_state = APP_CONN;
//                 hid_cid   = hid_subevent_connection_opened_get_hid_cid(pkt);
//                 led_on();
//             }
//             break;

//         case HID_SUBEVENT_CONNECTION_CLOSED:
//             app_state = APP_IDLE; hid_cid = 0; led_blink_start();
//             break;

//         case HID_SUBEVENT_CAN_SEND_NOW:
//             if(pending_rep){
//                 send_mouse_report(0, dx_accum, dy_accum, 0);
//                 dx_accum = dy_accum = 0;
//                 pending_rep = false;
//             } else {
//                 // envia 0 movimento para soltar botões se necessário
//                 send_mouse_report(0,0,0,0);
//             }
//             break;
//         default: break;
//         }
//         break;
//     default: break;
//     }
// }

// /* -----------------------------------------------------------
//  *  Main – configuração geral
//  * ----------------------------------------------------------- */
// #define WDT_TIMEOUT_MS 3000

// int btstack_main(int, const char**){
//     stdio_init_all();

//     /* LED */
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     led_set(false);

//     /* I²C */
//     i2c_init(I2C_PORT, 400 * 1000);
//     gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA_PIN);
//     gpio_pull_up(I2C_SCL_PIN);
//     sleep_ms(50);
//     mpu_init();

//     /* Timers */
//     imu_tmr.process = imu_poll;
//     btstack_run_loop_set_timer(&imu_tmr, 10);
//     btstack_run_loop_add_timer(&imu_tmr);
//     led_blink_start();

//     /* Watchdog */
//     watchdog_enable(WDT_TIMEOUT_MS, 0);
//     watchdog_update();

//     /* BTstack */
//     gap_discoverable_control(1);
//     gap_set_class_of_device(0x2580);                  // Mouse
//     gap_set_local_name("PicoW-HID-Mouse");
//     gap_set_default_link_policy_settings(
//         LM_LINK_POLICY_ENABLE_ROLE_SWITCH |
//         LM_LINK_POLICY_ENABLE_SNIFF_MODE);
//     gap_set_allow_role_switch(true);

//     l2cap_init(); sdp_init();

//     static uint8_t sdp_buf[300];
//     static hid_sdp_record_t rec = {
//         0x2580,           // COD
//         25,               // HID version 2.5
//         1,1,              // country code, virtual cable
//         1,1,              // reconnect, boot device
//         0,                // normally connectable
//         1600,3200,3200,   // intervals
//         HID_DESC,sizeof(HID_DESC),
//         "PicoW-HID-Mouse"
//     };
//     hid_create_sdp_record(sdp_buf, sdp_create_service_record_handle(), &rec);
//     sdp_register_service(sdp_buf);

//     hid_device_init(0, sizeof(HID_DESC), HID_DESC);
//     hid_device_register_packet_handler(pk_handler);

//     hci_power_control(HCI_POWER_ON);
//     return 0;
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*****************************************************************
 *  Raspberry Pi Pico W – HID Mouse via Bluetooth LE + MPU-6050  *
 *  Usa a lib mpu6050 (diretório mpu6050/ do .zip fornecido)     *
 *****************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "btstack.h"

/* --- biblioteca do sensor --- */
#include "mpu6050.h"

/* -----------------------------------------------------------
*  Hardware – ajuste se necessário
* ----------------------------------------------------------- */
#define LED_PIN       11
#define I2C_PORT      i2c_default
#define I2C_SDA_PIN    4
#define I2C_SCL_PIN    5

/* -----------------------------------------------------------
*  HID – descritor de mouse (Report ID = 1, 4 bytes)
* ----------------------------------------------------------- */
#define REPORT_ID  0x01
static const uint8_t HID_DESC[] = {
    0x05,0x01,0x09,0x02,0xA1,0x01,               /* Generic Desktop / Mouse */
        0x85,REPORT_ID,0x09,0x01,0xA1,0x00,       /* Report ID / Pointer     */
            0x05,0x09,0x19,0x01,0x29,0x03,
            0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,
            0x95,0x01,0x75,0x05,0x81,0x01,        /* 3 botões + padding      */
            0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
            0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x03,0x81,0x06,
        0xC0,
    0xC0
};

/* -----------------------------------------------------------
*  Estado global
* ----------------------------------------------------------- */
static btstack_timer_source_t led_tmr, imu_tmr;
static enum { APP_BOOT, APP_IDLE, APP_CONN } app_state = APP_BOOT;

static uint16_t hid_cid   = 0;
static bool     rep_ready = false;
static int8_t   dx_accum  = 0;
static int8_t   dy_accum  = 0;

/* -----------------------------------------------------------
*  Instância do sensor
* ----------------------------------------------------------- */
static imu_c mpu;               // struct definida na lib

/* -----------------------------------------------------------
*  Utilidades
* ----------------------------------------------------------- */
static inline void led_set(bool v){ gpio_put(LED_PIN, v); }

/* -----------------------------------------------------------
*  Conversão gyro → pixel
* ----------------------------------------------------------- */
#define GYRO_GAIN   (1.0f / 500.0f)     /* sensibilidade */

static void imu_poll(btstack_timer_source_t *ts)
{
    watchdog_update();

    int16_t gyro[3];
    if (mpu6050_read_gyro(mpu, gyro)) {
        /*  
         *  gyro[2] → yaw  → ΔX  (gira à direita ⇒ cursor direita)
         *  gyro[0] → pitch → ΔY  (nariz abaixa  ⇒ cursor sobe)
         */
        float fx = -gyro[2] * GYRO_GAIN;   /* yaw controla horizontal */
        float fy = -gyro[0] * GYRO_GAIN;   /* pitch invertido p/ vertical */

        dx_accum += (int8_t)fx;
        dy_accum += (int8_t)fy;

        /* saturação para intervalo HID */
        dx_accum = (dx_accum >  127) ?  127 : (dx_accum < -127) ? -127 : dx_accum;
        dy_accum = (dy_accum >  127) ?  127 : (dy_accum < -127) ? -127 : dy_accum;

        if ((dx_accum || dy_accum) && !rep_ready && app_state == APP_CONN) {
            rep_ready = true;
            hid_device_request_can_send_now_event(hid_cid);
        }
    }

    btstack_run_loop_set_timer(ts, 10);    /* 100 Hz            */
    btstack_run_loop_add_timer(ts);
}

/* -----------------------------------------------------------
*  LED heartbeat
* ----------------------------------------------------------- */
static void led_tick(btstack_timer_source_t *ts){
    static bool on = false;
    watchdog_update();
    on = !on; led_set(on);
    btstack_run_loop_set_timer(ts, 500);
    btstack_run_loop_add_timer(ts);
}
static void led_blink_start(void){
    btstack_run_loop_remove_timer(&led_tmr);
    led_tmr.process = led_tick;
    btstack_run_loop_set_timer(&led_tmr, 0);
    btstack_run_loop_add_timer(&led_tmr);
}
static void led_on(void){
    btstack_run_loop_remove_timer(&led_tmr);
    led_set(true);
}

/* -----------------------------------------------------------
*  Envio de relatório HID
* ----------------------------------------------------------- */
static void send_mouse_report(int8_t buttons, int8_t dx, int8_t dy, int8_t wheel){
    uint8_t pkt[] = { 0xA1, REPORT_ID, buttons, dx, dy, wheel };
    hid_device_send_interrupt_message(hid_cid, pkt, sizeof pkt);
}

/* -----------------------------------------------------------
*  Handler BTstack
* ----------------------------------------------------------- */
static void pk_handler(uint8_t type, uint16_t ch, uint8_t *packet, uint16_t size){
    (void)ch; (void)size;
    if (type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)){
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
            app_state = APP_IDLE;
            led_blink_start();
        }
        break;

    case HCI_EVENT_HID_META:
        switch (hci_event_hid_meta_get_subevent_code(packet)){
        case HID_SUBEVENT_CONNECTION_OPENED:
            if (hid_subevent_connection_opened_get_status(packet)){
                app_state = APP_IDLE; hid_cid = 0; led_blink_start();
            } else {
                app_state = APP_CONN;
                hid_cid   = hid_subevent_connection_opened_get_hid_cid(packet);
                led_on();
            }
            break;

        case HID_SUBEVENT_CONNECTION_CLOSED:
            app_state = APP_IDLE; hid_cid = 0; led_blink_start();
            break;

        case HID_SUBEVENT_CAN_SEND_NOW:
            if (rep_ready){
                send_mouse_report(0, dx_accum, dy_accum, 0);
                dx_accum = dy_accum = 0;
                rep_ready = false;
            } else {
                send_mouse_report(0,0,0,0);  /* solta botões */
            }
            break;
        default: break;
        }
        break;
    default: break;
    }
}

/* -----------------------------------------------------------
*  Main
* ----------------------------------------------------------- */
#define WDT_TIMEOUT_MS 3000

int btstack_main(int, const char **)
{
    stdio_init_all();

    /* LED */
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); led_set(false);

    /* --- configurar estrutura do sensor -------------------- */
    mpu6050_set_config(&mpu, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 0); /* acc_scale=±2 g */
    if (!mpu6050_init(mpu)){
        printf("MPU-6050 não encontrado!\n");
    }

    /* Timers */
    imu_tmr.process = imu_poll;
    btstack_run_loop_set_timer(&imu_tmr, 10);
    btstack_run_loop_add_timer(&imu_tmr);
    led_blink_start();

    /* Watchdog */
    watchdog_enable(WDT_TIMEOUT_MS, 0);
    watchdog_update();

    /* BTstack básico */
    gap_discoverable_control(1);
    gap_set_class_of_device(0x2580);           /* Mouse */
    gap_set_local_name("PicoW-IMU-Mouse");
    gap_set_default_link_policy_settings(
        LM_LINK_POLICY_ENABLE_ROLE_SWITCH |
        LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    l2cap_init();  sdp_init();

    static uint8_t sdp_buf[300];
    static hid_sdp_record_t rec = {
        0x2580, 25, 1,1, 1,1, 0, 1600, 3200, 3200,
        HID_DESC, sizeof HID_DESC,
        "PicoW-IMU-Mouse"
    };
    hid_create_sdp_record(sdp_buf, sdp_create_service_record_handle(), &rec);
    sdp_register_service(sdp_buf);

    hid_device_init(0, sizeof HID_DESC, HID_DESC);
    hid_device_register_packet_handler(pk_handler);

    hci_power_control(HCI_POWER_ON);
    return 0;
}
