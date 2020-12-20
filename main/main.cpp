// https://github.com/Silvanosky/PositionEstimator/tree/master/position/main  # similar project

#include <stdio.h>
#include "sdkconfig.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "driver/gpio.h"
#include "i2cw.h"
#include "ICM20948.h"
// #include "timerw.h"
// #include "driver/uart.h"
#include "driver/i2c.h"

// #include "MadgwickAHRS.h"


// void qmul(float p[], float q[], float *pq) {
//     pq[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
//     pq[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
//     pq[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
//     pq[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
// }


static void control_loop(void *arg)
{
    // setup 

    // Stopwatch timer;
    I2CMaster i2c(0, 26, 25, 400000); // configure i2c bus
    ICM20948 icm(i2c);


    // uart_config_t uart_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_APB,
    // };
    // uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0);
    // uart_param_config(UART_NUM_0, &uart_config);

    // const unsigned int N = 7;
    // float imudata[N];
    // float magdata[3];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t Ts = 50;
    // float imucal[] = {0.f,0.f,0.f,0.f,0.f,0.f,0.f};

    // calibrate gyroscope/accelerometer
    // uint K=100;
    // timer.start();
    // for(uint k=0; k<K; k++) {
    //     imu.read_imu(imudata+1);
    //     imudata[0] = timer.stop();
    //     for (uint n=0; n<N; n++)
    //         imucal[n] += imudata[n]/K;
    //     timer.start();
    //     vTaskDelayUntil( &xLastWakeTime, Ts );
    //     printf("cal %d\n", k);
    // }
    // printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", imucal[0], imucal[1], imucal[2], imucal[3], imucal[4], imucal[5], imucal[6]);

    // quaternion angular velocity integration
    // typedef float ftype;
    // const ftype rad = 4.*atan(1.)/180.;
    // ftype q[4], p[4], q0[4] = {1.,0.,0.,0.};
    // ftype s, dtheta, dt_2, nx, ny, nz;
    // timer.start();

    float d[10];
    while(1) {
        icm.readSensors(d);
        printf("% 6.3f % 6.3f % 6.3f, % 6.1f % 6.1f % 6.1f,  % 6.1f % 6.1f % 6.1f\n", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8]);

        // imu.readAHRS(data);
        // printf("%02x %02x %02x %02x %02x %02x\n", data[14], data[16], data[17], data[18], data[19], data[20]);
        // printf("%f, %f, %f\n", mdata[0], mdata[1], mdata[2]);
        // imu.read_imu(imudata+1);
        // imudata[1]-=imucal[1]; imudata[2]-=imucal[2]; imudata[3]-=imucal[3];
        // imudata[4]-=imucal[5]; imudata[5]-=imucal[5]; imudata[6]-=imucal[6];

        // dt_2 = imudata[0]/2.f;
        // nx = imudata[1]*rad*dt_2; ny = imudata[2]*rad*dt_2; nz = imudata[3]*rad*dt_2;
        // dtheta = sqrt(nx*nx + ny*ny + nz*nz);
        // nx /= dtheta, ny /= dtheta, nz /= dtheta;
        // s = sin(dtheta);
        // p[0] = 1.-s*s; p[1] = nx*s; p[2] = ny*s; p[3] = nz*s;
        // q[0] = q0[0]*p[0] - q0[1]*p[1] - q0[2]*p[2] - q0[3]*p[3];
        // q[1] = q0[0]*p[1] + q0[1]*p[0] + q0[2]*p[3] - q0[3]*p[2];
        // q[2] = q0[0]*p[2] - q0[1]*p[3] + q0[2]*p[0] + q0[3]*p[1];
        // q[3] = q0[0]*p[3] + q0[1]*p[2] - q0[2]*p[1] + q0[3]*p[0];
        // q0[0] = q[0]; q0[1] = q[1]; q0[2] = q[2]; q0[3] = q[3];
    
        // MadgwickAHRSupdateIMU(imudata[1]*rad, imudata[2]*rad, imudata[3]*rad, imudata[4], imudata[5], imudata[6]);
        // imu.read_magnetometer_data(magdata);

        // imudata[0] = timer.stop();
        // timer.start();
        vTaskDelayUntil( &xLastWakeTime, Ts );
        // printf("%.3f %.3f %.3f\n", magdata[0], magdata[1], magdata[2]);

        // uart_write_bytes(UART_NUM_0, (const char*) &imudata, sizeof(float)*N);
        // uart_write_bytes(UART_NUM_0, "\r\n", 2);
        // printf("%.3f %.3f %.3f %.3f \n", q0[0], q0[1], q0[2], q0[3]);
        // printf("%.3f %.3f %.3f %.3f %.3f %.3f \n", 1/imudata[0], imudata[0], q0, q1, q2, q3);
    }
}
    

extern "C" void app_main(void)
{
    xTaskCreate(control_loop, "control_loop", 1024*2, (void *)0, 5, NULL);
}
