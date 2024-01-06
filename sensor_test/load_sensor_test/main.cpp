#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico.h>

/*
test load sensor
*/

// define motor mosfet pin
const uint PDSCK_PIN = 0;
const uint DOUT_PIN = 1;


unsigned long read_count()
{
    unsigned long Count;
    unsigned char i;
    gpio_put(PDSCK_PIN, false);     // 使能AD（PD_SCK 置低）
    Count = 0;

    while (gpio_get(DOUT_PIN))      // AD转换未结束则等待，否则开始读取
    {
        tight_loop_contents();
    };

    sleep_us(1);

    for (i = 0; i < 24; i++)
    {
        gpio_put(PDSCK_PIN, true);  // PD_SCK 置高（发送脉冲）
        sleep_us(1);
        Count = Count << 1;         // 下降沿来时变量Count左移一位，右侧补零
        gpio_put(PDSCK_PIN, false); // PD_SCK 置低
        if (gpio_get(DOUT_PIN))
        {
            Count++;
        }
        sleep_us(1);
    }

    gpio_put(PDSCK_PIN, true);
    Count = Count ^ 0x800000;       // 第25个脉冲下降沿来时，转换数据
    gpio_put(PDSCK_PIN, false);
    return Count;
}

int main()
{
    stdio_init_all();

    gpio_init(PDSCK_PIN);
    gpio_set_dir(PDSCK_PIN, GPIO_OUT);

    gpio_init(DOUT_PIN);
    gpio_set_dir(DOUT_PIN, GPIO_IN);
    gpio_disable_pulls(DOUT_PIN);

    while (true)
    {
        // printf("output value = %d\n", read_count());
        printf("%d\n", read_count());
        sleep_ms(400);
    }
     
    return 0;
}