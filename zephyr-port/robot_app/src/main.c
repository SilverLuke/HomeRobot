#include <zephyr/kernel.h>

extern int esp_rom_printf(const char *fmt, ...);

int main(void)
{
    while (1) {
        for (volatile int i = 0; i < 5000000; i++);
        esp_rom_printf("ALIVE ROM\n");
    }
    return 0;
}
