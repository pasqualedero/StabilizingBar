#include "stm32f446xx.h"
#include "lcd_i2c_simple.h"
#include <string.h> // Per strlen se necessario

// --- Definisci indirizzi e pin --- //
#define I2C1_BASE 0x40005400
#define I2C1 ((I2C_TypeDef *) I2C1_BASE)

#define LCD_ADDR (0x27 << 1)
#define LCD_BACKLIGHT (1 << 3)

// --- Funzioni di supporto I2C (PER STM32F4) --- //

void i2c_wait_for_flag(uint32_t flag) {
    while(!(I2C1->SR1 & flag));
}

void i2c_write_byte(uint8_t data) {
    i2c_wait_for_flag(I2C_SR1_TXE);
    I2C1->DR = data;
}

void i2c_start(uint8_t addr) {
    I2C1->CR1 |= I2C_CR1_START;
    i2c_wait_for_flag(I2C_SR1_SB);
    I2C1->DR = addr & 0xFE;
    i2c_wait_for_flag(I2C_SR1_ADDR);
    (void) I2C1->SR2;
}

void i2c_stop(void) {
    i2c_wait_for_flag(I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
}

// --- Funzioni di supporto LCD --- //

void lcd_send_nibble(uint8_t nibble, uint8_t rs_mode) {
    uint8_t data = nibble | rs_mode | LCD_BACKLIGHT;
    uint8_t data_en = data | (1 << 2);
    uint8_t data_dis = data & ~(1 << 2);

    i2c_start(LCD_ADDR);
    i2c_write_byte(data_en);
    i2c_write_byte(data_dis);
    i2c_stop();

    for(volatile int i = 0; i < 100; i++);
}

void lcd_send_byte(uint8_t byte, uint8_t rs_mode) {
    lcd_send_nibble(byte & 0xF0, rs_mode);
    lcd_send_nibble((byte << 4) & 0xF0, rs_mode);
}

// --- Funzioni pubbliche --- //

void lcd_init(void) {
    // 1. Abilita clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // 2. Configura PB8/9
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
    GPIOB->MODER |= (2 << 16) | (2 << 18);
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk);
    GPIOB->PUPDR |= (1 << 16) | (1 << 18);
    GPIOB->AFR[1] &= ~(0xFF << 0);
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);

    // 3. Configura I2C1
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;

    // 4. Sequenza di inizializzazione LCD
    for(volatile int i = 0; i < 100000; i++);

    lcd_send_nibble(0x30, 0);
    for(volatile int i = 0; i < 10000; i++);
    lcd_send_nibble(0x30, 0);
    for(volatile int i = 0; i < 1000; i++);
    lcd_send_nibble(0x30, 0);
    for(volatile int i = 0; i < 1000; i++);
    lcd_send_nibble(0x20, 0);

    lcd_send_byte(0x28, 0);
    lcd_send_byte(0x0C, 0);
    lcd_send_byte(0x06, 0);
    lcd_send_byte(0x01, 0);
    for(volatile int i = 0; i < 10000; i++);
}

void lcd_write_string(char *str) {
    while(*str) {
        lcd_send_byte(*str++, 1);
    }
}

void lcd_clear(void) {
    lcd_send_byte(0x01, 0); // Comando clear display
    for(volatile int i = 0; i < 10000; i++); // Attesa
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0;
    address += col;
    lcd_send_byte(address, 0);
}

// --- Funzioni di conversione --- //

void int_to_string(int value, char* buffer, int buffer_size) {
    int i = 0;
    int is_negative = 0;

    if (value < 0) {
        is_negative = 1;
        value = -value;
    }

    // Gestisci il caso speciale del valore 0
    if (value == 0) {
        buffer[i++] = '0';
    } else {
        // Converti digit per digit (in ordine inverso)
        while (value > 0 && i < buffer_size - 1) {
            buffer[i++] = (value % 10) + '0';
            value /= 10;
        }
    }

    // Aggiungi il segno negativo se necessario
    if (is_negative && i < buffer_size - 1) {
        buffer[i++] = '-';
    }

    // Aggiungi il terminatore di stringa
    buffer[i] = '\0';

    // Inverti la stringa
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = buffer[start];
        buffer[start] = buffer[end];
        buffer[end] = temp;
        start++;
        end--;
    }
}
void float_to_string(float value, char* buffer, int decimals) {
    // Gestisci valori negativi
    if (value < 0) {
        buffer[0] = '-';
        value = -value;
        buffer++;
    }

    int int_part = (int)value;
    float frac_part = value - int_part;

    // Converti la parte intera
    int_to_string(int_part, buffer, 16);

    // Trova la lunghezza della parte intera
    int len = 0;
    while (buffer[len] != '\0') len++;

    // Aggiungi il punto decimale se richiesto
    if (decimals > 0) {
        buffer[len++] = '.';

        // Converti la parte decimale
        for (int i = 0; i < decimals; i++) {
            frac_part *= 10;
            int digit = (int)frac_part;
            buffer[len++] = digit + '0';
            frac_part -= digit;
            // Assicurati di non superare la dimensione del buffer
            if (len >= 15) break;
        }
    }

    buffer[len] = '\0';
}
