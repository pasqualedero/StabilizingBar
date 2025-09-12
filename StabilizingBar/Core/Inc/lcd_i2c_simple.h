#ifndef LCD_I2C_SIMPLE_H
#define LCD_I2C_SIMPLE_H

void lcd_init(void);
void lcd_write_string(char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);

// Aggiungi queste dichiarazioni
void int_to_string(int value, char* buffer, int buffer_size);
void float_to_string(float value, char* buffer, int decimals);

#endif
