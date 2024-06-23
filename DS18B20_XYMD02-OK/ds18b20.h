void set_pin_as_output(void);
void set_pin_as_input(void);
void onewire_reset(void);
void onewire_write(uint8_t data);
uint8_t onewire_read(void);
uint16_t read_temperature(void);