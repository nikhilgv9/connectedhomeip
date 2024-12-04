#include "Display.h"

// Function to write a byte to the AXP2101 register
esp_err_t Display::i2c_write(uint8_t axp_addr, uint8_t reg_addr, const uint8_t* data, uint8_t count) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    esp_err_t status = i2c_master_start(cmd_handle);
    status = i2c_master_write_byte(cmd_handle,axp_addr << 1, true);
    status = i2c_master_write_byte(cmd_handle,reg_addr, true);
    status = i2c_master_write(cmd_handle, data, count, true);
    status = i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 100000);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

int32_t Display::i2c_read(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t length) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (reg) {
        /* When reading specific register set the address pointer first. */
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, (i2c_ack_type_t)true);
        i2c_master_write(cmd, &reg, 1, (i2c_ack_type_t)true);
    } else {
//        ESP_LOGD(TAG, "Reading address 0x%02x", address);
    }

    /* Read length bytes from the current pointer. */
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd,
        (address << 1) | I2C_MASTER_READ,
        true
    );
    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1,(i2c_ack_type_t)false);
    }
    i2c_master_read_byte(cmd, buffer + length - 1, (i2c_ack_type_t)true);
    i2c_master_stop(cmd);

    result = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        1000 / portTICK_PERIOD_MS
    );
    i2c_cmd_link_delete(cmd);

    return result;
}

void Display::Write1Byte(uint8_t reg, uint8_t value) {
  i2c_write(AXP2101_ADDRESS, reg, &value, 1);
}

uint8_t Display::Read8bit(uint8_t reg) {
  uint8_t value;
  i2c_read(AXP2101_ADDRESS, reg, &value, 1);
  return value;
}


void Display::ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void Display::i2c_init() {
  // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        {{.clk_speed = 100000}}
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    uint8_t val = Read8bit(0x03);
    printf("nigeorge: VAL is %d\n", val);
    if (val == 0x03) {
      printf("nigeorge: It is AXP192!!. Not Expected");
    }
}

// DISPLAY METHODS NIGEORGE

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    if (keep_cs_active) {
      t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
{
    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);

    //get_id cmd
    lcd_cmd(spi, 0x04, true);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    // Release bus
    spi_device_release_bus(spi);

    return *(uint32_t*)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_BCKL));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = (gpio_pullup_t)true;
    gpio_config(&io_conf);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd, false);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }

    printf("Done sending all commands.\n");
}


/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;             //Column Address Set
    trans[1].tx_data[0] = 0;                //Start Col High
    trans[1].tx_data[1] = 0;                //Start Col Low
    trans[1].tx_data[2] = (320 - 1) >> 8;   //End Col High
    trans[1].tx_data[3] = (320 - 1) & 0xff; //End Col Low
    trans[2].tx_data[0] = 0x2B;             //Page address set
    trans[3].tx_data[0] = ypos >> 8;        //Start page high
    trans[3].tx_data[1] = ypos & 0xff;      //start page low
    trans[3].tx_data[2] = (ypos + PARALLEL_LINES - 1) >> 8;     //end page high
    trans[3].tx_data[3] = (ypos + PARALLEL_LINES - 1) & 0xff;   //end page low
    trans[4].tx_data[0] = 0x2C;             //memory write
    trans[5].tx_buffer = linedata;          //finally send the line data
    trans[5].length = 240 * 2 * 8 * PARALLEL_LINES;  //Data length, in bits
    trans[5].flags = 0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
void Display::display_from_buffer(spi_device_handle_t spi)
{
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;

    for (int y=0; y<320; y+=PARALLEL_LINES) {
        //Calculate a line.
        pretty_effect_calc_lines(lines[calc_line], y, PARALLEL_LINES);
        //Finish up the sending process of the previous line, if any
        if (sending_line!=-1) send_line_finish(spi);
        //Swap sending_line and calc_line
        sending_line=calc_line;
        calc_line=(calc_line==1)?0:1;
        //Send the line we currently calculated.
        send_lines(spi, y, lines[sending_line]);
        //The line set is queued up for sending now; the actual sending happens in the
        //background. We can go on to calculate the next line set as long as we do not
        //touch line[sending_line]; the SPI sending process is still reading from that.
    }

    send_line_finish(spi);
}

void Display::init_display(void) {
    // Step 1: I2C to PMU
    i2c_init();
    uint8_t val = Read8bit(0x80);
    Write1Byte(0x80, val & 0X5);

    Write1Byte(0x90, (Read8bit(0x30)) | 0X3A); // ALDO4 1000,

    Write1Byte(0x82, 0x1E); // AXP_ALDO3
    Write1Byte(0x84, 0xEB); // Very dangerous..!!! But carefully done.
    Write1Byte(0x95, 0x1E); // ALDO4 voltage.
    Write1Byte(0x96, 0x1E);

    val = Read8bit(0x18);
    Write1Byte(0x18, (val & 0xFD) + (1 << 1)); // Battery Charge Enable

    Write1Byte(0x69, 0b00110000); // System LED

    // Step 2: LEDC
    ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // Step 3: SPI to LCD
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .mosi_io_num=PIN_NUM_MOSI,
        .miso_io_num=PIN_NUM_MISO,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
        .mode=0,                                //SPI mode 0
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi);
}

void Display::display_image(void)
{
    //Initialize the effect displayed
    esp_err_t ret;
    ret=draw_smiley_face();
    ESP_ERROR_CHECK(ret);

    //Go do nice stuff.
    display_from_buffer(spi);
}

void Display::display_sad_image()
{
    //Initialize the effect displayed
    esp_err_t ret;
    ret=draw_sad_face();
    ESP_ERROR_CHECK(ret);

    //Go do nice stuff.
    display_from_buffer(spi);
}

void Display::open_eye(uint8_t eye) {
  esp_err_t ret;
  ret = draw_open_eye(eye);
  ESP_ERROR_CHECK(ret);

  display_from_buffer(spi);
}

void Display::close_eye(uint8_t eye) {
  esp_err_t ret;
  ret = draw_close_eye(eye);
  ESP_ERROR_CHECK(ret);

  display_from_buffer(spi);
}

void Display::neutral_lip() {
  esp_err_t ret;
  ret = draw_neutral_lips();
  ESP_ERROR_CHECK(ret);

  display_from_buffer(spi);
}

void Display::smile_lip() {
  esp_err_t ret;
  ret = draw_smile_lips();
  ESP_ERROR_CHECK(ret);

  display_from_buffer(spi);
}

Display::Display() {
    // Initialize the buffer
    init_blank_screen();

  //Allocate memory for the pixel buffers
  for (int i=0; i<2; i++) {
      lines[i]=(uint16_t*)heap_caps_malloc(240*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
      assert(lines[i]!=NULL);
  }
}
