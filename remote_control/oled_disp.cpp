#include "oled_disp.h"

oled_disp::oled_disp(i2c_inst_t* i2c_instance, uint8_t sda, uint8_t scl)
    : i2c_instance_(i2c_instance)
    , sda_(sda)
    , scl_(scl)
{

}

oled_disp::~oled_disp()
{

}

void oled_disp::init_dev()
{
    init_oled_i2c();
    init_oled_driver();
}

void oled_disp::init_oled_i2c()
{

}   

void oled_disp::init_oled_driver()
{
    oled_send_cmd(OLED_SET_DISP & 0x00u); // 1. close the display

    oled_send_cmd(OLED_SET_MEM_ADDR); // 2. memory address mode: horizontal addressing mode
    oled_send_cmd(0x00);

    oled_send_cmd(OLED_SET_SEG_REMAP | 0x01); // 3. set seg re-map, column address 127 map to SEG0

    oled_send_cmd(OLED_SET_MUX_RATIO); // 4. set MUX_RATIO to 63 (height - 1)
    oled_send_cmd(OLED_HEIGHT - 1);

    oled_send_cmd(OLED_SET_COM_OUT_DIR | 0x08); // 5. scan from bottom to up

    oled_send_cmd(OLED_SET_DISP_OFFSET); // 6. no offset
    oled_send_cmd(0x00);

    // timing and driving schema
    oled_send_cmd(OLED_SET_DISP_CLK_DIV); // 7. divide ratio
    oled_send_cmd(0x80);

    oled_send_cmd(OLED_SET_PRECHARGE); // 8. set pre-charge period, Vcc internally generated on our board 1111 0001
    oled_send_cmd(0xF1);

    oled_send_cmd(OLED_SET_VCOM_DESEL); // 9. set VCOMH deselect level
    oled_send_cmd(0x30);                // 0.83xVcc

    oled_send_cmd(OLED_SET_CONTRAST); // 10. set contrast control
    oled_send_cmd(0xFF);

    oled_send_cmd(OLED_SET_ENTIRE_ON | 0x00); // 11. set display follow RAM content

    oled_send_cmd(OLED_SET_NORM_INV); // 12. set normal (not inverted) display

    oled_send_cmd(OLED_SET_CHARGE_PUMP); // 13. charge pump，3.3v->7v~12V
    oled_send_cmd(0x14);

    oled_send_cmd(OLED_SET_SCROLL | 0x00); // 14 deactivate horizontal scrolling if set

    show_test_image();                     // 15 init ram content, show test image

    oled_send_cmd(OLED_SET_DISP | 0x01); // open display
}

void oled_disp::oled_send_cmd(uint8_t cmd)
{
    // ssd1306 command control byte is 0x80
    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(i2c_instance_, ADDR, buf, 2, false);
}

void oled_disp::oled_send_to_memory(uint8_t* buf, size_t buf_len)
{
    // send to memory control byte is 0x40
    // control byte should added into the buffer array
    if (buf[0] != 0x40)
    {
        return;
    }
    i2c_write_blocking(i2c_instance_, ADDR, buf, buf_len, false);
}

void oled_disp::show_test_image()
{
    oled_send_to_memory(test_image2, 1025);
}

void oled_disp::show_test_string()
{
    const std::string str = "hello, world123456789";
    write_text_to_dev(str);
}

oled_disp& oled_disp::operator<<(const std::string& str)
{
    write_text_to_dev(str);
    return *this;
}

void oled_disp::write_text_to_dev(const std::string& text)
{
    // copy template pattern image to array
    uint8_t data[1025];
    std::copy(std::begin(display_template_one), std::end(display_template_one), data);
    uint32_t count = 0;

    for (auto&& c : text)
    {
        uint32_t line = (count / 16) * 2;     // 字符所在的行数，一个字符一共占两行（8bit算一行）line为两行中的第一行，可以取的值为0246
        uint32_t line_offset = count % 16;    // 字符在行中的偏移量
        uint8_t* c_model = nullptr;           // 用于显示字符的数据，总共16bytes
        auto it = index_mapping.find(c);
        
        // get character model
        if (it != index_mapping.end())
        {
            if (c >= 'a' && c <= 'z')
            {
                c_model = lowcase_letter_8x16[it->second];
            } 
            else if (c >= 'A' && c <= 'Z')
            {
                c_model = upcase_letter_8x16[it->second];
            } 
            else if (c == '\n')
            {
                if (line != 6)
                {
                    count = 16 * (line / 2 + 1);
                    continue;
                } 
                else
                {
                    break;  // 超过屏幕显示的限制
                }
            } 
            else 
            {
                c_model = numbers_8x16[it->second];
            }
        }

        // write character to the template image
        if (c_model)
        {   
            for (size_t i = 0; i < 8; i++)
            {
                data[i + 128 * line + 8 * line_offset + 1] = c_model[i];             // 写入字符的上半部分
                data[i + 128 * (line + 1) + 8 * line_offset + 1] = c_model[i + 8];   // 写入字符的下半部分
            }
        }

        // the oled panel can only hold 64 characters
        if (++count > 63 )
        {
            break;
        }
    }
    oled_send_to_memory(data, 1025);
}
