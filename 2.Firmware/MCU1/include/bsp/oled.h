
#pragma once
#include <sstream>
#include <iomanip>
#include <U8g2lib.h>
#include <map>

namespace CubliMini
{
    namespace Bsp
    {
#define OLED_SEND g_oled_control->Display
#define OLED g_oled_control
#define U8G2 g_oled_control->GetU8G2()
#define TO_STRING g_oled_control->ToString

#define OLED_IIC_SDA (int)9
#define OLED_IIC_SCL (int)4
#define OLED_IIC_RST (int)21

/*
    单次刷新OLED，节约CPU
*/
class OledControl
{
public:
    enum oled_TYPE_E
    {
        CH1115_88x48 = 0,
    };

    enum FONT_TYPE_E
    {
        NUMBLE = 0,
        CHINESE
    };

    using Task = std::function<void()>;

    static std::string FloatToString(float num, int precision = 2) 
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << num;
        return oss.str();
    }

     static std::string ToString(const char* format, ...) 
    {
        char buffer[256];
        
        // 使用可变参数处理
        va_list args;
        va_start(args, format);          // 初始化可变参数列表
        
        // 安全格式化（自动截断防止溢出）
        vsnprintf(buffer, sizeof(buffer), format, args);
        
        va_end(args);                    // 清理可变参数列表
        
        return std::string(buffer);
    }

    OledControl(oled_TYPE_E type = CH1115_88x48) : type_(type)
    {
        u8g2_ = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R1, /* reset=*/ OLED_IIC_RST, OLED_IIC_SCL, OLED_IIC_SDA);
        if(type_ == CH1115_88x48)
        {
            w_ = 48;
            h_ = 88;
            oled_offset_x_ = oled_offset_y_ = 8;
        }
        x_queue_ = xQueueCreate(1, sizeof(Task));
    }

    void Init()
    {
        u8g2_->begin();
        u8g2_->setContrast(150); // 设置对比度（0-255）
        //u8g2.setFont(u8g2_font_ncenB08_tr); // 设置字体
        u8g2_->setFont(u8g2_font_wqy12_t_gb2312);
        u8g2_->enableUTF8Print();
    }

    void DrawNumbleInCenter(int y, const std::string & msg)
    {
        int start_w;
        int msg_len = msg.length() * 8;
        if(msg_len >= w_)
        {
            start_w = 0;
        }
        else
        {
            start_w = (w_ - msg_len) / 2;
        }
        u8g2_->drawUTF8(oled_offset_x_ + start_w, oled_offset_y_ + y, msg.c_str()); 
    }

    void DrawChineseInCenter(int y, const std::string & msg)
    {
        int start_w;
        int msg_len = msg.length() * 4;
        if(msg_len >= w_)
        {
            start_w = 0;
        }
        else
        {
            start_w = (w_ - msg_len) / 2;
        }
        u8g2_->drawUTF8(oled_offset_x_ + start_w, oled_offset_y_ + y, msg.c_str()); 
    }

    void DrawUTF8(u8g2_uint_t x, u8g2_uint_t y, const char *s)
    {
        u8g2_->drawUTF8(oled_offset_x_ + x, oled_offset_y_ + y, s); 
    }

    /*
        发送一次task刷新一次UI
    */
    void Display(Task task)
    {
        xQueueOverwrite(x_queue_, &task);
    }

    void Loop(TickType_t ms = portMAX_DELAY)
    {
        Task task;
        if (xQueueReceive(x_queue_, &task, ms) == pdTRUE)  // wait max 2000ms
        {
            u8g2_->firstPage();
            do{
                task();
            } while ( u8g2_->nextPage() );
        }
    }

    U8G2_SSD1306_128X64_NONAME_F_HW_I2C *GetU8G2() const {
        return u8g2_;
    }

private:
    QueueHandle_t x_queue_;
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2_;
    int w_;
    int h_;
    int oled_offset_x_;
    int oled_offset_y_;
    oled_TYPE_E type_;
};


extern OledControl *g_oled_control;

}
}