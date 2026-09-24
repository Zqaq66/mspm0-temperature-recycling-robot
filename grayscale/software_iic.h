#include "ti_msp_dl_config.h"
#include "gw_color_sensor.h"
#include "Time.h"

/* ×¢Òâ£º±¾ÎÄ¼şÊÇ¾É»Ò¶È´«¸ĞÆ÷µÄÈí¼şIICÇı¶¯£¬ÒÑ²»ÔÚ¹¦ÄÜÁ´Â·ÖĞ£¨ÒÑ±» Xinliu_AuroraT8_IIC È¡´ú£©¡£
 * PA0/PA1 ÏÖÓÉÓ²¼ş I2C0 Çı¶¯£¬ÏÂÃæ°Ñ¾ÉµÄÈí¼şIICÒı½Å±ğÃûÖ¸Ïò I2C0 µÄÒı½Åºê£¬
 * ½öÓÃÓÚ±£Ö¤¹¤³ÌÀïÈÔÒıÓÃ±¾ÎÄ¼şÊ±ÄÜ±àÒëÍ¨¹ı£»¸ÃÄ£ÄâÊ±ĞòÊµ¼Ê²»¿ÉÓÃ¡£ */
#define Software_iic_PORT       GPIO_I2C_0_SDA_PORT
#define Software_iic_SDA_PIN    GPIO_I2C_0_SDA_PIN
#define Software_iic_SCL_PIN    GPIO_I2C_0_SCL_PIN
#define Software_iic_SDA_IOMUX  GPIO_I2C_0_IOMUX_SDA

#define SDA_PIN Software_iic_SDA_PIN
#define SDA_PORT Software_iic_PORT
#define SCL_PIN Software_iic_SCL_PIN
#define SCL_PORT Software_iic_PORT

/* åŸºæœ¬I2Cæ“ä½œå® */
#define SDA_HIGH() DL_GPIO_setPins(SDA_PORT, SDA_PIN)
#define SDA_LOW()  DL_GPIO_clearPins(SDA_PORT, SDA_PIN)
#define SCL_HIGH() DL_GPIO_setPins(SCL_PORT, SCL_PIN)
#define SCL_LOW()  DL_GPIO_clearPins(SCL_PORT, SCL_PIN)
#define READ_SDA() DL_GPIO_readPins(SDA_PORT, SDA_PIN)

unsigned char Ping(void);
uint8_t IIC_WriteBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                           uint8_t *data, uint8_t len);
uint8_t IIC_WriteByte(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t data);
uint8_t IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t *Result, uint8_t len);