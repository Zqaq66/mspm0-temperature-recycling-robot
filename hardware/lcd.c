#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "Time.h"   /* DZZH: delay_ms() 在 BSP/Time.c */
#include "Timer.h"  /* DZZH: Get_Time() 1ms 时基 (刷屏节流用) */

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加方式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加方式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFixedNum1(u16 x,u16 y,int32_t value_Q10,u8 len,u16 fc,u16 bc,u8 sizey)
{
    u8 t, temp, sizex;
    u16 num1;
    sizex = sizey / 2;

    /* Q10 -> *10 / 1024 = (*10) >> 10, 保留一位小数 (如 17.8 -> 178) */
    num1 = (u16)((value_Q10 * 10) >> 10);

    for (t = 0; t < len; t++)
    {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if (t == (len - 2))
        {
            LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}




void LCD_ShowHex(u16 x,u16 y,u32 num,u8 len,u16 fc,u16 bc,u8 sizey)
{
    u8 i, hex[8];
    for (i = 0; i < len; i++) {
        hex[len - 1 - i] = num & 0x0F;
        num >>= 4;
    }
    for (i = 0; i < len; i++) {
        if (hex[i] < 10)
            LCD_ShowChar(x + i * (sizey / 2), y, hex[i] + 0x30, fc, bc, sizey, 0);
        else
            LCD_ShowChar(x + i * (sizey / 2), y, hex[i] - 10 + 0x41, fc, bc, sizey, 0);
    }
}


void LCD_ShowHexBuf(u16 x,u16 y,u8 *buf,u8 len,u16 fc,u16 bc,u8 sizey)
{
    u8 i, h, l, step = sizey / 2 + 2;
    for (i = 0; i < len; i++) {
        h = buf[i] >> 4;
        l = buf[i] & 0x0F;
        if (h < 10) LCD_ShowChar(x + i * step,     y, h + 0x30, fc, bc, sizey, 0);
        else        LCD_ShowChar(x + i * step,     y, h - 10 + 0x41, fc, bc, sizey, 0);
        if (l < 10) LCD_ShowChar(x + i * step + sizey/2, y, l + 0x30, fc, bc, sizey, 0);
        else        LCD_ShowChar(x + i * step + sizey/2, y, l - 10 + 0x41, fc, bc, sizey, 0);
    }
}

/*===========================================================================
 *  批量刷屏 + 文字层 (2026-10-06 新增)
 *    ★ 上面那些例程函数一个字没改: 需要老行为(逐字节 / 叠加模式 / 12~32 号字)照旧用。
 *
 *  为什么要加:
 *    老路径 1 个字符 = 263 次 LCD_Writ_Bus (7 次窗口命令 + 128 像素 x2), 16 字符一行
 *    约 4208 次事务 约 14ms, 8~10 行 约 114ms —— 5ms 控制环被顶成 8.7Hz
 *    (屏上诊断行 L 一直报 114/115), 跑车 100ms 盲开, 弯道直接甩出去。
 *
 *  现在怎么做:
 *   1) LCD_ShowRowFast(): 一行 16 个 16 号字 = 128x16 像素, CS 只翻一次推 4096 字节
 *      (0.82ms), 而且直接按行读 ascii_1608 字体, 没有中间显存 -> 不占 RAM。
 *      像素顺序/颜色/字节序与 LCD_ShowChar(非叠加模式) 完全一致。
 *   2) 文字层 LCD_RowSet() / LCD_RowFlush(): 调用者只写"文字影子"(纯 RAM, 几 us 就返回),
 *      真正推屏由 LCD_RowFlush() 在主循环里做, 而且 1ms 最多推 1 行 (闸门) ——
 *      单次最多占 1ms, 控制环的夹缝里就把屏刷完了, 谁也不会把谁卡住。
 *      后面加测温 / 语音 / LORA 也只是往自己那行 LCD_RowSet(...) 一下就完事。
 *   3) 内容没变就不刷: LCD_RowSet 先比对影子, 文字和颜色都一样就不置脏 ->
 *      静止画面 0 SPI 流量, 一个字节都不发。
 *===========================================================================*/

#define LCD_ROW_NUM   10                        /* 160 / 16 = 10 行 */
#define LCD_ROW_COLS  16                        /* 128 / 8  = 16 字符/行 */

static char lcd_txt[LCD_ROW_NUM][LCD_ROW_COLS]; /* 每行 16 字符 (无结束符, 不足的补空格) */
static u16  lcd_fc [LCD_ROW_NUM];               /* 每行颜色 */
static u16  lcd_dirty = 0;                      /* bit i = 第 i 行有变化, 等刷新 (10 行要 10 位!) */
static u8   lcd_rr    = 0;                      /* 下次从哪行开始找 (轮转, 防某行饿死) */
static u32  lcd_flush_ms = 0;                   /* 上次推屏时间戳 (1ms 闸门) */

/*---------------------------------------------------------------------------
 * 写一行文字影子 —— 只动 RAM, 不碰 SPI, 几 us 返回。
 *   不足 16 字符补空格 (盖掉上一帧残字), 超过 16 字符截断 (不许印到屏外)。
 *   文字和颜色都没变 -> 不置脏, 后面一次 SPI 都不会发。
 *---------------------------------------------------------------------------*/
void LCD_RowSet(u8 row, const char *txt, u16 fc)
{
    u8 i, changed = 0;

    if (row >= LCD_ROW_NUM) return;

    for (i = 0; i < LCD_ROW_COLS; i++)
    {
        /* ★ 必须"读到 '\0' 就停, 后面一律空格": 不能写成 txt[i] 逐个下标判 —
         *   那样会读过字符串结尾, 字符串字面量后面紧挨着别的字面量, 会把垃圾字符补进来
         *   (上屏就是乱码; 短串尤其明显, 比如授时未同步时传进来的空串)。
         *   老 empty.c 的 lcd_row 是先数到 '\0' 再补空格, 这里保持一致。 */
        char ch = ' ';
        if (txt && *txt) ch = *txt++;
        if (lcd_txt[row][i] != ch) { lcd_txt[row][i] = ch; changed = 1; }
    }
    if (changed || lcd_fc[row] != fc)
    {
        lcd_fc[row] = fc;
        lcd_dirty |= (u16)(1u << row);
    }
}

/*---------------------------------------------------------------------------
 * 一行 (x .. x+127, y .. y+15) 一次推完: 16 行 x 16 字 x 8 像素 x 2 字节 = 4096 字节。
 *   字体按行读: ascii_1608[ch-' '][r], 每个字节的 bit t 就是第 t 个像素(低位在左),
 *   与 LCD_ShowChar 非叠加模式逐点输出的结果一模一样。
 *---------------------------------------------------------------------------*/
void LCD_ShowRowFast(u16 x, u16 y, const char *s, u16 fc, u16 bc)
{
    u8 cc[LCD_ROW_COLS];                            /* 16 字节栈上小缓冲 (不占 RAM 区) */
    u8 r, c, t, b, zero = 0;

    /* 先把 16 个字符解析出来: 读到 '\0' 就停, 后面一律空格 (绝不读过字符串结尾),
     * 非 ASCII / 控制字符也按空格 —— LCD_ShowRowFast 允许直接传短字符串, 比如 "AB" */
    for (c = 0; c < LCD_ROW_COLS; c++)
    {
        u8 ch = 0x20;
        if (!zero && s)                             /* s == NULL 也安全: 全空格 */
        {
            ch = (u8)s[c];
            if (ch == 0) { zero = 1; ch = 0x20; }
        }
        if (ch < 0x20 || ch > 0x7E) ch = 0x20;      /* 只支持 ASCII, 越界按空格(防数组越界) */
        cc[c] = ch;
    }

    LCD_Address_Set(x, y, (u16)(x + LCD_ROW_COLS * 8 - 1), (u16)(y + 15));
    LCD_Writ_BusBegin();
    for (r = 0; r < 16; r++)                        /* 16 行像素 */
    {
        for (c = 0; c < LCD_ROW_COLS; c++)          /* 一行 16 个字符 */
        {
            b = ascii_1608[cc[c] - 0x20][r];
            for (t = 0; t < 8; t++)                 /* 一个字符 8 个像素宽 */
            {
                u16 col = (b & (1u << t)) ? fc : bc;
                LCD_Writ_BusStream((u8)(col >> 8)); /* 与 LCD_WR_DATA 一致: 高字节在前 */
                LCD_Writ_BusStream((u8)col);
            }
        }
    }
    LCD_Writ_BusEnd();
}

/*---------------------------------------------------------------------------
 * 整块填充 —— 参数与例程 LCD_Fill 相同, 但 CS 只翻一次。
 *   整屏 128x160 = 40960 字节 = 8.2ms (老路径约 0.28s, 上电自检 4 次纯色就是 1.1s)
 *   上电自检 / 清屏 / 画色块都该用它。
 *---------------------------------------------------------------------------*/
void LCD_FillFast(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    u32 n;
    u8  hi = (u8)(color >> 8), lo = (u8)(color & 0xFF);

    if (xend <= xsta || yend <= ysta) return;       /* 空区域直接返回 */
    n = (u32)(xend - xsta) * (u32)(yend - ysta);

    LCD_Address_Set(xsta, ysta, (u16)(xend - 1), (u16)(yend - 1));
    LCD_Writ_BusBegin();
    while (n--) { LCD_Writ_BusStream(hi); LCD_Writ_BusStream(lo); }
    LCD_Writ_BusEnd();
}

/*---------------------------------------------------------------------------
 * 主循环里反复调 —— 1ms 最多推 1 行 (闸门), 推了一行返回 1, 没脏行返回 0。
 *   放在 main() 的 while(1) 里: 控制环 5ms 一拍, 这里每次最多占 1ms, 夹缝刷完。
 *   闸门靠 Get_Time() (1ms 才变): 同 1ms 内第二次进来直接返回, 不会连着刷好几行。
 *---------------------------------------------------------------------------*/
u8 LCD_RowFlush(void)
{
    u32 now;
    u8  n, r;

    if (!lcd_dirty) return 0;                       /* 没变化: 一次 SPI 都不发 */
    now = Get_Time();
    if (now == lcd_flush_ms) return 0;              /* 1ms 闸门 */
    lcd_flush_ms = now;

    for (n = 0; n < LCD_ROW_NUM; n++)               /* 轮转找一行脏的 (不饿死任何一行) */
    {
        r = (u8)((lcd_rr + n) % LCD_ROW_NUM);
        if (lcd_dirty & (u16)(1u << r))
        {
            lcd_dirty &= (u16)~(1u << r);
            lcd_rr = (u8)((r + 1) % LCD_ROW_NUM);
            LCD_ShowRowFast(0, (u16)(r * 16), lcd_txt[r], lcd_fc[r], BLACK);
            return 1;
        }
    }
    return 0;
}
