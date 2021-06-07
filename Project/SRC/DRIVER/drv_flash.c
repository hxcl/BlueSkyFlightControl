/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_flash.c
 * @说明     单片机Flash读写驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "drv_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_flash.h"

/**********************************************************************************************************
*函 数 名: Flash_ReadByte
*功能说明: 从flash中读取一个字节
*形    参: 起始地址 地址偏移量
*返 回 值: 读取到的字节
**********************************************************************************************************/
uint8_t Flash_ReadByte(uint32_t start_addr, uint16_t cnt) {
    return *(__IO uint8_t *) (start_addr + cnt);
}

/**********************************************************************************************************
*函 数 名: Flash_ReadWord
*功能说明: 从flash中读取一个字（32位）
*形    参: flash地址
*返 回 值: 读取到的字
**********************************************************************************************************/
uint32_t Flash_ReadWord(uint32_t addr) {
    return *(uint32_t *) addr;
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address) {
    uint32_t sector = 0;

    if (((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2))) {
        sector = FLASH_SECTOR_0;
    } else if (((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2))) {
        sector = FLASH_SECTOR_1;
    } else if (((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2))) {
        sector = FLASH_SECTOR_2;
    } else if (((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2))) {
        sector = FLASH_SECTOR_3;
    } else if (((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2))) {
        sector = FLASH_SECTOR_4;
    } else if (((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2))) {
        sector = FLASH_SECTOR_5;
    } else if (((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2))) {
        sector = FLASH_SECTOR_6;
    } else if (((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2))) {
        sector = FLASH_SECTOR_7;
    } else {
        sector = FLASH_SECTOR_7;
    }

    return sector;
}

/**********************************************************************************************************
*函 数 名: Flash_WriteFLASHWORD
*功能说明: 在Flash的某个区域写入数据
*形    参: 写入地址 写入数据缓冲区指针 写入长度
*返 回 值: 写入状态
**********************************************************************************************************/
bool Flash_WriteFLASHWORD(uint32_t dest, uint8_t *src, uint32_t length) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    bool return_value = false;
    uint32_t start_addr = 0;
    uint32_t end_addr = 0;

    if (dest < FLASH_BASE_ADDR || dest > FLASH_END_ADDR) //非法地址
        return false;

    HAL_FLASH_Unlock();

    start_addr = dest;            //写入的起始地址
    end_addr = dest + length;    //写入的结束地址

    while (start_addr < end_addr) {
        if (Flash_ReadWord(start_addr) != 0XFFFFFFFF)//有非0XFFFFFFFF的地方，要擦除这个扇区
        {
            /* Fill EraseInit structure*/
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            EraseInitStruct.Banks = FLASH_BANK_1;
            EraseInitStruct.Sector = GetSector(start_addr);
            EraseInitStruct.NbSectors = 1;
            if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
                return_value = false; //发生错误了
                break;
            }
        } else
            start_addr += 4;
    }

    uint32_t Address = start_addr;

    while (Address < end_addr) {
        // FLASHWORD 256bit
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, *src) == HAL_OK) {
            Address += 32;
            src += 32;
        } else {
            return_value = false; //发生错误了
            break;
        }
    }

    HAL_FLASH_Lock();

    return return_value;
}
