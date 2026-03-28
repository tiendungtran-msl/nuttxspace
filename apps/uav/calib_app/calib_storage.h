/****************************************************************************
 * apps/uav/calib_app/calib_storage.h
 *
 * Calibration persistence via STM32H743VI Backup SRAM (D3 domain).
 *
 * Backup SRAM (4KB @ 0x38800000):
 *   - Ton tai qua NRST hardware reset
 *   - Ton tai qua software reset
 *   - Ton tai qua power-cycle NEU VBAT duoc cap nguon
 *   - Bi xoa khi first POR (power-on-reset) khong co VBAT
 *
 * IMPLEMENTATION NOTE:
 *   Backup SRAM tren STM32H743 duoc truy cap qua AHB4 bus (D3 domain),
 *   KHONG di qua D-Cache cua Cortex-M7. Vi vay:
 *   - Khong dung up_clean_dcache / up_invalidate_dcache
 *   - Dung volatile word-by-word access + DSB barrier de dam bao ghi toi SRAM
 *   - Phai set PWR_CR1.DBP = 1 va RCC_AHB4ENR.BKPSRAMEN = 1 truoc khi ghi
 *
 * Validation: magic word + version + XOR checksum.
 * Neu data khong hop le -> bo qua, khong ap dung calib.
 *
 * Dung chung boi calib_app va sensors_app.
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/* Constants                                                                 */
/****************************************************************************/

#define CALIB_BBRAM_BASE        0x38800000UL    /* D3 domain backup SRAM    */
#define CALIB_MAGIC             0xCAB1CAB1UL    /* "CALIB DATA" marker      */
#define CALIB_VERSION           ((uint16_t)2)   /* Increment khi doi struct  */

/* Bit flags trong truong `flags` */
#define CALIB_FLAG_GYRO_VALID   (1u << 0)       /* Gyro calib da duoc luu   */
#define CALIB_FLAG_ACCEL_VALID  (1u << 1)       /* Accel calib da duoc luu  */

/* STM32H743 register addresses (hardcoded, verified tu RM0433)             */
#define _CALIB_PWR_CR1_ADDR     0x58024800UL    /* PWR_CR1                  */
#define _CALIB_RCC_AHB4ENR_ADDR 0x580244E0UL    /* RCC_AHB4ENR              */
#define _CALIB_PWR_CR1_DBP      (1u << 8)       /* Disable backup domain WP */
#define _CALIB_BKPSRAMEN        (1u << 28)      /* Backup SRAM clock enable */

/****************************************************************************/
/* Data Structure in Backup SRAM                                             */
/* Total: 4+2+2+4 + 12+12+12 = 48 bytes = 12 uint32_t words                */
/****************************************************************************/

typedef struct calib_bbram_s
{
    uint32_t magic;           /* 0xCAB1CAB1 khi hop le                    */
    uint16_t version;         /* CALIB_VERSION                            */
    uint16_t flags;           /* CALIB_FLAG_GYRO_VALID | ACCEL_VALID      */
    uint32_t checksum;        /* XOR checksum qua 9 float data fields     */

    float    gyro_offset[3];  /* rad/s  - gyro bias offset                */
    float    accel_offset[3]; /* m/s2   - accelerometer bias              */
    float    accel_scale[3];  /* dimless- diagonal cua accel_T matrix     */
} calib_bbram_t;              /* 48 bytes = 12 x uint32_t                 */

/****************************************************************************/
/* Internal Low-Level Helpers                                                */
/****************************************************************************/

/**
 * Enable access to backup domain:
 *   PWR_CR1.DBP  = 1  (disable write protection)
 *   BKPSRAMEN   = 1  (backup SRAM clock)
 * Ham nay idempotent — goi nhieu lan van an toan.
 * Do not rely on board bringup: CONFIG_BOARD_LATE_INITIALIZE=n,
 * CONFIG_NSH_ARCHINIT=n nen stm32_bringup() co the khong chay.
 */
static inline void _calib_enable_bbram(void)
{
    volatile uint32_t *pwr_cr1    = (volatile uint32_t *)_CALIB_PWR_CR1_ADDR;
    volatile uint32_t *rcc_ahb4en = (volatile uint32_t *)_CALIB_RCC_AHB4ENR_ADDR;
    *pwr_cr1    |= _CALIB_PWR_CR1_DBP;
    *rcc_ahb4en |= _CALIB_BKPSRAMEN;
    __asm__ volatile ("dsb" ::: "memory");
}

/**
 * Doc 48 bytes (12 x uint32_t) tu backup SRAM vao local struct.
 * Goi _calib_enable_bbram() truoc de dam bao clock/DBP da mo.
 * Dung DSB + volatile word access, khong di qua D-Cache.
 */
static inline void _calib_read_bbram(calib_bbram_t *dest,
                                      const calib_bbram_t *src)
{
    _calib_enable_bbram();  /* dam bao BKPSRAM clock va DBP da set */

    const volatile uint32_t *s = (const volatile uint32_t *)src;
    uint32_t                *d = (uint32_t *)dest;
    unsigned int i;
    __asm__ volatile ("dsb" ::: "memory");  /* flush moi write truoc do */
    for (i = 0; i < sizeof(calib_bbram_t) / 4; i++) {
        d[i] = s[i];
    }
    __asm__ volatile ("dsb" ::: "memory");  /* dam bao du lieu doc xong */
}

/**
 * Ghi 48 bytes (12 x uint32_t) tu local struct vao backup SRAM.
 * Goi _calib_enable_bbram() truoc de dam bao clock/DBP da mo.
 * Dung volatile word access + DSB, khong di qua D-Cache.
 */
static inline void _calib_write_bbram(calib_bbram_t *dest,
                                       const calib_bbram_t *src)
{
    _calib_enable_bbram();  /* dam bao BKPSRAM clock va DBP da set */

    volatile uint32_t    *d = (volatile uint32_t *)dest;
    const    uint32_t    *s = (const uint32_t *)src;
    unsigned int i;
    for (i = 0; i < sizeof(calib_bbram_t) / 4; i++) {
        d[i] = s[i];
    }
    __asm__ volatile ("dsb" ::: "memory");  /* dam bao data toi SRAM truoc khi tiep tuc */
}

/****************************************************************************/
/* Public Inline Helpers                                                     */
/****************************************************************************/

static inline calib_bbram_t *calib_bbram_ptr(void)
{
    return (calib_bbram_t *)((uintptr_t)CALIB_BBRAM_BASE);
}

/**
 * Tinh XOR checksum qua 9 float data fields va flags/version.
 * Seed 0x5A5A5A5A de phan biet all-zero (chua khoi tao).
 * Luu y: s la pointer toi local struct (khong phai volatile SRAM).
 */
static inline uint32_t calib_compute_checksum(const calib_bbram_t *s)
{
    uint32_t csum = 0x5A5A5A5Au;
    uint32_t word;
    const unsigned char *p = (const unsigned char *)s->gyro_offset;
    unsigned int i;
    /* Safe byte-by-byte read de tranh strict-aliasing violation */
    for (i = 0; i < 9; i++) {
        memcpy(&word, p + i * 4, 4);
        csum ^= word;
    }
    csum ^= (uint32_t)s->flags;
    csum ^= (uint32_t)s->version;
    return csum;
}

/**
 * Kiem tra tinh hop le cua mot LOCAL COPY da doc san (khong truy cap SRAM).
 * Dung noi bo de tranh doc SRAM nhieu lan trong mot thao tac save/load.
 * @param snap  Pointer toi local calib_bbram_t (khong phai volatile SRAM).
 * @return true neu magic, version, va checksum deu hop le.
 */
static inline bool _calib_validate(const calib_bbram_t *snap)
{
    if (snap->magic   != CALIB_MAGIC)   { return false; }
    if (snap->version != CALIB_VERSION) { return false; }
    if (calib_compute_checksum(snap) != snap->checksum) { return false; }
    return true;
}

/**
 * Kiem tra tinh hop le cua du lieu trong backup SRAM.
 * Doc vao local copy qua volatile access truoc khi validate.
 * @return true neu magic, version, va checksum deu hop le.
 */
static inline bool calib_bbram_is_valid(const calib_bbram_t *s)
{
    calib_bbram_t snap;
    _calib_read_bbram(&snap, s);    /* enable + doc 1 lan */
    return _calib_validate(&snap);  /* validate tren local copy, khong doc lai SRAM */
}

/**
 * Luu gyro offset vao backup SRAM.
 * Bao ton gia tri accel neu da ton tai va hop le.
 * Doc SRAM dung 1 lan, validate tren local copy, ghi 1 lan.
 * @param offset_rad_s  Gyro bias [3] theo don vi rad/s
 */
static inline void calib_save_gyro(const float offset_rad_s[3])
{
    calib_bbram_t *s = calib_bbram_ptr();
    calib_bbram_t tmp;

    /* Doc SRAM 1 lan duy nhat vao local copy */
    _calib_read_bbram(&tmp, s);

    /* Neu du lieu khong hop le: reset ve mac dinh, giu accel scale = 1 */
    if (!_calib_validate(&tmp)) {
        memset(&tmp, 0, sizeof(tmp));
        tmp.accel_scale[0] = 1.0f;
        tmp.accel_scale[1] = 1.0f;
        tmp.accel_scale[2] = 1.0f;
    }

    tmp.magic          = CALIB_MAGIC;
    tmp.version        = CALIB_VERSION;
    tmp.gyro_offset[0] = offset_rad_s[0];
    tmp.gyro_offset[1] = offset_rad_s[1];
    tmp.gyro_offset[2] = offset_rad_s[2];
    tmp.flags         |= CALIB_FLAG_GYRO_VALID;
    tmp.checksum       = calib_compute_checksum(&tmp);

    /* Ghi vao backup SRAM bang volatile access + DSB */
    _calib_write_bbram(s, &tmp);
}

/**
 * Luu accel offset va scale vao backup SRAM.
 * Bao ton gia tri gyro neu da ton tai va hop le.
 * Doc SRAM dung 1 lan, validate tren local copy, ghi 1 lan.
 * @param offset_m_s2  Accel bias [3] theo m/s2
 * @param scale        Diagonal cua accel_T [3] (adimensional)
 */
static inline void calib_save_accel(const float offset_m_s2[3],
                                    const float scale[3])
{
    calib_bbram_t *s = calib_bbram_ptr();
    calib_bbram_t tmp;

    /* Doc SRAM 1 lan duy nhat vao local copy */
    _calib_read_bbram(&tmp, s);

    /* Neu du lieu khong hop le: reset ve mac dinh, giu gyro offset = 0 */
    if (!_calib_validate(&tmp)) {
        memset(&tmp, 0, sizeof(tmp));
    }

    tmp.magic           = CALIB_MAGIC;
    tmp.version         = CALIB_VERSION;
    tmp.accel_offset[0] = offset_m_s2[0];
    tmp.accel_offset[1] = offset_m_s2[1];
    tmp.accel_offset[2] = offset_m_s2[2];
    tmp.accel_scale[0]  = scale[0];
    tmp.accel_scale[1]  = scale[1];
    tmp.accel_scale[2]  = scale[2];
    tmp.flags          |= CALIB_FLAG_ACCEL_VALID;
    tmp.checksum        = calib_compute_checksum(&tmp);

    /* Ghi vao backup SRAM bang volatile access + DSB */
    _calib_write_bbram(s, &tmp);
}

/**
 * Xoa toan bo du lieu calibration trong backup SRAM.
 * Sau khi xoa, calib_bbram_is_valid() se tra ve false.
 */
static inline void calib_clear_bbram(void)
{
    calib_bbram_t zero;
    memset(&zero, 0, sizeof(zero));
    _calib_write_bbram(calib_bbram_ptr(), &zero);  /* enable nam trong write */
}


#ifdef __cplusplus
}
#endif
