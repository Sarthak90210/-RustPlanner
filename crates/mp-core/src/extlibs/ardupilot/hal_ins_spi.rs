//! Port of `ExtLibs/ArduPilot/hal_ins_spi.cs`

crate::cs_enum! {
    /// `MissionPlanner.ArduPilot.hal_ins_spi`
    pub hal_ins_spi: i32 {
        ms5611 = 0,
        ms5611_int = 1,
        ms5611_ext = 2,
        lps22h = 3,
        bmp280 = 4,
        mpu6000 = 5,
        mpu6000_ext = 6,
        lsm9ds0_g = 7,
        lsm9ds0_am = 8,
        lsm9ds0_ext_g = 9,
        lsm9ds0_ext_am = 10,
        mpu9250 = 11,
        mpu9250_ext = 12,
        mpu6500 = 13,
        icm20608 = 14,
        //icm20608-am,
        icm20608_ext = 15,
        hmc5843 = 16,
        lis3mdl = 17,
    }
}
