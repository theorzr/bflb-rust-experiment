//! BL808, M0 core (T-Head E907, RV32, CLIC).

pub use bflb_hal::arch::bl808 as pac;

/// Number of GPIO pins.
pub(crate) const PIN_COUNT: usize = 46;

/// Interrupt shared by all the pins, owned by the GPIO driver.
pub(crate) const GPIO_IRQ: bflb_hal::interrupt::Interrupt = bflb_hal::interrupt::GPIO_INT0;

embassy_hal_internal::peripherals! {
    // GPIO
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9,
    PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19,
    PIN_20, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27, PIN_28, PIN_29,
    PIN_30, PIN_31, PIN_32, PIN_33, PIN_34, PIN_35, PIN_36, PIN_37, PIN_38, PIN_39,
    PIN_40, PIN_41, PIN_42, PIN_43, PIN_44, PIN_45,

    // UART
    UART0,
    UART1,
    UART2,
}

impl_uart!(UART0, UART0, UART0, 0);
impl_uart!(UART1, UART1, UART1, 1);
impl_uart!(UART2, UART2, UART2, 2);

impl_pin!(PIN_0, 0);
impl_pin!(PIN_1, 1);
impl_pin!(PIN_2, 2);
impl_pin!(PIN_3, 3);
impl_pin!(PIN_4, 4);
impl_pin!(PIN_5, 5);
impl_pin!(PIN_6, 6);
impl_pin!(PIN_7, 7);
impl_pin!(PIN_8, 8);
impl_pin!(PIN_9, 9);
impl_pin!(PIN_10, 10);
impl_pin!(PIN_11, 11);
impl_pin!(PIN_12, 12);
impl_pin!(PIN_13, 13);
impl_pin!(PIN_14, 14);
impl_pin!(PIN_15, 15);
impl_pin!(PIN_16, 16);
impl_pin!(PIN_17, 17);
impl_pin!(PIN_18, 18);
impl_pin!(PIN_19, 19);
impl_pin!(PIN_20, 20);
impl_pin!(PIN_21, 21);
impl_pin!(PIN_22, 22);
impl_pin!(PIN_23, 23);
impl_pin!(PIN_24, 24);
impl_pin!(PIN_25, 25);
impl_pin!(PIN_26, 26);
impl_pin!(PIN_27, 27);
impl_pin!(PIN_28, 28);
impl_pin!(PIN_29, 29);
impl_pin!(PIN_30, 30);
impl_pin!(PIN_31, 31);
impl_pin!(PIN_32, 32);
impl_pin!(PIN_33, 33);
impl_pin!(PIN_34, 34);
impl_pin!(PIN_35, 35);
impl_pin!(PIN_36, 36);
impl_pin!(PIN_37, 37);
impl_pin!(PIN_38, 38);
impl_pin!(PIN_39, 39);
impl_pin!(PIN_40, 40);
impl_pin!(PIN_41, 41);
impl_pin!(PIN_42, 42);
impl_pin!(PIN_43, 43);
impl_pin!(PIN_44, 44);
impl_pin!(PIN_45, 45);

// GPIO_INT0 is intentionally missing, it is owned by the GPIO driver.
interrupt_mod!(
    BMX_MCU_BUS_ERR,
    BMX_MCU_TO,
    IPC_M0,
    AUDIO,
    RF_TOP_INT0,
    RF_TOP_INT1,
    LZ4D,
    GAUGE_ITF,
    SEC_ENG_ID1,
    SEC_ENG_ID0,
    SEC_ENG_ID1_CDET,
    SEC_ENG_ID0_CDET,
    SF_CTRL_ID1,
    SF_CTRL_ID0,
    SDH,
    MM_ALL,
    IR_TX,
    IR_RX,
    USB,
    AUPDM,
    EMAC,
    GPADC_DMA,
    EFUSE,
    SPI0,
    UART0,
    UART1,
    UART2,
    GPIO_DMA,
    I2C0,
    PWN,
    IPC_RSVD,
    IPC_LP,
    TIMER0_CH0,
    TIMER0_CH1,
    TIMER0_WDT,
    I2C1,
    I2S,
    ANA_OCP_OUT_TO_CPU0,
    ANA_OCP_OUT_TO_CPU1,
    ANA_OCP_OUT_TO_CPU2,
    DM,
    BL,
    M154_REQ_ACK,
    M154_INT,
    M154_AES,
    PDS_WAKE_UP,
    HBN_OUT0,
    HBN_OUT1,
    BOR,
    WIFI,
    BZ_PHY_INT,
    BLE,
    MAC_TX_RX_TIMER,
    MAC_TX_RX_MISC,
    MAC_RX_TRIGGER,
    MAC_TX_TRIGGER,
    MAC_GEN,
    MAC_PORT_TRIGGER,
    WIFI_IPC,
);
