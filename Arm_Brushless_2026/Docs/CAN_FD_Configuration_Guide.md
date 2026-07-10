# CAN FD Configuration Guide — B-G431B-ESC1 ESC

## Overview

This document details the CAN FD setup for the **B-G431B-ESC1** Discovery kit (STM32G431CB) using **STM32CubeMX**. The configuration enables CAN FD communication with bit rate switching between the ESC and a CANable adapter.

---

## Hardware Reference

- **MCU:** STM32G431CBU6 (LQFP-48)
- **CAN Transceiver:** On-board, UAVCAN-compliant with overcurrent fuse protection
- **CAN TX Pin:** PB9 (Pin 47)
- **CAN RX Pin:** PA11 (Pin 33)
- **CAN Termination:** PC14 (Pin 3) — GPIO-controlled 120 Ω resistor via analog switch
  - HIGH = termination ON
  - LOW = termination OFF (high impedance)
- **CAN Connector:** J1 solder pads (includes 5V, GND, CANH, CANL)

### CAN Bus Power

The J1 connector can accept 5.0–5.5 V from an external unit (e.g., flight controller with powered CAN). When powered via J1, the board generates 3.3 V for the MCU and transceiver, maintaining communication even when the main battery is removed or discharged.

---

## Clock Configuration

### System Clock (unchanged)

| Parameter        | Value    |
|------------------|----------|
| HSE Input        | 8 MHz    |
| PLLM             | /2       |
| PLLN             | x85      |
| PLLR             | /2       |
| SYSCLK           | 170 MHz  |
| HCLK             | 170 MHz  |
| PCLK1            | 170 MHz  |
| PCLK2            | 170 MHz  |

### FDCAN Clock Source

| Parameter            | Value   |
|----------------------|---------|
| **Source**           | PLL Q   |
| PLLQ Divider        | /2      |
| **FDCAN Clock**      | 170 MHz |

> **Why PLL Q instead of PCLK1?**
> Using PCLK1 would require lowering the system clock to get a clean FDCAN frequency (e.g., 40 MHz), which would reduce SYSCLK and degrade motor control performance (FOC loop timing, PWM resolution, ADC sampling). PLL Q provides an independent clock source — changing it does not affect any other peripheral.

> **Why 170 MHz?**
> 170 MHz divides cleanly into 2 Mbps data rate (170 / 5 / 17 = 2,000,000) and 500 kbps nominal rate (170 / 20 / 17 = 500,000). A value like 40 MHz would have required reducing the system clock. 4 Mbps was not achievable cleanly from 170 MHz, so 2 Mbps was chosen instead.

---

## FDCAN1 Parameter Settings

### Basic Parameters

| Parameter                  | Value                            |
|----------------------------|----------------------------------|
| Clock Divider              | Divide kernel clock by 1         |
| Frame Format               | FD mode with BitRate Switching   |
| Mode                       | Normal mode                      |
| Auto Retransmission        | Enable                           |
| Transmit Pause             | Enable                           |
| Protocol Exception         | Disable                          |
| Nominal Sync Jump Width    | 3                                |
| Data Prescaler             | 5                                |
| Data Sync Jump Width       | 3                                |
| Data Time Seg1             | 13                               |
| Data Time Seg2             | 3                                |
| Std Filters Nbr            | 1                                |
| Ext Filters Nbr            | 0                                |
| Tx Fifo Queue Mode         | FIFO mode                        |

### Bit Timing Parameters (Nominal — Arbitration Phase)

| Parameter              | Value                |
|------------------------|----------------------|
| Nominal Prescaler      | 20                   |
| Nominal Time Seg1      | 13                   |
| Nominal Time Seg2      | 3                    |
| **Nominal Baud Rate**  | **~500 kbps**        |
| Nominal Sample Point   | 82.4%                |
| Nominal Time Quantum   | ~117.6 ns            |
| Nominal Time per Bit   | 2000 ns              |

**Math:**
- TQ clock = 170 MHz / 20 = 8.5 MHz
- TQ per bit = 1 (sync) + 13 (seg1) + 3 (seg2) = 17
- Bitrate = 8,500,000 / 17 = 500,000 bit/s (displayed as 499,999 due to rounding)
- Sample point = (1 + 13) / 17 = 82.4%

### Bit Timing Parameters (Data Phase)

| Parameter              | Value                |
|------------------------|----------------------|
| Data Prescaler         | 5                    |
| Data Time Seg1         | 13                   |
| Data Time Seg2         | 3                    |
| **Data Baud Rate**     | **2 Mbps**           |
| Data Sample Point      | 82.4%                |
| Data Time Quantum      | ~29.4 ns             |
| Data Time per Bit      | 500 ns               |

**Math:**
- TQ clock = 170 MHz / 5 = 34 MHz
- TQ per bit = 1 (sync) + 13 (seg1) + 3 (seg2) = 17
- Bitrate = 34,000,000 / 17 = 2,000,000 bit/s
- Sample point = (1 + 13) / 17 = 82.4%

---

## Bit Timing Concepts Explained

### Time Quanta (TQ)

Every CAN bit is subdivided into smaller time slices called time quanta. The prescaler divides the peripheral clock to produce the TQ frequency:

```
Peripheral Clock (170 MHz)  →  ÷ Prescaler  →  TQ Clock  →  ÷ TQ per bit  →  Bitrate
```

### Bit Structure

Each bit consists of three segments:

```
|  SYNC  |     TIME_SEG1     |  TIME_SEG2  |
| (1 TQ) |    (configurable) | (configurable)|
                              ^
                        Sample Point
```

- **SYNC (1 TQ):** Edge transition for synchronization — always exactly 1 TQ
- **TIME_SEG1:** Wait time before sampling — longer = later sample point
- **TIME_SEG2:** Remainder after sampling

### Sample Point

The moment within each bit when the receiver reads the bus value (1 or 0). Expressed as a percentage:

```
Sample Point = (1 + Seg1) / (1 + Seg1 + Seg2) × 100%
```

Higher sample points (~80–87.5%) give the signal more time to stabilize before reading, improving reliability especially on longer buses or noisy environments.

### Sync Jump Width (SJW)

Compensates for clock drift between nodes. No two oscillators are perfectly matched, so bit timing gradually drifts. SJW defines how many TQ the receiver can shift per edge to re-align. A value of 3 allows up to 3 TQ of adjustment per edge. Cannot exceed TIME_SEG2.

### Why Bit Rate Switching?

CAN FD frames have two phases:

1. **Arbitration phase (nominal rate — 500 kbps):** Multiple nodes compete for bus access simultaneously. Slow speed gives all nodes time to see each other's bits for reliable priority resolution. Higher speeds limit bus length and reduce reliability during contention.

2. **Data phase (data rate — 2 Mbps):** Only one node transmits after winning arbitration. No contention, so higher speeds are safe. Signal quality is the only concern, which is manageable with proper termination and short buses.

Without bit rate switching, the entire frame runs at one speed — either too slow for data throughput or too fast for reliable arbitration.

---

## NVIC Interrupt Configuration

| Interrupt            | Enabled | Priority | Purpose                                      |
|----------------------|---------|----------|----------------------------------------------|
| FDCAN1 interrupt 0   | Yes     | 5–6      | RX FIFO 0 new message, TX complete            |
| FDCAN1 interrupt 1   | Yes     | 5–6      | RX FIFO 1, error and status events            |

> **Priority note:** Keep CAN interrupt priority lower (higher number) than motor control timer and ADC interrupts to avoid disrupting FOC loop execution.

---

## GPIO Configuration

| Pin  | Label     | Mode        | Purpose                                  |
|------|-----------|-------------|------------------------------------------|
| PC14 | CAN_TERM  | GPIO_Output | Controls 120 Ω CAN termination resistor |

- Drive HIGH if the ESC is at an end of the CAN bus
- Drive LOW if the ESC is in the middle of the bus

---

## FIFO and Filter Settings

| Parameter                 | Value |
|---------------------------|-------|
| Std Filters Nbr           | 1     |
| Rx Fifo0 Elmts Nbr        | 3     |
| Rx Fifo1 Elmts Nbr        | 3     |
| Tx Fifo Queue Elmts Nbr   | 3     |

---

## Firmware Initialization (Post Code Generation)

After generating code from CubeMX, the following steps are required in `main.c` before CAN FD is operational:

1. **Configure at least one receive filter** — without a filter, all messages are rejected
2. **Activate RX notification interrupt** using `HAL_FDCAN_ActivateNotification()` with `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`
3. **Start the FDCAN peripheral** with `HAL_FDCAN_Start()`
4. **Implement the callback** `HAL_FDCAN_RxFifo0Callback()` to handle incoming messages
5. **When transmitting**, set `BitRateSwitch` to `FDCAN_BRS_ON` in `FDCAN_TxHeaderTypeDef` to engage rate switching per frame

---

## CANable Adapter Settings

The CANable must be configured to match these exact bit timing parameters:

| Parameter       | Nominal (Arbitration) | Data Phase |
|-----------------|-----------------------|------------|
| Bitrate         | 500 kbps              | 2 Mbps     |
| Sample Point    | 82.4%                 | 82.4%      |

---

## Quick Reference — Key Design Decisions

| Decision                    | Choice          | Reason                                                    |
|-----------------------------|-----------------|-----------------------------------------------------------|
| FDCAN clock source          | PLL Q (170 MHz) | Independent from PCLK1; no impact on motor control clocks |
| Data bitrate                | 2 Mbps          | Clean division from 170 MHz; safer than 5 Mbps on CANable |
| Nominal bitrate             | 500 kbps        | Robust arbitration; common industry standard               |
| Bit rate switching          | Enabled         | Required to use different nominal and data rates           |
| Transmit Pause              | Enabled         | Fair bus access for multi-node systems                     |
| Auto Retransmission         | Enabled         | Failed transmissions are automatically retried             |
| CAN termination             | Software-controlled | PC14 GPIO toggles on-board 120 Ω resistor             |
