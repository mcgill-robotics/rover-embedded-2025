# Debug LED Configuration — B-G431B-ESC1 ESC

## Overview

This document details the debug/status LED setup on the B-G431B-ESC1 Discovery kit. The LED provides visual feedback during CAN FD development and runtime diagnostics.

---

## Hardware Reference

| Parameter        | Value                          |
|------------------|--------------------------------|
| **Pin**          | PC6 (Pin 29)                   |
| **Signal Name**  | STATUS                         |
| **LED**          | Red user-configurable LED (LD2)|
| **CubeMX Label** | LED_STATUS                     |

> **Note:** The board also has a green LED (D5) on the 3.3 V rail. This is not software-controllable — it simply indicates power is present. The red LED on PC6 is the only user-programmable LED.

---

## STM32CubeMX Configuration

### GPIO Settings

| Parameter          | Value        |
|--------------------|--------------|
| Pin                | PC6          |
| Mode               | GPIO_Output  |
| Output Type        | Push-Pull    |
| Pull-up/Pull-down  | No pull      |
| Speed              | Low          |
| User Label         | LED_STATUS   |

### Generated Initialization Code

CubeMX generates the following in `MX_GPIO_Init()`:

```c
/* Configure GPIO pin Output Level */
HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
```

The LED starts OFF at initialization.

---

## Firmware API

### Basic Control

```c
/* Turn ON */
HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

/* Turn OFF */
HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

/* Toggle */
HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
```

---

## Suggested Debug Patterns

### CAN FD Development

During CAN FD bring-up, the LED can indicate communication state at a glance:

| Pattern              | Meaning                        | Where to implement                        |
|----------------------|--------------------------------|-------------------------------------------|
| Toggle on RX         | CAN frame received             | `HAL_FDCAN_RxFifo0Callback()`             |
| Toggle on TX         | CAN response sent              | After `HAL_FDCAN_AddMessageToTxFifoQ()`   |
| Solid ON             | Bus error (error passive/off)  | FDCAN interrupt 1 error handler           |
| 3× blink at startup  | FDCAN initialized successfully | `main()`, after `HAL_FDCAN_Start()`       |

### Example — Toggle on CAN RX

The simplest and most useful pattern during initial CAN FD testing. A flickering LED confirms frames are arriving from the CANable adapter:

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

        /* Existing RX handling ... */
    }
}
```

### Example — Startup Blink

A quick visual confirmation that the MCU booted and FDCAN started without hanging:

```c
/* After HAL_FDCAN_Start() in main() */
for (int i = 0; i < 3; i++)
{
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
}
```

---

## Integration with CAN_processing_v2

The LED can be toggled inside the stub handlers to verify the full RX-parse-TX pipeline:

- **`Handle_Run_Command()`** — toggle LED to confirm a run command was parsed
- **`Handle_Read_Command()`** — toggle LED to confirm a read command was parsed and a dummy response was sent back

This lets you visually verify that frames received by the FDCAN hardware are making it all the way through `CAN_Parse_MSG()` and into the application-level dispatch.

---

## Quick Reference

| Item                  | Detail                                      |
|-----------------------|---------------------------------------------|
| Pin                   | PC6                                         |
| CubeMX label          | LED_STATUS                                  |
| Initial state         | OFF (GPIO_PIN_RESET)                        |
| Primary debug use     | Toggle on CAN RX during bring-up            |
| Interrupt priority    | N/A — GPIO output only, no interrupt needed |
