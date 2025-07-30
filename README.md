# STM32H743ZI GPIO & EXTI Bare‑Metal Examples

A minimal bare‑metal demo for the STM32H743ZI MCU showing three GPIO use cases:

1. **LED Blink** – simple LED toggle in a loop  
2. **Button Polling** – read USER button (PC13) and toggle LED on press  
3. **Button Interrupt** – configure EXTI on PC13 rising edge to toggle LED in an ISR

 ## 🔌 Hardware Connections

- **LED** → PB14  **LED** → PB7
- **USER Button** → PC13   
