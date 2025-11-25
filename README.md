## Labs Summary

### **Lab 1 — Assembly, C, and Optimization**  
*Focus:*  
- Integrating C + ARM assembly  
- Using CMSIS-DSP for optimized math  
- ITM instrumentation and SWV timeline profiling

*Key Skills:* Function timing, max-value implementation, ARMv7-M ISA

### **Lab 2 — GPIO, DAC, ADC**  
*Focus:*  
- Using GPIO for digital I/O  
- Generating DAC waveforms (triangle, saw, sine)  
- Sampling ADC internal channels for CPU temperature  
- Debugging with SWV Data Trace
  
*Key Skills:* Analog peripherals, waveform generation, ADC calibration

### **Lab 3 — Timers, Interrupts, DMA, DFSDM Mic**  
*Focus:*  
- Button interrupts (EXTI)  
- Timer-driven DAC → global interrupt mode + DMA mode  
- DFSDM microphone capture and processing
  
*Key Skills:* Real-time audio, DMA circular buffers, interrupt flow

### **Lab 4 — I2C, UART, FreeRTOS**  
*Focus:*  
- Reading I2C sensors via BSP  
- Printing sensor data over UART  
- Implementing multithreaded FreeRTOS tasks  
- QSPI Flash logging
  
*Key Skills:* RTOS task scheduling, sensor reads, memory-mapped flash


## Final Project — Reflex-Based Timing Game  
A multi-modal reaction game built on the STM32L4 board using:

- **Audio cues** (DAC + TIM2 + DMA)  
- **LED cues** (GPIO)  
- **Button reaction detection** (EXTI)  
- **Sound reaction detection** (DFSDM microphone)  
- **Tilt reaction detection** (I2C accelerometer)  
- **UART game UI** (score, logs, high score)  
- **QSPI Flash storage** (persistent statistics)  
- **FreeRTOS** task-based architecture  

### Game Flow
1. Display rules and previous player statistics  
2. Wait for user start with button press
3. Play 3-tone countdown for game-start  
4. Present 30 randomized cues that the user must respond to with button press, tilting the board left/right, or making a sound
5. Detect matching reaction within a 2-second time window  
6. Score PASS/MISS on UART
7. End-of-game tones + score summary  
8. Restart game on button press  
---

## 📄 License
These projects are for academic purposes as part of **ECSE 444 (Microprocessors)** at McGill University.
