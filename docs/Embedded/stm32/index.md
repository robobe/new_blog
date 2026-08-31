---
title: STM32 Learning path
tags:
    - stm32
    - f3discovery
    - learning
---

# STM32 learning path

Follow the stages in order, from identifying the board hardware to running an RTOS.

<div class="grid-container">
    <div class="grid-item">
        <a href="00_board_orientation/">
            <p>Stage 0 — Board Orientation</p>
        </a>
        <details>
            <summary>Topics</summary>
            <ul>
                <li>Identify the MCU</li>
                <li>Identify ST-LINK</li>
                <li>Identify LEDs and buttons</li>
                <li>Identify headers</li>
            </ul>
        </details>
    </div>
    <div class="grid-item">
        <p>Stage 1 — MCU Basics</p>
        <details>
            <summary>Planned topics</summary>
            <ul>
                <li>Cortex-M4</li>
                <li>Flash and SRAM</li>
                <li>Memory map</li>
                <li>Registers</li>
            </ul>
        </details>
    </div>
    <div class="grid-item">
        <a href="hello_led/">
            <p>Stage 2 — GPIO</p>
        </a>
        <details>
            <summary>Topics</summary>
            <ul>
                <li>RCC peripheral clock</li>
                <li>Input and output modes</li>
                <li>MODER</li>
                <li>IDR, ODR, and BSRR</li>
                <li>Pull-up and pull-down resistors</li>
            </ul>
        </details>
    </div>

    <div class="grid-item">
        <p>Stage 3 — Debugging</p>
        <details>
            <summary>Planned topics</summary>
            <ul>
                <li>ST-LINK</li>
                <li>OpenOCD</li>
                <li>GDB</li>
                <li>Breakpoints</li>
                <li>Inspecting registers</li>
            </ul>
        </details>
    </div>

    <div class="grid-item">
        <p>Stage 4 — Interrupts</p>
        <details>
            <summary>Planned topics</summary>
            <ul>
                <li>NVIC</li>
                <li>Interrupt service routines</li>
                <li>Button interrupts</li>
            </ul>
        </details>
    </div>

    <div class="grid-item">
        <p>Stage 5 — Timers</p>
        <details>
            <summary>Planned topics</summary>
            <ul>
                <li>Hardware timers</li>
                <li>Timer interrupts</li>
                <li>PWM</li>
            </ul>
        </details>
    </div>

    <div class="grid-item">
        <p>Stage 6 — Communication</p>
        <details>
            <summary>Planned topics</summary>
            <ul>
                <li>UART</li>
                <li>SPI</li>
                <li>I²C</li>
            </ul>
        </details>
    </div>

    <div class="grid-item">
        <p>Stage 7 — DMA</p>
        <small>Planned</small>
    </div>

    <div class="grid-item">
        <p>Stage 8 — ADC</p>
        <small>Planned</small>
    </div>

    <div class="grid-item">
        <p>Stage 9 — FreeRTOS and Zephyr</p>
        <small>Planned</small>
    </div>
</div>
