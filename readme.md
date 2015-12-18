# Gaming controller

Gaming controller project can emulate a gamepad, joystick or other gaming controller via USB HID class. It runs on STM32F3 Discovery board.

## Sources
You can use 16 buttons, 4 rotary encoders, 8 ADC channels or onboard accelerometer.

## Input buttons

| Button | Pin    ||Button  | Pin    |
| ------ |:------:||:------:|:------:|
| 1      | PD 0   || 9      | PD 8   |
| 2      | PD 1   || 10     | PD 9   |
| 3      | PD 2   || 11     | PD 10  |
| 4      | PD 3   || 12     | PD 11  |
| 5      | PD 4   || 13     | PB 12  |
| 6      | PD 5   || 14     | PB 13  |
| 7      | PD 6   || 15     | PB 14  |
| 8      | PD 7   || 16     | PB 15  |

## Rotary encoders

| Encoder | Pin 1 | Pin 2 | Timer |
| ------- |:-----:|:-----:|:-----:|
| 1       | PA 8  | PA 9  | TIM 1 |
| 2       | PB 4  | PA 4  | TIM 3 |
| 3       | PD 12 | PD 13 | TIM 4 |
| 4       | PC 6  | PC 7  | TIM 8 |

## ADC channels

| Channel | Pin   |
| ------- |:-----:|
| 1       | PA 1  |
| 2       | PA 2  |
| 3       | PA 3  |
| 4       | PF 4  |
| 5       | PC 0  |
| 6       | PC 1  |
| 7       | PC 2  |
| 8       | PC 3  |
