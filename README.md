# Pi Pico TMC220X Driver

This repository is a rewrite of https://github.com/kjk25/TMC2209_ESP32. I've made the motor calls nonblocking using `uasyncio` and done some refactoring, but huge props to [kjk25](https://github.com/kjk25) for the initial library.