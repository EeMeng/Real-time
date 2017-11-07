# Real-time

The plan is to have 5(6) threads in total:

1. Main thread (will wait to join the following three threads)

Thread parent: Main thread
2. Main keyboard thread
3. DAC manager thread
4. ADC thread

Thread parent: DAC Manager thread
5. DAC0 output thread
6. DAC1 output thread (optional)
