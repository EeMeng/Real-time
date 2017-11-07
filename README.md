# Real-time

The plan is to have 5(6) threads in total: <br />

1. Main thread (will wait to join the following three threads)<br />
<br />
Thread parent: Main thread<br />
2. Main keyboard thread<br />
3. DAC manager thread<br />
4. ADC thread<br />
<br />
Thread parent: DAC Manager thread<br />
5. DAC0 output thread<br />
6. DAC1 output thread (optional)<br />
