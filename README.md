# Embedded-Real-Time-Systems-Software

- What interrupts are, how they work and how you write them for a typical micro-controller
- Typical problems when using interrupt service routines and integrating them with other parts of an application: shared variables, critical sections and atomic actions
- How to configure and use common peripheral devices – counters/timers, digital I/O, ADCs
- The trade-offs in CPU load when using polling software vs. interrupts
- How to achieve real-time precision and timing
- Refreshing your programming capabilities and knowledge of C



## Tasks for this Problem
- [x] Get the on board buttons to control the movement of the buggy using interrupts
- [ ] Use the optical encoders and interrupts to monitor and measure how far each wheel has travelled
- [ ] Implement an interrupt driven tune player that allows the tune to be started, stopped, paused, tempo changed and pitch shifted on demand.
- [ ] Devise a way that allows the CPU usage to be measured
- [ ] Display information from the above sub-systems on the OLED / 7 segment display / LED arrays (e.g. indicate tune status on the OLED, show current ‘drive gear’ on the 7 Segment display etc…)
