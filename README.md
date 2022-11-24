# BHY-Micropython-Driver
Micropython porting of the BOSH BHY driver 

This module was created for the MuHack Badge, tested only on a BHI160B chip.
The included RAM patch are only for the BHI160B.
Even if a decent amount of debugging and testing have been done, expect bugs and crashes.

# Know bug:
- The FIFO reading procedure is slow and does not avoid large data buffer, causing the system to crash for RAM exaustion. Try to be as fast as possible to work on that data, expecially with high sensor frequency (below 50Hz should be fine)
- The flush FIFO request does not work properly