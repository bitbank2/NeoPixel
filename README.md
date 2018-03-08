NeoPixel
--------

Larry's NeoPixel (WS2812B) library<br>
Copyright (c) 2018 BitBank Software, Inc.<br>
Project started 3/7/2018<br>
Written by Larry Bank (bitbank@pobox.com)<br>
<br>
NeoPixels (aka WS2812B) are small, independent RGB LED modules containing an integrated controller.
The controller can produce any 24-bit color by modulating the brightness of the red/green/blue LEDs with PWM.
The controller uses a 1-wire communication protocol that allows them to be strung together in almost any length.
The data is sent as 24 bits in GRB order (MSB first). The first module in the chain captures the first 24-bits
then forwards the successive bits to the next in line. The modules latch/display their data when there is a pause
in the data stream. This means that for a string of N modules, Nx24 bits must be transmitted every time any of
the LEDs are to change their color.
<br>
Why re-invent the wheel? The challenge is to run a long string of LEDs on microcontrollers with limited
resources (like the ATTiny AVRs). This code allows you to create long runs of color patterns without needing
RAM to back each pixel. It accomplishes this by dynamically generating the colors based on a run-length encoding
scheme. On ATTiny controllers, the CPU is not fast enough to handle dynamicly calculated color gradients, so the
color delta values have to be explicitly defined in the data stream, but the effect is still good.
<br>

The run-length encoded byte stream defines a mode+length followed by the palette color<br>
 +-+-+-+-+-+-+-+-+<br>
 |M|M|L|L|L|L|L|L|   MM = 2 bits for mode, LLLLLL = 6 bits for length (1-64)<br>
 +-+-+-+-+-+-+-+-+<br>
 palette values follow the mode/length byte<br>

