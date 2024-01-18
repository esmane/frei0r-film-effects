# frei0r-film-effects
Frei0r effects that simulate the look of film and also analog video

## Gate Weave
The gate weave effect simulates the subtle motion of the image side to side as it passes through the projector. The way this works is by randomly generating points and then sliding the image towards these points. The interval parameter changes the amount of time between when new points are generated. A higher interval means slower movement. A value of 0.2 or less is probably going to get you the best results. The other two parameters adjust the maximum value that the random points could be. A value of 1 is equal to a maximum movement of 10 pixels to the left or the right. A value of zero means no movement at all on that axis. A value of 0.3 (equal to 3 pixels) or less is probably going to look the best. To create a realistic fake film effect you want it to be very subtle. Anything too much will look very corny. With higher resolutions you may need more movement however.

## Film Grain
The film grain effect simulates the actual grain of the film. The grain amount adjusts the overall amount of grain applied to the image. The red, green and blue values adjust the percentage of the overall grain applied to each color channel. This allows you to adjust the overall tint of the image. The dust amount adjust the frequency of which "dust particles" (pixels that are randomly set to full white or full black) will appear on the image. Even with this set to the maximum value however, dust particles are very rare. The blur amount adjusts the softness of the image. a higher value softens the image more. This works similar to a box blur but the values averaged together are given different, random weights so that the blur is not uniform across the image. This is important because with actual film the size of the crystals vary and therefore the resolution and softness of a piece of film is not constant. The final parameter, flicker, simulates the flicker of the shutter as the film passes through the projector. A value of 0.25 - 0.3 provides the best subtle effect in my opinion.

## NTSC-CRT
This effect simulates NTSC analog video and/or VHS tape noise. This one uses https://github.com/LMP88959/NTSC-CRT for the actual effect. I removed many of the options provided by the original and sort of messily combined it all together to better work as a video editor effect. The noise parameter controls the amount of signal noise, the VHS parameter toggles VHS emulation, and the progressive parameter toggles progressive vs interlaced scan modes.

## How to use
These effects should work with any video editing program that supports Frei0r effects. If you are using kdenlive on Windows you can simply download the dlls from this page and you should be good to go. Just copy them into the folder with the rest of the Frei0r dlls (Program Files/kdenlive/lib/frei0r-1) and the effects should appear automatically. Similarly, if you are using Linux you can download the .so's and copy them to the directory with the rest of the Frei0r effects.

I recommend applying these to the master channel after you finish your movie. The first effect in the chain should be gate weave. Then you need to do a very small zoom/transform effect so the ever-changing edge of the gateweaved frame is not visible. After that I recommend an extremely subtle vignette that is only really visible in the very corners of the image, and finally you should do film grain as your final effect.
