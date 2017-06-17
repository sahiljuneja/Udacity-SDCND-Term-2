### Reflection

## Describe the effect each of the P, I, D components had in your implementation. 

Unfortunately, my hardware restrictions couldn't allow me to record a video of my implementation.

The P gain value tends to "pull" the car towards the center of the lane. Depending on the value it could oscillate around that center line. Initially, I had a very high value for this gain and the car oscillated a lot right at the beginning and went off the track. I reduced it drastically thereafter, till I found a value that worked best, and the car still oscillated about the center line, but it didn't go off-track. It was relatively stable for now.

After the above, I decided to set a value for the D gain (which was 0 till now). I set it to a value which I thought could be a baseline for me to decide whether to increase or decrease it. Turned out that value helped quite a bit in reducing the above oscillations. The D gain essentially helped decide how "quickly" the car returns to the center if it starts to oscillate. There were (and are) some jerky movements in the car as a result, but it's still quite effective.

The I gain was 0 till now as well. From prior projects, I knew that the I gain is usually kept to quite a small value. Based on the above two parameters, I noticed the car was biasing turning left a bit more than turning right, which was expected given the track as per me. I set the I gain value to negate that effect to an extent. Setting that value ensured that my car wasn't going over the edge.

## Describe how the final hyperparameters were chosen.

The gain values were chosen based on the behavior noticed as I describe above.

- Initially I set all values to 0.
- I then defined quite a high value for P ( > 1.0) which caused lots of oscillations. I drastically reduced it to 0.01 and tested what worked best to maintain the car's oscillation to a minimal level. I gradually reached the 0.15 value.
- I then defined the D gain to 1.0 and fine-tuned it a bit to use the current value. This helped reduce the oscillation but did increase some jerky movements. Luckily setting the value to 1.0 right off the bat helped and I didn't have to mess around by much.
- The car tended to go over the edge slightly for some parts of the track as I discuss above so I then added the I gain value. As I mentioned, I selected a very small value for this gain and with slight fine-tuning I set it to a value that would pass this project.

I must admit, manual tuning for this track was very easy (took me just a few iterations), but I didn't aim for the best results or better techniques. That's something I plan to do on my own time now. 