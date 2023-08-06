# Tuning (Calibrating) the SpeedCachedSwerve

The <tt>SpeedCachedSwerve</tt> is a software layer for the swerve <tt>DriveSubsystem</tt> that, given a field position
sometime in the past, will predict where the robot is on the field now.

## What? Why?

Our initial programming to use april tags resulted in either extremely slow, or wildly unpredictable (oscillating)
robot motion when trying to reach the target position.
April tags were the primary visual field position markings for *2023 Charged Up*. They presented a challenge because
there is a latency in the april tag processing that can range from roughly 20ms to 100ms. So the robot position
data is sufficiently old that when the the april tag position is used as input to a PID robot position controller,
instability (oscillation) can only be prevented if PID constants are ridiculously small - resulting in the robot
creeping to the desired position.

During the *2023 Charged Up* season we developed a <tt>SpeedCachedSwerve</tt> to address this problem. The idea was
that this cache would save the timestamped commands to the swerve drive subsystem, and, given a sensor (april tag)
position sometime in the past, predict the current position of the robot so the optimal PID correction can be
computed. While this was reasonably successful, this was only an empirical evaluation (seemed better than without it).
So on the off-season, we decided to really investigate haw to best use april tags.

## The Testing Methodology

We elected to use a testing strategy where we would drive the robot around an april tag and record the april tag
sensed positions, the <tt>SpeedCachedSwerve</tt> commands, the predicted robot positions when the april tag positions
were read, and then plot all this stuff so we could really try to understand what is happening and how to best use
april tags. We had no idea what to expect when we started this exercise. The results were very interesting.

### First Test 2023-07-12 (12 July 2023)

For this test we dragged in the carpet, tried to stretch it and put heavy desks/tables on it to anchor it, taped an
april tag to the wall, and drove around a bit. Though we had some heavy furniture on the carpet, it was no stretched
out, so it moved a bit.