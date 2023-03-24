* **version:** 0.0.13
* **status:** first used for FRC **2023 Charged Up**
* **comments:** This code was extracted from our 2022 code base to provide a stable and growing code
  base to jump start future years.

# a05annexRobot

The A05annex base library for a Swerve Drive Base with NavX, autonomous paths with actions, and selection of
autonomous path (there are generally many possible paths depending on field start position and alliance strategy)
and driver preferences (drivers are conditioned by the games they play, the driver preference lets the driver natch
the robot response to their favorite game).

## Change Log

<details>
  <summary>version 0.0.1 to 0.0.13 (for <b>2023 Charged Up</b>):</summary>

  * 0.0.1 - Initial internal release;
  * 0.0.2 - Optional mirroring of autonomous for <b>2023 Charged Up</b>;
  * 0.0.4 - AbsoluteTranslateCommand - move by position;
  * 0.0.8 - Added getClosestDownField(), getClosestUpField(), getClosestDownOrUpField() to
            NavX.HeadingInfo;
  * 0.0.9 - Added max speed and heading correction at end of translate;
  * 0.0.10 - AbsoluteSmartTranslateCommand - first smart motion implementation;
  * 0.0.11 - Tuned smart motion coefficients;
  * 0.0.12 - Made heading correction after translate optional;
  * 0.0.13 - Added a NavX calibration factor to minimize rotational drift.
  * 0.0.15 - Current limited swerve
</details>

## Creating a Robot Project Using this Library

We have already done this for you. see [a05annexTemplate](https://github.com/A05annex/a05annexTemplate)


## Robot Swerve Drive

This project base assumes we are using a
[swerve drive specialties MK4 drive](https://www.swervedrivespecialties.com/products/mk4-swerve-module?variant=39376675143793)
swerve drive base. The specifics of the base geometry are ***VERY*** important in getting this software to correctly
control the drive, and the motor controllers and encoders must have the CAN bus IDs as shown below:
![alt text](./resources/SwerveConfiguration.jpg "Swerve Configuration")

We spent a lot of time working on optimal module performance, and this is
the [Swerve Programming paper](./resources/SwerveProgramming.pdf) that describes the details.

## Drive Control and Driver Tuning

Over multiple seasons we discovered that there are 2 driving modes for swerve drives that are critical:
* **Field Relative** - The driver is in a fixed position watching the robot on the field, and there is often no
  clear front/back to a robot. The speed-direction stick describes where the driver wants the robot to got relative
 to the field
* **Robot Relative** - This is what we always did with a conventional tank drive (right and left banks of wheels
  that here steered by setting some delta between right and left). The control is as though the driver is sitting
  in the robot (which is backwards when the robot is moving towards the driver). However, when you are performing 
  a precision task watching the screen display of the robot camera - you now need driver mode.

### Field Relative

What is happening in field-relative mode is we simply difference the stick direction with the
robot heading to transform the field relative direction to a robot relative direction.

### Robot Relative

What is happening in robot-relative mode is that stick direction is the robot-relative direction. Note that if
the camera is not facing directly forward, it is easy to change the robot-relative to camera relative by simply
differencing the stick direction with the
camera heading (relative to the robot) to transform the camera relative direction to a robot relative direction.

## NavX


## Autonomous Paths
