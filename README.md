* **version:** 0.0.31
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
  <summary>version 0.0.1 to 0.0.31 (for <b>2023 Charged Up</b>):</summary>

  * 0.0.1 - Initial internal release;
  * 0.0.2 - Optional mirroring of autonomous for <b>2023 Charged Up</b>;
  * 0.0.4 - AbsoluteTranslateCommand - move by position;
  * 0.0.8 - Added getClosestDownField(), getClosestUpField(), getClosestDownOrUpField() to
            NavX.HeadingInfo;
  * 0.0.9 - Added max speed and heading correction at end of translate;
  * 0.0.10 - AbsoluteSmartTranslateCommand - first smart motion implementation;
  * 0.0.11 - Tuned smart motion coefficients;
  * 0.0.12 - Made heading correction after translate optional;
  * 0.0.13 - Added a NavX calibration factor to minimize rotational drift;
  * 0.0.15 - Current limited swerve drive and spin motors.
  * 0.0.16 - Changed ordering of swerves in translate commands to try to reduce rotational drift.
  * 0.0.17 - ISwerveDrive can be set for the A05DriveCommand allowing extension of the DriveSubsystem
             that add game-specific functionality.
  * 0.0.18 - Added a recalibrate method for the swerve so it could be recalibrated prior to any enable, this was
             a band-aid to not burning configuration into the Spark and having occasional configuration
             issues.
  * 0.0.19 - Fixed a burning configuration state into the Sparks problem introduced in 0.0.18.
  * 0.0.20 - Added a methods to IServeDrive to get the actual underlying subsystem.
  * 0.0.21 - Code to burn default configuration into the Sparks.
  * 0.0.23 - Post-competition cleanup. Moving common Spark-NEO and Spark-NOE550 combination into
             a tested wrapper that formalizes our 95% use case into a sample and repeatable pattern.
  * 0.0.29 - Added SpeedCachedSwerve
  * 0.0.30 - Added A05AprilTagPositionCommand
  * 0.0.31 - Cleanup and testing of SwerveSpeedCache phase adjustment.
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

<details>
<summary>Over multiple seasons we discovered that there are 2 driving modes for swerve drives that are critical:
<ul>
    <li><b>Field Relative</b> - The driver is in a fixed position watching the robot on the field, and there is often
     no clear front/back to a robot. The speed-direction stick describes where the driver wants the robot to go 
     relative to the field/driver;</li>
    <li><b>Robot Relative</b> - This is what we always did with a conventional tank drive (right and left banks of
     wheels that here steered by setting some delta between right and left). The control is as though the driver is
     sitting in the robot (which is backwards when the robot is moving towards the driver). However, when you are
     performing a precision task watching the screen display of the robot camera - you now need driver mode.</li>
</ul>
We also discovered that the drive control should be tuned to the driver. Occasional drivers or drivers during robot
demonstrations should be highly constrained so the robot doesn't smash things at high speeds. Competition drivers
should tune control to match their favorite game.
</summary>

### Field Relative

What is happening in field-relative mode is we simply difference the stick direction with the
robot heading to transform the field relative direction to a robot relative direction.

### Robot Relative

What is happening in robot-relative mode is that stick direction is the robot-relative direction. Note that if
the camera is not facing directly forward, it is easy to change the robot-relative to camera relative by simply
differencing the stick direction with the
camera heading (relative to the robot) to transform the camera relative direction to a robot relative direction.

### Driver Tuning

The most important aspect of driver tuning is the realization that there are potentially many different drivers of
the robot with very different driving skill sets and that it is desirable to be able to specify a specific driver
or generalized driver skill set, and reset the robot control to reflect that. How do we do that? We save a variety
of driver profiles and support loading the appropriate driver profile when the robot is powered-up.

</details>

## NavX

## Drive Calibration

<details>
<summary>Drive calibration is primarily focused on characterizing the performance of the <code>DriveSubsystem</code>
and <code>NavX</code> so that the robot behaves as expected during driver control, and, so that autonomous paths and
the <code>SwerveSpeedCache</code> can be accurately mapped to robot behaviour.
</summary>

</details>

## Autonomous Paths
