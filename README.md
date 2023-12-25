* **version:** 0.0.32
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
  <ul>
  <li>0.0.1 - Initial internal release;</li>
  <li>0.0.2 - Optional mirroring of autonomous for <b>2023 Charged Up</b>;</li>
  <li>0.0.4 - AbsoluteTranslateCommand - move by position;</li>
  <li>0.0.8 - Added getClosestDownField(), getClosestUpField(), getClosestDownOrUpField() to
            NavX.HeadingInfo;</li>
  <li>0.0.9 - Added max speed and heading correction at end of translate;</li>
  <li>0.0.10 - AbsoluteSmartTranslateCommand - first smart motion implementation;</li>
  <li>0.0.11 - Tuned smart motion coefficients;</li>
  <li>0.0.12 - Made heading correction after translate optional;</li>
  <li>0.0.13 - Added a NavX calibration factor to minimize rotational drift;</li>
  <li>0.0.15 - Current limited swerve drive and spin motors;</li>
  <li>0.0.16 - Changed ordering of swerves in translate commands to try to reduce rotational drift;</li>
  <li>0.0.17 - ISwerveDrive can be set for the A05DriveCommand allowing extension of the DriveSubsystem
             that add game-specific functionality;</li>
  <li>0.0.18 - Added a recalibrate method for the swerve so it could be recalibrated prior to any enable, this was
             a band-aid to not burning configuration into the Spark and having occasional configuration
             issues;</li>
  <li>0.0.19 - Fixed a burning configuration state into the Sparks problem introduced in 0.0.18;</li>
  <li>0.0.20 - Added a methods to IServeDrive to get the actual underlying subsystem;</li>
  <li>0.0.21 - Code to burn default configuration into the Sparks;</li>
  <li>0.0.23 - Post-competition cleanup. Moving common Spark-NEO and Spark-NOE550 combination into
             a tested wrapper that formalizes our 95% use case into a sample and repeatable pattern;</li>
  <li>0.0.29 - Added SpeedCachedSwerve;</li>
  <li>0.0.30 - Added A05AprilTagPositionCommand;</li>
  <li>0.0.31 - Cleanup and testing of SwerveSpeedCache phase adjustment.</li>
  </ul>
</details>
<details>
  <summary>version 0.0.31 to 0.0.?? (for <b>2024 Crescendo</b>):</summary>
  <ul>
  <li>0.0.32 - 2024 WPIlib beta version 2024.1.1-beta-4 integration:</li>
    <ul>
    <li>update build to gradle 8.4;</li>
    <li>Java support to Java 17;</li>
    <li>update all vendor dependencies;</li>
    <li>replaced deprecated edu.wpi.first.wpilibj2.command.CommandBase class with
      edu.wpi.first.wpilibj2.command.Command;</li>
    <li>Migrate from CTRE Phoenix5 to Phoenix6 software (only applies to the CAN coders on the swerve
      modules (a painful change in the programming model and the initialization defaults for the
      CANcoder).</li>
    </ul>
  </ul>
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
We use the
We also discovered that the drive control should be tuned to the driver. Occasional drivers or guest drivers during
robot demonstrations should be highly constrained so the robot doesn't smash things at high speeds. Competition
drivers should tune control to match their favorite game.
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

The most important aspect of driver tuning is the realization that it is a necessity, not an option. There are
potentially many different drivers of the robot with very different driving skill sets and that it is desirable to
be able to specify a specific driver or generalized driver skill set, and reset the robot control to maximize
the probability of success for that driver. How do we do that? We save a variety
of driver profiles and support loading the appropriate driver profile when the robot is powered-up.

We have tried a number of different controller types and configurations through the years and have settled on the
use of Xbox style controllers, one for the driver, and one for the operator.
* **Driver Xbox Controller** - Everything that is about manually driving the robot including:
  * Either field relative (driving from the driver station) or robot relative (driving by the robot mounted camera);
  * Aids for orienting the robot downfield, upfield, or in operation-specific headings;
  * Accelerated (*boost*) motion override;
  * Fine (*slow*) motion override.
* **Operator Xbox Controller** - the operator does everything else - including adjusting the drive tuning parameters
  at the drivers direction. We have a robot code branch specifically for tuning where all the operator controls
  are mapped to tuning parameters that map controller actions to robot behaviour.

#### What Do We Tune?

![alt text](./resources/driver_tuning.jpg "Driver Tuning")
There are a number of control parameters we tune. Review the illustration above - what driver tuning does is map the
raw signal from the stick to a conditioned signal representing the desired driver action. The sensitivity is an
exponent applied to the signal, so a gain of 1.0, deadband of 0.0, and exponent of 1.0 will result in the driver
tuned signal being identical to the raw signal. 

We have found that robot translation and rotation are most intuitively decoupled, that is; one stick should control
translation (movement forward, backward, right,and left - with no change in robot heading), and the other should
control robot rotation (field heading). Similarly, the tuning for translation and rotation should be similarly
decoupled. Additionally, we have separated: normal driving conditions, a *boost* mode, and a *slow* mode. These modes
can be better described as:
* default (normal driving conditions) - similar to how you would want you car to behave in city traffic. You need a
  balance of control and speed, but you would seldom, if ever, use the maximum speed or minimum turning radius. This
  is your default driving profile for a match;
* *boost* - like getting on the freeway. The path is clear ahead, your robot is correctly oriented, and you want to
  use the maximum speed available.
* *slow* - like wanting to parallel park, you need control, not speed. An example of this would be positioning your
  robot to attach to a climbing bar for and end-game robot hang.

The important parameters for driver tuning are:
* **gain** (speed, boost, slow, or rotation) - This is the maximum translation speed (forward, backward, left,
  and/or right) for *normal*, *boost*, or *slow* driving; or maximum rotation speed.
* **deadband** (speed or rotation) - Deadband is especially important for rotation because when there is no rotation
  input (rotation = 0.0), the DriveCommand maintains robot heading (i.e. if heading is changed due to drive
  drift, robot contact with field elements, or robot contact with other robots, the drive will automatically
  restore the heading. The deadband
  should be large enough to assure that minor unintentional pressure on the rotation stick, or failure of the stick
  to center to exactly 0.0 is not interpreted as a rotation command.
* **sensitivity** (speed or rotation) - This is an exponent applied to the signal. An exponent greater than 1.0
  flattens the curve next to 0.0 and gives better control at low speeds. An exponent less than one accelerates the
  robot quickly, and gives better control at the high speeds.
* **maximum acceleration** (speed or rotation) - This is a limit on the change in speed or rotation in a command
  cycle. This limit should be set so the wheels don't go into a skid mode as that makes field position telemetry
  completely unreliable.

#### How Do We Tune?

We tune using the
[*drive-tuner* branch of Ao5annex/ao5annexTemplate](https://github.com/A05annex/a05annexTemplate/tree/drive_tuner)
which is simply a swerve drive robot with the driver control and a mapping of operator controls to manipulate the
tuning parameters. This is a good first exercise in driver-operator communication as the driver needs to communicate
to the operator the desired changes in driver settings while driving and testing settings.

#### How Do We Save a Driver Profile?

All driver profiles are saved in *.json* files, and those files are in the *./src/main/deploy/drivers* directory
of your robot project so that they will be downloaded to the robot with the rest of your robot code.
The driver profile is saved as a simple list of driver coefficients in a dictionary with these keys:
- **<tt>"DRIVE_DEADBAND"</tt>**: (required,double) The distance from 0.0 that is the 'dead' area of the stick for robot
  speed. Generally as small as possible without introducing drift when a driver thinks they are going pure
  forward-back, or left-right. Generally <= 0.05.
- **<tt>"DRIVE_SPEED_SENSITIVITY"</tt>**: (required,double) Speed sensitivity. A sensitivity of 1.0 matches the linear
  raw performance of the stick. We find 2.0 to 3.0 is generally a range where the driver feels they have sufficient
  low-speed, or 'fine' control.
- **<tt>"DRIVE_SPEED_GAIN"</tt>**: (required,double) The maximum robot speed the driver feels comfortable with in
  normal driving where 0.0 is the robot stopped, and 1.0 is the fastest the robot can possibly go.
- **<tt>"DRIVE_SPEED_MAX_INC"</tt>**: (required,double) The maximum speed increment (either accelerate or decelerate)
  in a command cycle. Generally something in the 0.05 to 0.15 range gives good control without sending the robot into
  skid modes when the driver does something extreme.
- **<tt>"ROTATE_DEADBAND"</tt>**: (required,double) See <tt>DRIVE_DEADBAND</tt>. The near zero dead zone for the
  rotation stick. It is important that this be large enough to prevent un-intended rotation commands as any rotation
  command resets the desired heading for the robot.
- **<tt>"ROTATE_SENSITIVITY"</tt>**: (required,double) Rotation sensitivity, see <tt>DRIVE_SPEED_SENSITIVITY</tt>.
- **<tt>"ROTATE_GAIN"</tt>**: (required,double)  The maximum robot rotation the driver feels comfortable with in
  normal driving where 0.0 is the robot stopped, and 1.0 is the fastest the robot can possibly spin.
- **<tt>"ROTATE_MAX_INC"</tt>**: (required,double)  The maximum rotation increment (either accelerate or decelerate)
  in a command cycle. Generally something in the 0.05 to 0.15 range gives good control without sending the robot into
  skid modes when the driver does something extreme.
- **<tt>"BOOST_TRIGGER"</tt>**: (required,string) <tt>LEFT</tt> or <tt>RIGHT</tt> - the trigger that
  invokes *boost* mode.
- **<tt>"BOOST_GAIN"</tt>**: (required,double) The speed gain when the driver has the robot lined up and wants to go
  at the fastest speed the robot can attain. Generally 1.0 for competition drivers, and perhaps a little higher than
  <tt>DRIVE_SPEED_GAIN</tt> for non-competition drivers.
- **<tt>"SLOW_TRIGGER"</tt>**: (required,string)  <tt>LEFT</tt> or <tt>RIGHT</tt> - the trigger that
  invokes *slow* mode. Must not be the same as <tt>BOOST_TRIGGER</tt>.
- **<tt>"SLOW_GAIN"</tt>**: (required,double) The speed gain for precision positioning of the robot, often using the
  camera and *Robot Relative* drive control. Generally about 0.5 times the <tt>DRIVE_SPEED_GAIN</tt>.

This is an example of a driver profile file (for our primary 2023 competition driver):
```
{
  "DRIVE_DEADBAND": 0.05,
  "DRIVE_SPEED_SENSITIVITY": 2.0,
  "DRIVE_SPEED_GAIN": 0.6,
  "DRIVE_SPEED_MAX_INC": 0.075,
  "ROTATE_DEADBAND": 0.05,
  "ROTATE_SENSITIVITY": 1.5,
  "ROTATE_GAIN": 0.4,
  "ROTATE_MAX_INC": 0.075,
  "BOOST_TRIGGER": "LEFT",
  "BOOST_GAIN": 1.0,
  "SLOW_TRIGGER": "RIGHT",
  "SLOW_GAIN": 0.4
}
```

#### How Do We Configure Driver Setting for a Match?

The steps in making the driver profile configurable are:
* Build a driver profile and adopt a naming convention for the driver profiles, we normally just use the
  driver's first name;
* Put the profile in the *./src/main/deploy/drivers* directory;
* Assuming you have used the [a05annexTemplate](https://github.com/A05annex/a05annexTemplate) as a starting point for
  your project (or are just referring to that templete in *github*) - in <code>frc.robot.Constants.java</code> find the
  <code>A05Constants.DriverSettings[] DRIVER_SETTINGS</code> array, and add an entry for your driver. The
  <code>id</code> is the driver selection switch value for that driver, and is used to make sure switches are being
  mapped to the correct driver file.
* We configure the driver by setting switches during robot setup on the field (really, while we are in the match queue),
  to select the driver profile. This means the driver profile is loaded at robot power-up, regardless of the driver
  station state or the field control system state. This has been 100% reliable for us (barring human failure to throw
  the correct switches) during competition. Inspecting <code>A05RobotContainer</code> constructor, you will see that
  the first thing that happens is reading the configuration switches (switches 1 and 2, 1 being the first switch which
  is wired to the Roborio DIO 0 port, and 2 being the second switch which is wired to the Roborio DIO 1 port) and
  loading the corresponding driver profile.
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
