package org.a05annex.frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DriverStation;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;

/**
 * This class represents and controls an
 * <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS MK4</a>
 * Swerve Drive module powered by <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motors controlled
 * using <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark Max</a> motor controllers and with a
 * <a href="https://store.ctr-electronics.com/cancoder/">CTRE CANcoder</a> as the position encoder for
 * absolute wheel direction.
 */
public class Mk4NeoModule {

    // -----------------------------------------------------------------------------------------------------------------
    // The control constants for this specific serve module (The MK4 with Neo motors, Spark Max controllers,
    // and a CANcoder calibration encoder) with the standard gear ratio 8:14 to 1, and a Neo unadjusted free
    // speed of 12.0ft/sec. We measured the unloaded max speed of the Neo at about 5700RPM, here we limit the
    // maximum speed to 5000rpn, which is .8772 of the maximum free speed (which is specified as 12 ft/sec), or
    // 10.5ft/sec (3.2m/sec)
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * The maximum speed (RPM) we will ever request from the drive motor. The actual not load maximum RPM with
     * fully charged batteries is about 5700RPM, so this gives about 12% headroom for friction, drag, battery
     * depletion, etc.
     */
    static final double MAX_DRIVE_RPM = 5000;
    /**
     * The maximum speed of the module with this module implementation. This implementation sets the
     * max NEO RPM to {@link #MAX_DRIVE_RPM} of the
     * <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> documented 5676RPM free (unloaded) speed.
     * The
     * <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS MK4</a>
     * documented free (unloaded) velocity with the standard gear ratio is 12.0ft/sec or 3.6576m/sec.
     * <p>
     * This lets us estimate the velocity at max speed as:<br>
     * &nbsp;&nbsp;{@code MAX_METERS_PER_SEC} = ({@link #MAX_DRIVE_RPM} / 5676rpm) * 3.6576m/sec = 3.222m/sec<br>
     * However, we will express this using the calculation in the event we change {@link #MAX_DRIVE_RPM}
     *
     */
    public static final double MAX_METERS_PER_SEC = (MAX_DRIVE_RPM / 5676) * 3.6576;
    /**
     * The empirically measured drive motor position change per meter of travel. The constant name
     * {@code TICS_PER_METER} is in reference to motor position encoder tics, however in the REV Neo motors on
     * the REV Spark controller with the current REV drivers, the encoder tics are converted to revolutions, so
     * this is actually drive motor revolutions per meter of travel.
     * <p>
     * This value is probably changes with wheel wear, game surface, and other variables - so this is just
     * an approximate value. Empirically measured by Ethan Ready using the practice robot The practice carpet
     * on 10-mar-2023. At 5000prm, this is 3.136m/sec as a max speed.
     */
    public static final double TICS_PER_METER = 26.571;
    /**
     * Based on telemetry feedback, 1 wheel direction revolution maps to 12.7999 spin encoder revolutions
     */
    static final double RADIANS_TO_SPIN_ENCODER = 12.7999 / AngleD.TWO_PI.getRadians();

    /**
     * A tolerance used for determining when the final position has been reached. This
     * tolerance corresponds to {@code TARGET_POSITION_TOLERANCE} / {@link #TICS_PER_METER}, or
     * .007m (0.27in)
     */
    static final double TARGET_POSITION_TOLERANCE = 0.4;

    // PID values for the spin spark motor encoder position controller PID loop
    static double SPIN_kP = 0.5;
    static double SPIN_kI = 0.0;

    // PID values for the drive spark motor controller speed PID loop
    static double DRIVE_kP = 0.00003;
    static double DRIVE_kI = 0.000002;
    static double DRIVE_kFF = 0.000174;
    static double DRIVE_IZONE = 200.0;

    // PID values for the drive spark motor controller position PID loop
    static double DRIVE_POS_kP = 0.13;
    static double DRIVE_POS_kI = 0.0002;
    static double DRIVE_POS_IZONE = 2.0;

    // PID values for the drive spark motor controller smart motion PID loop
    static double SMART_MOTION_kP = 0.00005;
    static double SMART_MOTION_kI = 0.0000001;
    static double SMART_MOTION_kFF = 0.000174;
    static double SMART_MOTION_IZONE = 0.75;
    // -----------------------------------------------------------------------------------------------------------------
    // The module physical hardware
    // -----------------------------------------------------------------------------------------------------------------
    private final String swerveDrivePosition;
    // This is the physical hardware wired to the roborio
    @SuppressWarnings("FieldCanBeLocal")
    private final CANSparkMax driveMotor;
    @SuppressWarnings("FieldCanBeLocal")
    private final CANSparkMax directionMotor;
    private final CANCoder calibrationEncoder;

    // These are the components of the physical hardware
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;
    private final RelativeEncoder directionEncoder;
    private final SparkMaxPIDController directionPID;

    // -----------------------------------------------------------------------------------------------------------------
    // The module physical state
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * The CANcoder reading when the wheel is aligned directly forward.
     */
    private double calibrationOffset;
    /**
     * A multiplier for the speed that is either 1.0 (forward) or -1.0 (backwards) because the shortest
     * spin to the desired direction may be the backwards direction of the wheel, which requires the speed
     * to be reversed.
     */
    private double speedMultiplier = 1.0;
    /**
     * The last angle the wheel was set to, in radians. this may be either the front or the back of
     * the wheel - see {@link #speedMultiplier ) documentation for determining whether this is the
     * orientation of the front or the back. This will be in the range -pi to pi.
     */
    private final AngleD lastDirection = new AngleD(AngleUnit.RADIANS, 0.0);
    /**
     * The last direction encoder value that was set. Note, we always set the next spin by using a change angle.
     * This means the encoder setting can be anywhere from -infinity to +infinity.
     */
    private double lastDirectionEncoder;
    /**
     * The last speed value that was set for this module, in the range 0.0 to 1.0.
     */
    private double lastSpeed = 0.0;

    /**
     * The control mode that is currently active.
     */
    CANSparkMax.ControlType driveMode = CANSparkMax.ControlType.kVelocity;
    enum DriveSlotId {
        VELOCITY(0),
        SMART_MOTION(1),
        POSITION(2);

        final int value;
        DriveSlotId(int value) {
            this.value = value;
        }
    }


    /**
     * The target position if {@link #driveMode}{@code  != }{@link CANSparkMax.ControlType#kVelocity}.
     */
    private double targetPosition;

    /**
     * The factory that creates the DriveModule given the physical component software objects. This factory exists
     * so that we can pass in mock object for testing code without the presence of physical hardware to run the
     * tests on.
     *
     * @param swerveDrivePosition (string, not null) The named position of this module in the swerve drive for error
     *                            messaging.
     * @param driveCAN          (int) CAN address for the motor that drives the wheel forward.
     * @param spinCAN           (int) CAN address for the motor that spins the wheel around.
     * @param calibrationCAN    (int) CAN address for the CANcoder.
     * @return (not null) Returns the initialized drive module.
     */
    public static Mk4NeoModule factory(@NotNull String swerveDrivePosition, int driveCAN,
                                       int spinCAN, int calibrationCAN) {
        // basic code representations for physical hardware
        CANSparkMax driveMotor = new CANSparkMax(driveCAN, MotorType.kBrushless);
        CANSparkMax spinMotor = new CANSparkMax(spinCAN, MotorType.kBrushless);
        CANCoder calibrationEncoder = new CANCoder(calibrationCAN);
        // derived representations of components embedded in the physical hardware
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        RelativeEncoder spinEncoder = spinMotor.getEncoder();
        SparkMaxPIDController spinPID = spinMotor.getPIDController();
        return new Mk4NeoModule(swerveDrivePosition, driveMotor, driveEncoder, drivePID,
                spinMotor, spinEncoder, spinPID, calibrationEncoder);
    }

    /**
     * Instantiate a DriveModule. All of instanced robot hardware control classes are passed in so this
     * module can be tested using the JUnit test framework.
     *
     * @param swerveDrivePosition (string, not null) The named position of this module in the swerve drive for error
     *                            messaging.
     * @param driveMotor         (CANSparkMax, not null) The drive motor controller.
     * @param driveEncoder       (RelativeEncoder, not null) The drive motor encoder.
     * @param drivePID           (CANPIDController, not null) The drive motor PID controller.
     * @param directionMotor     (CANSparkMax, not null) The spin motor controller.
     * @param directionEncoder   (RelativeEncoder, not null) The spin motor encoder.
     * @param directionPID       (CANPIDController, not null) The spin motor PID controller.
     * @param calibrationEncoder (CANCoder, not null) The spin analog position encoder which provides
     *                           the absolute spin position of the module.
     */
    public Mk4NeoModule(@NotNull String swerveDrivePosition,
                        @NotNull CANSparkMax driveMotor, @NotNull RelativeEncoder driveEncoder,
                        @NotNull SparkMaxPIDController drivePID, @NotNull CANSparkMax directionMotor,
                        @NotNull RelativeEncoder directionEncoder, @NotNull SparkMaxPIDController directionPID,
                        @NotNull CANCoder calibrationEncoder) {
        this.swerveDrivePosition = swerveDrivePosition;

        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.drivePID = drivePID;
        this.directionMotor = directionMotor;
        this.directionEncoder = directionEncoder;
        this.directionPID = directionPID;
        this.calibrationEncoder = calibrationEncoder;

        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            // Initialize the calibration CANcoder
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.sensorCoefficient = 2 * Math.PI / 4096.0;
            config.unitString = "rad";
            config.sensorDirection = true;
            while (true) {
                ErrorCode errorCode = calibrationEncoder.configAllSettings(config);
                if ((ErrorCode.OK == errorCode) || (/*In Testing*/null == errorCode)) {
                    break;
                }
                DriverStation.reportWarning(
                        String.format("Swerve (%s) CANCoder config error: CAN id = %d; error =  %d", swerveDrivePosition,
                                calibrationEncoder.getDeviceID(), errorCode.value), false);
            }


            // reset motor controllers to factory default
            while (true) {
                REVLibError errorCode = this.driveMotor.restoreFactoryDefaults();
                if (REVLibError.kOk == errorCode) {
                    break;
                }
                DriverStation.reportWarning(
                        String.format("Swerve (%s) drive motor config error: CAN id = %d; error =  %d", swerveDrivePosition,
                                this.driveMotor.getDeviceId(), errorCode.value), false);
            }
            while (true) {
                REVLibError errorCode = this.directionMotor.restoreFactoryDefaults();
                if (REVLibError.kOk == errorCode) {
                    break;
                }
                DriverStation.reportWarning(
                        String.format("Swerve (%s) spin motor config error: CAN id = %d; error =  %d", swerveDrivePosition,
                                this.directionMotor.getDeviceId(), errorCode.value), false);
            }
        }

        this.directionMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,60000);
        this.directionMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4,60000);
        this.directionMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5,60000);
        this.directionMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6,60000);
        this.driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3,60000);
        this.driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4,60000);
        this.driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5,60000);
        this.driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6,60000);
    }


    /**
     * Set the calibration offset, initialize the direction motor encoder, spin the wheel until it faces
     * forward (the direction is 0.0), and set up the internal tracking/setting of direction relative to the last
     * direction and last direction motor position encoder.
     *
     * @param calibrationOffset The position reported by the calibration CANcoder wen the wheel is physically
     *                          positioned to be facing forward.
     */
    public void setCalibrationOffset(double calibrationOffset) {
        this.calibrationOffset = calibrationOffset;
    }

     private void initPID(SparkMaxPIDController pid, double kFF, double kP, double kI, double kIZone) {
        initPID(pid, kFF, kP, kI, kIZone, 0);
    }

    private void initPID(SparkMaxPIDController pid, double kFF, double kP, double kI, double kIZone, int slotId) {
        pid.setFF(kFF, slotId);
        pid.setP(kP, slotId);
        pid.setI(kI, slotId);
        pid.setD(0.0, slotId);
        pid.setIZone(kIZone, slotId);
        pid.setOutputRange(-1.0, 1.0, slotId);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Methods to question the module about its state and what its doing.
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * Returns the drive motor velocity (RPM) as read from the encoder
     *
     * @return The drive motor velocity (RPM)
     */
    public double getDriveEncoderVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the drive motor position as read from the encoder.
     *
     * @return The drive motor position as read from the encoder.
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the drive motor control mode that is currently active.
     *
     * @return The drive motor control mode that is currently active.
     */
    public CANSparkMax.ControlType getSparkControlType() {
        return driveMode;
    }

    /**
     * Returns the direction motor position as read from the encoder.
     *
     * @return The direction motor position as read from the encoder.
     */
    public double getDirectionPosition() {
        return directionEncoder.getPosition();
    }

    /**
     * Returns the value of the calibration encoder as a double. The value goes from 0.0 to 2pi, wrapping
     * around when the boundary between 0 and 2pi is reached. This method is provided primarily to read
     * the calibration encoder to determine the calibrationOffset that should be used for initialization.
     *
     * @return The analog direction encoder position.
     */
    public double getCalibrationPosition() {
        return calibrationEncoder.getAbsolutePosition();
    }

    /**
     * Returns the last speed that was set for this module in m/sec.
     *
     * @return The last speed that was set in m/sec.
     */
    public double getLastSpeed() {
        return lastSpeed * MAX_DRIVE_RPM;
    }

    /**
     * Returns the last speed that was set for this module normalized to 0.0-1.0.
     *
     * @return the last normalized speed that was set.
     */
    public double getLastNormalizedSpeed() {
        return lastSpeed;
    }

    /**
     * Get the last direction set for the module.
     *
     * @return the last direction set.
     */
    public AngleD getLastDirection() {
        return lastDirection;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Methods to tell the module to do something
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * This should be calledSet the NEO direction encoder value using the absolute direction encoder, so that
     * forward is an encoder reading of 0 tics.
     */
    void calibrate() {
        // --------------------------------------------------------
        // make, sure the motor setup is correct
        // --------------------------------------------------------
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            // invert the spin so positive is a clockwise spin
            directionMotor.setInverted(true);
            // current limit the motor to prevent popping breakers
            directionMotor.setSmartCurrentLimit(45, 20, 3000);
            // initialize the PIDs for the direction controller
            initPID(directionPID, 0.0, SPIN_kP, SPIN_kI, 0.0);

            // current limit the motor to prevent popping breakers
            driveMotor.setSmartCurrentLimit(60, 20, 2000);
            // initialize the PIDs for the drive controller
            initPID(drivePID, DRIVE_kFF, DRIVE_kP, DRIVE_kI, DRIVE_IZONE, DriveSlotId.VELOCITY.value);
            initPID(drivePID, SMART_MOTION_kFF, SMART_MOTION_kP, SMART_MOTION_kI,
                    SMART_MOTION_IZONE, DriveSlotId.SMART_MOTION.value);
            drivePID.setSmartMotionMinOutputVelocity(-0.0, DriveSlotId.SMART_MOTION.value);
            drivePID.setSmartMotionAllowedClosedLoopError(TARGET_POSITION_TOLERANCE, DriveSlotId.SMART_MOTION.value);
            initPID(drivePID, 0.0, DRIVE_POS_kP, DRIVE_POS_kI, DRIVE_POS_IZONE, DriveSlotId.POSITION.value);

            if (A05Constants.getSparkBurnConfig()) {
                directionMotor.burnFlash();
                driveMotor.burnFlash();
            }
        }


        double absolutePosition;
        // get the absolute position of the calibration CANcoder. The difference between this and the
        // calibration offset is the change indirection required to set the direction to 0. Note, this loop
        // is here because it occasionally takes a bit for the CANcoder to correctly initialize, and
        // out of range values may be reported.
        do {
            absolutePosition = calibrationEncoder.getAbsolutePosition();
        } while (absolutePosition < 0.0 || absolutePosition > Math.PI*2);
        // Now we know where the wheel actually is pointing, initialize the direction encoder on direction
        // motor controller to reflect the actual position of the wheel.
        directionEncoder.setPosition(
                (calibrationEncoder.getAbsolutePosition() - calibrationOffset) * RADIANS_TO_SPIN_ENCODER);
        // set the wheel direction to 0.0 (son the wheel is now facing forward), and setup the remembered
        // last wheel direction and last direction encoder  position.
        directionPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        lastDirection.setValue(AngleUnit.RADIANS, 0.0);
        driveMode = CANSparkMax.ControlType.kVelocity;
        lastDirectionEncoder = 0.0;
        lastSpeed = 0.0;
        speedMultiplier = 1.0;
    }

    /**
     * Set the module direction in radians. This code finds the closest forward-backward direction and sets the
     * foward-backaward multiplier for speed.
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     */
    public void
    setDirection(AngleConstantD targetDirection) {
        // The real angle of the front of the wheel is 180 degrees away from the current angle if the wheel
        // is going backwards (i.e. the lastDirection was the last target angle for the module
        AngleD realLastDirection = (speedMultiplier > 0.0) ? lastDirection :
                lastDirection.isLessThan(AngleD.ZERO) ? lastDirection.add(AngleD.PI) : lastDirection.subtract(AngleD.PI);
        AngleD deltaDirection = new AngleD(targetDirection).subtract(realLastDirection);
        speedMultiplier = 1.0;

        // Since there is wrap-around at -180.0 and 180.0, it is easy to create cases where only a small correction
        // is required, but a very large deltaDegrees results because the spin is in the wrong direction. If the
        // angle is greater than 180 degrees in either direction, the spin is the wrong way. So the next section
        // checks that and changes the direction of the spin is the wrong way.
        if (deltaDirection.isGreaterThan(AngleD.PI)) {
            deltaDirection.subtract(AngleD.TWO_PI);
        } else if (deltaDirection.isLessThan(AngleD.NEG_PI)) {
            deltaDirection.add(AngleD.TWO_PI);
        }

        // So, the next bit is looking at whether it better to spin the front of the wheel to the
        // target and drive forward, or, to spin the back of the wheel to the target direction and drive
        // backwards - if the spin is greater than 90 degrees (pi/2 radians) either direction, it is better
        // to spin the shorter angle and run backwards.
        if (deltaDirection.isGreaterThan(AngleD.PI_OVER_2)) {
            deltaDirection.subtract(AngleD.PI);
            speedMultiplier = -1.0;
        } else if (deltaDirection.isLessThan(AngleD.NEG_PI_OVER_2)) {
            deltaDirection.add(AngleD.PI);
            speedMultiplier = -1.0;
        }

        // Compute and set the spin value
        lastDirection.setValue(targetDirection);
        lastDirectionEncoder += (deltaDirection.getRadians() * RADIANS_TO_SPIN_ENCODER);

        directionPID.setReference(lastDirectionEncoder, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the direction and speed of the drive wheel in this module.
     *
     * @param targetDirection (double) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     * @param speed           (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
     *                        forward velocity.
     */
    public void setDirectionAndSpeed(AngleConstantD targetDirection, double speed) {
        if (driveMode != CANSparkMax.ControlType.kVelocity) {
            // going out of something other than speed control to speed control
            driveMode = CANSparkMax.ControlType.kVelocity;
        }
        setDirection(targetDirection);
        // Compute and set the speed value
        lastSpeed = speed;
        speed *= MAX_DRIVE_RPM * speedMultiplier;
        drivePID.setReference(speed, driveMode, DriveSlotId.VELOCITY.value);
    }

    /**
     * <p>
     * Set the direction and distance in encoder tics that the module should move. We use this for
     * targeting when we are trying to get very fast response and a very solid lock on the target either
     * by robot rotation (aiming) or translation (positioning). The key here is that we have
     * a targeting error that we can approximately convert to heading or position correction,
     * and we are using this approximation to specify the next move that gets us to target.
     * target.
     * </p><p>
     * This starts a move that may take multiple command cycles to achieve. Calling
     * this method again before the move is complete resets the target position. Calling
     * {@link #setDirectionAndSpeed(AngleConstantD, double)} cancels a move to position.
     * </p>
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     * @param deltaTics       (double) The number of tics the drive motor should move.
     * @param maxSpeed The maximum speed, in the range 0.0 to 1.0.
     */
    public void setDirectionAndDistance(AngleD targetDirection, double deltaTics, double maxSpeed) {
        if (driveMode != CANSparkMax.ControlType.kPosition) {
            // going out of something other than distance control to distance control
            // reset speed to zero
            drivePID.setReference(0.0, CANSparkMax.ControlType.kVelocity, DriveSlotId.VELOCITY.value);
            // setup the position PID
            driveMode = CANSparkMax.ControlType.kPosition;
        }
        drivePID.setOutputRange(-maxSpeed, maxSpeed, DriveSlotId.POSITION.value);

        setDirection(targetDirection);
        double targetTics = getDriveEncoderPosition() + (deltaTics * speedMultiplier);
        targetPosition = targetTics;
        drivePID.setReference(targetTics, driveMode, DriveSlotId.POSITION.value);
    }

    /**
     * <p>
     * Set the direction and distance in encoder tics that the module should move. We use this for
     * targeting when we are trying to get very fast response and a very solid lock on the target either
     * by robot rotation (aiming) or translation (positioning). The key here is that we have
     * a targeting error that we can approximately convert to heading or position correction,
     * and we are using this approximation to specify the next move that gets us to target.
     * target.
     * </p><p>
     * This starts a move that may take multiple command cycles to achieve. Calling
     * this method again before the move is complete resets the target position. Calling
     * {@link #setDirectionAndSpeed(AngleConstantD, double)} cancels a move to position.
     * </p>
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     * @param deltaTics       (double) The number of tics the drive motor should move.
     * @param maxSpeed        (double) This is the speed mapped to the range 0.0 to 1.0.
     * @param maxAcceleration (double) This is a REV Spark smart motion max RPM/sec -- so, 10000 gets the
     *                        robot to max speed in about .5 seconds.
     */
    public void setDirectionAndSmartMotionDistance(AngleD targetDirection, double deltaTics,
                                                   double maxSpeed, double maxAcceleration) {
        if (driveMode != CANSparkMax.ControlType.kSmartMotion) {
            // going out of something other than smart motion control to smart motion control
            // reset speed to zero
            drivePID.setReference(0.0, CANSparkMax.ControlType.kVelocity, DriveSlotId.VELOCITY.value);
            // And as far as I can tell from the documentation, this mode uses a max acceleration/deceleration
            // and max speed to control the motion until close to the target and then waffles around the target
            // until it gets in tolerance.
            driveMode = CANSparkMax.ControlType.kSmartMotion;
        }

        // now set up the smart motion speed and acceleration constants
        drivePID.setSmartMotionMaxVelocity(maxSpeed * MAX_DRIVE_RPM, DriveSlotId.SMART_MOTION.value);
        drivePID.setSmartMotionMaxAccel(maxAcceleration, DriveSlotId.SMART_MOTION.value);
        setDirection(targetDirection);
        double targetTics = getDriveEncoderPosition() + (deltaTics * speedMultiplier);
        targetPosition = targetTics;
        drivePID.setReference(targetTics, driveMode, DriveSlotId.SMART_MOTION.value);
    }

    /**
     * Test whether a move to a specified position (a target distance) has completed.
     *
     * @return {@code true} if the target distance has been reached or the module is no longer
     * being controlled by position, {@code false} otherwise.
     */
    public boolean isAtTargetDistance() {
        double currentPosition = getDriveEncoderPosition();
        // If driving by speed, he the move by distance is done. Otherwise, test for a tolerance
        // of 0.2 -> which converts to roughly +-0.25"
        return (driveMode == CANSparkMax.ControlType.kVelocity) ||
                ((currentPosition > targetPosition - TARGET_POSITION_TOLERANCE * 3) &&
                        (currentPosition < targetPosition + TARGET_POSITION_TOLERANCE * 3));
    }
}
