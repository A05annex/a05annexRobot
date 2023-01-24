package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

/**
 * This is the code that controls the A05annex default swerve base with MK4 drive modules. In your
 * {@code RobotContainer} constructor, please call the
 * {@link #setDriveGeometry(double, double, double, double, double, double)}
 * method to setup drive module calibration and geometry before any attempts are made to send
 * drive commands.
 */
public class DriveSubsystem extends SubsystemBase implements ISwerveDrive {
    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static DriveSubsystem INSTANCE;

    public static final boolean FIELD_RELATIVE = true;
    public static final boolean ROBOT_RELATIVE = false;

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     *
     * @return Returns this {@link DriveSubsystem}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DriveSubsystem();
        }
        return INSTANCE;
    }

    // create drive modules
    // first letter is right or left, second is front or rear
    private final Mk4NeoModule m_rf;
    private final Mk4NeoModule m_rr;
    private final Mk4NeoModule m_lf;
    private final Mk4NeoModule m_lr;

    // create NavX - the drive subsystem owns the NavX and is responsible for the heading update
    // cycle.
    private final NavX m_navx = NavX.getInstance();

    // These are the constants for the drive geometry. They will vary with different frames, and they are
    // initially not set - which will result in an error is you call any of the drive commands.
    /**
     * is the drive geometry set? Initially {@code false}, but sell be set to {@code true} when the
     * drive geometry and calibration has been initialized.
     */
    private boolean isDriveGeometrySet = false;
    private double DRIVE_LENGTH;
    private double DRIVE_WIDTH;
    private double LENGTH_OVER_DIAGONAL;
    private double WIDTH_OVER_DIAGONAL;
    // The maximum spin of the robot when only spinning. Since the robot drive centers are
    // rectangular, all wheels are aligned so their axis passes through the center of that rectangle, and all wheels
    // follow the same circular path at a radius of DRIVE_DIAGONAL/2.0 at MAX_METERS_PER_SEC. So this is
    // computed from DRIVE_DIAGONAL and MAX_METERS_PER_SEC:
    //     Max [radians/sec] = max speed [meters/sec] / (PI * radius) [meters/radian]
    //     Max [radians/sec] = MAX_METERS_PER_SEC / (Math.PI * DRIVE_DIAGONAL * 0.5)
    private double MAX_RADIANS_PER_SEC;
    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    private double DRIVE_TICS_PER_RADIAN;

    // keep track of last angles
    private final AngleD m_RF_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_RR_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_LF_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_LR_lastRadians = new AngleD(AngleD.ZERO);

    // keep track of the last chassis speeds for odometry
    private double m_thisChassisForward = 0.0;
    private double m_thisChassisStrafe = 0.0;
    private long m_lastTime = System.currentTimeMillis();
    private final AngleD m_lastHeading = new AngleD(AngleD.ZERO);
    private double m_lastChassisForward = 0.0;
    private double m_lastChassisStrafe = 0.0;

    private double m_fieldX = 0.0;
    private double m_fieldY = 0.0;
    private final AngleD m_fieldHeading = new AngleD(AngleD.ZERO);

    private boolean fieldRelative = true;

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        // initialize drive modules. The IDs are as described in the README for the project. We could make these
        // constants somewhere, but that does not make it anymore understandable. This is just what it is.
        m_rf = Mk4NeoModule.factory("right-front", 1, 2, 20);
        m_rr = Mk4NeoModule.factory("right-rear", 3, 4, 21);
        m_lf = Mk4NeoModule.factory("left-front", 7, 8, 23);
        m_lr = Mk4NeoModule.factory("left-rear", 5, 6, 22);
    }

    /**
     * Call this at the beginning of any method that requires a functioning swerve drive. If the drive
     * geometry has not been set, then there will not be a functioning swerve drive.
     */
    private void testGeometryIsSet() {
        if (!isDriveGeometrySet) {
            System.out.println();
            System.out.println("**********************************************************************");
            System.out.println("***** THE DRIVE GEOMETRY MUST BE SET BEFORE YOU TRY TO DRIVE !!! *****");
            System.out.println("**********************************************************************");
            System.out.println();
            throw new IllegalStateException("The drive geometry has not been set!");
        }
    }

    /**
     * Set the swerve drive geometry and calibration constants. This must be called in your {@code Robot.robotInit()}
     * to describe the geometry and calibration of the swerve drive before any commands using this geometry
     * (like drive commands) are called. It should <i><b>NEVER</b></i> be called a second time.
     *
     * @param driveLength   (double) The length of the drive in meters.
     * @param driveWidth    (double) The width of the drive in meters.
     * @param rfCalibration (double) The reading of the right front spin encoder when the wheel is facing
     *                      directly forward.
     * @param rrCalibration (double) The reading of the right rear spin encoder when the wheel is facing
     *                      directly forward.
     * @param lfCalibration (double) The reading of the left front spin encoder when the wheel is facing
     *                      directly forward.
     * @param lrCalibration (double) The reading of the left rear spin encoder when the wheel is facing
     *                      directly forward.
     */
    @Override
    public void setDriveGeometry(double driveLength, double driveWidth,
                                 double rfCalibration, double rrCalibration,
                                 double lfCalibration, double lrCalibration) {
        if (isDriveGeometrySet) {
            throw new IllegalStateException("The drive geometry has already been set for this swerve drive.");
        }
        isDriveGeometrySet = true;
        DRIVE_LENGTH = driveLength;
        DRIVE_WIDTH = driveWidth;
        double driveDiagonal = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
        LENGTH_OVER_DIAGONAL = DRIVE_LENGTH / driveDiagonal;
        WIDTH_OVER_DIAGONAL = DRIVE_WIDTH / driveDiagonal;
        // For a module to travel 1radian, it moves a distance of:
        //    radius = (0.5 * driveDiagonal)
        // Note, this is in meters. Max m/sec / m/rad = rad/sec, so
        //    max m/sec / (0.5 * driveDiagonal) =  max rad/sec
        MAX_RADIANS_PER_SEC = Mk4NeoModule.MAX_METERS_PER_SEC / (0.5 * driveDiagonal);
        // tics/m * m/rad = tics/rad
        DRIVE_TICS_PER_RADIAN = Mk4NeoModule.TICS_PER_METER * 0.5 * driveDiagonal;
        m_rf.setCalibrationOffset(rfCalibration);
        m_rr.setCalibrationOffset(rrCalibration);
        m_lf.setCalibrationOffset(lfCalibration);
        m_lr.setCalibrationOffset(lrCalibration);
    }

    @Override
    public double getDriveLength() {
        testGeometryIsSet();
        return DRIVE_LENGTH;
    }

    @Override
    public double getDriveWidth() {
        testGeometryIsSet();
        return DRIVE_WIDTH;
    }

    @Override
    public double getMaxMetersPerSec() {
        testGeometryIsSet();
        return Mk4NeoModule.MAX_METERS_PER_SEC;
    }

    @Override
    public double getMaxRadiansPerSec() {
        testGeometryIsSet();
        return MAX_RADIANS_PER_SEC;
    }

    /**
     * Get the right-front drive module. Useful is you want to get information from the module (like the
     * absolute spin encoder value for spin calibration).
     *
     * @return Returns the right-front drive module.
     */
    @SuppressWarnings("unused")
    public Mk4NeoModule getRFModule() {
        return m_rf;
    }

    /**
     * Get the right-rear drive module. Useful is you want to get information from the module (like the
     * absolute spin encoder value for spin calibration).
     *
     * @return Returns the right-rear drive module.
     */
    @SuppressWarnings("unused")
    public Mk4NeoModule getRRModule() {
        return m_rr;
    }

    /**
     * Get the left-front drive module. Useful is you want to get information from the module (like the
     * absolute spin encoder value for spin calibration).
     *
     * @return Returns the left-front drive module.
     */
    @SuppressWarnings("unused")
    public Mk4NeoModule getLFModule() {
        return m_lf;
    }

    /**
     * Get the left-rear drive module. Useful is you want to get information from the module (like the
     * absolute spin encoder value for spin calibration).
     *
     * @return Returns the left-rear drive module.
     */
    @SuppressWarnings("unused")
    public Mk4NeoModule getLRModule() {
        return m_lr;
    }

    /**
     *
     */
    @SuppressWarnings("unused")
    public void resetDrivePID() {
        m_rf.setDrivePID();
        m_rr.setDrivePID();
        m_lf.setDrivePID();
        m_lr.setDrivePID();
    }

    /**
     * Print all module angles to SmartDashboard. Should be called in DriveSubsystem periodic if used.
     */
    public void printAllAngles() {
        SmartDashboard.putNumber("RF cal angle", m_rf.getCalibrationPosition());
        SmartDashboard.putNumber("RR cal angle", m_rr.getCalibrationPosition());
        SmartDashboard.putNumber("LF cal angle", m_lf.getCalibrationPosition());
        SmartDashboard.putNumber("LR cal angle", m_lr.getCalibrationPosition());

        SmartDashboard.putNumber("RF angle", m_rf.getDirectionPosition());
        SmartDashboard.putNumber("RR angle", m_rr.getDirectionPosition());
        SmartDashboard.putNumber("LF angle", m_lf.getDirectionPosition());
        SmartDashboard.putNumber("LR angle", m_lr.getDirectionPosition());
    }

    // begin swerve methods

    /**
     * The internal method to run, or prepare to run, the swerve drive with the specified {@code  forward},
     * {@code strafe}, and {@code rotation} chassis relative components.
     *
     * @param forward   Drive forward. From -1 (full backwards) to 1 (full forwards).
     * @param strafe    Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation  Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     * @param setSpeeds (boolean) {@code true} if module speeds should be set to run the modules, {@code false} if
     *                  this method is being called to prepare (set the direction of) the modules to run this command.
     *                  If {@code false}, module speeds will be 0.0 and there should be no robot motion.
     */
    private void setModulesForChassisMotion(double forward, double strafe,
                                            double rotation, boolean setSpeeds) {
        // calculate a, b, c and d variables
        double a = strafe - (rotation * LENGTH_OVER_DIAGONAL);
        double b = strafe + (rotation * LENGTH_OVER_DIAGONAL);
        double c = forward - (rotation * WIDTH_OVER_DIAGONAL);
        double d = forward + (rotation * WIDTH_OVER_DIAGONAL);

        // calculate wheel speeds
        double rfSpeed = Utl.length(b, c);
        double lfSpeed = Utl.length(b, d);
        double lrSpeed = Utl.length(a, d);
        double rrSpeed = Utl.length(a, c);

        // normalize speeds
        double max = Utl.max(rfSpeed, lfSpeed, lrSpeed, rrSpeed);
        if (max > 1.0) {
            rfSpeed /= max;
            lfSpeed /= max;
            lrSpeed /= max;
            rrSpeed /= max;
            forward /= max;
            strafe /= max;
        }

        // if speed is small or 0, (i.e. essentially stopped), use the last angle because its next motion
        // will probably be very close to its current last motion - i.e. the next direction will probably
        // be very close to the last direction.
        double SMALL = 0.000001;
        if (rfSpeed > SMALL) {
            m_RF_lastRadians.atan2(b, c);
        }
        if (lfSpeed > SMALL) {
            m_LF_lastRadians.atan2(b, d);
        }
        if (lrSpeed > SMALL) {
            m_LR_lastRadians.atan2(a, d);
        }
        if (rrSpeed > SMALL) {
            m_RR_lastRadians.atan2(a, c);
        }

        // run wheels at speeds and angles
        m_rf.setDirectionAndSpeed(m_RF_lastRadians, setSpeeds ? rfSpeed : 0.0);
        m_lf.setDirectionAndSpeed(m_LF_lastRadians, setSpeeds ? lfSpeed : 0.0);
        m_lr.setDirectionAndSpeed(m_LR_lastRadians, setSpeeds ? lrSpeed : 0.0);
        m_rr.setDirectionAndSpeed(m_RR_lastRadians, setSpeeds ? rrSpeed : 0.0);

        // save the values we set for use in odometry calculations
        m_thisChassisForward = setSpeeds ? forward : 0.0;
        m_thisChassisStrafe = setSpeeds ? strafe : 0.0;
    }

    /**
     * Run the swerve drive with the specified {@code  forward}, {@code strafe}, and {@code rotation} chassis
     * relative components.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    @Override
    public void swerveDriveComponents(double forward, double strafe,
                                      double rotation) {
        testGeometryIsSet();
        setModulesForChassisMotion(forward, strafe, rotation, true);
    }

    /**
     * Prepare the swerve drive to run with the swerve drive with the specified {@code  forward}, {@code strafe},
     * and {@code rotation} chassis relative components. 'Prepare', in this context, means orient all the modules
     * so they are ready to perform this command but set the module speeds to 0.0; In this way movement can start
     * smoothly without additional module reorientation. This method is used to initialize the robot before the
     * start of an autonomous path.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    @Override
    public void prepareForDriveComponents(double forward, double strafe,
                                          double rotation) {
        testGeometryIsSet();
        setModulesForChassisMotion(forward, strafe, rotation, false);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            //  If this is interrupted it is because the robot is being shut down - that is OK
        }
    }

    /**
     * Swerve drive with a robot-relative direction, a speed and a rotation speed.
     *
     * @param direction (AngleConstantD) The robot direction in radians from -PI to
     *                  PI. NOTE that this direction will be conditioned by whether the drive mode is robot-relative
     *                  or field-relative
     * @param speed     (double) Speed from 0.0 to 1.0.
     * @param rotation  (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    @Override
    public void swerveDrive(AngleConstantD direction, double speed, double rotation) {
        if (fieldRelative) {
            AngleD chassisDirection = new AngleD(direction).subtract(m_navx.getHeading());
            swerveDriveComponents(chassisDirection.cos() * speed,
                    chassisDirection.sin() * speed, rotation);
        } else {
            swerveDriveComponents(direction.cos() * speed,
                    direction.sin() * speed, rotation);
        }
    }

    @Override
    public void toggleDriveMode() {
        fieldRelative = !fieldRelative;
    }

    @Override
    public boolean getDriveMode() {
        return fieldRelative;
    }

    @Override
    public void setDriveMode(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
    }

    // end swerve methods
    // -------------------------------
    // begin odometry methods

    /**
     * Set the field position of the robot. This is typically called at the beginning of the autonomous
     * command as the command that is run should know where the robot has been placed on the field. It
     * could also be called during play if machine vision, sensors, or some other method is available
     * o locate the robot on the field.
     *
     * @param fieldX  (double) The X location of the robot on the field.
     * @param fieldY  (double) The Y location of the robot on the field.
     * @param heading (AngleD) The heading of the robot on the field.
     */
    @Override
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading) {
        m_fieldX = fieldX;
        m_fieldY = fieldY;
        m_fieldHeading.setValue(heading);
        m_navx.initializeHeadingAndNav(m_fieldHeading);
        m_lastTime = System.currentTimeMillis();
    }

    /**
     * Get what the swerve drive estimates to be the current X location of the robot on the field.
     *
     * @return The estimated X location of the robot on the field
     */
    @SuppressWarnings("unused")
    public double getFieldX() {
        return m_fieldX;
    }

    /**
     * Get what the swerve drive estimates to be the current Y location of the robot on the field.
     *
     * @return The estimated Y location of the robot on the field
     */
    @SuppressWarnings("unused")
    public double getFieldY() {
        return m_fieldY;
    }

    /**
     * Returns the heading of the robot on the field.
     *
     * @return (AngleD) A copy of the heading of the robot.
     */
    @SuppressWarnings("unused")
    public AngleD getFieldHeading() {
        return m_fieldHeading.cloneAngleD();
    }

    /**
     * Rotate the chassis to the specified heading with no field translation. This controls the module using distance
     * (i.e. moving a specified number of ticks) rather than speed because this adjustment of heading is faster
     * and more reliable.
     *
     * @param targetHeading (AngleConstantD) The desired chassis heading on the field.
     */
    public void setHeading(AngleConstantD targetHeading) {
        m_RF_lastRadians.atan2(DRIVE_LENGTH, -DRIVE_WIDTH);
        m_LF_lastRadians.atan2(DRIVE_LENGTH, DRIVE_WIDTH);
        m_LR_lastRadians.atan2(-DRIVE_LENGTH, DRIVE_WIDTH);
        m_RR_lastRadians.atan2(-DRIVE_LENGTH, -DRIVE_WIDTH);

        double deltaTics = new AngleD(targetHeading).subtract(m_navx.getHeading()).getRadians()
                * DRIVE_TICS_PER_RADIAN;

        m_rf.setDirectionAndDistance(m_RF_lastRadians, deltaTics);
        m_lf.setDirectionAndDistance(m_LF_lastRadians, deltaTics);
        m_lr.setDirectionAndDistance(m_LR_lastRadians, deltaTics);
        m_rr.setDirectionAndDistance(m_RR_lastRadians, deltaTics);

        m_thisChassisForward = 0.0;
        m_thisChassisStrafe = 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the NavX heading
        m_navx.recomputeHeading(false);
        // Update the odometry for the drive. OK, the scam here is that there was a previous heading set
        // in the last command cycle when we were setting the new direction/speed/rotation for the
        // chassis, and the heading we are at now. For odometry, assume the average of the last heading and current
        // heading approximates the path of the robot and that the last speed set happened pretty
        // instantaneously. In that case, we can make a pretty good guess how the robot moved on the field.

        // Get the average speed and heading for this interval
        AngleD currentHeading = m_navx.getHeading();
        AngleD aveHeading = currentHeading.cloneAngleD().add(m_lastHeading).mult(0.5);
        double aveForward = (m_lastChassisForward + m_thisChassisForward) * 0.5;
        double aveStrafe = (m_lastChassisStrafe + m_thisChassisStrafe) * 0.5;

        // the maximum distance we could travel in this interval at max speed
        long now = System.currentTimeMillis();
        double maxDistanceInInterval = Mk4NeoModule.MAX_METERS_PER_SEC * (double) (now - m_lastTime) / 1000.0;

        // compute the distance in field X and Y and update the field position
        double sinHeading = aveHeading.sin();
        double cosHeading = aveHeading.cos();
        m_fieldX += ((aveForward * sinHeading) + (aveStrafe * cosHeading)) * maxDistanceInInterval;
        m_fieldY += ((aveForward * cosHeading) - (aveStrafe * sinHeading)) * maxDistanceInInterval;

        // save the current state as the last state
        m_lastHeading.setValue(currentHeading);
        m_fieldHeading.setValue(currentHeading);
        m_lastChassisForward = m_thisChassisForward;
        m_lastChassisStrafe = m_thisChassisStrafe;
        m_lastTime = now;

        // telemetry
        printAllAngles();
    }
}

