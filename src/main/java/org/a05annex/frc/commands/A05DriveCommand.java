package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

/**
 * <p>The basic drive command for controlling the swerve base from an Xbox controller. The actual robot project
 * will probably override this command to add competition-specific functionality like targeting, position tracking
 * or field position goals, etc. An override of this class will probably add some instance variables for the added
 * functionality, instantiation or initialization setting/defaulting of these instance  variables, and an
 * override of the execute command that implements these additional capabilities and falls-back to basic
 * driver functionalities if the additional capabilities are not being summoned.
 * <p>
 * This base implementation of drive control includes:
 * <ul>
 *     <li><b>Xbox drive control</b> with translation (field motion relative to the drive) on the left stick, and robot
 *     rotation on the right stick;</li>
 *     <li><b>deadband around the stick center</b> to make sure the driver means to apply a robot motion before it is
 *     applied to the robots (on many controllers sticks don't quite go to 0,0 when you release them - this gives
 *     you the 0,0 you expect when you release a stick);</li>
 *     <li><b>Speed and Rotation sensitivity</b> to apply a power function to the linearity of the stick motion to
 *     actual output. All human perceptions are logarithmic, not linear. This adjustment helps make the robot more
 *     controllable (more sensitive) at low speeds with the stick.</li>
 *     <li><b>acceleration/deceleration limits</b> - when a driver lats go of a stick (1.0 to 0.0) or smashes
 *     a stick to the max (0.0 to 1.0), this asks the drives to do something not physically possible and results
 *     in the tires skidding (they are moving faster that the robot can accelerate, or slowing faster than the
 *     robot can decelerate, so they skid). These limits are a bit like anti-skid brakes in cars, because you
 *     (the driver) have more control when you are not skidding. These limits  dictate the maximum change in speed
 *     or rotation per command cycle to try to minimize skidding.</li>
 * </ul>
 */
public class A05DriveCommand extends CommandBase {

    /**
     * The swerve drive subsystem.
     */
    protected final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    /**
     * The NavX that provides (and maintains the expected) inertial navigation heading for the robot
     */
    protected final NavX m_navx = NavX.getInstance();

    /**
     * The drive controller for the robot.
     */
    protected XboxController m_driveXbox;

    /**
     * The custom driver settings for the current driver.
     */
    private final A05Constants.DriverSettings m_driver;

    /**
     * The raw stick Y read from the controller and corrected for direction
     */
    protected double m_rawStickY;
    /**
     * The raw stick X read from the controller
     */
    protected double m_rawStickX;
    /**
     * The raw rotation read from the controller
     */
    protected double m_rawStickRotate;

    /**
     * The requested field-relative direction determined after the stick values are conditioned.
     */
    protected AngleD m_conditionedDirection = new AngleD(AngleUnit.DEGREES,0.0);
    /**
     * The requested speed in the field-relative direction (from 0 to 1) conditioned from the stick values
     */
    protected double m_conditionedSpeed;
    /**
     * The requested rotation after the stick values are conditioned.
     */
    protected double m_conditionedRotate;
    // ---------------------------------------------
    // save last conditioned values, which are used to limit rate of change for the next execution of the
    // command. In the 2020 'at-home' competition we noted that really abrupt driver actions skidded the
    // robot making behaviour erratic and tearing up the treads. To counteract this we implemented maximum
    // speed and rotation deltas per command cycle which improve response because it reduces uncontrolled
    // skids, traction is maintained (like anti-skid brakes).
    /**
     * The conditioned direction.
     */
    protected AngleD m_lastConditionedDirection = new AngleD(AngleUnit.DEGREES,0.0);
    /**
     * The last conditioned speed.
     */
    protected double m_lastConditionedSpeed = 0.0;
    /**
     * The last conditioned rotation value.
     */
    protected double m_lastConditionedRotate = 0.0;

    /**
     * How far the trigger needs to be depressed before the boost or slow is activated.
     */
    public static final double TRIGGER_THRESHOLD = 0.5;

    private ISwerveDrive iSwerveDrive = m_driveSubsystem;

    /**
     * The constructor for the drive command.
     * @param xbox The driver xbox controller.
     * @param driver The custom driver settings for the current driver.
     */
    public A05DriveCommand(XboxController xbox, A05Constants.DriverSettings driver) {
        addRequirements(m_driveSubsystem);
        m_driveXbox = xbox;
        m_driver = driver;
    }

    /**
     * This does the basic driving with the swerve drive. If you want to add any custom code like targeting control
     * of rotation so the robot is always facing a target, override this method and add your targeting code, or
     * special behavior code between conditioning the stick, and sending the conditioned values to the drive
     * subsystem. For example, if targeting were on and the driver could fully control the robot field position, when
     * targeting was active it would be replacing the {@link #m_conditionedRotate} with a rotation it computed.
     */
    @Override
    public void execute() {
        // condition the left stick XY to be field relative speed and direction, and the right stick to be rotation.
        conditionStick();

        // now ask the drive subsystem to do that.
        iSwerveDrive.swerveDrive(m_conditionedDirection, m_conditionedSpeed, m_conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Read and save the raw stick values {@link #m_rawStickX}, {@link #m_rawStickX},
     * and {@link #m_rawStickRotate}. Condition the raw stick to set {@link #m_conditionedDirection},
     * {@link #m_conditionedSpeed}, and {@link #m_conditionedRotate}.
     * <p>
     * Conditioning the stick means reading the stick position and applying corrections to go from the physical stick positions
     * to effective values that should be used as the actual drive inputs. These corrections are:
     * <ul>
     *     <li>Determine whether a gain boost or slow should be applied. This is a temporary reset of the
     *         normal driving gain for an immediate increased gain (which is generally less controllable and used for
     *         simple maneuvers requiring speed - like racing to the other end of the field), or a decrease in gain
     *         (which is generally more controllable and used when fine control for robot positioning is
     *         required - like positioning for hang). If used, boost or slow is activated by left and right.</li>
     *     <li>Get the raw values from the control sticks.</li>
     *     <li>Convert the direction XY stick values into the field-relative direction and raw speed requested.</li>
     *     <li>Do the deadband and sensitivity conditioning of speed and rotation</li>
     *     <li>Do the gain correction and delta-limiting of speed and rotation</li>
     *     <li>Save these newly computed direction, speed, and rotation as the last conditioned direction,
     *         speed, and rotation so they are available for delta limiting them the next time this method
     *         is called.</li>
     * </ul>
     */
    protected void conditionStick() {
        // if pressing boost button, set gain to boost gain
        double gain = m_driver.getDriveSpeedGain();
        if ((null != m_driver.getBoostTrigger()) &&
                (m_driveXbox.getRawAxis(m_driver.getBoostTrigger().value) >= TRIGGER_THRESHOLD)) {
            gain = m_driver.getBoostGain();
        } else if ((null != m_driver.getSlowTrigger()) &&
                (m_driveXbox.getRawAxis(m_driver.getSlowTrigger().value) >= TRIGGER_THRESHOLD)) {
            gain = m_driver.getSlowGain();
        }

        // get the raw stick values - these the values ae read from the controller. The Y value is negated
        // because forward is negative from the stick, but positive for forward motion.
        // * left stick Y for field-relative forward/backward speed
        m_rawStickY = -m_driveXbox.getLeftY(); // Y is inverted so up is 1 and down is -1
        // * left stick X for strafe speed
        m_rawStickX = m_driveXbox.getLeftX();
        // * right stick X for rotation speed
        m_rawStickRotate = m_driveXbox.getRightX();

        // But we really want field-relative direction, distance from 0,0 (which corresponds to speed),
        // and rotation, and then we need to condition distance and rotation for deadband and sensitivity.
        // * the distance the stick has moved from 0,0, clipped to a maximum of 1.0 because the speed can
        //   never be greater than 1.0
        double speedDistance = Utl.clip(Utl.length(m_rawStickY, m_rawStickX), 0.0, 1.0);
        // * the direction is computed from the raw stick X and Y values. Note that is the speed distance is less
        //   than the DRIVE_DEADBAND, then we assume we are still going in the last direction until the driver moves
        //   the stick enough to tell us what is happening next.
        if (speedDistance < m_driver.getDriveDeadband()) {
            m_conditionedDirection.setValue(m_lastConditionedDirection);
        } else {
            m_conditionedDirection.atan2(m_rawStickX, m_rawStickY);
        }

        // to condition the speedDistance, we apply the deadband correction and then the sensitivity. lastly,
        // we limit the change in speed from the last speed to reduce skidding. We do the same conditioning for
        // deadband
        // * first, lets do speed - remember that in getting speedDistance the value is 0.0 to 1.0
        //   * deadband and sensitivity
        if (speedDistance < m_driver.getDriveDeadband()) {
            speedDistance = 0.0;
        } else {
            speedDistance = (speedDistance - m_driver.getDriveDeadband()) / (1.0 - m_driver.getDriveDeadband());
            speedDistance = Math.pow(speedDistance, m_driver.getDriveSpeedSensitivity()) * gain;
        }
        //   * delta limiting
        m_conditionedSpeed = Utl.clip(speedDistance, m_lastConditionedSpeed - m_driver.getDriveSpeedMaxInc(),
                m_lastConditionedSpeed + m_driver.getDriveSpeedMaxInc());
        // * Now let's do rotation - note, tha value here ranges from -1.0 to 1.0
        //   * take out rotation sign and store it for later
        double rotationMult = (m_rawStickRotate < 0.0) ? -1.0 : 1.0;
        //   * Need the rotation to be between 0.0 and 1.0 to apply deadband and sensitivity
        double rotation = Math.abs(m_rawStickRotate);
        // are we rotating?
        if (rotation < m_driver.getRotateDeadband()) {
            // no rotate, keep current heading or 0 if no NavX
            NavX.HeadingInfo headingInfo = m_navx.getHeadingInfo();
            if (headingInfo != null) {
                // This is a little PID correction to maintain heading
                rotation = new AngleD(headingInfo.expectedHeading).subtract(new AngleD(headingInfo.heading))
                        .getRadians() * A05Constants.getDriveOrientationkp();
                // clip and add speed multiplier
                rotation = Utl.clip(rotation, -0.5, 0.5) * this.m_conditionedSpeed;
                if (A05Constants.getPrintDebug()) {
                    System.out.println("**********");
                    System.out.println("Expected Heading: " + headingInfo.expectedHeading.getRadians());
                    System.out.println("Actual Heading:   " + headingInfo.heading.getRadians());
                    System.out.println("Rotation:         " + rotation);
                }
            } else {
                // no NavX
                rotation =  0.0;
            }
        } else {
            // rotating
            // adjust for deadband
            rotation = (rotation - m_driver.getRotateDeadband()) / (1.0 - m_driver.getRotateDeadband());
            // update expected heading
            m_navx.setExpectedHeadingToCurrent();
            // add sensitivity, gain and sign
            rotation =  Math.pow(rotation, m_driver.getRotateSensitivity()) * m_driver.getRotateGain() * rotationMult;
        }
        m_conditionedRotate = Utl.clip(rotation, m_lastConditionedRotate - m_driver.getRotateMaxInc(),
                m_lastConditionedRotate + m_driver.getRotateMaxInc());

        // set the last values as references for the delta limiting in the next call to this method
        m_lastConditionedDirection = m_conditionedDirection;
        m_lastConditionedSpeed = m_conditionedSpeed;
        m_lastConditionedRotate = m_conditionedRotate;
    }

    protected void setISwerveDrive(ISwerveDrive iSwerveDrive) {
        this.iSwerveDrive = iSwerveDrive;
    }

}
