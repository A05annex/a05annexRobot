package org.a05annex.frc.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;

/**
 * This class is the packaging for a <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motor
 * powered by a <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark MAX</a> motor controller. It binds
 * together the {@link com.revrobotics.CANSparkMax}, {@link com.revrobotics.RelativeEncoder}, and
 * {@link com.revrobotics.SparkMaxPIDController} into a single object.
 * <p>
 * The genesis of this class is that we had a lot of issues in previous years that we really resolved in the 2022-2023
 * competition season. Our standard pattern of <i>Spark Max - Neo motor</i> was:
 * <ul>
 *     <li>We previously accepted the Spark Max defaults for a number of things and discovered this was a bad
 *     idea, specifically:
 *     <ul>
 *         <li><b>Current Limits</b> - Brushless motors have very low resistance at or near stall and can draw
 *         huge  current without appropriate current limits, this will pop motor breakers and burn out motors,
 *         both of which we have experienced. So, these are the things to consider for current limits:
 *         <ul>
 *             <li><b>Motor Stall Tests</b> - REV does stall testing (Locked-rotor Testing) and these are the
 *             results for the <a href="https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/">NEO
 *             Brushless Motor - Locked-rotor Testing</a>. These tests tell us how long a different single NEO lasted
 *             at for each different test current (i.e. they only fried 1 NEO at each of the test currents). In
 *             engineering it is typical to use a safety factor of at least 2 for static structures (breaking
 *             strength of cables in a suspension bridge), and 5 in a dynamic use situation (like the cables it
 *             a crane). The tests were run at limits of 40Amp, 50Amp, 60Amp, and 80Amp (NOTE, 80Amp is the default
 *             current limit fora Spark MAX). Motors failed in the 60 and 80Amp tests, for 40 and 50Amp we guessed
 *             that the failure happened around 150&deg;C and extended the temperature curves to estimate a failure
 *             time. Here is a table of test results and various safety factors:
 *             <table>
 *                 <tr>
 *                     <th>Amps</th>
 *                     <th>failure (sec)</th>
 *                     <th>SF 2 (sec)</th>
 *                     <th>SF 3 (sec)</th>
 *                     <th>SF 5 (sec)</th>
 *                 </tr>
 *                 <tr>
 *                     <th>40</th>
 *                     <th>420</th>
 *                     <th>210</th>
 *                     <th>140</th>
 *                     <th>84</th>
 *                 </tr>
 *                 <tr>
 *                     <th>50</th>
 *                     <th>190</th>
 *                     <th>95</th>
 *                     <th>63</th>
 *                     <th>38</th>
 *                 </tr>
 *                 <tr>
 *                     <th>60</th>
 *                     <th>100</th>
 *                     <th>50</th>
 *                     <th>33</th>
 *                     <th>20</th>
 *                 </tr>
 *                 <tr>
 *                     <th>80</th>
 *                     <th>45</th>
 *                     <th>23</th>
 *                     <th>15</th>
 *                     <th>9</th>
 *                 </tr>
 *             </table>
 *             NOTE: a match is only 150sec long. We seldom see current over 20Amp with the exception of drive
 *             motors during extreme defensive play where it was not unusual to see 60-70Amps in short bursts (not
 *             much longer than 5sec because longer will draw penalties).
 *             </li>
 *             <li><b>Breaker Amperage and Supplier</b> - FRC uses thermal self-resetting breakers in sizes from 10Amp
 *             to 40Amp ordered from
 *             <a href="https://www.revrobotics.com/content/docs/REV-11-1860-1863-DS.pdf">REV</a>
 *             or <a href="https://cdn.andymark.com/media/W1siZiIsIjIwMTkvMDEvMjMvMTEvMzQvNDEvNzMzMGE1NTYtOGZmZC00ZDVlLWI1MjgtNmFkZDQzM2E2MWY1L01YNVNwZWNTaGVldC5wZGYiXV0/MX5SpecSheet.pdf?sha=b903664b28d21599">AndyMark</a>.
 *             While the breakers between the two have the same ratings, they do not behave the same, and the spec
 *             sheet reports things sufficiently differently that they are difficult to compare. The common things
 *             is that they will carry the rated amperage almost indefinitely and they will carry above the rated
 *             current for a <iu>short</iu> time - the higher over rated amperage the faster the breaker will trip.
 *             What is published is time to trip at a couple different loads - 135%, and 200% as:
 *             <table>
 *                 <tr>
 *                     <th>MFG</th>
 *                     <th>load</th>
 *                     <th>min (sec)</th>
 *                     <th>typ(sec)</th>
 *                     <th>max (sec)</th>
 *                 </tr>
 *                 <tr>
 *                     <th>REV</th>
 *                     <th>135%</th>
 *                     <th>30</th>
 *                     <th>70</th>
 *                     <th>1800</th>
 *                 </tr>
 *                 <tr>
 *                     <th>AndyMark</th>
 *                     <th>135%</th>
 *                     <th>2.8</th>
 *                     <th>??</th>
 *                     <th>1800</th>
 *                 </tr>
 *                 <tr>
 *                     <th>REV</th>
 *                     <th>200%</th>
 *                     <th>5</th>
 *                     <th>18</th>
 *                     <th>60</th>
 *                 </tr>
 *                 <tr>
 *                     <th>AndyMark</th>
 *                     <th>200%</th>
 *                     <th>1.5</th>
 *                     <th>??</th>
 *                     <th>3.9</th>
 *                 </tr>
 *             </table>
 *             </li>
 *             The main things to notice are that the trip time is highly variable, and that minimum trip time on the
 *             AndyMark breakers is considerably less than the REV breakers. Our swerve drive motors were on 40Amp
 *             (maximum size allowed for FRC) AndyMark breakers, and they would trip during extreme defensive play.
 *             <li><b>Motor Use</b> - How a motor is used affects how we should approach a determining an appropriate
 *             maximum current limit. Typically we see these use scenarios:
 *             <ul>
 *                 <li><b>Free Spinning</b> -</li>
 *                 <li><b>Occasional Stall</b> - </li>
 *                 <li><b>Prolonged Stall</b></li>
 *                 <li><b>Position (Always Stalled)</b></li>
 *             </ul>
 *             motors that are essentially free spinning motors
 *             </li>
 *         </ul>
 *         Current limiting uses the SparkMAX to control the current to the motor. This can protect both the motor
 *         from currents that would cause damage, and the breaker to prevent conditions that would cause the breaker
 *         to trip and disable the motor.
 * <p>
 *         </li>
 *         <li>Unnecessary CAN activity</li>
 *     </ul>
 *     </li>
 *     <li>We were previously always configuring starting with {@link CANSparkMax#restoreFactoryDefaults()} and then
 *     setting everything we needed. This was best practice with TalonSRX controllers several years ago. We
 *     experienced occasional initialization problems where it appeared the reset had not completed, and subsequent
 *     configuration calls were ignored or written over (the configuration was still the default value). Current best
 *     practice is to only configure this way during tuning, then burn the tuned configuration into the Spark Max
 *     flash memory so the Spark Max is correctly configured at power up. We experienced no startup problems after
 *     doing this.</li>
 *     <li>that we always used PID slot 0 which worked pretty well because we were not running multiple move modes
 *     on motors
 *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         hen we switched control mode we were also resetting PID constants - which led to unpredictable </li>
 *     </ul>
 * <p>
 * It provides a simple interface for configuration; PID control modes for:
 * <ul>
 *     <li>{@link com.revrobotics.CANSparkMax.ControlType#kVelocity} - use the {@link #setTargetRPM(double)}
 *     method to set the motor velocity RPM.</li>
 *     <li>{@link com.revrobotics.CANSparkMax.ControlType#kSmartMotion} </li>
 *     <li>{@link com.revrobotics.CANSparkMax.ControlType#kPosition}</li>
 * </ul>
 */
public class SparkNeo {

    /**
     * The factory for {@link SparkNeo} objects for the physical robot. When the robot is powered up and this
     * object is first created, it will represent in its powered up configuration
     *
     * @param canId The CAN id of this Spark MAX
     * @return
     */
    @NotNull
    public static SparkNeo factory(int canId) {
        CANSparkMax sparkMax = new CANSparkMax(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
        return new SparkNeo(sparkMax, sparkMax.getEncoder(), sparkMax.getPIDController());
    }

    /**
     *
     */
    enum PIDtype {
        RPM(0),
        SMART_MOTION(1),
        POSITION(2);

        final int slotId;

        PIDtype(int slotId) {
            this.slotId = slotId;
        }
    }

    enum UseType {
        FREE_SPINNING(0),
        RPM_OCCASIONAL_STALL(1),
        RPM_PROLONGED_STALL(2),
        POSITION(3);

        final int index;

        UseType(int index) {
            this.index = index;
        }
    }

    enum BreakerAmps {
        Amps10(0),
        Amps20(1),
        Amps30(2),
        Amps40(3);

        final int index;

        BreakerAmps(int index) {
            this.index = index;
        }
    }

    enum Direction {
        FORWARD(false),
        REVERSE(true);

        final boolean reversed;

        Direction(boolean reversed) {
            this.reversed = reversed;
        }
    }

    static final int[][] maxCurrentMatrix = {
            // UseType.FREE_SPINNING - a think that is essentially free-spinning. Like a pickup roller which is
            // essentially free spinning except for the momentary power blip during pick up. Current is expected
            // to be low, and since stall is completely unexpected, should be limited to a value the motor and
            // breaker can sustain forever.
            {10, 20, 30, 30},   // 10, 20, 30, 40 Amp
            // UseType.RPM_OCCASIONAL_STALL - you occasionally might do a thing that stalls the motor - like
            // hitting a physical stop.
            {10, 20, 30, 30},   // 10, 20, 30, 40 Amp
            // UseType.RPM_PROLONGED_STALL - speed control (like, of the drive) where you may be stalled (like,
            // playing defense), and you don't want to fry the motor or throw the breaker. The key here is the
            // prolonged stall is intentional and the driver will choose to continue rather than trying to recover
            // from the conditions causing the stall.
            {15, 30, 40, 50},   // 10, 20, 30, 40 Amp
            // UseType.POSITION - position control is essentially always holding the motor at a stall that
            // maintains the position.
            {10, 20, 30, 40}    // 10, 20, 30, 40 Amp
    };

    boolean inConfig = false;
    public final CANSparkMax sparkMax;
    public final RelativeEncoder encoder;
    public final SparkMaxPIDController sparkMaxPID;


    public SparkNeo(@NotNull CANSparkMax sparkMax, @NotNull RelativeEncoder encoder,
                    @NotNull SparkMaxPIDController sparkMaxPID) {
        this.sparkMax = sparkMax;
        this.encoder = encoder;
        this.sparkMaxPID = sparkMaxPID;
    }

    protected void verifyInConfig(boolean expectedInConfig, @NotNull String method) {
        if (this.inConfig != expectedInConfig) {
            System.out.println();
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.println("***** %s() may only be called %s configuration".formatted(method,
                    expectedInConfig ? "during" : "outside of"));
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.println();
            throw new IllegalStateException("%s() may only be called %s configuration".formatted(method,
                    expectedInConfig ? "during" : "outside of"));
        }
    }

    /**
     * Get the maximum free RPM. This is published in the
     * <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> summary as 5676RPM
     *
     * @return Returns the maximum free RPM
     */
    static public double getMaxFreeRPM() {
        // per REV Neo datasheet
        return 5676.0;
    }

    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void setEncoderPosition(double position) {
        verifyInConfig(false, "setEncoderPosition");
        encoder.setPosition(position);
    }

    /**
     * Call this to start your configuration of the SparkMAX, call {@link #endConfig()} to end the
     * configuration and optionally burn the configuration to the flash memory so it will be the
     * power-up configuration. Only methods specified as configuration methods may be called between
     * {@code startConfig()} and {@link #endConfig()}.
     */
    public void startConfig() {
        verifyInConfig(false, "startConfig");
        inConfig = true;
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            while (true) {
                REVLibError errorCode = sparkMax.restoreFactoryDefaults();
                if (REVLibError.kOk == errorCode) {
                    break;
                }
                DriverStation.reportWarning(
                        String.format("SparkMAX config error: CAN id = %d; error =  %d",
                                sparkMax.getDeviceId(), errorCode.value), false);
            }
        }

    }

    /**
     * Set the current limit. This is a configuration method and can only be called between {@link #startConfig()}
     * and {@link #endConfig()}.
     *
     * @param useType         The motor use type, which characterizes likelihood and duration of stall.
     * @param breakertAmps    The breaker amperage, which allows computing the breaker overload.
     */
    public void setCurrentLimit(@NotNull UseType useType,  @NotNull BreakerAmps breakertAmps) {
        verifyInConfig(true, "setCurrentLimit");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            int maxAmps = maxCurrentMatrix[useType.index][breakertAmps.index];
            sparkMax.setSmartCurrentLimit(maxAmps, maxAmps, 10000);
        }
    }

    /**
     * @param direction
     */
    public void setDirection(Direction direction) {
        verifyInConfig(true, "setReversed");
        sparkMax.setInverted(direction.reversed);
    }

    /**
     * @param idleMode
     */
    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        sparkMax.setIdleMode(idleMode);
    }


    /**
     * @param kP             The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI             The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone         The PID loop will not include the integral component until the current position or speed is
     *                       within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                       is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF            The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param maxRPM
     * @param maxRPMs
     * @param minRPMs
     * @param allowableError
     */
    public void setSmartMotion(double kP, double kI, double kIZone, double kFF,
                               double maxRPM, double maxRPMs, double minRPMs, double allowableError) {
        setSmartMotion(kP, kI, kIZone, kFF, 0.0, -1.0, 1.0, maxRPM, maxRPMs, minRPMs, allowableError);
    }

    /**
     * @param kP             The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI             The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone         The PID loop will not include the integral component until the current position or speed is
     *                       within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                       is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF            The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param kD
     * @param min
     * @param max
     * @param maxRPM
     * @param maxRPMs
     * @param minRPM
     * @param allowableError
     */
    public void setSmartMotion(double kP, double kI, double kIZone, double kFF, double kD, double min, double max,
                               double maxRPM, double maxRPMs, double minRPM, double allowableError) {
        verifyInConfig(true, "setSmartMotion");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            setPID(PIDtype.SMART_MOTION, kP, kI, kIZone, kFF, kD, min, max);
            int slotId = PIDtype.SMART_MOTION.slotId;
            sparkMaxPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, slotId);
            sparkMaxPID.setSmartMotionMaxVelocity(maxRPM, slotId);
            sparkMaxPID.setSmartMotionMaxAccel(maxRPMs, slotId);
            sparkMaxPID.setSmartMotionMinOutputVelocity(minRPM, slotId);
            sparkMaxPID.setSmartMotionAllowedClosedLoopError(allowableError, slotId);
        }
    }

    /**
     * Sets the PID constants for the specified PID control type..
     *
     * @param pidType The PID control type.
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     */
    public void setPID(@NotNull PIDtype pidType, double kP, double kI, double kIZone, double kFF) {
        setPID(pidType, kP, kI, kIZone, kFF, 0.0, -1.0, 1.0);
    }

    /**
     * Sets the PID constants for the specified PID control type..
     *
     * @param pidType The PID control type.
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param kD
     * @param min
     * @param max
     */
    public void setPID(@NotNull PIDtype pidType, double kP, double kI, double kIZone, double kFF,
                       double kD, double min, double max) {
        verifyInConfig(true, "setPID");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            int slotId = pidType.slotId;
            sparkMaxPID.setP(kP, slotId);
            sparkMaxPID.setI(kI, slotId);
            sparkMaxPID.setIZone(kIZone, slotId);
            sparkMaxPID.setFF(kFF, slotId);
            sparkMaxPID.setD(kD, slotId);
            sparkMaxPID.setOutputRange(min, max, slotId);
        }
    }


    /**
     *
     */
    public void endConfig() {
        verifyInConfig(true, "endConfig");
        inConfig = false;
        if (A05Constants.getSparkConfigFromFactoryDefaults() && A05Constants.getSparkBurnConfig()) {
            sparkMax.burnFlash();
        }
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 500);
    }

    // setIdle

    public void setTargetRPM(double targetRpm) {
        verifyInConfig(false, "setTargetRPM");
        sparkMaxPID.setReference(targetRpm, CANSparkMax.ControlType.kVelocity, PIDtype.RPM.slotId);
    }

    public void setSmartMotionTarget(double targetPosition) {
        verifyInConfig(false, "setSmartMotionTarget");
        sparkMaxPID.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, PIDtype.SMART_MOTION.slotId);
    }

    public void setTargetPosition(double targetPosition) {
        verifyInConfig(false, "setTargetPosition");
        sparkMaxPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition, PIDtype.POSITION.slotId);
    }
}
