package org.a05annex.frc.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;

import static org.a05annex.frc.subsystems.SparkNeo.UseType.FREE_SPINNING;

/**
 * This class is the packaging for a <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motor
 * powered by a <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark MAX</a> motor controller. It binds
 * together the {@link com.revrobotics.spark.SparkMax}, {@link com.revrobotics.RelativeEncoder}, and
 * {@link com.revrobotics.spark.SparkClosedLoopController} into a single object.
 * <p>
 * <b>Motivation:</b>
 * <p>
 * The genesis of this class is that we had a lot of issues in the last 3 seasons that we really resolved in
 * the 2022-2023 competition season. So we wanted a container class that made
 * it easier to use a SparkMax/Neo or SparkMax/Neo550 the <i>correct</i> way, which we are still learning.
 * We just finished our 3<sup>rd</sup> season of using the
 * <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark MAX</a> to control
 * <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motors on a swerve base. This season we went to all
 * REV controllers and motors, adding the <a href="https://www.revrobotics.com/rev-21-1651/">REV Neo 550</a> motor
 * to our bag of tricks. We have done a lot of things wrong in the past; and we wanted to contain out past mistakes,
 * so they are not (at least less frequently) repeated.
 * <p>
 * <b>Use:</b>
 * <p>
 * Here are a couple hints on the best use of {@code SparkNeo}, and the associated class {@link SparkNeo550}:
 * <ul>
 *     <li>Create one using the {@link SparkNeo#factory(int)} or {@link SparkNeo550#factory(int)}. While there
 *     is a constructor for either it exists for creating either a real implementation or a mocked test implementation,
 *     and is not publicly accessible. The factory method creates what you want to use with the CAN id
 *     as the only argument.</li>
 *     <li>Do all your configuration within a {@link SparkNeo#startConfig()} and {@link SparkNeo#endConfig()} block.
 *     This lets the {@code SparkNeo} handle all the details of how the configuration is handled and burned into the
 *     controllers. It helps you do configuration in the right place in code. NOTE: sometimes you have special
 *     needs - like during PID tuning. You can get to any of the wrapped {@link com.revrobotics.spark.SparkMax},
 *     {@link com.revrobotics.RelativeEncoder}, and {@link com.revrobotics.spark.SparkClosedLoopController} classes
 *     through the {@link SparkNeo#sparkMax}, {@link SparkNeo#encoder}, and {@link SparkNeo#sparkMaxPID} if
 *     you need to bypass the restrictions of the {@code SparkNeo} implementation - for example, in code that
 *     tunes PID settings.</li>
 *     <li>Controller PID slots - yeah, the REV documentation is confusing. {@code SparkNeo} manages all 4 slots
 *     as {@link PIDtype#RPM}, {@link PIDtype#POSITION},{@link PIDtype#MAX_MOTION_RPM}, and
 *     {@link PIDtype#MAX_MOTION_POSITION}. The {@code SparkNeo} also hides the management of the slots by providing
 *     the {@link #setRpmPID(double, double, double, double)},
 *     {@link #setPositionPID(double, double, double, double)},
 *     {@link #setMAXMotionRPM(double, double, double, double, double, double, double)}, and
 *     {@link #setMAXMotionPosition(double, double, double, double, double, double, double)}.
 *     </li>
 * </ul>
 * <p>
 * <b>Detail:</b>
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
 *                 <caption>Motor failure time as a function of AMPS</caption>
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
 *             current for a <i>short</i> time - the higher over rated amperage the faster the breaker will trip.
 *             What is published is time to trip at a couple different loads - 135%, and 200% as:
 *             <table>
 *                 <caption>Breaker trip time as a function of AMPS</caption>
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
 *             The main things to notice are that the trip time is highly variable, and that minimum trip time on the
 *             AndyMark breakers is considerably less than the REV breakers. Our swerve drive motors were on 40Amp
 *             (maximum size allowed for FRC) AndyMark breakers, and they would trip during extreme defensive play.
 *             </li>
 *             <li><b>Motor Use</b> - How a motor is used affects how we should approach a determining an appropriate
 *             maximum current limit. Typically we see these use scenarios:
 *             <ul>
 *                 <li><b>Free Spinning</b> - </li>
 *                 <li><b>Occasional Stall</b> - </li>
 *                 <li><b>Prolonged Stall</b> - </li>
 *                 <li><b>Position (Always Stalled)</b> - </li>
 *             </ul>
 *             motors that are essentially free spinning motors
 *             </li>
 *         </ul>
 *         Current limiting uses the SparkMAX to control the current to the motor. This can protect both the motor
 *         from currents that would cause damage, and the breaker to prevent conditions that would cause the breaker
 *         to trip and disable the motor.
 *         </li>
 *         <li>Unnecessary CAN activity</li>
 *     </ul>
 *     </li>
 *     <li>that we always used PID slot 0 which worked pretty well because we were not running multiple move modes
 *     on motors
 *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         hen we switched control mode we were also resetting PID constants - which led to unpredictable </li>
 *     </ul>
 * <p>
 * It provides a simple interface for configuration; PID control modes for:
 * <ul>
 *     <li>{@link com.revrobotics.spark.SparkMax.ControlType#kVelocity} - use the {@link #setTargetRPM(double)}
 *     method to set the motor velocity RPM.</li>
 *     <li>{@link com.revrobotics.spark.SparkMax.ControlType#kPosition} - use the {@link #setTargetPosition(double)}
 *     method to set the desired motor position.</li>
 *     <li>{@link com.revrobotics.spark.SparkMax.ControlType#kMAXMotionVelocityControl} - use</li>
 *     <li>{@link com.revrobotics.spark.SparkMax.ControlType#kMAXMotionPositionControl} </li>
  * </ul>
 */
public class SparkNeo {

    /**
     * The factory for {@link SparkNeo} objects for the physical robot. When the robot is powered up and this
     * object is first created, it will represent the SparkMax/Neo in its powered up configuration (i.e. it will
     * have the configuration burned into the SparkMax).
     *
     * @param canId The CAN id of the Spark MAX controlling the Neo motor.
     * @return The created {@link SparkNeo}
     */
    @NotNull
    public static SparkNeo factory(int canId) {
        SparkMax sparkMax = new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);
        return new SparkNeo(sparkMax, sparkMax.getEncoder(), sparkMax.getClosedLoopController());
    }

    /**
     * The PID slot. There are 4 PID slots available. Slots 0-2 are reserved for the speed, smart-motion, and position
     * PIDs. Slot 3 is the custom slot and can be used for any PID need not addressed by the {@link SparkNeo} wrapper.
     */
    public enum PIDtype {
        /**
         * The motor speed (RPM) PID slot managed by this wrapper.
         */
        RPM(ClosedLoopSlot.kSlot0),
        /**
         * The position PID slot managed by this wrapper.
         */
        POSITION(ClosedLoopSlot.kSlot1),
        /**
         * The MAX motion RPM PID slot managed by this wrapper.
         */
        MAX_MOTION_RPM(ClosedLoopSlot.kSlot2),
        /**
         * The MAX motion position PID slot managed by this wrapper.
         */
        MAX_MOTION_POSITION(ClosedLoopSlot.kSlot3);

        final ClosedLoopSlot slotId;

        PIDtype(ClosedLoopSlot slotId) {
            this.slotId = slotId;
        }
    }

    /**
     * The motor use types for guessing the best current limits
     */
    public enum UseType {
        /**
         * The motor is expected to be essentially free spinning - as in a pickup that catches and delivers game
         * elements to the next stage. Stall is unexpected, so the current limits is low enough that the motor
         * is expected to survive forever at this current.
         */
        FREE_SPINNING(0),
        /**
         * The motor is expected to occasionally stall, and needs a bit of extra power to break the stall. This
         * gives a bit more current to break the stall - but not enough to be damaging if the stall continues
         * through the match.
         */
        RPM_OCCASIONAL_STALL(1),
        /**
         * There are driver induced prolonged stalls, an interesting case like the driver playing defence and stalling
         * against another robot. the driver needs the most power available for short periods of time. The current
         * must not exceed that which will damage the motor or throw the breaker for several consecutive 5 second
         * bursts of stall (5 seconds being the typical max defensive engagement time before penalty).
         */
        RPM_PROLONGED_STALL(2),
        /**
         * The motor is position controlled - which means it is essentially always stalled. The current limit must
         * not exceed a current that would damage the motor if sustained, or throw the breaker.
         */
        POSITION(3);

        final int index;

        UseType(int index) {
            this.index = index;
        }
    }

    /**
     * The breakers that may be used in the PDP with this motor.
     */
    public enum BreakerAmps {
        /**
         * A 10 amp breaker.
         */
        Amps10(0),
        /**
         * A 20 amp breaker.
         */
        Amps20(1),
        /**
         * A 30 amp breaker.
         */
        Amps30(2),
        /**
         * A 40 amp breaker.
         */
        Amps40(3);

        final int index;

        BreakerAmps(int index) {
            this.index = index;
        }
    }

    /**
     * The motor spin direction when positive power is applied.
     */
    public enum Direction {
        /**
         * The default spin direction of the motor.
         */
        DEFAULT(false),
        /**
         * The reverse of the default spin direction.
         */
        REVERSE(true);

        final boolean reversed;

        Direction(boolean reversed) {
            this.reversed = reversed;
        }
    }

    static final int[][] maxCurrentMatrix = {
            // UseType.FREE_SPINNING - a thing that is essentially free-spinning. Like a pickup roller which is
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

    SparkMaxConfig config = new SparkMaxConfig();
    boolean inConfig = false;
    boolean isConfigured = false;
    boolean currentLimitIsSet = false;
    /**
     * The low level REV code controlling the REV Spark MAX motor controller.
     */
    public final SparkMax sparkMax;
    /**
     * The low level REV code controlling interacting with the encoder of the motor plugged into the
     * REV Spark MAX controller.
     */
    public final RelativeEncoder encoder;
    /**
     * The low level REV code controlling the PID loops in the REV Spark MAX controller.
     */
    public final SparkClosedLoopController  sparkMaxPID;


    /**
     * The constructor for a {@code SparkNeo}. Always use {@link #factory(int)} to create the {@code SparkNeo}
     * <i>unless</i> you are creating a mock {@code SparkNeo} for testing.
     *
     * @param sparkMax The {@link SparkMax}.
     * @param encoder The {@link RelativeEncoder} of the {@link SparkMax}.
     * @param sparkMaxPID The {@link SparkClosedLoopController} of the {@link SparkMax}.
     */
    protected SparkNeo(@NotNull SparkMax sparkMax, @NotNull RelativeEncoder encoder,
                    @NotNull SparkClosedLoopController sparkMaxPID) {
        this.sparkMax = sparkMax;
        this.encoder = encoder;
        this.sparkMaxPID = sparkMaxPID;
    }

    /**
     * Verify this motor is currently <i>in</i>, or, <i>not in</i> configuration mode
     * @param expectedInConfig {@code true} to verify the motor is <i>in</i> configuration mode, and {@code false}
     *                                     to verify the motor is <i>not in</i> configuration mode.
     * @param method The name of the method calling this function for use in error messaging.
     * @throws IllegalStateException thrown if the motor is not in the expected mode.
     */
    protected void verifyInConfig(boolean expectedInConfig, @NotNull String method) {
        if (this.inConfig != expectedInConfig) {
            System.out.println();
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.printf("***** %s() may only be called %s configuration%n", method,
                    expectedInConfig ? "during" : "outside of");
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.println();
            throw new IllegalStateException("%s() may only be called %s configuration".formatted(method,
                    expectedInConfig ? "during" : "outside of"));
        }
    }

    /**
     * Verify this motor has been configured.
     *
     * @param method The name of the method calling this function for use in error messaging.
     * @throws IllegalStateException thrown if this motor has not been configured.
     */
    protected void verifyIsConfigured(@NotNull String method) {
        if (!this.isConfigured) {
            System.out.println();
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.printf("***** %s() may only be called after configuration%n", method);
            System.out.println("**********************************************************************");
            System.out.println("**********************************************************************");
            System.out.println();
            throw new IllegalStateException("%s() may only be called after configuration".formatted(method));
        }
    }

    /**
     * Get the maximum free RPM. This is published in the
     * <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> summary as 5676RPM. Note that the actual
     * achievable speed is dependent on the load (weight being lifted, friction, inertia, etc.) and is likely
     * 0.8 to 0.9 times the maximum free speed depending on use.
     *
     * @return Returns the maximum free RPM.
     */
    static public double getMaxFreeRPM() {
        // per REV Neo datasheet
        return 5676.0;
    }

    /**
     * Gets the motor velocity (RPM) reported by the encoder.
     * @return The motor velocity (RPM) reported by the encoder.
     */
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the motor position (revolutions) reported by the encoder.
     * @return The motor position (revolutions) reported by the encoder.
     */
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    /**
     * Set the encoder position. The encoder position may only be set during configuration.
     *
     * @param position The encoder position.
     */
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
        // It seems this does not apply any more.
//        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
//            while (true) {
//                REVLibError errorCode = sparkMax.restoreFactoryDefaults();
//                if (REVLibError.kOk == errorCode) {
//                    break;
//                }
//                DriverStation.reportWarning(
//                        String.format("SparkMAX config error: CAN id = %d; error =  %d",
//                                sparkMax.getDeviceId(), errorCode.value), false);
//            }
//        }
    }

    /**
     * Set the current limit. This is a configuration method and can only be called between {@link #startConfig()}
     * and {@link #endConfig()}.
     *
     * @param useType         The motor use type, which characterizes likelihood and duration of stall.
     * @param breakerAmps    The breaker amperage, which allows computing the breaker overload.
     */
    public void setCurrentLimit(@NotNull UseType useType,  @NotNull BreakerAmps breakerAmps) {
        verifyInConfig(true, "setCurrentLimit");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            int maxAmps = maxCurrentMatrix[useType.index][breakerAmps.index];
            config.smartCurrentLimit(maxAmps, maxAmps, 10000);
        }
        currentLimitIsSet = true;
    }

    /**
     * Set the positive direction of the motor. This is a configuration method and can only be called between
     * {@link #startConfig()} and {@link #endConfig()}.
     *
     * @param direction The positive direction of the motor.
     */
    public void setDirection(Direction direction) {
        verifyInConfig(true, "setDirection");
        config.inverted(direction.reversed);
    }

    /**
     * Sets whether the power=0.0 mode should be free spinning
     * ({@link com.revrobotics.spark.config.SparkBaseConfig.IdleMode#kCoast})
     * or brake ({@link com.revrobotics.spark.config.SparkBaseConfig.IdleMode#kBrake}). NOTE: this is one
     * of the few methods that can be called inside or outside configuration.
     *
     * @param idleMode The idle mode.
     */
    public void setIdleMode(SparkBaseConfig.IdleMode idleMode) {
        // if we are in config, we are setting this as part of the config object (i.e. it is the real default
        // configuration), otherwise, this is a configuration change that needs to be applied now.
        if (inConfig) {
            // this is part of default configuration for this motor
            config.idleMode(idleMode);
        } else {
            // This is a 'temporary' change in configuration
            SparkMaxConfig lclConfig = new SparkMaxConfig();
            lclConfig.idleMode(idleMode);
            // Don't persist parameters since it takes time and this change is temporary
            sparkMax.configure(lclConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters);
        }
    }

    /**
     * The most common setup for MAX motion control of the motor RPM.
     *
     * @param kP             The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI             The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone         The PID loop will not include the integral component until the current position or speed is
     *                       within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                       is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF            The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param maxRPM         The maximum forward RPM, typically 0.8 to 0.9 times the {@link #getMaxFreeRPM()}.
     * @param maxRPMs        The maximum RPM acceleration per second, typically 1.0 to 4.0 times
     *                       the {@link #getMaxFreeRPM()}.
     * @param allowableError The allowable error in final RPM at which MAX motion will consider the goal
     *                       achieved.
     */
    public void setMAXMotionRPM(double kP, double kI, double kIZone, double kFF,
                                double maxRPM, double maxRPMs, double allowableError) {
        setMAXMotion(PIDtype.MAX_MOTION_RPM, kP, kI, kIZone, kFF, 0.0, -1.0, 1.0,
                maxRPM, maxRPMs, allowableError);
    }
    /**
     * The most common setup for MAX motion control of the motor position.
     *
     * @param kP             The PID proportional constant <i>K<sub>p</sub></i>
     * @param kI             The PID integral constant <i>K<sub>i</sub></i>
     * @param kIZone         The PID loop will not include the integral component until the current position or speed is
     *                       within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                       is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kD            The PID derivative constant <i>K<sub>d</sub></i>
     * @param maxRPM         The maximum forward RPM, typically 0.8 to 0.9 times the {@link #getMaxFreeRPM()}.
     * @param maxRPMs        The maximum RPM acceleration per second, typically 1.0 to 4.0 times
     *                       the {@link #getMaxFreeRPM()}.
     * @param allowableError The allowable error in final distance at which MAX motion will consider the goal
     *                       achieved.
     */
    public void setMAXMotionPosition(double kP, double kI, double kIZone, double kD,
                                double maxRPM, double maxRPMs, double allowableError) {
        setMAXMotion(PIDtype.MAX_MOTION_POSITION, kP, kI, kIZone, 0.0, kD, -1.0, 1.0,
                maxRPM, maxRPMs, allowableError);
    }

    /**
     * The configuration for all PID constants for SmartMotion (position) control.
     *
     * @param pidType        The PID type, must be either {@link PIDtype#MAX_MOTION_RPM} or
     *                       {@link PIDtype#MAX_MOTION_POSITION}.
     * @param kP             The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI             The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone         The PID loop will not include the integral component until the current position or speed is
     *                       within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                       is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF            The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param kD             The PID differential constant <i>K<sub>d</sub></i>.
     * @param min            The minimum RPM that will be requested, -1.0 is full reverse speed.
     * @param max            The maximum RPM that will be requested, 1.0 is full forward speed.
     * @param maxRPM         The maximum forward RPM, typically 0.8 to 0.9 times the {@link #getMaxFreeRPM()}.
     * @param maxRPMs        The maximum RPM acceleration per second, typically 1.0 to 4.0 times
     *                       the {@link #getMaxFreeRPM()}.
     * @param allowableError The allowable error in final distance or RPM at which smart motion will consider the goal
     *                       achieved.
     */
    public void setMAXMotion(@NotNull PIDtype pidType, double kP, double kI, double kIZone, double kFF, double kD,
                             double min, double max, double maxRPM, double maxRPMs,
                             double allowableError) {
        verifyInConfig(true, "setMAXMotion");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            ClosedLoopConfig clConfig = setPID(pidType, kP, kI, kIZone, kFF, kD, min, max);
            MAXMotionConfig mmConfig = new MAXMotionConfig();
            mmConfig.maxVelocity(maxRPM, pidType.slotId);
            mmConfig.maxAcceleration(maxRPMs, pidType.slotId);
            mmConfig.allowedClosedLoopError(allowableError, pidType.slotId);
            mmConfig.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal,pidType.slotId);
            clConfig.apply(mmConfig);
            config.apply(clConfig);

//            // the old code for reference
//            int slotId = PIDtype.SMART_MOTION.slotId;
//            sparkMaxPID.setSmartMotionAccelStrategy(SparkClosedLoopController.AccelStrategy.kTrapezoidal, slotId);
//            sparkMaxPID.setSmartMotionMaxVelocity(maxRPM, slotId);
//            sparkMaxPID.setSmartMotionMaxAccel(maxRPMs, slotId);
//            sparkMaxPID.setSmartMotionMinOutputVelocity(minRPM, slotId);
//            sparkMaxPID.setSmartMotionAllowedClosedLoopError(allowableError, slotId);
        }
    }

    /**
     * The most common configuration of the PID constants for RPM (speed) control.
     *
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     */
    public void setRpmPID(double kP, double kI, double kIZone, double kFF) {
        setPID(PIDtype.RPM, kP, kI, kIZone, kFF, 0.0, -1.0, 1.0);
    }
    /**
     * The most common configuration of the PID constants for position control.
     *
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     */
    public void setPositionPID(double kP, double kI, double kIZone, double kFF) {
        setPID(PIDtype.POSITION, kP, kI, kIZone, kFF, 0.0, -1.0, 1.0);
    }

    /**
     * The configuration for all PID constants for RPM (speed) control.
     *
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>.
     * @param kD      The PID differential constant <i>K<sub>d</sub></i>.
     * @param min     The minimum RPM that will be requested, -1.0 is full reverse speed.
     * @param max     The maximum RPM that will be requested, 1.0 is full forward speed.
     */
    public void setRpmPID(double kP, double kI, double kIZone, double kFF, double kD, double min, double max) {
        setPID(PIDtype.RPM, kP, kI, kIZone, kFF, kD, min, max);
    }
    /**
     * The configuration for all PID constants for position control. It is generally NOT recommended that you call
     * this directly, but that you use the PID configuration methods specific to the control type.
     *
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param kD      The PID differential constant <i>K<sub>d</sub></i>.
     * @param min     The minimum RPM that will be requested, -1.0 is full reverse speed.
     * @param max     The maximum RPM that will be requested, 1.0 is full forward speed.
     */
    public void setPositionPID(double kP, double kI, double kIZone, double kFF,double kD, double min, double max) {
        setPID(PIDtype.POSITION, kP, kI, kIZone, kFF, kD, min, max);
    }

    /**
     * Sets the PID constants for the specified PID control type.
     *
     * @param pidType The PID control type.
     * @param kP      The PID proportional constant <i>K<sub>p</sub></i>.
     * @param kI      The PID integral constant <i>K<sub>i</sub></i>.
     * @param kIZone  The PID loop will not include the integral component until the current position or speed is
     *                within this distance or RPM from the target. This zone helps prevent overshoot as the integral
     *                is only accumulated once the <i>K<sub>p</sub></i> has brought the system close to the target
     * @param kFF     The PID feed-forward constant <i>K<sub>ff</sub></i>
     * @param kD      The PID differential constant <i>K<sub>d</sub></i>.
     * @param min     The minimum RPM that will be requested, -1.0 is full reverse speed.
     * @param max     The maximum RPM that will be requested, 1.0 is full forward speed.
     */
    public ClosedLoopConfig setPID(@NotNull PIDtype pidType, double kP, double kI, double kIZone, double kFF,
                       double kD, double min, double max) {
        verifyInConfig(true, "setPID");
        ClosedLoopSlot slotId = pidType.slotId;
        ClosedLoopConfig clConfig = new ClosedLoopConfig()
                .pidf(kP, kI, kD, kFF, slotId)
                .iZone(kIZone, slotId)
                .outputRange(min, max, slotId);
        config.apply(clConfig);
        return clConfig;
    }

    /**
     * Sets the soft limits of a motor (programmed stops). This is a configuration method and can only be called between
     * {@link #startConfig()} and {@link #endConfig()}
     *
     * @param min The stop limit in the reverse direction (null for no limit)
     * @param max The stop limit in the forward direction (null for no limit)
     */
    public void setSoftLimits(Double min, Double max) {
        verifyInConfig(true, "setSoftLimits");
        SoftLimitConfig slConfig = new SoftLimitConfig();
        if(min == null) {
            slConfig.reverseSoftLimitEnabled(false);
        } else {
            slConfig.reverseSoftLimitEnabled(true);
            slConfig.reverseSoftLimit(min);
        }

        if(max == null) {
            slConfig.forwardSoftLimitEnabled(false);
        } else {
            slConfig.forwardSoftLimitEnabled(true);
            slConfig.forwardSoftLimit(max.floatValue());
        }
        config.apply(slConfig);
    }

    /**
     * End the configuration by performing the following tasks:
     * <ul>
     *     <li>verifying we are currently in config;</li>
     *     <li>burning the current configuration into the SparkMax flash if requested;</li>
     *     <li>reducing the reporting frequency of non-essential SparkMax state variables to reduce
     *     CAN traffic.</li>
     * </ul>
     */
    public void endConfig() {
        verifyInConfig(true, "endConfig");
//        // This was the old way to do this - and the config was set before we set the periodic stuff.
//        if (A05Constants.getSparkConfigFromFactoryDefaults() && A05Constants.getSparkBurnConfig()) {
//            sparkMax.burnFlash();
//        }
        config.signals.absoluteEncoderPositionAlwaysOn(true);
        config.signals.absoluteEncoderVelocityAlwaysOn(true);
        // OK - these will take a bit more research. These are a bunch of things we don't use, so we had set the
        // reporting time way long to reduce CAN traffic.
//        // These cannot be burned into the configuration - so they must be set at configuration
//        sparkMax.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
//        sparkMax.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus4, 500);
//        sparkMax.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus5, 500);
//        sparkMax.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus6, 500);
        // current limit configuration IS REQUIRED. If the current limit has not been set - pick the safest, and
        // of course, the lowest possible breaker amperage.
        if (!currentLimitIsSet) {
            setCurrentLimit(FREE_SPINNING, BreakerAmps.Amps10);
        }
        // and this is the new way we set the configuration for the SparkMAX - they now say we should always burn
        // the real config
        sparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // this spark is now configured
        inConfig = false;
        isConfigured = true;
    }

    /**
     * Stop this motor.
     */
    public void stopMotor() {
        verifyInConfig(false, "stopMotor");
        sparkMax.stopMotor();
    }

    /**
     * Set a target RPM for this motor. The velocity PID will be used to achieve this RPM.
     *
     * @param targetRpm The target RPM.
     */
    public void setTargetRPM(double targetRpm) {
        verifyIsConfigured("setTargetRPM");
        sparkMaxPID.setReference(targetRpm, SparkMax.ControlType.kVelocity, PIDtype.RPM.slotId);
    }

    /**
     * Set a target position for this motor. The position PID will be used to achieve this.
     *
     * @param targetPosition The target position.
     */
    public void setTargetPosition(double targetPosition) {
        verifyIsConfigured("setTargetPosition");
        sparkMaxPID.setReference(targetPosition, SparkMax.ControlType.kPosition, PIDtype.POSITION.slotId);
    }
    /**
     * Set a target RPM for this motor. The MAX Motion velocity PID will be used to achieve this RPM.
     *
     * @param targetRpm The target RPM.
     */
    public void setTargetMAXMotionRPM(double targetRpm) {
        verifyIsConfigured("setTargetRPM");
        sparkMaxPID.setReference(targetRpm, SparkMax.ControlType.kMAXMotionVelocityControl,
                PIDtype.MAX_MOTION_RPM.slotId);
    }
    /**
     * Set a smart motion target position for this motor. The MAX Motion position PID will be used to achieve this.
     *
     * @param targetPosition The target position.
     */
    public void setTargetMAXMotionPosition(double targetPosition) {
        verifyIsConfigured("setSmartMotionTarget");
        sparkMaxPID.setReference(targetPosition, SparkMax.ControlType.kMAXMotionPositionControl,
                PIDtype.MAX_MOTION_POSITION.slotId);
    }

 }
