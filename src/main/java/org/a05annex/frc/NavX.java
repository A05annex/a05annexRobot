package org.a05annex.frc;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;


/**
 * This is a class that initializes and updates from the NavX board to maintain current information about the
 * location and orientation of the robot on the field. Specifically, this class maintains current heading, fused
 * heading, and displacements for the robot throughout a match. These are the maintained elements:
 * <ul>
 *     <li>heading - A heading based on the yaw gyro on the NavX board. NOTE: the yaw gyro on every NavX2 board has
 *       a different raw response to rotation and must be calibrated, see
 *       <a href="https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/">Gyro/Accelerometer
 *       Calibration</a>. NOTE: gyros are subject to drift over multiple rotations, which, in the case of
 *       the NavX2 board is an incorrect scaling of the reported rotation. Calibration sets the correct scaling
 *       factor in the NavX2 board to minimize drift.</li>
 *     <li>fused heading -  A heading based on the yaw gyro with periodic correction whenever a magnetic
 *       heading can be read. Using fused heading requires the
 *       <a href="https://pdocs.kauailabs.com/navx-mxp/guidance/magnetometer-calibration/">Magnetometer
 *       Calibration</a>. NOTE: my reading of this is that this needs to happen in each competition arena
 *       to accurately work during the competition.</li>
 *     <li>displacement - The NavX2 board has code to integrate gyro and accelerometer readings to compute
 *       a displacement of the robot since time of power-up. The best discussion of this in the NavX website is the
 *       last question of the <a href="https://pdocs.kauailabs.com/navx-mxp/intro/frequently-asked-questions/">NavX
 *       FAQ</a>.We might be able to use this with the
 *       {@link org.a05annex.frc.subsystems.SpeedCachedSwerve} to provide an additional evaluation of change
 *       in robot position over time. Our initial testing of displacements looked like they re pretty half-baked
 *       right now, and unreliable.</li>
 * </ul>
 * Please read <a href="https://pdocs.kauailabs.com/navx-mxp/guidance/best-practices/">NavX best practices</a> to
 * get some insight on how to best use the NavX in your robot design and operation.
 */
public class NavX {

    /**
     * This is the NavX inertial navigation board connection.
     */
    private final AHRS ahrs;

    /**
     * The heading we are trying to track with the robot, i.e. this is the heading the robot is expected to
     * be on given the commands we have sent to the robot.
     */
    private final AngleD expectedHeading = new AngleD(AngleD.ZERO);

    /**
     * This is the update count from the NavX. This count is incremented whenever this NavX class updates its
     * heading/displacement information. The NavX board updates at a faster rate than the WPI command loop rate,
     * so, the update count should always change between command cycles unless there is a NavX communication problem.
     */
    private double updateCt;

    /**
     * Heading, fused heading, and displacement is updated every command cycle. If the NavX has reported new
     * information since the last command cycle, this will be {@code true}, otherwise this will be {@code false},
     * indicating that this is not current information.
     */
    private boolean isHeadingCurrent;

    /**
     * The actual heading of the robot from -&infin; to &infin;, so the spins are included in this
     * heading.
     */
    private final AngleD heading = new AngleD(AngleD.ZERO);
    /** The fused heading of the robot - this is the heading from the gyro fused with (corrected by) the magnetic
     *  heading if it can be read. Specifically, the gyro reading is subject to drift and this is supposed to use
     *  the magnetic heading to correct for drift - but it looks half-baked in the documentation, so we really
     *  don't expect it to work yet.
     */
    private final AngleD fusedHeading = null;

    // --------------------------------------------------
    // Reference values - these are the values at initialization of the NavX recording the position of the robot
    // on the field in terms of the readings of the NavX and the field heading of the robot at initialization. We
    // refer to these as the reference values as everything that happens after initialization is evaluated
    // with reference to (or as the change relative to this reference).
    // --------------------------------------------------
    /**
     * The NavX reported pitch at the time the NavX in initialized.
     */
    private final AngleD refPitch = new AngleD(AngleD.ZERO);

    /**
     * The NavX reported yaw at the time the NavX in initialized.
     */
    private final AngleD refYaw = new AngleD(AngleD.ZERO);

    /**
     * The NavX reported roll at the time the NavX in initialized.
     */
    private final AngleD refRoll = new AngleD(AngleD.ZERO);

    /**
     * The actual field heading of the robot at the time the NavX in initialized.
     */
    private final AngleD refHeading = new AngleD(AngleD.ZERO);

    /**
     * {@code true} if the fused heading should be reported in the {@link HeadingInfo}, {@code false} otherwise.
     */
    boolean includeFusedHeading = false;

    /**
     * The X displacement reported by the NavX board.
     */
    private float displacementX = 0.0f;

    /**
     * The Y displacement reported by the NavX boar.
     */
    private float displacementY = 0.0f;


    /**
     * Instantiate the NavX. We have had problems here where the NavX does not respond because it is somehow
     * unreachable (disconnected) - which means this instantiation never finishes, and the robot sits on the
     * starting block and does not participate in the match. We have not fully worked out the details of
     * continuing without inertial navigation.
     */
    private NavX() {
        // So, if there is no navX, there is no error - it just keeps trying to connect forever, so this
        // needs to be on a thread that can be killed if it doesn't connect in time ......
        // TODO: figure out the threading, error handling, and redundancy.
        ahrs = new AHRS(SPI.Port.kMXP);
        try {
            // the reset starts a calibration process, so we want to make sure that finishes before we do
            // anything else, This is happening when the robot is first powered up, so this blocking any other
            // code from running until the NavX gyros are done calibrating
            while (ahrs.isCalibrating()) {
                     //noinspection BusyWait
                    Thread.sleep(100);
            }
            // 26-oct-2024
            // and, unfortunately, the board/software is still not really ready to be used. We discovered that
            // trying to initialize the heading stuff did not work unless we waited, tried some different times,
            // 100ms was reliable
            Thread.sleep(100);
            // OK, let's finish initialization
            updateCt = ahrs.getUpdateCount();
            initializeHeadingAndNav();
        } catch (InterruptedException ignored) {
            // we only get here if one of the Thread.sleep() calls is interrupted because the robot program is being
            // killed externally.
        }
    }

    /**
     * Sets the reference start heading to 0.0 (the robot facing down-field) and navigation reference positions
     * to the current values. This should be called immediately at the start of autonomous.
     */
    public void initializeHeadingAndNav() {
        initializeHeadingAndNav(AngleD.ZERO);
    }

    /**
     * Sets the reference start heading to the specified heading, and navigation reference positions to the
     * current values. This should be called immediately at the start of autonomous.
     *
     * @param heading (AngleConstantD) The current field heading of the robot as it is positioned at the start
     *                of autonomous.
     */
    public void initializeHeadingAndNav(AngleConstantD heading) {
        // In the past we have always initialized with the front of the robot facing down field, so the
        // heading was 0.0 at initialization. In this case we are initializing to some other heading.
        refPitch.setDegrees(ahrs.getPitch());
        refRoll.setDegrees(ahrs.getRoll());
        refHeading.setValue(heading);
        // reset the Yaw gyro to read 0.0
        ahrs.reset();
        // set the adjustment angle so the ahrs.getAngle() will return the specified heading
        // in the current NavX board position.
        ahrs.setAngleAdjustment(heading.getDegrees() - ahrs.getAngle());
        // Set the expected heading to the specified initialize heading
        expectedHeading.setValue(refHeading);
    }

    /**
     * Set whether the fused NavX heading should be returned with {@link HeadingInfo}. By default, this is
     * {@code false}. Note that fused heading information is only available if magnetic calibration has been performed
     * on the NavX board. If magnetic calibration has not been performed, then a call to this method is ignored and
     * this remains {@code false}.
     *
     * @param includeFusedHeading {@code true} to request the inclusion of fused heading information, {@code false}
     *                                       otherwise.
     * @return {@code true} if fused heading information will be included in {@link HeadingInfo}, {@code false}
     * otherwise. Note: if you try to set {@code includeFusedHeading} to {@code true}, and the NavX has not been
     * magnetically calibrated, this will return {@code false}.
     */
    public boolean setIncludeFusedHeading(boolean includeFusedHeading) {
        this.includeFusedHeading = includeFusedHeading && ahrs.isMagnetometerCalibrated();
        return this.includeFusedHeading;
    }

    /**
     * Will fused heading we returned {@link HeadingInfo}?
     *
     * @return {@code true} if fused heading information will be included in {@link HeadingInfo}, {@code false}
     *  otherwise.
     */
    public boolean getIncludeFusedHeading() {
        return includeFusedHeading;
    }

    /**
     * Call the {@link AHRS#resetDisplacement()} function for the NavX board. Since we are pretty foggy on
     * how displacement calculations work, we are really not sure what this does yet.
     */
    public void resetDisplacement() {
        ahrs.resetDisplacement();
    }
    /**
     * Change the expected heading by the specified angle.
     *
     * @param delta The change to the expected heading.
     */
    @SuppressWarnings("unused")
    public void incrementExpectedHeading(AngleD delta) {
        expectedHeading.add(delta);
    }

    /**
     * Set the expected heading.
     *
     * @param expectedHeading (not null, AngleD) The expected heading.
     */
    @SuppressWarnings("unused")
    public void setExpectedHeading(AngleConstantD expectedHeading) {
        this.expectedHeading.setValue(expectedHeading);
    }

    /**
     * Set the expected heading to the current heading.
     */
    public void setExpectedHeadingToCurrent() {
        expectedHeading.setValue(heading);
    }

    /**
     * Recompute the heading as reported by the NavX and adjusted to be always increasing when rotation is
     * clockwise, and decreasing when rotation is counter-clockwise. This means that the heading range is continuous
     * from -&infin; to +&infin; so heading PID loops sre not subject to special logic for the &plusmn;180&deg;
     * boundary typical in reporting heading.
     * <p>
     * This method is called in the {@link A05Robot#robotPeriodic()} override of
     * {@link edu.wpi.first.wpilibj.TimedRobot#robotPeriodic()} so the
     * same NavX heading information is available to all subsystems and commands during each command cycle.
     * </p>
     */
    public void recomputeHeading() {
        // first question - do we actually have new data from the NavX board
        double updateCt = ahrs.getUpdateCount();
        isHeadingCurrent = updateCt > this.updateCt;
        if (isHeadingCurrent) {
            // this is a new report from the NavX board, update all the heading info.
            this.updateCt = updateCt;
            // returns the accumulated yaw deviation (continuous -infinity to +infinity)
            heading.setDegrees(-ahrs.getAngle());
            if (includeFusedHeading) {
                // not at all sure what to do here - documentation says the fused heading is in the range 0-360
                // degrees, so it would need some adjustment to get it into the continuous -infinity to +infinity
                // range. Right now the best we can do is report what the NavX reports, so we can confirm what it
                // does and figure out how to convert that to the continuous -infinity to +infinity representation.
                fusedHeading.setDegrees(-ahrs.getFusedHeading());
            }

            displacementX = -ahrs.getDisplacementX();
            displacementY = ahrs.getDisplacementY();
        }
    }

    /**
     * Returns a copy of the current robot chassis heading. Note that the robot makes a revolution the heading does
     * not reset when the heading crosses the &pi;, -&pi; boundary - so the actual bounds of the heading are
     * -&infin; to &infin;.
     *
     * @return (not null, AngleD) A copy of the current robot chassis heading.
     */
    @NotNull
    public AngleD getHeading() {
        return heading.cloneAngleD();
    }

    /**
     * Get the current heading information for the robot. NOTE: the returned {@link HeadingInfo} reflects snapshot
     * in time and is not updated to reflect future conditions. Please call this method whenever you need
     * {@link HeadingInfo} rather than caching it.
     *
     * @return Returns the heading info, returns {@code null} if there is a problem with the NavX.
     */
    public HeadingInfo getHeadingInfo() {
        if (null == ahrs) {
            return null;
        }
        return new HeadingInfo(heading.cloneAngleD(),
                includeFusedHeading ? fusedHeading.cloneAngleD() : null,
                isHeadingCurrent, expectedHeading, displacementX, displacementY);
    }

    /**
     * Get the navigation info from the NavX. NOTE: the returned {@link NavInfo} reflects snapshot
     *      * in time and is not updated to reflect future conditions. Please call this method whenever you need
     *      * {@link NavInfo} rather than caching it.
     *
     * @return Returns the navigation info, returns {@code null} if there is a problem with the NavX.
     */
    @SuppressWarnings("unused")
    public NavInfo getNavInfo() {
        if (null == ahrs) {
            return null;
        }
        // The subtraction of the ref values adjusts for the construction bias of not having the NavX perfectly
        // mounted, or there being some bias in the NavX - i.e. the ref represents the value first reported when
        // the reference position is set, see initializeHeadingAndNav().
        return new NavInfo(
                new AngleConstantD(AngleUnit.DEGREES, ahrs.getPitch() - refPitch.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, -ahrs.getYaw() - refYaw.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, ahrs.getRoll() - refRoll.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, ahrs.getPitch()),
                new AngleConstantD(AngleUnit.DEGREES, -ahrs.getYaw()),
                new AngleConstantD(AngleUnit.DEGREES, ahrs.getRoll()));
    }

    /**
     * The data class for returning information about the robot heading.
     */
    public static class HeadingInfo {
        /**
         * The current heading in radians of the robot as computed in the last call
         * to {@link NavX#initializeHeadingAndNav()}. The current heading ranges from -&infin; to &infin; as the
         * heading reflects the spins made by the robot and is a continuous function, as opposed to yaw readings
         * which have a discontinuity at &plusmn;&pi;(&plusmn;180&deg;).
         */
        public final AngleConstantD heading;

        /**
         * This is a NavX thing that fuses the gyroscope reported yww with the magnetic heading to help correct
         * yaw drift of the yaw gyroscope - read more details here
         */
        public final AngleConstantD fusedHeading;

        /**
         * {@code true} if the heading information was updated for this frame
         */
        public final boolean isHeadingCurrent;

        /**
         * This is the expected heading based on NavX initialization and calls to
         * {@link NavX#incrementExpectedHeading(AngleD)} and {@link NavX#setExpectedHeadingToCurrent()}.
         */
        public final AngleConstantD expectedHeading;
        /**
         * The X displacement reported by the NavX board. Highly experimental and not reliable.
         */
        public final float displacementX;
        /**
         * The Y displacement reported by the NavX board. Highly experimental and not reliable.
         */
        public final float displacementY;

        /**
         * Initialize the {@link NavX.HeadingInfo} at construction.
         *
         * @param heading          The actual robot heading, as determined by the yaw gyro
         * @param fusedHeading     The actual fused setting , as determined by the yaw gyro with correction
         *                         by the magnetometer when the robot is in a state where the magnetic
         *                         (compass) direction can be sensed. {@code null} if either the NavX
         *                         magnetometer has not been calibrated, or fused heading has not been
         *                         requested
         * @param isHeadingCurrent {@code true} is the NavX has updated the heading this command cycle, and
         *                         {@code false} otherwise.
         * @param expectedHeading  The expected robot heading.
         * @param displacementX
         * @param displacementY
         */
        HeadingInfo(@NotNull AngleD heading, @Nullable AngleD fusedHeading, boolean isHeadingCurrent,
                    @NotNull AngleD expectedHeading, float displacementX, float displacementY) {
            this.heading = heading;
            this.fusedHeading = fusedHeading;
            this.isHeadingCurrent = isHeadingCurrent;
            this.expectedHeading = expectedHeading;
            this.displacementX = displacementX;
            this.displacementY = displacementY;
        }

        /**
         * Get the down-field heading that is closest to the current heading. This is useful if you have
         * a driving or targeting operation that requires the robot heading to stay
         * down-field.
         *
         * @return The closest down-field heading.
         */
        @NotNull
         public AngleD getClosestDownField() {
            return new AngleD(AngleUnit.DEGREES,360.0 * Math.round(heading.getDegrees() / 360.0));
        }

        /**
         * Get the up-field heading that is closest to the current heading. This is useful if you have
         * a driving or targeting operation that requires the robot heading to stay
         * up-field.
         *
         * @return The closest up-field heading.
         */
        @NotNull
        public AngleD getClosestUpField() {
            return new AngleD(AngleUnit.DEGREES,180.0 + (360.0 * Math.round((heading.getDegrees() - 180.0) / 360.0)));
        }

        /**
         * Get the robot heading that is closest to current heading to achieve the specified field heading.
         *
         * @param fieldHeading The desired field heading.
         * @return The closest robot heading.
         */
        public AngleD getClosestHeading(AngleD fieldHeading) {
            return new AngleD(AngleUnit.DEGREES, fieldHeading.getDegrees() +
                    (360 * Math.round((heading.getDegrees() - fieldHeading.getDegrees()) / 360.0)));
        }

        /**
         * Get the down-field or up-field heading that is closest to the current heading. This is useful if you have
         * a driving mode allowing the driver to get the robot into a somewhat down-field or
         * up-field heading and then lock on to a directly down-field or up-field heading to
         * thwart defense attempts to disrupt the robot while traversing the field.
         *
         * @return The closest down-field or up-field heading.
         */
        @NotNull
        public AngleD getClosestDownOrUpField() {
            double currentHeadingDeg = heading.getDegrees();
            int mod = (int)currentHeadingDeg % 360;
            if ((mod > -90 && mod < 90) || (mod > 270) || (mod < -270)) {
                // The closest direction is down-field.
                return getClosestDownField();
            }
            return getClosestUpField();
        }
    }

    /**
     * The data class for the 'raw' navigation info from the NavX, corrected by when the reference was last set.
     */
    public static class NavInfo {
        /**
         * The pitch (lean forward or backward) of the robot, with negative being forwards, from when the robot
         * was first initialized, or relative to when the NavX orientation was last reset.
         */
        public final AngleConstantD pitch;
        /**
         * The pitch (lean forward or backward) as reported by the NavX.
         */
        public final AngleConstantD rawPitch;
        /**
         * The yaw (rotation or turn) of the robot, with positive being clockwise (to the right), from when the
         * robot was first initialized, or relative to when the NavX orientation was last reset.
         */
        public final AngleConstantD yaw;
        /**
         * The yaw (rotation or turn) as reported by the NavX.
         */
        public final AngleConstantD rawYaw;
        /**
         * The roll (lean sideways) of the robot, with positive being the robot falling over on it's left
         * side, from when the robot was first initialized.
         */
        public final AngleConstantD roll;
        /**
         * The roll (lean sideways) as reported by the NavX.
         */
        public final AngleConstantD rawRoll;

        NavInfo(AngleConstantD pitch, AngleConstantD yaw, AngleConstantD roll,
                AngleConstantD rawPitch, AngleConstantD rawYaw, AngleConstantD rawRoll) {
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.rawPitch = rawPitch;
            this.rawYaw = rawYaw;
            this.rawRoll = rawRoll;
        }
    }

    /**
     * The Singleton instance of this NavX. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static NavX INSTANCE = new NavX();

    /**
     * Returns the Singleton instance of this NavX. This static method
     * should be used -- {@code NavX.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     *
     * @return (NavX) returns the NavX instance.
     */
    public static NavX getInstance() {
        return INSTANCE;
    }
}
