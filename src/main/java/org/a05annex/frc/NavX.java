package org.a05annex.frc;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;


/**
 * <p>This is a class that initializes and tracks the NavX board to maintain current information, specifically
 * heading, for the robot. We have been having a degrees vs. radians debate;
 * since all of the math trig libraries
 * use radians. we decided that staying in radians would build up minimal round-off error.
 * </p><p>
 * Originally this class was written to support NavX on a conventional drive that had PID direction loops
 * concerned with matching actual heading to expected heading. Right now we are a little unclear how that
 * relates to the A05annex 2021 season swerve drive.
 * </p>
 */
public class NavX {

    /**
     * This is the NavX inertial navigation board connection.
     */
    private final AHRS m_ahrs;

    /**
     * The heading we are trying to track with the robot, i.e. this is the heading the robot is expected to
     * be on given the commands we have sent to the robot.
     */
    private final AngleD m_expectedHeading = new AngleD(AngleD.ZERO);

    /**
     * This is the update count from the NavX. This count is incremented whenever the NavX updates its position
     * information. The NavX updates at a faster rate than the WPI command loop rate, so, the update count should
     * always change between command cycles of there is a NavX communication problem.
     */
    private double m_updateCt;

    /**
     * The raw heading, not corrected for the spins, read directly from the NavX, in the range
     * -180 to +180 degrees. Used for determining whether the boundary between -180 and 180 has been crossed.
     */
    private final AngleD m_headingRawLast = new AngleD(AngleD.ZERO);

    /**
     * The number of complete revolutions the robot has made.
     */
    private int m_headingRevs = 0;

    /**
     * The actual heading of the robot from -&infin; to &infin;, so the spins are included in this
     * heading.
     */
    private final AngleD m_heading = new AngleD(AngleD.ZERO);
    private boolean m_setExpectedToCurrent = false;

    // --------------------------------------------------
    // Reference values - these are the values at initialization of the NavX recording the position of the robot
    // on the field in terms of the readings of the NavX and the field heading of the robot at initialization. We
    // refer to these as the reference values as everything that happens after initialization is evaluated
    // with reference to (or as the change relative to this reference).
    // --------------------------------------------------
    /**
     * The NavX reported pitch at the time the NavX in initialized.
     */
    private final AngleD m_refPitch = new AngleD(AngleD.ZERO);

    /**
     * The NavX reported yaw at the time the NavX in initialized.
     */
    private final AngleD m_refYaw = new AngleD(AngleD.ZERO);

    /**
     * The NavX reported roll at the time the NavX in initialized.
     */
    private final AngleD m_refRoll = new AngleD(AngleD.ZERO);

    /**
     * The actual field heading of the robot at the time the NavX in initialized.
     */
    private final AngleD m_refHeading = new AngleD(AngleD.ZERO);

    /**
     * Instantiate the NavX. We have had problems here where the NavX does not respond because it is somehow
     * unreachable (disconnected) - which means this instantiation never finishes, and the robot sits on the
     * starting block and does not participate in the match. We have not fully worked out the details of
     * continuing without inertial navigation.
     */
    private NavX() {
        // So, if there is no navx, there is no error - it just keeps trying to connect forever, so this
        // needs to be on a thread that can be killed if it doesn't connect in time ......
        // TODO: figure out the threading, error handling, and redundancy.
        m_ahrs = new AHRS(SPI.Port.kMXP);
        m_ahrs.reset();
        while (m_ahrs.isCalibrating()) {
            try {
                //noinspection BusyWait
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
        }
        m_updateCt = m_ahrs.getUpdateCount();
        initializeHeadingAndNav();
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
        m_refPitch.setDegrees(m_ahrs.getPitch());
        m_refYaw.setDegrees(m_ahrs.getYaw());
        m_refRoll.setDegrees(m_ahrs.getRoll());
        m_refHeading.setValue(heading);
        m_headingRawLast.setValue(AngleD.ZERO);
        m_expectedHeading.setValue(m_refHeading);
        m_headingRevs = 0;
    }

    /**
     * Change the expected heading by the specified angle.
     *
     * @param delta The change to the expected heading.
     */
    @SuppressWarnings("unused")
    public void incrementExpectedHeading(AngleD delta) {
        m_expectedHeading.add(delta);
    }

    /**
     * Set the expected heading.
     *
     * @param expectedHeading (not null, AngleD) The expected heading.
     */
    @SuppressWarnings("unused")
    public void setExpectedHeading(AngleConstantD expectedHeading) {
        m_expectedHeading.setValue(expectedHeading);
    }

    /**
     * Set the expected heading to the current heading.
     */
    public void setExpectedHeadingToCurrent() {
        m_expectedHeading.setValue(m_heading);
    }

    /**
     * Recompute the heading as reported by the NavX and adjusted to be always increasing when rotation is
     * clockwise. This heading computation was introduced by Jason Barringer to the FRC 6831 AO5 Annex code base
     * in the 2017 season to make using PID loops to control heading with the IMU easier to write, and more
     * predictable. If there is a discontinuity in the sensor output, this means there needs to be special logic
     * in the PID code to deal with the discontinuity. This handles the discontinuity in a single place where
     * the heading is computed.
     *
     * @param setExpectedToCurrent (boolean) {@code true} if the expected heading should be set to the current
     *                             heading, {@code false} otherwise. This would normally be {@code true} during
     *                             robot-relative driving when the driver is turning (the expected heading is
     *                             where the driver is turning to). This would normally be {@code false} during
     *                             field-relative driving or autonomous when the program is setting a target
     *                             heading and the robot is the expected to move along, or turn towards, the
     *                             expected heading; or when robot-relative driving without any turn.
     */
    public void recomputeHeading(boolean setExpectedToCurrent) {
        m_setExpectedToCurrent = setExpectedToCurrent;
        AngleD heading_raw = new AngleD(AngleUnit.DEGREES, m_ahrs.getYaw());
        // This is the logic for detecting and correcting for the IMU discontinuity at +180degrees and -180degrees.
        if (m_headingRawLast.isLessThan(AngleD.NEG_PI_OVER_2) && heading_raw.isGreaterThan(AngleD.ZERO)) {
            // The previous raw IMU heading was negative and close to the discontinuity, and it is now positive. We
            // have gone through the discontinuity, so we decrement the heading revolutions by 1 (we completed a
            // negative revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs--;
        } else if (m_headingRawLast.isGreaterThan(AngleD.PI_OVER_2) && heading_raw.isLessThan(AngleD.ZERO)) {
            // The previous raw IMU heading was positive and close to the discontinuity, and it is now negative. We
            // have gone through the discontinuity, so we increment the heading revolutions by 1 (we completed
            // positive revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs++;
        }
        m_headingRawLast.setValue(heading_raw);

        m_heading.setRadians(m_headingRevs * AngleD.TWO_PI.getRadians())
                .add(heading_raw).subtract(m_refYaw).add(m_refHeading);

        if (setExpectedToCurrent) {
            m_expectedHeading.setValue(m_heading);
        }
    }

    /**
     * Returns a copy of the current robot chassis heading. Note that the robot makes a revolution the heading does
     * not reset when the heading crosses the &pi;, -&pi; boundary - so the actual bounds of the heading is -&infin; to
     * &infin;.
     *
     * @return (not null, AngleD) A copy of the current robot chassis heading.
     */
    @NotNull
    public AngleD getHeading() {
        return m_heading.cloneAngleD();
    }

    /**
     * Get the current heading information for the robot. NOTE: the returned {@link HeadingInfo} reflects snapshot
     * in time and is not updated to reflect future conditions. Please call this method whenever you need
     * {@link HeadingInfo} rather than caching it.
     *
     * @return Returns the heading info, returns {@code null} if there is a problem with the NavX.
     */
    public HeadingInfo getHeadingInfo() {
        if (null == m_ahrs) {
            return null;
        }
        double updateCt = m_ahrs.getUpdateCount();
//        if (updateCt <= m_updateCt) {
//            // there is a problem communication with the NavX - the results we would get from NavX queries
//            // are unreliable.
//            return null;
//        }
        m_updateCt = updateCt;
        return new HeadingInfo(m_heading, m_expectedHeading, m_setExpectedToCurrent);
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
        if (null == m_ahrs) {
            return null;
        }
        // The subtraction of the ref values adjusts for the construction bias of not having the NavX perfectly
        // mounted, or there being some bias in the NavX - i.e. the ref represents the value first reported when
        // the reference position is set, see initializeHeadingAndNav().
        return new NavInfo(
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getPitch() - m_refPitch.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getYaw() - m_refYaw.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getRoll() - m_refRoll.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getPitch()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getYaw()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getRoll()));
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
         * This is the expected heading based on NavX initialization and calls to
         * {@link NavX#incrementExpectedHeading(AngleD)} and {@link NavX#setExpectedHeadingToCurrent()}.
         */
        public final AngleConstantD expectedHeading;

        /**
         * {@code true} if the expected heading was reset to the current heading at the last call to
         * {@link NavX#initializeHeadingAndNav()}, and {@code false} otherwise.
         */
        public final boolean isExpectedTrackingCurrent;

        /**
         * Initialize the {@link NavX.HeadingInfo} at construction.
         *
         * @param heading                   The actual robot heading.
         * @param expectedHeading           The expected robot heading.
         * @param isExpectedTrackingCurrent {@code true} if the expected heading was reset to the current heading
         *                                  at the last call to {@link NavX#initializeHeadingAndNav()}, and
         *                                  {@code false} otherwise.
         */
        HeadingInfo(AngleD heading, AngleD expectedHeading, boolean isExpectedTrackingCurrent) {
            this.heading = heading;
            this.expectedHeading = expectedHeading;
            this.isExpectedTrackingCurrent = isExpectedTrackingCurrent;
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
         * Get the down-field or up-field heading that is closest to the current heading. This is useful if you have
         * a driving mode allowing the driver to get the robot into a somewhat down-field or
         * up-field heading and then lock opn to a directly down-field or up-field heading to
         * thwart defense attempts to the robot while traversing the field.
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
