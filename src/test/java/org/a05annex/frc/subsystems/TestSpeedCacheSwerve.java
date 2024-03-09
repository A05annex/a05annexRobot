package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * These are tests of the basic functionality of the {@code SpeedCacheSwerve}, a cache of commands to the swerve
 * drive with telemetry calculations that let us predict robot position from a known past position (usually from some
 * visual target) in the past (because of the latency of the targeting pipeline).
 */
public class TestSpeedCacheSwerve {

    /**
     * These are the setting for the calibrated competition robot from '2023 Charged Up' used in testing
     */
    A05Constants.RobotSettings robotSettings = new A05Constants.RobotSettings(0, "Competition",
            0.5461, 0.5461, 2.700, 1.161,
             2.723, 2.448, 1.026,0.9650);

    private final int TEST_CACHE_LENGTH = 10;

    /**
     * Standard initialization of the {@code SpeedCacheSwerve} (SCS) for these tests.
     * @return The initialized {@code SpeedCacheSwerve} (SCS).
     */
    SpeedCachedSwerve getInitializedSCS() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        A05Constants.RobotSettings cc = robotSettings;
        SCS.setDriveGeometry(cc.length, cc.width,
                cc.rf, cc.rr, cc.lf, cc.lr,
                cc.maxSpeedCalibration);
        SCS.setCacheLength(TEST_CACHE_LENGTH);
        return SCS;
    }

    @Test
    @DisplayName("test cacheConfiguration")
    void TestConfiguration() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        A05Constants.RobotSettings cc = robotSettings;
        assertEquals(cc.length, SCS.getDriveLength());
        assertEquals(cc.width, SCS.getDriveWidth());
        assertEquals(Mk4NeoModule.MAX_METERS_PER_SEC * cc.maxSpeedCalibration, SCS.getMaxMetersPerSec());
        double driveDiagonal = Utl.length(cc.length, cc.width);
        assertEquals(SCS.getMaxMetersPerSec() / (0.5 * driveDiagonal), SCS.getMaxRadiansPerSec());
    }

    @Test
    @DisplayName("test setCacheLength")
    void TestSetCacheLength() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        assertEquals(10, SCS.getCacheLength());

    }

    @Test
    @DisplayName("test recordControlRequests")
    void TestRecordControlRequests() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        for (int i = 0; i < 10; i++) {
            double currentTime = Timer.getFPGATimestamp();
            SCS.swerveDriveComponents((i * 0.1) - 0.5, (i * 0.1) - 0.9, (i * 0.1) - 0.1);
            assertEquals(i, SCS.mostRecentControlRequest);
            assertEquals((i * 0.1) - 0.5, SCS.controlRequests[SCS.mostRecentControlRequest].forward);
            assertEquals((i * 0.1) - 0.9, SCS.controlRequests[SCS.mostRecentControlRequest].strafe);
            assertEquals((i * 0.1) - 0.1, SCS.controlRequests[SCS.mostRecentControlRequest].rotation);
            //assertTrue(currentTime <= SCS.controlRequests[SCS.mostRecentControlRequest].timeStamp);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        assertEquals(9, SCS.mostRecentControlRequest);
    }

    @Test
    @DisplayName("test getPositionSinceTime")
    void TestGetPositionSinceTime() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        SCS.addControlRequest(4.0, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(4.02, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, -0.1);
        SCS.addControlRequest(4.04, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, -0.1);
        SCS.addControlRequest(4.06, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, -0.1);
        SCS.addControlRequest(4.08, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        // at default phase = 0.0
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(4.078, 4.028);
        assertEquals(.05 * 0.2 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(.05 * 0.1 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
        assertEquals(0.0,position.heading.getRadians(), 0.0000001);
        assertEquals(false, position.cacheOverrun);
        // at phase = 0.5
        SCS.setPhase(0.5);
        position = SCS.getRobotRelativePositionSince(4.078, 4.028);
        assertEquals(.05 * 0.2 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(.05 * 0.1 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
        assertEquals(0.0,position.heading.getRadians(), 0.0000001);
        assertEquals(false, position.cacheOverrun);
        // at phase = 1.0
        SCS.setPhase(1.0);
        position = SCS.getRobotRelativePositionSince(4.078, 4.028);
        assertEquals(.05 * 0.2 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(.05 * 0.1 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
        assertEquals(0.0,position.heading.getRadians(), 0.0000001);
        assertEquals(false, position.cacheOverrun);
    }

    @Test
    @DisplayName("test cache overflow")
    void TestCacheOverflow() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        double nextRequestTime = 2.0;
        // populate the cache a wrap around the end (more entries than the length of the cache.
        for (int i = 0; i < 1.5 * TEST_CACHE_LENGTH; i++) {
            nextRequestTime += 0.02;
            SCS.addControlRequest(nextRequestTime, AngleConstantD.ZERO,
                    AngleConstantD.ZERO, 0.2, 0.1, 0.01);
        }
        SCS.setPhase(0.5);

        // OK, do something in the cache range, (0.02 * (TEST_CACHE_LENGTH - 1)) - .001, the .001 is to get just past
        // the time of the last point to be included.
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(nextRequestTime,
                        nextRequestTime - (0.02 * (TEST_CACHE_LENGTH - 2)) - .001);
        assertEquals(false, position.cacheOverrun);
        assertEquals((TEST_CACHE_LENGTH - 2) * 0.02 * 0.2 * SCS.getMaxMetersPerSec(),
                position.forward, 0.0007);
        assertEquals((TEST_CACHE_LENGTH - 2) * 0.02 * 0.1 * SCS.getMaxMetersPerSec(),
                position.strafe, 0.0007);
        assertEquals(0.0,position.heading.getRadians(), 0.0000001);
        // OK, now do something outside the cache range, and we should only get the cached number of values added, and
        // the cacheOverrun flag should be set.
        position = SCS.getRobotRelativePositionSince(nextRequestTime,
                nextRequestTime - (0.02 * (TEST_CACHE_LENGTH + 2)));
        assertEquals(true, position.cacheOverrun);
        assertEquals(0.0,
                position.forward, 0.0000001);
        assertEquals(0.0,
                position.strafe, 0.0000001);
        assertEquals(0.0,position.heading.getRadians(), 0.0000001);
    }


    @Test
    @DisplayName("test getExpectedHeadingDeltaAt() for a very low latency camera")
    void TestLowLatencyCamera() {
        // So, the deal here is that when we started to use the orange pi 5, the latency was very low (less
        // than the 20ms command cycle) asa result the 'since time' might be after the last recorded command
        // time - which caused the cache to crash - so let's test and fix that.
        SpeedCachedSwerve SCS = getInitializedSCS();
        // lets populate the cache with some constant speed profile
        double nextCommandTime = 2.0;
        for (int i = 0; i < .5 * TEST_CACHE_LENGTH; i++) {
            nextCommandTime += 0.02;
            SCS.addControlRequest(nextCommandTime, AngleConstantD.ZERO,
                    AngleConstantD.ZERO, 0.2, 0.1, 0.00);
        }
        // So we've populated the cache, and we will look at what happens if the last visual target time is after
        // the last command - yeah failed - fixed that, let's test if the returned position is correct.
        nextCommandTime += 0.02;
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(nextCommandTime,  nextCommandTime - 0.01);
        // OK, so the robot is moving at a constant velocity of forward 0.2 and strafe 0.1, the interval between the
        // last recorded position and now is 0.01 seconds - so the robot should have gone
        assertEquals(false, position.cacheOverrun);
        assertEquals(0.2 * 0.01 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(0.1 * 0.01 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
    }

    @Test
    @DisplayName("test getExpectedHeadingDeltaAt() for a pretty low latency camera")
    void TestPrettyLowLatencyCamera() {
        // So, the deal here is that when we started to use the orange pi 5, the latency was very low (less
        // than the 20ms command cycle) asa result the 'since time' might be after the last recorded command
        // time - which caused the cache to crash - so let's test and fix that.
        SpeedCachedSwerve SCS = getInitializedSCS();
        // lets populate the cache with some constant speed profile
        double nextCommandTime = 2.0;
        for (int i = 0; i < .5 * TEST_CACHE_LENGTH; i++) {
            nextCommandTime += 0.02;
            SCS.addControlRequest(nextCommandTime, AngleConstantD.ZERO,
                    AngleConstantD.ZERO, 0.2, 0.1, 0.00);
        }
        // So we've populated the cache, and we will look at what happens if the last visual target time is after
        // the last command - yeah failed - fixed that, let's test if the returned position is correct.
        nextCommandTime += 0.02;
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(nextCommandTime,  nextCommandTime - 0.03);
        // OK, so the robot is moving at a constant velocity of forward 0.2 and strafe 0.1, the interval between the
        // last recorded position and now is 0.03 seconds - so the robot should have gone
        assertEquals(false, position.cacheOverrun);
        assertEquals(0.2 * 0.03 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(0.1 * 0.03 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
    }


    @Test
    @DisplayName("test getExpectedHeadingDeltaAt()")
    void TestGetExpectedHeadingDelta() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        AngleConstantD smallActual = new AngleConstantD(AngleUnit.RADIANS, 0.1);
        // load a short path into the cache
        SCS.addControlRequest(0.0, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.02, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.04, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.06, smallActual, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.08, smallActual, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.10, smallActual, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        // test the interpolated values between 0.04 and 0.06 are correct.
        assertEquals(0.0, SCS.getExpectedHeadingDeltaAt(0.03).getRadians(), 0.0000001);
        assertEquals(0.0, SCS.getExpectedHeadingDeltaAt(0.04).getRadians(), 0.0000001);
        assertEquals(0.025, SCS.getExpectedHeadingDeltaAt(0.045).getRadians(), 0.0000001);
        assertEquals(0.050, SCS.getExpectedHeadingDeltaAt(0.05).getRadians(), 0.0000001);
        assertEquals(0.075, SCS.getExpectedHeadingDeltaAt(0.055).getRadians(), 0.0000001);
        assertEquals(0.1, SCS.getExpectedHeadingDeltaAt(0.06).getRadians(), 0.0000001);
        assertEquals(0.1, SCS.getExpectedHeadingDeltaAt(0.07).getRadians(), 0.0000001);
    }
    @Test
    @DisplayName("test getExpectedHeadingDeltaAt() overruns")
    void TestGetExpectedHeadingDeltaOverrun() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        AngleConstantD smallActual = new AngleConstantD(AngleUnit.RADIANS, 0.1);
        // load a short path into the cache
        SCS.addControlRequest(0.02, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.04, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.06, smallActual, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        SCS.addControlRequest(0.08, smallActual, AngleConstantD.ZERO, 0.2, 0.1, 0.3);
        // Ask for the heading delta at a time before the start of the cache. This is a cache overrun that generates
        // an IllegalArgumentException
        assertEquals(null, SCS.getExpectedHeadingDeltaAt(0.00));
        assertThrows(IllegalArgumentException.class, () -> SCS.getExpectedHeadingDeltaAt(0.10));
    }
 }
