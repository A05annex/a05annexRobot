package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestSpeedCacheSwerve {

    A05Constants.RobotSettings robotSettings = new A05Constants.RobotSettings(0, "Competition",
            0.5461, 0.5461, 2.700, 1.161,
             2.723, 2.448, 1.026,0.9650);

    private final int TEST_CACHE_LENGTH = 10;

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
            assertTrue(currentTime <= SCS.controlRequests[SCS.mostRecentControlRequest].timeStamp);

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
        SCS.addControlRequest(0.2, 0.1, 0.3, 4.0);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.02);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.04);
        SCS.addControlRequest(0.2, 0.1, -0.1, 4.06);
        SCS.addControlRequest(0.2, 0.1, 0.3, 4.08);
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(4.10, 4.03);
        assertEquals(.06 * 0.2 * SCS.getMaxMetersPerSec(), position.forward, 0.0000001);
        assertEquals(.06 * 0.1 * SCS.getMaxMetersPerSec(), position.strafe, 0.0000001);
        assertEquals(.02 * 0.1 * SCS.getMaxRadiansPerSec(),
                position.heading.getRadians(), 0.0000001);
        assertEquals(false, position.cacheOverrun);
    }

    @Test
    @DisplayName("test cache overflow")
    void TestCacheOverflow() {
        SpeedCachedSwerve SCS = getInitializedSCS();
        double nextRequestTime = 2.0;
        // populate the cache a wrap around the end (more entries than the length of the cache.
        for (int i = 0; i < 1.5 * TEST_CACHE_LENGTH; i++) {
            SCS.addControlRequest(0.2, 0.1, 0.01, nextRequestTime);
            nextRequestTime += 0.02;
        }
        // OK, do something in the cache range, (0.02 * (TEST_CACHE_LENGTH - 1)) - .001, the .001 is to get just past
        // the time of the last point to be included.
        SpeedCachedSwerve.RobotRelativePosition position =
                SCS.getRobotRelativePositionSince(nextRequestTime,
                        nextRequestTime - (0.02 * (TEST_CACHE_LENGTH - 1)) - .001);
        assertEquals((TEST_CACHE_LENGTH - 1) * 0.02 * 0.2 * SCS.getMaxMetersPerSec(),
                position.forward, 0.0000001);
        assertEquals((TEST_CACHE_LENGTH - 1) * 0.02 * 0.1 * SCS.getMaxMetersPerSec(),
                position.strafe, 0.0000001);
        assertEquals((TEST_CACHE_LENGTH - 1) * 0.02 * 0.01 * SCS.getMaxRadiansPerSec(),
                position.heading.getRadians(), 0.0000001);
        assertEquals(false, position.cacheOverrun);
        // OK, now do something outside the cache range, and we should only get the cached number of values added, and
        // the cacheOverrun flag should be set.
        position = SCS.getRobotRelativePositionSince(nextRequestTime,
                nextRequestTime - (0.02 * (TEST_CACHE_LENGTH + 2)));
        assertEquals(TEST_CACHE_LENGTH * 0.02 * 0.2 * SCS.getMaxMetersPerSec(),
                position.forward, 0.0000001);
        assertEquals(TEST_CACHE_LENGTH * 0.02 * 0.1 * SCS.getMaxMetersPerSec(),
                position.strafe, 0.0000001);
        assertEquals(TEST_CACHE_LENGTH * 0.02 * 0.01 * SCS.getMaxRadiansPerSec(),
                position.heading.getRadians(), 0.0000001);
        assertEquals(true, position.cacheOverrun);
    }
}
