package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.a05annex.frc.subsystems.SpeedCachedSwerve.RobotRelativePosition;

import static org.junit.jupiter.api.Assertions.*;

/**
 * So we had this question about the time it takes for the drice subsystem to get to the requested speed -
 * specifically, do the motors get to the new settings sistantly, does it take until the next commandis executed
 * (about 20ms) or does it happen somewhere inbetween? We think that inbetween is most likely, that it depends on
 * the magnitude of the requested change, but that there is some <i>phase</i> which will give us a reasonable
 * approximation. We imagined a phase from 0.0 (happens immediately at the beginning of the command cycle) to 1.0,
 * happens about time time the next request happens (at the end of the command cycle - or by the beginning of
 * the next command cycle). So we built a test set that would let us test this.
 * <p/>
 * The test is a short set 6 drive requests at time 0.0, 0.02, 0.04, 0.06, 0.08, and 0.10 seconds with forward speeds
 * of  0.0, 0.0, 0.1, 0.2, 0.3, 0.4. Using a drive that is capable of a max speed of 1m/sec, we should compute
 * for t = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10. 0.12) forward postions of:
 * <ul>
 *     <li>for phase = 0.0: (0.0, 0.0  0.0, 0.002, 0.006, 0.012, 0.020)</li>
 *     <li>for phase = 0.5: (0.0, 0.0  0.0, 0.001, 0.004, 0.009, 0.016)</li>
 *     <li>for phase = 1.0: (0.0, 0.0  0.0, 0.0, 0.002, 0.006, 0.012)</li>
 * </ul>
 *
 */
public class TestPssedCacheSwervePhase {
    /**
     * These are the setting for the calibrated competition robot from '2023 Charged Up' used in testing
     */
    A05Constants.RobotSettings robotSettings = new A05Constants.RobotSettings(0, "Competition",
            0.5461, 0.5461, 2.700, 1.161,
            2.723, 2.448, 1.026,1.0/Mk4NeoModule.MAX_METERS_PER_SEC);

    private final int TEST_CACHE_LENGTH = 10;

    SpeedCachedSwerve initializeAndLoadSCS() {
        SpeedCachedSwerve SCS = SpeedCachedSwerve.getInstance();
        A05Constants.RobotSettings cc = robotSettings;
        SCS.setDriveGeometry(cc.length, cc.width,
                cc.rf, cc.rr, cc.lf, cc.lr,
                cc.maxSpeedCalibration);
        SCS.setCacheLength(TEST_CACHE_LENGTH);
        SCS.addControlRequest(-0.02, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.0, 0.0, 0.0);
        SCS.addControlRequest(0.00, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.0, 0.0, 0.0);
        SCS.addControlRequest(0.02, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.0, 0.0, 0.0);
        SCS.addControlRequest(0.04, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.1, 0.0, 0.0);
        SCS.addControlRequest(0.06, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.2, 0.0, 0.0);
        SCS.addControlRequest(0.08, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.3, 0.0, 0.0);
        SCS.addControlRequest(0.10, AngleConstantD.ZERO, AngleConstantD.ZERO, 0.4, 0.0, 0.0);
        return SCS;
    }

    SpeedCachedSwerve SCS = initializeAndLoadSCS();

    @Test
    @DisplayName("test phase = 0.0")
    void TestPhase_0x0() {
        SCS.setPhase(0.0);
        RobotRelativePosition rrp = SCS.getRobotRelativePositionSince(0.0, -0.02);
        assertEquals(0.0, rrp.forward);
        assertEquals(false, rrp.cacheOverrun);
        rrp = SCS.getRobotRelativePositionSince(0.0, 0.0);
        assertEquals(0.0, rrp.forward);
        assertEquals(false, rrp.cacheOverrun);
        rrp = SCS.getRobotRelativePositionSince(0.02, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.04, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.06, 0.0);
        assertEquals(0.002, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.08, 0.0);
        assertEquals(0.006, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.10, 0.0);
        assertEquals(0.012, rrp.forward, 0.00001);
        // 1 & 2 cycles into the future
        rrp = SCS.getRobotRelativePositionSince(0.12, 0.0);
        assertEquals(0.020, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.14, 0.0);
        assertEquals(0.028, rrp.forward, 0.00001);
        // should fail at 3 cycles into the future
        assertThrows(IllegalArgumentException.class, () -> SCS.getRobotRelativePositionSince(0.18, 0.0));
    }

    @Test
    @DisplayName("test phase = 0.5")
    void TestPhase_0x5() {
        SCS.setPhase(0.5);
        RobotRelativePosition rrp = SCS.getRobotRelativePositionSince(0.0, -0.02);
        assertEquals(true, rrp.cacheOverrun);
        rrp = SCS.getRobotRelativePositionSince(0.0, 0.0);
        assertEquals(0.0, rrp.forward);
        assertEquals(false, rrp.cacheOverrun);
        rrp = SCS.getRobotRelativePositionSince(0.02, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.04, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.06, 0.0);
        assertEquals(0.001, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.08, 0.0);
        assertEquals(0.004, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.10, 0.0);
        assertEquals(0.009, rrp.forward, 0.00001);
        // 1 & 2 cycles into the future
        rrp = SCS.getRobotRelativePositionSince(0.12, 0.0);
        assertEquals(0.016, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.14, 0.0);
        assertEquals(0.024, rrp.forward, 0.00001);
        assertThrows(IllegalArgumentException.class, () -> SCS.getRobotRelativePositionSince(0.18, 0.0));
    }
    @Test
    @DisplayName("test phase = 1.0")
    void TestPhase_1x0() {
        SCS.setPhase(1.0);
        RobotRelativePosition rrp = SCS.getRobotRelativePositionSince(0.0, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.02, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.04, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.06, 0.0);
        assertEquals(0.0, rrp.forward);
        rrp = SCS.getRobotRelativePositionSince(0.08, 0.0);
        assertEquals(0.002, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.10, 0.0);
        assertEquals(0.006, rrp.forward, 0.00001);
        // 1 & 2 cycles into the future
        rrp = SCS.getRobotRelativePositionSince(0.12, 0.0);
        assertEquals(0.012, rrp.forward, 0.00001);
        rrp = SCS.getRobotRelativePositionSince(0.14, 0.0);
        assertEquals(0.020, rrp.forward, 0.00001);
        // should fail at 3 cycles into the future
        assertThrows(IllegalArgumentException.class, () -> SCS.getRobotRelativePositionSince(0.18, 0.0));
    }

}
