package org.a05annex.frc.subsystems;

import com.revrobotics.*;
import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.a05annex.frc.subsystems.SparkNeo.maxCurrentMatrix;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@Suite
public class TestSparkNeo {

    static double DEFAULT_POSITION = 0.73;

    SparkNeo newSparkNeo() {
        // mocked representations for physical hardware
        final CANSparkMax spark = mock(CANSparkMax.class);
        final RelativeEncoder encoder = mock(RelativeEncoder.class);
        final SparkPIDController pid = mock(SparkPIDController.class);
        when(spark.restoreFactoryDefaults()).thenReturn(REVLibError.kOk);
        when(encoder.getPosition()).thenReturn(DEFAULT_POSITION);
        return new SparkNeo(spark, encoder, pid);
    }

    /**
     * Verify smart motion PID motor controller constants were set as expected.
     *
     * @param pid       (SparkMaxPIDController, not null) The mocked PID motor controller.
     * @param kP        (double, readonly) The expected proportional multiplier.
     * @param kI        (double, readonly) The expected integral multiplier.
     * @param kIZone    (double, readonly) The expected integral zone.
     * @param kFF       (double, readonly) The expected forward multiplier.
     * @param resetMock (boolean) {@code true} to reset the mock function call cache, {@code false} otherwise.
     */
    static void verifyPid(@NotNull SparkPIDController pid, int slotId, final double kP, final double kI,
                          final double kIZone, final double kFF, double kD, double min, double max, boolean resetMock) {
        verify(pid, times(1)).setP(kP, slotId);
        verify(pid, times(1)).setI(kI, slotId);
        verify(pid, times(1)).setIZone(kIZone, slotId);
        verify(pid, times(1)).setFF(kFF, slotId);
        verify(pid, times(1)).setD(kD, slotId);
        verify(pid, times(1)).setOutputRange(min, max, slotId);
        if (resetMock) {
            reset(pid);
        }
    }

    static void verifySmartMotion(@NotNull SparkPIDController pid, double kP, double kI, double kIZone,
                                  double kFF, double kD, double min, double max,
                                  double maxRPM, double maxRPMs, double minRPMs, double allowableError,
                                  boolean resetMock) {
        int slotId = SparkNeo.PIDtype.SMART_MOTION.slotId;
        verify(pid, times(1)).
                setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, slotId);
        verify(pid, times(1)).setSmartMotionMaxVelocity(maxRPM, slotId);
        verify(pid, times(1)).setSmartMotionMaxAccel(maxRPMs, slotId);
        verify(pid, times(1)).setSmartMotionMinOutputVelocity(minRPMs, slotId);
        verify(pid, times(1)).setSmartMotionAllowedClosedLoopError(allowableError, slotId);
        verifyPid(pid, slotId, kP, kI, kIZone,kFF, kD, min, max, resetMock);
    }

    private void verifyUnusedCAN(@NotNull CANSparkMax spark) {
        verify(spark, times(1)).
                setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        verify(spark, times(1)).
                setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
        verify(spark, times(1)).
                setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
        verify(spark, times(1)).
                setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);
    }

    private void verifySetCurrentLimit(@NotNull CANSparkMax spark, @NotNull SparkNeo.UseType useType,
                                       @NotNull SparkNeo.BreakerAmps breakertAmps) {
        int maxAmps = maxCurrentMatrix[useType.index][breakertAmps.index];
        verify(spark, times(1)).
                setSmartCurrentLimit(maxAmps, maxAmps, 10000);
        reset(spark);
    }

    private void verifySetDirection(@NotNull CANSparkMax spark, SparkNeo.Direction direction) {
        assertTrue(SparkNeo.Direction.REVERSE.reversed);
        verify(spark, times(1)).setInverted(direction.reversed);
        reset(spark);
    }

    private void verifyIdleMode(@NotNull CANSparkMax spark, CANSparkMax.IdleMode idleMode) {
        verify(spark, times(1)).setIdleMode(idleMode);
        reset(spark);
    }

    @Test
    @DisplayName("test config from factory defaults")
    void test_configFromFactoryDefaults() {
        A05Constants.setSparkConfig(true,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verify(sparkNeo.sparkMax, times(1)).restoreFactoryDefaults();
        verify(sparkNeo.sparkMax, times(0)).burnFlash();
        verifyUnusedCAN(sparkNeo.sparkMax);
    }

    @Test
    @DisplayName("test config from factory defaults and flash")
    void test_configFromFactoryDefaultsAndBurnFlash() {
        A05Constants.setSparkConfig(true,true);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verify(sparkNeo.sparkMax, times(1)).restoreFactoryDefaults();
        verify(sparkNeo.sparkMax, times(1)).burnFlash();
        verifyUnusedCAN(sparkNeo.sparkMax);
        // verify that flash does not get burned if config from defaults is false
        A05Constants.setSparkConfig(false,true);
        sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verify(sparkNeo.sparkMax, times(0)).restoreFactoryDefaults();
        verify(sparkNeo.sparkMax, times(0)).burnFlash();
        verifyUnusedCAN(sparkNeo.sparkMax);
    }

    @Test
    @DisplayName("test config at power-up")
    void test_configAtPowerUp() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verify(sparkNeo.sparkMax, times(0)).restoreFactoryDefaults();
        verify(sparkNeo.sparkMax, times(0)).burnFlash();
        verifyUnusedCAN(sparkNeo.sparkMax);
    }

    /**
     * This test exercises all the configuration methods: that can only be called in configuration; and that
     * can be called both in configuration and out of configuration.
     */
    @Test
    @DisplayName("test config methods")
    void test_configMethods() {
        A05Constants.setSparkConfig(true,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        // What we are testing here is the right (test) values are in the right places in the PID controller of
        // the spark. These values are not expected to be realistic values - but simply values we can unambiguously
        // track/verify through the call stack.
        // NOTE: here we call the sparkNeo.setXXX() and immediately verify - which resets counters. This lets us test
        // all the variants of each setXXX() method.

        // test setDirection()
        sparkNeo.setDirection(SparkNeo.Direction.REVERSE);
        verifySetDirection(sparkNeo.sparkMax, SparkNeo.Direction.REVERSE);
        sparkNeo.setDirection(SparkNeo.Direction.DEFAULT);
        verifySetDirection(sparkNeo.sparkMax, SparkNeo.Direction.DEFAULT);

        // test steIdle()
        sparkNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        verifyIdleMode(sparkNeo.sparkMax, CANSparkMax.IdleMode.kBrake);

        // setCurrentLimit()
        sparkNeo.setCurrentLimit(SparkNeo.UseType.RPM_PROLONGED_STALL, SparkNeo.BreakerAmps.Amps40);
        verifySetCurrentLimit(sparkNeo.sparkMax, SparkNeo.UseType.RPM_PROLONGED_STALL, SparkNeo.BreakerAmps.Amps40);

        // test the setSmartMotion() variants.
        sparkNeo.setSmartMotion( 0.01, 0.02, 1.0, 0.03, 0.04, -0.95, 0.95,
                10000.0,3000.0, 500.0, 0.5);
        verifySmartMotion(sparkNeo.sparkMaxPID,0.01, 0.02, 1.0,
                0.03, 0.04, -0.95, 0.95,
                10000.0,3000.0, 500.0, 0.5, true);
        sparkNeo.setSmartMotion( 0.01, 0.02, 1.0, 0.03,
                10000.0,3000.0, 500.0, 0.5);
        verifySmartMotion(sparkNeo.sparkMaxPID, 0.01, 0.02, 1.0,
                0.03, 0.0, -1.0, 1.0,
                10000.0,3000.0, 500.0, 0.5, true);

        // test the setPID() variants
        sparkNeo.setRpmPID(0.05, 0.06, 2.0,0.07, 0.08, -0.9, 0.9);
        verifyPid(sparkNeo.sparkMaxPID, SparkNeo.PIDtype.RPM.slotId, 0.05, 0.06, 2.0,0.07,
                0.08, -0.9, 0.9, true);
        sparkNeo.setRpmPID(0.05, 0.06, 2.0,0.07);
        verifyPid(sparkNeo.sparkMaxPID, SparkNeo.PIDtype.RPM.slotId, 0.05, 0.06, 2.0,0.07,
                0.0, -1.0, 1.0, true);

        sparkNeo.setPositionPID(0.09, 0.10, 3.0,0.20, 0.30, -0.85, 0.85);
        verifyPid(sparkNeo.sparkMaxPID, SparkNeo.PIDtype.POSITION.slotId, 0.09, 0.10, 3.0,0.20,
                0.30, -0.85, 0.85, true);
        sparkNeo.setPositionPID(0.09, 0.10, 3.0,0.20);
        verifyPid(sparkNeo.sparkMaxPID, SparkNeo.PIDtype.POSITION.slotId, 0.09, 0.10, 3.0,0.20,
                0.0, -1.0, 1.0, true);

        sparkNeo.endConfig();
        verifyUnusedCAN(sparkNeo.sparkMax);

        // We are now outside configuration
        // test setIdle()
        sparkNeo.setIdleMode(CANSparkMax.IdleMode.kCoast);
        verifyIdleMode(sparkNeo.sparkMax, CANSparkMax.IdleMode.kCoast);

    }

    @Test
    @DisplayName("test invalid in config")
    void test_invalidInConfig() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        assertThrows(IllegalStateException.class,
                sparkNeo::startConfig);
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setEncoderPosition(5.0));
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setTargetRPM(4000.0));
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setSmartMotionTarget(340.0));
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setTargetPosition(37.0));
    }

    @Test
    @DisplayName("test invalid outside of config")
    void test_invalidNotInConfig() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        assertThrows(IllegalStateException.class,
                sparkNeo::endConfig);
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps40));
    }

    @Test
    @DisplayName("test setTargetRPM no config")
    void test_invalidNotConfigured_position() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setTargetRPM(1000.0));
    }

    @Test
    @DisplayName("test setSmartMotionTarget no config")
    void test_invalidNotConfigured_rpm() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setSmartMotionTarget(1000.0));
    }

    @Test
    @DisplayName("test setTargetPosition no config")
    void test_invalidNotConfigured_smartMotion() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setTargetPosition(400.0));
    }

    /**
     * Test that the default current limits are correctly set.
     */
    @Test
    @DisplayName("test the default current limits.")
    void test_defaultCurrentLimits() {
        A05Constants.setSparkConfig(true,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verifySetCurrentLimit(sparkNeo.sparkMax, SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps10);
    }



}
