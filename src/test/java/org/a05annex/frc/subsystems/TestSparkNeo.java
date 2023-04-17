package org.a05annex.frc.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.mock;

@Suite
public class TestSparkNeo {

    SparkNeo newSparkNeo() {
        // mocked representations for physical hardware
        final CANSparkMax spark = mock(CANSparkMax.class);
        final RelativeEncoder encoder = mock(RelativeEncoder.class);
        final SparkMaxPIDController pid = mock(SparkMaxPIDController.class);
        return new SparkNeo(spark, encoder, pid);
    }

    /** This method verifies that the PID motor controller constants were set as expected.
     * @param pid (SparkMaxPIDController, not null) The mocked PID motor controller.
     * @param kFF (double, readonly) The expected forward multiplier.
     * @param kP (double, readonly) The expected proportional multiplier.
     * @param kI (double, readonly) The expected integral multiplier.
     * @param kIZone (double, readonly) The expected integral zone.
     */
    private void verifyPid(@NotNull SparkMaxPIDController pid, final double kFF, final double kP,
                           final double kI, final double kIZone, double kD, double min, double max, int slotId) {
        verify(pid, times(1)).setFF(kFF, slotId);
        verify(pid, times(1)).setP(kP, slotId);
        verify(pid, times(1)).setI(kI, slotId);
        verify(pid, times(1)).setD(kD, slotId);
        verify(pid, times(1)).setIZone(kIZone, slotId);
        verify(pid, times(1)).setOutputRange(min, max, slotId);
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
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 500);

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
        // verify that flash does not get burned is vonfig from defaults is false
        A05Constants.setSparkConfig(false,true);
        sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        sparkNeo.endConfig();
        verify(sparkNeo.sparkMax, times(0)).restoreFactoryDefaults();
        verify(sparkNeo.sparkMax, times(0)).burnFlash();
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
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 500);
        verify(sparkNeo.sparkMax, times(1)).
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 500);
    }

    @Test
    @DisplayName("test invalid in config")
    void test_invalidInConfig() {
        A05Constants.setSparkConfig(false,false);
        SparkNeo sparkNeo = newSparkNeo();
        sparkNeo.startConfig();
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.startConfig());
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
                () -> sparkNeo.endConfig());
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setReversed());
        assertThrows(IllegalStateException.class,
                () -> sparkNeo.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerSupplier.REV,
                        SparkNeo.BreakerAmps.Amps40));
    }
}
