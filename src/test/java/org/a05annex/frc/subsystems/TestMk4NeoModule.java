package org.a05annex.frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

/**
 *
 */
@Suite
public class TestMk4NeoModule {

    /**
     * This is a class that creates an initialized drive module using mocked motor controllers, motor encoders,
     * motor PID and analog encoder. The mocked module uses the actual module code, but mocks the controllers and
     * encoders, so we can input sensor (encoder) values and commands we want to respond to and verify the module
     * sends the correct (expected) commands to the controllers.
     */
    private class InitializedMk4NeoModule {
        // mocked representations for physical hardware
        final CANcoder analogEncoder = mock(CANcoder.class);
        final CANcoderConfigurator configurator = mock(CANcoderConfigurator.class);
        // mocked derived representations of components embedded in the physical hardware (i.e. the
        // PID controllers and encoders embedded in the neo motor and spark controller pair)
        final CANSparkMax driveMotor = mock(CANSparkMax.class);
        final RelativeEncoder driveEncoder = mock(RelativeEncoder.class);
        final SparkPIDController drivePID = mock(SparkPIDController.class);
        final SparkNeo driveSparkNeo = new SparkNeo(driveMotor, driveEncoder, drivePID);
        final CANSparkMax spinMotor = mock(CANSparkMax.class);
        final RelativeEncoder spinEncoder = mock(RelativeEncoder.class);
        final SparkPIDController spinPID = mock(SparkPIDController.class);
        final SparkNeo spinSparkNeo = new SparkNeo(spinMotor, spinEncoder, spinPID);
        final Mk4NeoModule driveModule;

        /**
         * Instantiate, initialize, and verify initialization called everything as expected. Reset all the mock
         * objects that were touched in initialization before exiting this instantiation.
         */
        public InitializedMk4NeoModule() {
            final String modulePosition = "test-mk4";
            final StatusSignal<Double> position = mock(StatusSignal.class);
            when(position.getValue()).thenReturn(0.25);
            when(analogEncoder.getConfigurator()).thenReturn(configurator);
            when(configurator.refresh(any(CANcoderConfiguration.class))).thenReturn(StatusCode.OK);
            when(configurator.apply(any(CANcoderConfiguration.class))).thenReturn(StatusCode.OK);
            when(analogEncoder.getAbsolutePosition()).thenReturn(position);
            when(driveMotor.restoreFactoryDefaults()).thenReturn(REVLibError.kOk);
            when(spinMotor.restoreFactoryDefaults()).thenReturn(REVLibError.kOk);
            driveModule = new Mk4NeoModule(modulePosition, driveSparkNeo,
                    spinSparkNeo, analogEncoder);
            driveModule.setCalibrationOffset(-(Math.PI / 2.0));
            driveModule.calibrate();
            assertEquals(Math.PI / 2.0, driveModule.getCalibrationPosition());
            assertEquals(modulePosition, driveModule.getModulePosition());
            // In this test example, the wheel is facing directly backwards, so the position should be set to
            // half a direction revolution.
            TestSparkNeo.verifyPid(drivePID, SparkNeo.PIDtype.RPM.slotId,Mk4NeoModule.DRIVE_kP,
                    Mk4NeoModule.DRIVE_kI, Mk4NeoModule.DRIVE_IZONE, Mk4NeoModule.DRIVE_kFF,
                    0.0, -1.0, 1.0, false);
            TestSparkNeo.verifySmartMotion(drivePID, Mk4NeoModule.SMART_MOTION_kP,
                    Mk4NeoModule.SMART_MOTION_kI, Mk4NeoModule.SMART_MOTION_IZONE, Mk4NeoModule.SMART_MOTION_kFF,
                    0.0, -1.0, 1.0, Mk4NeoModule.SMART_MOTION_MAX_RPM,
                    Mk4NeoModule.SMART_MOTION_MAX_RPMs, Mk4NeoModule.SMART_MOTION_MIN_RPM,
                    Mk4NeoModule.SMART_MOTION_TARGET_TOLERANCE, false);
            TestSparkNeo.verifyPid(drivePID, SparkNeo.PIDtype.POSITION.slotId, Mk4NeoModule.DRIVE_POS_kP,
                    Mk4NeoModule.DRIVE_POS_kI, Mk4NeoModule.DRIVE_POS_IZONE,Mk4NeoModule.DRIVE_POS_kFF,
                    0.0, -1.0, 1.0, false);
            TestSparkNeo.verifyPid(spinPID, SparkNeo.PIDtype.POSITION.slotId, Mk4NeoModule.SPIN_kP, Mk4NeoModule.SPIN_kI,
                    Mk4NeoModule.SPIN_IZONE, 0.0, 0.0, -1.0, 1.0, false);
            verify(spinEncoder, times(1)).setPosition(Math.PI * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER);
            verify(spinPID, times(1)).
                    setReference(0.0, CANSparkMax.ControlType.kPosition, SparkNeo.PIDtype.POSITION.slotId);
            reset(spinEncoder, drivePID, spinPID);
        }
    }

//    /** This method verifies that the PID motor controller constants were set as expected.
//     * @param pid (SparkMaxPIDController, not null) The mocked PID motor controller.
//     * @param kFF (double, readonly) The expected forward multiplier.
//     * @param kP (double, readonly) The expected proportional multiplier.
//     * @param kI (double, readonly) The expected integral multiplier.
//     * @param kIZone (double, readonly) The expected integral zone.
//     */
//    private void verifyPid(@NotNull SparkMaxPIDController pid, final double kFF, final double kP,
//                           final double kI, final double kIZone, double min, double max, int slotId) {
//        verify(pid, times(1)).setFF(kFF, slotId);
//        verify(pid, times(1)).setP(kP, slotId);
//        verify(pid, times(1)).setI(kI, slotId);
//        verify(pid, times(1)).setD(0.0, slotId); // we are not using D, this is always 0.0
//        verify(pid, times(1)).setIZone(kIZone, slotId);
//        verify(pid, times(1)).setOutputRange(min, max, slotId);
//    }

    /**
     * This method verifies what we think we programmed a module to do is what it actually does.
     * </p><p>
     * The deal here is that we are setting a direction and speed for the module, but the actual direction
     * may be reversed 180, or the module may have gone over the 180 degree spin boundary, so the actual
     * may not be what was really set, but something that gives the same result. So we are providing the
     * orientation and speed we are asking for - and the orientation and speed we expect from the module, so
     * we can verify it is going to the expected closest orientation and running in the correct direction.
     *
     * @param dm (InitializedMk4NeoModule, not null) The mock motor controller.
     * @param radians (double, readonly) The requested direction.
     * @param speed (double, readonly) The requested speed.
     * @param actualRadians (double, readonly) The expected actual direction.
     * @param actualSpeed (double, readonly) The expected actual speed.
     */
    private void verifyDirectionAndSpeed(@NotNull InitializedMk4NeoModule dm, double radians, double speed,
                                         double actualRadians, double actualSpeed) {
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, radians), speed);
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER, .00001),
                ArgumentMatchers.eq(CANSparkMax.ControlType.kPosition), ArgumentMatchers.eq(SparkNeo.PIDtype.POSITION.slotId));
        verify(dm.drivePID, times(1)).setReference(actualSpeed * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity, SparkNeo.PIDtype.RPM.slotId);
        // make encoder readings slightly different from what was actually set so that when we get these
        // things we know we are really getting them from the encoder.
        double spinEncPosition = (actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER) + .001;
        when(dm.spinEncoder.getPosition()).thenReturn(spinEncPosition);
        double driveEncPosition = Math.random() * 1000.0; // we have no idea what happens with this
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncPosition);
        double driveEncVelocity = (actualSpeed * Mk4NeoModule.MAX_DRIVE_RPM) + 3.0;
        when(dm.driveEncoder.getVelocity()).thenReturn(driveEncVelocity);

        assertEquals(radians, dm.driveModule.getLastDirection().getRadians());
        assertEquals(speed, dm.driveModule.getLastNormalizedSpeed());
        assertEquals(speed * Mk4NeoModule.MAX_DRIVE_RPM, dm.driveModule.getLastSpeed());

        assertEquals(spinEncPosition, dm.driveModule.getDirectionPosition());
        assertEquals(driveEncPosition, dm.driveModule.getDriveEncoderPosition());
        assertEquals(driveEncVelocity, dm.driveModule.getDriveEncoderVelocity());
    }

    /**
     * This just does the instantiation and verifies the initial calibration. If there is a problem here
     * then all the other tests will probably fail.
     */
    @Test
    @DisplayName("Test Calibration")
    void test_calibration() {
        new InitializedMk4NeoModule();
    }


    /**
     * Test a basic move - 10 degrees clockwise, full speed forward.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(10.0),1.0)")
    void test_set_10_1() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(10.0), 1.0, Math.toRadians(10.0), 1.0);
    }

    /**
     * Test a basic move specified in degrees - 10 degrees clockwise, full speed forward.
     */
    @Test
    @DisplayName("Test setDegreesAndSpeed(10.0,1.0)")
    void test_set_degrees_10_1() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(10.0), 1.0, Math.toRadians(10.0), 1.0);
    }

    /**
     * Test a close to, but less than, 90 degree clockwise, half speed. Spin should be clockwise and speed forward
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(80.0),0.5)")
    void test_set_80_1() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(80.0), 0.5, Math.toRadians(80.0), 0.5);
    }

    /**
     * Test a slightly greater than 90 degree clockwise, full speed. The closest rotation is to orient
     * the back of the wheel and go backwards - make sure this happens.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(100.0),0.25)")
    void test_set_100_1() {
        // Should spin negatively and go backwards
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(100.0), 0.25, Math.toRadians(-80.0), -0.25);
    }

    /**
     * Test a basic move - 10 degrees counter-clockwise, full speed.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-10.0),1.0)")
    void test_set_neg_10_1() {
        // Should spin negatively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(-10.0), 1.0, Math.toRadians(-10.0), 1.0);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-80.0),1.0)")
    void test_set_neg_80_1() {
        // Should spin negatively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(-80.0), 0.5, Math.toRadians(-80.0), 0.5);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-100.0),1.0)")
    void test_set_neg_100_1() {
        // Should spin positively and go backwards
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm, Math.toRadians(-100.0), 0.25, Math.toRadians(80.0), -0.25);
    }

    @Test
    @DisplayName("Test 180 boundary clockwise")
    void test_180_boundary_clockwise() {
        // Should spin positively and keep spinning positively at the 180 boundary - really a pain because we
        // need 2 less than 90 degree steps to get the front of the wheel there.
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, 85.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, 170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID, dm.drivePID);
        verifyDirectionAndSpeed(dm, Math.toRadians(-170.0), 0.35, Math.toRadians(190.0), 0.35);
    }

    @Test
    @DisplayName("Test 180 boundary counter clockwise")
    void test_180_boundary_counter_clockwise() {
        // Should spin negatively and keep spinning negatively at the 180 boundary - really a pain because we
        // need 2 less than -90 steps to get the front of the wheel there.
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, -85.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, -170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID, dm.drivePID);
        verifyDirectionAndSpeed(dm, Math.toRadians(170.0), 0.35, Math.toRadians(-190.0), 0.35);
    }

    /**
     * The forwards-backwards logic finds the least spin of the wheel (>= 90.0) and the appropriate forward/backward
     * multiplier. A concern it the programming has been making sure the forward-backward states are handled
     * correctly. Specifically, if I have 2 forward commands it a row or 2 backwards commands in a row, does the
     * module correctly remember what it is doing and not do a 180 degree spin.
     */
    @Test
    @DisplayName("Test backwards forwards")
    void test_forward_backward() {
        // Go forward 2 steps
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, 0.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, 0.0), 1.0);
        verify(dm.spinPID, times(2)).setReference(0.0,
                CANSparkMax.ControlType.kPosition, SparkNeo.PIDtype.POSITION.slotId);
        verify(dm.drivePID, times(2)).setReference(Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity, SparkNeo.PIDtype.RPM.slotId);
        assertEquals(CANSparkMax.ControlType.kVelocity, dm.driveModule.getSparkControlType());
        // Go backwards (180 degrees) 2 steps - should be no spin and negative speed
        reset(dm.spinPID, dm.drivePID);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, Math.PI), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, Math.PI), 1.0);
        verify(dm.spinPID, times(2)).setReference(0.0,
                CANSparkMax.ControlType.kPosition, SparkNeo.PIDtype.POSITION.slotId);
        verify(dm.drivePID, times(2)).setReference(-1.0 * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity, SparkNeo.PIDtype.RPM.slotId);
        // Go forwards again steps - should be no spin and positive speed
        reset(dm.spinPID, dm.drivePID);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, 0.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, 0.0), 1.0);
        verify(dm.spinPID, times(2)).setReference(0.0,
                CANSparkMax.ControlType.kPosition, SparkNeo.PIDtype.POSITION.slotId);
        verify(dm.drivePID, times(2)).setReference(Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity, SparkNeo.PIDtype.RPM.slotId);
    }


    private void verifyDirectionAndDistance(InitializedMk4NeoModule dm, double radians, double deltaTics,
                                            double actualRadians) {
        double driveEncStartPosition = Math.random() * 1000.0; //generate an arbitrary start position
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition);
        // call the method being tested
        dm.driveModule.setDirectionAndDistance(new AngleD(AngleUnit.RADIANS, radians), deltaTics, 1.0);
        // Verify the direction of the wheel was correctly set (set to the spin PID)
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER, .00001),
                ArgumentMatchers.eq(CANSparkMax.ControlType.kPosition),
                ArgumentMatchers.eq(SparkNeo.PIDtype.POSITION.slotId));
        // test the last call to drive PID setReference is for the correct target position, and is asking for position
        // (not speed).
        verify(dm.drivePID).setReference(driveEncStartPosition + deltaTics,
                CANSparkMax.ControlType.kPosition, SparkNeo.PIDtype.POSITION.slotId);
        assertEquals(CANSparkMax.ControlType.kPosition, dm.driveModule.getSparkControlType());
        // make encoder readings slightly different from what was actually set so that when we get these
        // things we know we are really getting them from the encoder.
        double spinEncPosition = (actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER) + .001;
        when(dm.spinEncoder.getPosition()).thenReturn(spinEncPosition);
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition + deltaTics);

        assertEquals(spinEncPosition, dm.driveModule.getDirectionPosition());
        assertEquals(driveEncStartPosition + deltaTics, dm.driveModule.getDriveEncoderPosition());
    }

    /**
     * Test the setDirectionAndDistance(AngleD targetDirection, double deltaTics, double maxSpeed) method
     */
    @Test
    @DisplayName("Test setDirectionAndDistance(Math.toRadians(10.0),20.0)")
    void test_set_direction_distance_10_20() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndDistance(dm, Math.toRadians(10.0), 20.0, Math.toRadians(10.0));
    }
    private void verifyDirectionAndSmartMotionDistance(InitializedMk4NeoModule dm, double radians, double deltaTics,
                                                       double maxSpeed, double maxAccel,
                                                       double actualRadians) {
        double driveEncStartPosition = Math.random() * 1000.0; //generate an arbitrary start position
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition);
        // call the method being tested
        dm.driveModule.setDirectionAndSmartMotionDistance(new AngleD(AngleUnit.RADIANS, radians), deltaTics,
                maxSpeed, maxAccel);
        // Verify the direction of the wheel was correctly set (set to the spin PID)
        REVLibError revLibError = verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER, .00001),
                ArgumentMatchers.eq(CANSparkMax.ControlType.kPosition),
                ArgumentMatchers.eq(SparkNeo.PIDtype.POSITION.slotId));
        // test the last call to drive PID setReference is for the correct target position, and is asking for position
        // (not speed).
        verify(dm.drivePID).setReference(driveEncStartPosition + deltaTics,
                CANSparkMax.ControlType.kSmartMotion, SparkNeo.PIDtype.SMART_MOTION.slotId);
        assertEquals(CANSparkMax.ControlType.kSmartMotion, dm.driveModule.getSparkControlType());
        // make encoder readings slightly different from what was actually set so that when we get these
        // things we know we are really getting them from the encoder.
        double spinEncPosition = (actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER) + .001;
        when(dm.spinEncoder.getPosition()).thenReturn(spinEncPosition);
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition + deltaTics);

        assertEquals(spinEncPosition, dm.driveModule.getDirectionPosition());
        assertEquals(driveEncStartPosition + deltaTics, dm.driveModule.getDriveEncoderPosition());
    }
    /**
     * Test the setDirectionAndDistance(AngleD targetDirection, double deltaTics, double maxSpeed) method
     */
    @Test
    @DisplayName("Test setDirectionAndSmartMotionDistance(Math.toRadians(10.0),20.0,5000.0,600000.0)")
    void test_set_direction_smart_motion_distance_10_20() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSmartMotionDistance(dm, Math.toRadians(10.0), 20.0,
                5000.0, 600000.0, Math.toRadians(10.0));
    }
}
