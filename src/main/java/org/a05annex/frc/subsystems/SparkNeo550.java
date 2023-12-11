package org.a05annex.frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;

/**
 * This class is the packaging for a <a href="https://www.revrobotics.com/rev-21-1651/">REV Neo 550</a> motor
 * powered by a <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark Max</a> motor controller. It binds
 * together the {@link com.revrobotics.CANSparkMax}, {@link com.revrobotics.RelativeEncoder}, and
 * {@link com.revrobotics.SparkMaxPIDController} into a single object. See {@link SparkNeo} for details.
 */
 public class SparkNeo550 extends SparkNeo{

    /**
     * The factory for {@link SparkNeo550} objects for the physical robot. When the robot is powered up and this
     * object is first created, it will represent the SparkMax/Neo550 in its powered up configuration (i.e. it will
     * have the configuration burned into the SparkMax).
     *
     * @param canId The CAN id of the Spark MAX controlling the Neo550 motor.
     * @return The created {@link SparkNeo}
     */
    @NotNull
    public static SparkNeo550 factory(int canId) {
        CANSparkMax sparkMax = new CANSparkMax(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
        return new SparkNeo550(sparkMax, sparkMax.getEncoder(), sparkMax.getPIDController());
    }

    static final int[][] maxCurrentMatrix = {
            // UseType.FREE_SPINNING - a think that is essentially free-spinning. Like a pickup roller which is
            // essentially free spinning except for the momentary power blip during pick up. Current is expected
            // to be low, and since stall is completely unexpected, should be limited to a value the motor and
            // breaker can sustain forever.
            {10, 20, 20, 20},   // 10, 20, 30, 40 Amp
            // UseType.RPM_OCCASIONAL_STALL - you occasionally might do a thing that stalls the motor - like
            // hitting a physical stop.
            {10, 20, 20, 20},   // 10, 20, 30, 40 Amp
            // UseType.RPM_PROLONGED_STALL - speed control (like, of the drive) where you may be stalled (like,
            // playing defense), and you don't want to fry the motor or throw the breaker. The key here is the
            // prolonged stall is intentional and the driver will choose to continue rather than trying to recover
            // from the conditions causing the stall.
            {15, 25, 25, 25},   // 10, 20, 30, 40 Amp
            // UseType.POSITION - position control is essentially always holding the motor at a stall that
            // maintains the position.
            {10, 20, 20, 20}    // 10, 20, 30, 40 Amp
    };

    /**
     * The constructor for a {@code SparkNeo}. Always use {@link #factory(int)} to create the {@code SparkNeo550}
     * <i>unless</i> you are creating a mock {@code SparkNeo550} for testing.
     *
     * @param sparkMax The {@link CANSparkMax}.
     * @param encoder The {@link RelativeEncoder} of the {@link CANSparkMax}.
     * @param sparkMaxPID The {@link SparkMaxPIDController} of the {@link CANSparkMax}.
     */
    public SparkNeo550(@NotNull CANSparkMax sparkMax, @NotNull RelativeEncoder encoder,
                       @NotNull SparkMaxPIDController sparkMaxPID) {
        super(sparkMax, encoder, sparkMaxPID);
    }

    /**
     * Get the maximum free RPM. This is published in the
     * <a href="https://www.revrobotics.com/rev-21-1651/">REV Neo 550</a> summary as 11000RPM. Note that the actual
     * achievable speed is dependent on the load (weight being lifted, friction, inertia, etc.) and is likely
     * 0.8 to 0.9 times the maximum free speed depending on use.
     * @return Returns the maximum free RPM
     */
    static public double getMaxFreeRPM() {
        // per REV Neo 550 datasheet
        return 11000.0;
    }

    @Override
    public void setCurrentLimit(@NotNull UseType useType,  @NotNull BreakerAmps breakerAmps) {
        verifyInConfig(true, "setCurrentLimit");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            int maxAmps = maxCurrentMatrix[useType.index][breakerAmps.index];
            sparkMax.setSmartCurrentLimit(maxAmps, maxAmps, 10000);
        }
    }

}
