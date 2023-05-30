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

    public SparkNeo550(@NotNull CANSparkMax sparkMax, @NotNull RelativeEncoder encoder,
                       @NotNull SparkMaxPIDController sparkMaxPID) {
        super(sparkMax, encoder, sparkMaxPID);
    }

    /**
     * Get the maximum free RPM. This is published in the
     * <a href="https://www.revrobotics.com/rev-21-1651/">REV Neo 550</a> summary as 11000RPM
     * @return Returns the maximum free RPM
     */
    static public double getMaxFreeRPM() {
        // per REV Neo 550 datasheet
        return 11000.0;
    }

    @Override
    public void setCurrentLimit(@NotNull UseType useType,  @NotNull BreakerAmps breakertAmps) {
        verifyInConfig(true, "setCurrentLimit");
        if (A05Constants.getSparkConfigFromFactoryDefaults()) {
            int maxAmps = maxCurrentMatrix[useType.index][breakertAmps.index];
            sparkMax.setSmartCurrentLimit(maxAmps, maxAmps, 10000);
        }
    }

}
