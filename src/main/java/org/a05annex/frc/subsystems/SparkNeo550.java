package org.a05annex.frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
}
