package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX flywheelMotor1;
    private final WPI_TalonFX flywheelMotor2;
    private final WPI_TalonFX turretMotor;
    private final WPI_TalonFX hoodMotor;

    public static final double MAX_FORWARD = 0.1;
    public static final double FLYWHEEL_VELOCITY = 10;
    public static final double MAX_REVERSE = -0.1;
    public static final double STOP_MOTOR = 0;
    public static final double DEGREES_TO_ENCODER_TICKS = 2048 / 360; // 2048 ticks per 360 degrees
    public static final double MIN_TURRET_ANGLE = 0; // These are currently unused
    public static final double MAX_TURRET_ANGLE = 180; // Placeholder, maybe need to update
    public static final double TURRET_MAX_SPEED = 0.1;
    public static final double TURRET_MIN_SPEED = -0.1;
    public static final double TURRET_P = 0.01; // Placeholder PID constants
    public static final double TURRET_I = 0;
    public static final double TURRET_D = 0;

    public ShooterSubsystem(WPI_TalonFX flywheelMotor1, WPI_TalonFX flywheelMotor2, WPI_TalonFX turretMotor,
            WPI_TalonFX hoodMotor) {
        var limit = new SupplyCurrentLimitConfiguration(true, 40, 40, 500);
        flywheelMotor1.configSupplyCurrentLimit(limit);
        flywheelMotor2.configSupplyCurrentLimit(limit);
        flywheelMotor2.setInverted(true);
        turretMotor.configSupplyCurrentLimit(limit);
        hoodMotor.configSupplyCurrentLimit(limit);

        this.flywheelMotor1 = flywheelMotor1;
        this.flywheelMotor2 = flywheelMotor2;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        // Make sure turret is using builtin encoder, may be unneccessary
        this.turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // Configure turret PID
        this.turretMotor.config_kP(0, TURRET_P);
        this.turretMotor.config_kI(0, TURRET_I);
        this.turretMotor.config_kD(0, TURRET_D);
    }

    public void hoodMotorExtend() {
        hoodMotor.set(MAX_FORWARD);
    }

    public void hoodMotorRetract() {
        hoodMotor.set(MAX_REVERSE);
    }

    public void hoodMotorStop() {
        hoodMotor.set(STOP_MOTOR);
    }

    // TODO make sure these motors are going in the right direction
    public void startFlywheel() {
        flywheelMotor1.set(ControlMode.Velocity, FLYWHEEL_VELOCITY);
        flywheelMotor2.set(ControlMode.Velocity, FLYWHEEL_VELOCITY);
    }

    public void stopFlywheel() {
        flywheelMotor1.set(STOP_MOTOR);
        flywheelMotor2.set(STOP_MOTOR);
    }

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    // TODO use limit switches to reset the encoder
    public void resetTurretEncoder() {
        turretMotor.setSelectedSensorPosition(0);
    }

    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
    }

    /**
     * Sets the turret's angle to the given angle (Does not check angle limits).
     * 
     * @param angle
     *            the angle (in degrees) to set the turret to (negative for
     *            counterclockwise)
     */
    public void setTurretAngle(double angle) {
        turretMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * angle);
    }

    public void updateTurretAngle(double deltaAngle) {
        double currentAngle = turretMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
        setTurretAngle(currentAngle + deltaAngle);
    }

    public void setTurretSpeed(double speed) {
        turretMotor.set(Math.min(Math.max(speed, TURRET_MIN_SPEED), TURRET_MAX_SPEED));
    }
}
