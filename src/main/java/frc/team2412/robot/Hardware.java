package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.team2412.robot.util.Mk4Configuration;
import frc.team2412.robot.util.MultiplexedColorSensor;

import org.frcteam2910.common.robot.drivers.NavX;
import org.photonvision.PhotonCamera;

import static frc.team2412.robot.Hardware.HardwareConstants.*;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Hardware {
    public static class HardwareConstants {
        //drive can ids are range 1-19
        public static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));

        public static final SPI.Port GYRO_PORT = SPI.Port.kMXP;

        //cameras
        public static final String LIMELIGHT = "limelight", FRONT_CAM = "front";

        //shooter can ids are range 20-29
        public static final int FLYWHEEL_1 = 0, FLYWHEEL_2 = 0, TURRET = 0, HOOD = 0;

        //intake can ids are range 30-39
        public static final int INTAKE_1 = 0, INTAKE_2 = 0, INTAKE_UP = 0, INTAKE_DOWN = 0;

        //index can ids are range 40-49
        public static final int INDEX = 0;

        //climb can ids are range 50-59
        public static final int CLIMB_FIXED_1 = 0, CLIMB_FIXED_2 = 0, CLIMB_ANGLED_1 = 0, CLIMB_ANGLED_2 = 0, CLIMB_ANGLE_UP = 0, CLIMB_ANGLE_DOWN = 0;

        //default address of TCA9548A
        public static final int I2C_MULTIPLEXER_ADDRESS = 0x70;
        public static final Port I2C_MULTIPLEXER_PORT = I2C.Port.kOnboard;
        //which port the I2C device plugged in on the multiplexer
        public static final int LEFT_COLORSENSOR_PORT = 1, RIGHT_COLORSENSOR_PORT = 2, CENTER_COLORSENSOR_PORT = 3;
    }

    //drive
    public SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public NavX navX;

    //cameras
    public PhotonCamera limelight, frontCamera;

    //shooter
    public WPI_TalonFX flywheelMotor1, flywheelMotor2, turretMotor, hoodMotor;

    //intake
    public WPI_TalonFX intakeMotor1, intakeMotor2;
    public DoubleSolenoid intakeSolenoid;
    public I2C i2c_multiplexer;
    public MultiplexedColorSensor leftColorSensor;
    public MultiplexedColorSensor rightColorSensor;
    public MultiplexedColorSensor centerColorSensor;

    //climb
    public WPI_TalonFX climbFixed1, climbFixed2, climbAngled1, climbAngled2;
    public DoubleSolenoid climbAngle;

    //index
    public WPI_TalonFX indexMotor;

    public Hardware() {
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons();
            frontRightModule = FRONT_RIGHT_CONFIG.falcons();
            backLeftModule = BACK_LEFT_CONFIG.falcons();
            backRightModule = BACK_RIGHT_CONFIG.falcons();
            navX = new NavX(GYRO_PORT);
        }
        if (CLIMB_ENABLED) {
            climbFixed1 = new WPI_TalonFX(CLIMB_FIXED_1);
            climbFixed2 = new WPI_TalonFX(CLIMB_FIXED_2);
            climbAngled1 = new WPI_TalonFX(CLIMB_ANGLED_1);
            climbAngled2 = new WPI_TalonFX(CLIMB_ANGLED_2);
            climbAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP, CLIMB_ANGLE_DOWN);
        }
        if (INTAKE_ENABLED) {
            intakeMotor1 = new WPI_TalonFX(INTAKE_1);
            intakeMotor2 = new WPI_TalonFX(INTAKE_2);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_UP, INTAKE_DOWN);
            /**
             * example code from https://www.chiefdelphi.com/t/multiple-rev-color-sensors-using-an-i2c-multiplexer-java/377776
             * also possible to put a wrapper for the ColorSensor on Multiplexer
             */
            if (I2C_MUX_ENABLED) {
                this.i2c_multiplexer = new I2C(I2C_MULTIPLEXER_PORT, I2C_MULTIPLEXER_ADDRESS);
                this.leftColorSensor = new MultiplexedColorSensor(i2c_multiplexer, LEFT_COLORSENSOR_PORT);
                this.rightColorSensor = new MultiplexedColorSensor(i2c_multiplexer, RIGHT_COLORSENSOR_PORT);
                this.centerColorSensor = new MultiplexedColorSensor(i2c_multiplexer, CENTER_COLORSENSOR_PORT);
            }
        }
        if (INDEX_ENABLED) {
            indexMotor = new WPI_TalonFX(INDEX);
        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = new WPI_TalonFX(FLYWHEEL_1);
            flywheelMotor2 = new WPI_TalonFX(FLYWHEEL_2);
            turretMotor = new WPI_TalonFX(TURRET);
            hoodMotor = new WPI_TalonFX(HOOD);
        }
        if (DRIVER_VIS_ENABLED) {
            frontCamera = new PhotonCamera(FRONT_CAM);
        }
        if (GOAL_VIS_ENABLED) {
            limelight = new PhotonCamera(LIMELIGHT);
        }
    }
}
