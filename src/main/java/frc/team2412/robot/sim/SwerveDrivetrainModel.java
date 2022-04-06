package frc.team2412.robot.sim;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Rotation2;

import java.util.ArrayList;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.sim.TalonFXSimProfile.TalonFXConstants.ENCODER_TICKS_PER_REVOLUTION;
import static frc.team2412.robot.sim.TalonFXSimProfile.TalonFXConstants.RPM_TO_VELOCITY;

public class SwerveDrivetrainModel {
    final SwerveModule[] realModules;
    final Gyroscope gyro;
    final Field2d field;
    final QuadSwerveSim swerveSim;
    ArrayList<SwerveModuleSim> simModules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);
    SwerveDrivePoseEstimator poseEstimator;

    // ArrayList<SteerController> steerControllers = new
    // ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    // ArrayList<DriveController> driveControllers = new
    // ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    // ArrayList<AbsoluteEncoder> steerEncoders = new
    // ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    double lastSimulationStepTime;
    Pose2d endActualPose; // Ending pose after each simulation time step
    Pose2d endEstimatedPose; // Ending estimated pose after each simulation time step

    private static final double POSE_ESTIMATOR_NOMINAL_SAMPLE_RATE_SEC = 0.02;

    public SwerveDrivetrainModel(SwerveModule[] realModules, Gyroscope gyro, Field2d field) {
        assert (RobotBase.isSimulation());

        this.realModules = realModules;
        this.gyro = gyro;
        this.field = field;

        simModules.add(createSimModule(realModules[0], "FL"));
        simModules.add(createSimModule(realModules[1], "FR"));
        simModules.add(createSimModule(realModules[2], "BL"));
        simModules.add(createSimModule(realModules[3], "BR"));

        swerveSim = new QuadSwerveSim(TRACKWIDTH,
                WHEELBASE,
                MASS_KG,
                MOI_KGM2,
                simModules);

        lastSimulationStepTime = Timer.getFPGATimestamp();
        endActualPose = new Pose2d();

        // Trustworthiness of the internal model of how motors should be moving
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        // Trustworthiness of gyro in radians of standard deviation.
        var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

        // Trustworthiness of the vision system
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

        poseEstimator = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(gyro.getAngle().toDegrees()),
                endActualPose, WPI_DRIVEKINEMATICS, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
                POSE_ESTIMATOR_NOMINAL_SAMPLE_RATE_SEC);
    }

    private SwerveModuleSim createSimModule(SwerveModule module, String namePrefix) {
        ModuleConfiguration config = GEAR_RATIO.getConfiguration();
        return new SwerveModuleSim(DCMotor.getFalcon500(1),
                DCMotor.getFalcon500(1),
                config.getWheelDiameter() / 2,
                1 / config.getSteerReduction(),
                1 / config.getDriveReduction(),
                1.0,
                1 / config.getDriveReduction(),
                1.1,
                0.8,
                MASS_KG * 9.81 / QuadSwerveSim.NUM_MODULES,
                0.01,
                namePrefix);
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     *
     * @param pose
     */
    public void modelReset(Pose2d pose) {
        field.setRobotPose(pose);
        swerveSim.modelReset(pose);
    }

    /**
     * Advance the simulation forward by one step
     *
     * @param isDisabled
     *            Whether the robot is disabled or not
     * @param batteryVoltage
     *            Current battery voltage
     */
    public void update(boolean isDisabled, double batteryVoltage) {

        // Check if the user moved the robot with the Field2D
        // widget, and reset the model if so.
        Pose2d startPose = field.getRobotPose();
        Transform2d deltaPose = startPose.minus(endActualPose);
        if (deltaPose.getRotation().getDegrees() > 0.01 || deltaPose.getTranslation().getNorm() > 0.01) {
            modelReset(startPose);
        }

        double currentTime = Timer.getFPGATimestamp();

        // Calculate and update input voltages to each motor.
        if (isDisabled) {
            for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
                simModules.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            StringBuilder sb = new StringBuilder(currentTime + ": ");
            for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
                double steerAngle = ((WPI_TalonFX)realModules[idx].getSteerMotor()).get// realModules[idx].getReferenceAngle(); //getSteerAngle();
                double wheelVolts = ((WPI_TalonFX) realModules[idx].getDriveMotor()).getMotorOutputPercent()
                        * BATTERY_VOLTAGE;
                sb.append("(" + wheelVolts + "," + steerAngle + ") ");
                simModules.get(idx).setInputVoltageAndAngle(wheelVolts, steerAngle);
            }
            System.out.println(sb);
        }

        // Update the main drivetrain plant model
        swerveSim.update(currentTime - lastSimulationStepTime);
        lastSimulationStepTime = currentTime;
        endActualPose = swerveSim.getCurPose();

        // Update each encoder
        for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
            double steerMotorPosTicks = ENCODER_TICKS_PER_REVOLUTION * simModules.get(idx).getAzimuthMotorPositionRev()
                    / GEAR_RATIO.getConfiguration().getSteerReduction();
            double wheelPosTicks = ENCODER_TICKS_PER_REVOLUTION * simModules.get(idx).getWheelEncoderPositionRev()
                    / GEAR_RATIO.getConfiguration().getDriveReduction();

            double steerVelocity = simModules.get(idx).getAzimuthMotorVelocityRPM() * RPM_TO_VELOCITY;
            double wheelVelocity = simModules.get(idx).getWheelEncoderVelocityRPM() * RPM_TO_VELOCITY;

            // Absoluate encoder not on sim bot, so no need to set it
            // realModules[idx].getAbsoluteEncoder().setAbsoluteEncoder(azmthShaftPos, azmthShaftVel);
            ((WPI_TalonFX) realModules[idx].getSteerMotor()).getSimCollection()
                    .setIntegratedSensorRawPosition((int) steerMotorPosTicks);
            ((WPI_TalonFX) realModules[idx].getSteerMotor()).getSimCollection()
                    .setIntegratedSensorVelocity((int) steerVelocity);
            ((WPI_TalonFX) realModules[idx].getDriveMotor()).getSimCollection()
                    .setIntegratedSensorRawPosition((int) wheelPosTicks);
            ((WPI_TalonFX) realModules[idx].getDriveMotor()).getSimCollection()
                    .setIntegratedSensorVelocity((int) wheelVelocity);
        }

        // Update associated devices based on drivetrain motion
        field.setRobotPose(endActualPose);
        gyro.setSimAngle(Rotation2.fromDegrees(swerveSim.getCurPose().getRotation().getDegrees()));

        /*
         * // Based on gyro and measured module speeds and positions, estimate where our
         * // robot should have moved to.
         * Pose2d previousEstimatedPose = endEstimatedPose;
         * if (states != null) {
         * endEstimatedPose = poseEstimator.update(getGyroscopeRotation(), states[0], states[1], states[2],
         * states[3]);
         *
         * // Calculate a "speedometer" velocity in ft/sec
         * Transform2d chngPose = new Transform2d(prevEstPose, endEstimatedPose);
         * curSpeed = Units.metersToFeet(chngPose.getTranslation().getNorm()) /
         * SimConstants.CTRLS_SAMPLE_RATE_SEC;
         * }
         */
    }
}
