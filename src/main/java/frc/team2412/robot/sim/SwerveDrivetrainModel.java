package frc.team2412.robot.sim;

import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import org.frcteam2910.common.drivers.Gyroscope;

import java.util.ArrayList;

import static frc.team2412.robot.Hardware.*;

public class SwerveDrivetrainModel {
    final QuadSwerveSim swerveSim;
    final SwerveModule[] realModules;
    ArrayList<SwerveModuleSim> simModules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    final Gyroscope gyro;

    public SwerveDrivetrainModel(SwerveModule[] realModules, Gyroscope gyro) {
        assert (RobotBase.isSimulation());

        this.gyro = gyro;
        this.realModules = realModules;

        simModules.add(createSimModule(realModules[0], "FL"));
        simModules.add(createSimModule(realModules[1], "FR"));
        simModules.add(createSimModule(realModules[2], "BL"));
        simModules.add(createSimModule(realModules[3], "BR"));

        swerveSim = new QuadSwerveSim(TRACKWIDTH,
                WHEELBASE,
                MASS_KG,
                MOI_KGM2,
                simModules);
    }

    private SwerveModuleSim createSimModule(SwerveModule module, String namePrefix) {
        ModuleConfiguration config = GEAR_RATIO.getConfiguration();
        return new SwerveModuleSim((DCMotor) module.getSteerMotor(),
                (DCMotor) module.getDriveMotor(),
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
}
