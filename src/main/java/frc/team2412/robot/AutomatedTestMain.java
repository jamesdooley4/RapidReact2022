// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to change
 * the parameter class to the startRobot call.
 */
public final class AutomatedTestMain {
    private AutomatedTestMain() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        RobotBase.startRobot(new Supplier<RobotBase>() {
            @Override
            public RobotBase get() {
                return Robot.getInstance(Robot.RobotType.AUTOMATED_TEST);
            }
        });
    }
}