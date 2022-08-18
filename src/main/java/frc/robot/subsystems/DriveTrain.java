/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around, such as directly setting motor power,
 * handling the control mode, and auto-turning.
 */
public class DriveTrain extends SubsystemBase {
    // -------------------- Motor Controllers -------------------- \\
    private final WPI_VictorSPX FLMC = new WPI_VictorSPX(4);
    private final WPI_VictorSPX BLMC = new WPI_VictorSPX(3);
    private final WPI_VictorSPX FRMC = new WPI_VictorSPX(5);
    private final WPI_VictorSPX BRMC = new WPI_VictorSPX(1);

    public DriveTrain() {
        FLMC.setInverted(true);
        BLMC.setInverted(true);
        FRMC.setInverted(false);
        BRMC.setInverted(false);
    }

    public void moveLeftWheel(double speed) {
        FLMC.set(speed);
        BLMC.set(speed);
    }

    public void moveRightWheel(double speed) {
        FRMC.set(speed);
        BRMC.set(speed);
    }

    /**
     * Initializes a drive mode where only one joystick controls the drive motors.
     * 
     * @param x The joystick's forward/backward tilt. Any value from -1.0 to 1.0.
     * @param z The joystick's vertical "twist". Any value from -1.0 to 1.0.
     */
    public void singleJoystickDrive(double x, double z) {

    }
}
