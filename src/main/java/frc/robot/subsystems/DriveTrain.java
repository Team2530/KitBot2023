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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around, such as directly setting motor power,
 * handling the control mode, and auto-turning.
 */
public class DriveTrain extends SubsystemBase {
    // -------------------- Motors -------------------- \\
    private WPI_VictorSPX leftMotor = new WPI_VictorSPX(0);
    private WPI_VictorSPX rightMotor = new WPI_VictorSPX(1);

    public DriveTrain() {
       
    }

    public void moveLeftMotor(double speed) {
        leftMotor.set(speed);
    }

    public void moveRightMotor(double speed) {
        rightMotor.set(speed);
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
