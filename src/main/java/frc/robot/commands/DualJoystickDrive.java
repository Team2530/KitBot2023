/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.libraries.Deadzone;

public class DualJoystickDrive extends CommandBase {
    /**
     * Creates a new SingleJoystickDrive.
     */
    DriveTrain m_drivetrain;
    Joystick leftStick;
    Joystick rightStick;

    public DualJoystickDrive(DriveTrain m_drivetrain, Joystick leftStick, Joystick rightStick) {
        this.m_drivetrain = m_drivetrain;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightStickY = 0.5 * Deadzone.deadZone(rightStick.getY(), 0.05);
        double leftStickZ = -0.5 * Deadzone.deadZone(leftStick.getZ(), 0.05);
        m_drivetrain.moveLeftWheel(rightStickY + leftStickZ);
        m_drivetrain.moveRightWheel(rightStickY - leftStickZ);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
