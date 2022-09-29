package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class LimeLight extends CommandBase {
  /** Creates a new Limelight. */

  static NetworkTable table;
  /** X offset from target */
  static NetworkTableEntry tx = table.getEntry("tx");
  static NetworkTableEntry ty = table.getEntry("ty");
  static NetworkTableEntry ta = table.getEntry("ta");

  static double xoff;
  static double yoff;
  static double area;
  /** Any targets? */
  static double tv;

  static double turnRate;

  /** Gain for turning for the LimeLight */
  double limekP = 0.3;
  /** If the turn value is really low, we add to it so it still moves */
  double minCommand = 0.05;
  /**  Amount we are willing to compromise for in our distance */
  double disttolerance = 0.9;

  
  // not sure if this is right or not
  public int lightMode = 3;

  int cameraMode = 0;

  DriveTrain driveTrain;

  public LimeLight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    table = NetworkTableInstance.getDefault().getTable("limelight");
    this.driveTrain = driveTrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateValues();
    showValues();
    double distanceFromTarget = distanceToTarget();
    System.out.println(distanceFromTarget);
    SmartDashboard.putNumber("Lime Distance", distanceFromTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Updates the LimeLight's values
   */
  public static void updateValues() {
    xoff = tx.getDouble(0.0);
    yoff = ty.getDouble(0.0);
    // tv = table.getEntry("tv").getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  public void showValues(){
    SmartDashboard.putNumber("LimelightX", xoff);
    SmartDashboard.putNumber("LimelightY", yoff);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  public double toRadians(double input) {
    return input * (Math.PI / 180.0);
  }

  public double distanceToTarget() {
    double r = toRadians(Constants.limeAngle + yoff);
    return (Constants.goalHeight - Constants.limeHeight) / Math.tan(r);
  }

  public void changeMode() {
    if (lightMode == 3) {
      lightMode = 1;
    } else {
      lightMode = 3;
    }
  }

  /**
   * Assume that there is a valid target, we will turn to aim at it
   */
  public void aimAtTarget() {
    double error = -xoff;

    if (xoff < 1) {
      turnRate = limekP * error + minCommand;
    } else {
      turnRate = limekP * error - minCommand;
    }
    // Use this method to turn to robot at the speeds

  }

  /**
   * Backs up to a given distance based on maths
   * <p> Meant to be used called multiple times whilst preparing to shooting is occuring
   * @param dist Distance away from goal
   */
  public void backToDistance(double dist) {
    double currentdist = distanceToTarget();

  }
}