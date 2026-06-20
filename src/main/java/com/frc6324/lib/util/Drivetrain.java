package com.frc6324.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem {
  DrivePID getAutoDrivePID();

  void runRobotRelative(ChassisSpeeds speeds);

  void runFieldRelative(ChassisSpeeds speeds);

  void stop();
}
