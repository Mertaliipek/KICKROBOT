// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final Victor victorLeft1 = new Victor(0);
  private final Victor victorLeft2 = new Victor(1);
  private final Victor victorRight1 = new Victor(2);
  private final Victor victorRight2 = new Victor(3);
  private final MotorControllerGroup left_m = new MotorControllerGroup(victorLeft1, victorLeft2);
  private final MotorControllerGroup right_m = new MotorControllerGroup(victorRight1, victorRight2);
  private final DifferentialDrive m_drive = new DifferentialDrive(left_m,right_m);
  // navx gyro mor olan
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gyro.reset();
    

    }
  

  public void setMotors(double leftSpeed,double rightSpeed) { 
    left_m.set(leftSpeed);
    right_m.set(-rightSpeed);
  }

  public void letsKickrobot(double rotateSpeed,double rotateSpeedSlow) {
// 3 dereceden kucukse sag tekeri hizlandır diğerini yavaşlat
if(Math.abs(gyro.getAngle()) <=3) {
  left_m.set(0.6);
  right_m.set(-0.4);

} else if(Math.abs(gyro.getAngle()) <=10) {       // derece degidikce hız da degişir
  if (gyro.getAngle() > 0) {
    left_m.set(0.5);
    right_m.set(-1);
   } else if (gyro.getAngle() < 0) {
    left_m.set(1);
    right_m.set(-0.5);
}
}
else
if (gyro.getAngle() > 0) {      // eger 0 dan buyuk ise loop a donuyor ve loop 0 dan buyuk olana kadar devam ediyor buyuk oldugun da ust tarafa gidiyor
  while (gyro.getAngle() > 10) {
   left_m.set(-rotateSpeed);
   right_m.set(-rotateSpeed);
  }
 while (gyro.getAngle() > 0) {
  left_m.set(-rotateSpeedSlow);
  right_m.set(-rotateSpeedSlow);
 }
 while (gyro.getAngle() < 0) {
  left_m.set(rotateSpeedSlow);
  right_m.set(rotateSpeedSlow);
 }
} else{
 while (gyro.getAngle() < -10) {
  left_m.set(rotateSpeed);
  right_m.set(rotateSpeed);
 }
 while (gyro.getAngle() > 0) {
  left_m.set(rotateSpeedSlow);
  right_m.set(rotateSpeedSlow);
 }
 while (gyro.getAngle() < 0) {
  left_m.set(-rotateSpeedSlow);
  right_m.set(-rotateSpeedSlow);
 }
}


  }




  @Override
  public void simulationPeriodic() {
    // debug
    SmartDashboard.putNumber("Gyro value", gyro.getAngle());
    SmartDashboard.putBoolean("Gyro connected", gyro.isConnected());
    // This method will be called once per scheduler run during simulation
    
    
  }
}
