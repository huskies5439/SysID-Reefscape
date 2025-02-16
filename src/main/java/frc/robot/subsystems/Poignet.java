// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Poignet extends SubsystemBase {

  // moteur + config
  private SparkFlex moteur = new SparkFlex(14, MotorType.kBrushless);
  private SparkFlexConfig moteurConfig = new SparkFlexConfig();
  private double conversionEncodeur;


  private final MutVoltage voltageApplique = Volts.mutable(0);
  private final MutAngle angle = Radians.mutable(0);
  private final MutAngularVelocity vitesseAngulaire = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(2), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              voltage -> setVoltage(voltage),//Il faut changer la méthode "setVoltage" pour qu'elle prenne des Volts
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("Poignet")
                    .voltage(
                        voltageApplique.mut_replace(
                            moteur.getBusVoltage(), Volts))
                    .angularPosition(angle.mut_replace(getAngle(), Rotations))
                    .angularVelocity(
                        vitesseAngulaire.mut_replace(getVitesse(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));





  public Poignet() {
     // set parametres des configs
    moteurConfig.inverted(true);
    moteurConfig.idleMode(IdleMode.kBrake);

    // gearbox 4 pour 1 , 9 pour 1
    // 360 degrés par tour
    conversionEncodeur = (1 / 4.0) * (1 / 9.0) * 360;

    moteurConfig.encoder.positionConversionFactor(conversionEncodeur);
    moteurConfig.encoder.velocityConversionFactor(conversionEncodeur / 60.0);

    moteur.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetEncodeurStartUp();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle Poignet", getAngle());
    SmartDashboard.putNumber("Vitesse Poignet", getVitesse());
  }

///////////////// MOTEUR
public void setVoltage(Voltage voltage) {
  moteur.setVoltage(voltage.in(Volts));
}

  ///////////////// ENCODEUR

  public double getAngle() {
    return moteur.getEncoder().getPosition();
  }

  public double getVitesse() {
    return moteur.getEncoder().getVelocity();
  }

  public void resetEncodeurStartUp(){
    moteur.getEncoder().setPosition(90); 
  }

  /////// Commande SysID

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }




}
