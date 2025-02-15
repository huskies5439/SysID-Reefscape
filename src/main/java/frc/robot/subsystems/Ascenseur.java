// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Ascenseur extends SubsystemBase {
  
   // moteur + config
  private SparkFlex moteur1 = new SparkFlex(9, MotorType.kBrushless);
  private SparkFlex moteur2 = new SparkFlex(10, MotorType.kBrushless);

  private SparkFlexConfig moteurConfig = new SparkFlexConfig();
  private double conversionVortex;

  // Encodeur
  private Encoder encoder = new Encoder(2, 3);

   // capteur
  private final DigitalInput limitSwitchGauche = new DigitalInput(0);
  private final DigitalInput limitSwitchDroite = new DigitalInput(1);

  ////SYSID
  private final MutVoltage voltageApplique = Volts.mutable(0);
  private final MutDistance hauteur = Meters.mutable(0);
  private final MutLinearVelocity vitesseLineaire = MetersPerSecond.mutable(0);


  ///////////PRÉSENTEMENT EN MODE VORTEX
  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              voltage -> setVoltage(voltage),//Il faut changer la méthode "setVoltage" pour qu'elle prenne des Volts
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("Ascenseur Vortex")
                    .voltage(
                        voltageApplique.mut_replace(
                            moteur1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(hauteur.mut_replace(getPositionVortex(), Meters))
                    .linearVelocity(
                        vitesseLineaire.mut_replace(getVitesseVortex(), MetersPerSecond));

                /*log.motor("Ascenseur Externe")
                    .voltage(
                        voltageApplique.mut_replace(
                            moteur1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(hauteur.mut_replace(getPositionExterne(), Meters))
                    .linearVelocity(
                        vitesseLineaire.mut_replace(getVitesseExterne(), MetersPerSecond));*/
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));


  public Ascenseur() {
    // set parametres des configs
    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);

    // pignons 14 dents fait tourner 80 dents. Après, poulie 3/4 de pouce.
    conversionVortex = (14.0 / 80) * Units.inchesToMeters(0.75) * Math.PI;

    moteurConfig.encoder.positionConversionFactor(conversionVortex); // en mètre
    moteurConfig.encoder.velocityConversionFactor(conversionVortex / 60.0); // mètre par secondes

    // associe les configs aux moteurs
    moteur1.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    moteur2.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ///////À revoir quand ça va marcher dans l'autre code !
    encoder.setDistancePerPulse((Math.PI * 70.0 / 1000.0) / 360); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vitesse Vortex", getVitesseVortex());
    SmartDashboard.putNumber("Hauteur Vortex", getPositionVortex());

    SmartDashboard.putNumber("Vitesse Externe", getVitesseExterne()); 
    SmartDashboard.putNumber("Hauteur Externe", getPositionExterne());
   

    if (isLimitSwitch()) {
      resetEncoders();
    }

  }


  /////////////////// MOTEUR

  // Donne un voltage aux moteurs
  //Il faut modifier la méthode du moteur pour que ça soit des Volts
  public void setVoltage(Voltage voltage) {
    moteur1.setVoltage(voltage.in(Volts));
    moteur2.setVoltage(voltage.in(Volts));
  }
  
  ////////////////////////// ENCODEUR VORTEX

  // Retourne la position de l'encodeur VORTEX
  public double getPositionVortex() {
    return moteur1.getEncoder().getPosition();
  }

  // reset les encodeurs des vortex
  public void resetEncodersVortex() {
    moteur1.getEncoder().setPosition(0);
    moteur2.getEncoder().setPosition(0);
  }

  public double getVitesseVortex() {
    return moteur1.getEncoder().getVelocity();
  }


  //////////////////// Encodeur Externe

  public double getPositionExterne() {
    return encoder.getDistance();
  }

  public double getVitesseExterne() {
    return encoder.getRate();
  }

  public void resetEncodeurExterne() {
    encoder.reset();
  }

  public void resetEncoders() {
    resetEncodersVortex();
    resetEncodeurExterne();
  }


    //////////////////// Limit switch

    public boolean isLimitSwitch() {
      return !limitSwitchGauche.get() || !limitSwitchDroite.get();
  
    }


    /////// Commande SysID

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }


}


