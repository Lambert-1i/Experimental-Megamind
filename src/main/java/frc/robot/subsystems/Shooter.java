// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.units.VoltageUnit;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;




public class Shooter extends SubsystemBase {
    
    //Initalizes the motors
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;

    //sets up velocity PID for slot 0
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Calculates the max Revolutions Per Second
    private final double maxRPM = 5500;


    BangBangController controller = new BangBangController();

    InterpolatingDoubleTreeMap shooterRPS;

    /*Notes:
    10%: 550
    20%  1100
    30%  1650
    40%  2200
    50%  2750
    60%  3300
    70%  3850
    80%  4400
    90%  4950
    100% 5500
    */

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS,Constants.ShooterConstants.kV,Constants.ShooterConstants.kA);

    Double testRPS;

    SysIdRoutine routine;
    public Shooter() {

        //m_shooterFeedback.setTolerance(Constants.ShooterConstants.kShooterToleranceRPS);
        
        //Sets settings for the Lower Flywheel
        
        TalonFXConfiguration globalConfigs = new TalonFXConfiguration();
        globalConfigs.CurrentLimits.withStatorCurrentLimit(40);
        globalConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        //setups the PID value for the intake
      
        globalConfigs.Slot0.kS = Constants.ShooterConstants.s_Value;
        globalConfigs.Slot0.kV = Constants.ShooterConstants.v_Value; // A velocity target of 1 rps results in 0.12 V output
        globalConfigs.Slot0.kP = Constants.ShooterConstants.p_Value;

        globalConfigs.Voltage.withPeakForwardVoltage(8)
        .withPeakReverseVoltage(-8);
        
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        
        lowerFlyMotor.getConfigurator().apply(globalConfigs);

        //Sets settings for Upper Flywheel
        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);

        upperFlyMotor.getConfigurator().apply(globalConfigs);

        upperFlyMotor.setControl(new Follower(lowerFlyMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        lowerFlyMotor.setNeutralMode(NeutralModeValue.Coast);
        upperFlyMotor.setNeutralMode(NeutralModeValue.Coast);


        /* 
          Context to Interpolating Double Tree Map:
          
            *Creates a plot of data points
            *Uses data points to estimate value based off key
        */

        /* Old shooter table values
        shooterRPS = new InterpolatingDoubleTreeMap();
        //Key: distance 
        //Value: velocity of shooter

        //Will change
        // new Numbers as of 3/16/2026
        shooterRPS.put(1.9266899545847438,25.42); //Front of tower      
        shooterRPS.put(1.9823946179552907,25.73); //Mid right
        shooterRPS.put(2.661263186996523,28.6); //Corner
        shooterRPS.put(2.2590145219495,28.4); //Near Left Trench
        shooterRPS.put(2.157440140835241,27.5);//Near Right Trench

        //Attempt #2
        NewshooterRPS = new InterpolatingDoubleTreeMap();
        
        NewshooterRPS.put(2.5055061242497207,34.0);
        NewshooterRPS.put(1.4178527091316189,27.0);
        NewshooterRPS.put(1.6360094234791194,29.0);
        NewshooterRPS.put(3.2319892011747924,36.0);
        NewshooterRPS.put(1.1445679370483035,25.73);
        //test later

        NewershooterRPS = new InterpolatingDoubleTreeMap();
        //With new silicon wrapping
        NewershooterRPS.put(1.926470572923554,25.99); //Auto Position
        NewershooterRPS.put(2.4573944066702516,27.3); // next to tower
        NewershooterRPS.put(3.026646586664895,31.04); // front of depot
        NewershooterRPS.put(3.428045861139117,34.01); // Corner (rough max)

        //NewershooterRPS.put(1.1445679370483035,25.73); //
      */


      shooterRPS = new InterpolatingDoubleTreeMap();
        //Key: distance in meters
        //Value: velocity of shooter

      shooterRPS.put(2.084979071669848,24.55); //close (tune again)
      shooterRPS.put(3.0008923066606354,27.02); //Standard
      shooterRPS.put(4.434153557029914,34.01); //Far





        //After Week 3 feature
        m_sysIdRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
                  Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("Shooter state", state.toString())
        ),
          new SysIdRoutine.Mechanism(
          (volts) -> lowerFlyMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );

      SmartDashboard.putNumber("Shooter Test RPS", 27.02);
      //35
        
    }

  
    /*//sets up a command for the speed of the motors
    public Command setSpeed(double percent){
      return run(
          () -> {
            setLowerFly(percent);
          }
      );
    }
    //Made seperate because it doesn't use setControl
    public Command stopMotors(){
      return run(
          () -> {
            setLowerFly(0);
          }
      );
    }

    //Sets the speed of the Lower Flywheel
    public void setLowerFly(double OutputPercent)
    {
      OutputPercent /= 100.0;
      lowerFlyMotor.set(OutputPercent);
      SmartDashboard.putNumber("Flywheel speed", OutputPercent);
      //lowerFlyMotor.setControl(m_request.withVelocity(RPS*OutputPercent));
    }*/
    

 
    public Command PIDrunMotors(double FlyRPS){
      return run(
          () -> {
            setTargetRPS(FlyRPS);
          }
      );
    }
    public Command stopMotors(){
      return run(
          () -> {
            lowerFlyMotor.set(0);
          }
      );
    }

    public Command PIDtreeRunMotors(double distance){
      return run(
          () -> {
            setTargetRPS(shooterRPS.get(distance));
          }
      );
    }
    /*public Command PIDtestRunMotors(){
      return run(
          () -> {
            setTargetRPS(testRPS);
          }
      );
    }*/
    //Just Bang Bang
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(testRPS);
        }
      );
    }

    public void BBrps(double RPS){
        lowerFlyMotor.setVoltage(controller.calculate(lowerFlyMotor.getVelocity().getValueAsDouble(), RPS)*12 + 0.9*feedforward.calculate(RPS));
    }

    public void setTargetRPS(double RPS){
      SmartDashboard.putNumber("RPS", lowerFlyMotor.getVelocity().getValueAsDouble());
      lowerFlyMotor.setControl(m_request.withVelocity(RPS));
    }
    //After week 3 feature
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }

    //Continuously runs
   @Override
   public void periodic() {
    testRPS = SmartDashboard.getNumber("Shooter Test RPS", 27.02);
    SmartDashboard.putNumber("Shooter RPS", lowerFlyMotor.getVelocity().getValueAsDouble());
  }

}
