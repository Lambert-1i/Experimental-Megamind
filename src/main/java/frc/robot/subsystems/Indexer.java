// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volt;


public class Indexer extends SubsystemBase {

    //Initializes the Indexer Motor
    TalonFX indexerMotor;

    private BangBangController controller = new BangBangController();

    Double testRPS;

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.IndexerConstants.kS,Constants.IndexerConstants.kV,Constants.IndexerConstants.kA);


  

    public Indexer() {

        //Sets up the settings for the Indexer Motor
        indexerMotor = new TalonFX(Constants.IndexerConstants.indexMotor);
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(65);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        indexerMotor.getConfigurator().apply(indexerConfigs);

        indexerMotor.setNeutralMode(NeutralModeValue.Coast); 

        SmartDashboard.putNumber("Index Test RPS", 76);

        //After Week 3 feature
        m_sysIdRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
                  Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("Index state", state.toString())
        ),
          new SysIdRoutine.Mechanism(
          (volts) -> indexerMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }


    //Sets the speed for the Indexer Motor
    public Command setSpeed(double OutputPercent){
        return run(
            () -> {
                setIndexer(OutputPercent);
            }
        );
    }
    //Just Bang Bang
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(testRPS);
        }
      );
    } 

    public void BBrps(double RPS){
        indexerMotor.setVoltage(controller.calculate(indexerMotor.getVelocity().getValueAsDouble(), RPS)*12 + 0.9*feedforward.calculate(RPS));
    }

    //Sets the speed of the Indexer Motor
    public void setIndexer(double OutputPercent)
    {
        OutputPercent /= 100.0;
        indexerMotor.set(OutputPercent);
    }

    public Command unjam(){
        return setSpeed(90).withTimeout(0.01) //90
            .andThen(Commands.waitSeconds(1.25))
            .andThen(setSpeed(-90).withTimeout(0.01))
            .andThen(Commands.waitSeconds(0.25))
            ;
    }
     //Continuously runs
    @Override
    public void periodic() {
      testRPS = SmartDashboard.getNumber("Index Test RPS", 76);
      SmartDashboard.putNumber("Index RPS", indexerMotor.getVelocity().getValueAsDouble());
    } 
}
