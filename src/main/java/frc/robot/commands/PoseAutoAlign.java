package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PoseAutoAlign extends Command {

    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.FieldCentric robotDrive;

    public double targetRotation;

    public double AngleError;

    private static final Translation2d shooterOffset = new Translation2d(0, 0.184);


    public PoseAutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);

    }

    // runs
    public void execute() {
        
        Pose2d pose = DRIVETRAIN.getPose();

        Translation2d shooterPos = pose.getTranslation().plus(
            shooterOffset.rotateBy(pose.getRotation())
        );

        Translation2d toHub = DRIVETRAIN.setHub().minus(shooterPos);
        targetRotation = toHub.getAngle().getRadians();

        // Error between desired heading and current robot heading
        AngleError = MathUtil.angleModulus(
            targetRotation - pose.getRotation().getRadians()
        );

        DRIVETRAIN.setControl(
            robotDrive.withVelocityX(0)
                      .withVelocityY(0)
                      .withRotationalRate(AngleError)
        );      
    }

    // sets boolean for its done to stop the comand
    @Override
    public boolean isFinished() {
        return Math.abs(AngleError) < Math.toRadians(2);
    }

    @Override
    public void end(boolean interrupted){
        DRIVETRAIN.setControl(
            new SwerveRequest.Idle() 
        );  
    }
}