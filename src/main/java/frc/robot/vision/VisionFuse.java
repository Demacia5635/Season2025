// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import javax.xml.crypto.KeySelector.Purpose;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import frc.robot.vision.subsystem.Tag;

/** Add your docs here. */
public class VisionFuse {
  
    private Tag[] tags = new Tag[4];

    public VisionFuse(Tag reefTag, Tag fiderTag, Tag bargeTag, Tag backTag) {

        this.tags[0] = reefTag;
        this.tags[1] = fiderTag;
        this.tags[2] = bargeTag;
        this.tags[3] = backTag;

    }

    private double getColectedConfidence(){
        double confidence = 0;
        for (Tag tag : tags) {
            if(tag.getPose() != null){
                confidence += tag.getPoseEstemationConfidence();
            }
        }
        return confidence;
    }

    private double normalizeConfidence(double confidence){
        return confidence * (1/getColectedConfidence()); 
    }

    public Pose2d getPoseEstemation(){
        double x = 0;
        double y = 0;
        double angle = 0;
        for (Tag tag : tags) {
            if (tag.getPose() == null) continue;
            x += tag.getPose().getX() * normalizeConfidence(tag.getPoseEstemationConfidence());
            y += tag.getPose().getY() * normalizeConfidence(tag.getPoseEstemationConfidence());
            angle += tag.getPose().getRotation().getRadians() * normalizeConfidence(tag.getPoseEstemationConfidence());
        }
        return new Pose2d(x, y, new Rotation2d(angle));
    }
}
