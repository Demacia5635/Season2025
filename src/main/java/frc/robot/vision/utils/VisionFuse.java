// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        return (x == 0 && y == 0) ? null : new Pose2d(x, y, new Rotation2d(angle));
    }

    public double getVisionTimestamp() {
        double timestamp = 0;
        for (Tag tag : tags) {
            if(tag.getPose() != null){
                timestamp += tag.getTimestamp() * normalizeConfidence(tag.getPoseEstemationConfidence());
            }
        }
        return timestamp;
    }

    private Integer getBestCamera(){
        Integer bestCamera = null;
        double highestConfidence = 0.0;
    
        for (int i = 0; i < tags.length; i++) {
            double currentConfidence = tags[i].getPoseEstemationConfidence();
    
            if (currentConfidence > highestConfidence && currentConfidence > 0.1) {
                highestConfidence = currentConfidence;
                bestCamera = i;
            }
        }
    
        return bestCamera;
    }

    public Rotation2d getVisionEstimatedAngle() {
        return getBestCamera() != null ? tags[getBestCamera()].getRobotAngle() : null;
    }

    public double getVisionConfidence() {
        return Math.max(getColectedConfidence(),1);
    }
}
