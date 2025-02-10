package frc.robot.util;

import java.util.ArrayList;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Container {
    public static class  Multi_SparkFlex {
        private IdleMode m_lastIdleMode;
        private ArrayList<SparkFlex> m_sparkFlexArray;
        public Multi_SparkFlex(SparkFlex...sparkFlexArray){
            m_sparkFlexArray = new ArrayList<>();
            for(SparkFlex sparkFlex : sparkFlexArray){
                m_sparkFlexArray.add(sparkFlex);
            }
        }
        public SparkFlex getLeader(){
            return m_sparkFlexArray.get(0);//Leader Must be The First[0] in the Array and then the followers
        }
        // public void setLeader(){
        //     var config = new SparkFlexConfig();
        //     for(SparkFlex sparkFlex : m_sparkFlexArray){
        //         config.fol
        //     }
        // }
        // public void setIdleMode(IdleMode idleMode){
        //     if(idleMode == m_lastIdleMode){return;} 
        //     var config = new SparkFlexConfig();
        //     for(SparkFlex sparkFlex : m_sparkFlexArray){
        //         config.idleMode(idleMode);
        //     }
        //    }

        
    }
    
}
