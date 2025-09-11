#include "sonia_bt_runner/navigation/MainAlignment.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation{

    class MainAlignment{

        // ----------------- Helper implementations -----------------

        float bboxCenterX(const AiDetection& d)
        {
            // Average all four corners (robust to minor skew)
            return 0.25f * (d.top_left_x + d.top_right_x + d.bottom_left_x + d.bottom_right_x);
        }

        float bboxCenterY(const AiDetection& d)
        {
            // Average all four corners (robust to minor skew)
            return 0.25f * (d.top_left_y + d.top_right_y + d.bottom_left_y + d.bottom_right_y);
        }

        float GetCenterImageAngleX(const AiDetection& det)
        {
            // We get the x distance between the left side of the image and the center of the object.
            float u_px = bboxCenterX(det);
            std::cout <<"center x pixel : "<< u_px << std::endl;
            std::cout <<"center dist x pixel norm : "<< (u_px - CameraInfoZedMiniHD720::cx)  << std::endl;

            // We get the adjacent value of the rectangle triangle with the camera angle (fovx) and the adjacent value width.
            float fx=(CameraInfoZedMiniHD720::width/2)/(tan(CameraInfoZedMiniHD720::fovx*M_PI/180/2));

            // We get the angle between the normal of the cam and the direction to the detected object.  
            return atan((u_px - CameraInfoZedMiniHD720::cx) /(fx));
        }

        float GetCenterImageAngleY(const AiDetection& det)
        {
            // We get the y distance between the top and the center of the detected object
            float u_py = bboxCenterY(det);
            std::cout <<"center x pixel : "<< u_py << std::endl;
            std::cout <<"center dist x pixel norm : "<< (u_py - CameraInfoZedMiniHD720::cx)  << std::endl;

            // We get the adjacent value of the rectangle triangle with the camera angle (fovx) and the adjacent value width.
            float fx=(CameraInfoZedMiniHD720::width/2)/(tan(CameraInfoZedMiniHD720::fovx*M_PI/180/2));

            // We get the angle between the normal of the cam and the direction to the detected object.  
            return atan((u_py - CameraInfoZedMiniHD720::cx) /(fx));
        }
        
        Trajectory ComputeTrajectory(const AlignResult res)
        {
            // int i;
            // bool i_initialized=false;
            // if (i_initialized==false)
            //     i=0;

            // Optional temporal smoothing (IIR) for stability
            // if (do_smooth)
            // {
            //     if (std::isfinite(prev_bearing_)){
            //         // std::cout <<"bearing rad before smoothing: "<< i << "step"<< res.bearing_rad << std::endl;
            //         res.bearing_rad = alpha * res.bearing_rad + (1.0f - alpha) * prev_bearing_;
            //         // std::cout <<"bearing rad after smoothing: "<< i << "step"<< res.bearing_rad << std::endl;
            //         i++;
            //     }
            //     if (std::isfinite(prev_normx_)) res.norm_x = alpha * res.norm_x + (1.0f - alpha) * prev_normx_;
            //     if (res.has_metric && std::isfinite(prev_lateral_)){
            //         std::cout <<"lateral_m before smoothing: "<< i << "step"<< res.lateral_m << std::endl;
            //         res.lateral_m = alpha * res.lateral_m + (1.0f - alpha) * prev_lateral_;
            //         std::cout <<"lateral_m after smoothing: "<< i << "step"<< res.lateral_m << std::endl;
            // }

            // }
            // std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;


            // Publish outputs
            // setOutput("bearing_rad", res.bearing_rad);
            // setOutput("norm_x", res.norm_x);
            // std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

            // setOutput("has_metric", res.has_metric ? 1 : 0);
            // if (mode){
            //     if (res.lateral_m<=error_positionY && res.lateral_m>=-error_positionY){
            //         res.in_interval=true;
            //     }
            //     // if(prev_bearing_ == res.bearing_rad)
            //     //     res.in_interval=true;

            // }
            // else {
            //     if (res.bearing_rad<=error_orientationY && res.bearing_rad>=-error_orientationY){
            //         res.in_interval=true;
            //     }
                
            //     // if(prev_lateral_ = res.lateral_m)
            //     //     res.in_interval=true;

            // }
            // if (res.has_metric)
            // {
            //     setOutput("lateral_m", res.lateral_m);
            //     prev_lateral_ = res.lateral_m;
            // }

            // prev_bearing_ = res.bearing_rad;
            // std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

            // prev_normx_ = res.norm_x;

            // std::cout <<"res has metric : "<< res.has_metric << std::endl;
            // if (res.has_metric){
                TrajectoryPose t;
                // t.positionY = 0.0;
                // if(mode==1 or camera==false){
                //     t.positionY = res.lateral_m;
                //     std::cout <<"translation y : "<< res.lateral_m << std::endl;
                //     }
                // setOutput("positionY", positionY);
                t.positionX = 0.0;
                // if (camera==false){
                //     t.positionY = res_y.lateral_m;
                //     std::cout <<"translation x : "<< t.positionY << std::endl;
                // }
                // setOutput("positionX", positionX);
                t.positionZ = 0.0;
                // setOutput("positionZ", positionZ);
                t.orientationX = 0.0;
                // setOutput("orientationX", orientationX);
                t.orientationY = 0;
                // setOutput("orientationY", orientationY);
                t.orientationZ = 0;
                // std::cout << "position en x: "<<t.positionX;
                // if (!mode){
                //     t.orientationZ = res.bearing_rad / M_PI * 180;
                //     std::cout <<"orientation z : "<< res.bearing_rad << std::endl;

                // }
                // int frame = 1;
                t.frame = 1;
                // setOutput("frame", frame);
                // int speed = 2;
                // setOutput("speed", speed);
                t.speed = 2;
                t.precision = 0;
                t.long_rotation=false;
                // setOutput("precision", precision);
                // bool long_rotation = false;
                // setOutput("longRotation", long_rotation);
                std::cout << "position en x: "<<t.positionX<< std::endl;
                std::cout<< "position en y: "<<t.positionY<< std::endl;
                std::cout << "position en z: "<<t.positionZ<< std::endl;
                std::cout << "Orientation en x: "<<t.orientationX<< std::endl;
                std::cout << "Orientation en y: "<<t.orientationY<< std::endl;
                std::cout << "Orientation en z: "<<t.orientationZ<< std::endl;
                std::cout << "frame: "<<t.frame<< std::endl;
                std::cout << "speed: "<<t.speed<< std::endl;
                std::cout << "precision: "<<t.precision<< std::endl;
                std::cout << "long rotation: "<<t.long_rotation<< std::endl;
                // Trajectory traj = getInput<Trajectory>("traj").value();
                // traj.trajectory.push_back(t);

            // }
            // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
            // return res.has_metric ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
            return t;
        }

    }

}

