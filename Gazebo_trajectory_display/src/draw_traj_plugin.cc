/************************************************************************
* @file draw_traj_plugin.cc
* @brief Draw trajectory of the robot in Gazebo
* Modefied from Unitree Robotics
*
* @author Luchuanzhao
* @version 1.0.0
* @date 2023.6.26
************************************************************************/

/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <gazebo/rendering/rendering.hh>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>

namespace gazebo
{
    class UnitreeDrawForcePlugin : public VisualPlugin
    {
        public:
        UnitreeDrawForcePlugin(){}
        ~UnitreeDrawForcePlugin(){
            this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            // scene = rendering::get_scene();
            this->visual = _parent; //parent就是xacro文件中的gazebo reference
            // this->visual = scene->GetVisual();
            this->visual_namespace = "visual/";

            this->topic_name = "/pf_state";

            this->rosnode = new ros::NodeHandle(this->visual_namespace);
            this->rosnode->param("/model/sim_cables", sim_cables_, false);

            XmlRpc::XmlRpcValue p,wp;
            this->rosnode->getParam("/model/points", p);
            this->rosnode->getParam("/Tra/waypoints", wp);

            Eigen::Vector3d t;
            for(int i = 0; i < p.size(); i++)
            {
                for(auto elem: p[i])
                {
                    t[0] = elem.second[0];
                    t[1] = elem.second[1];
                    t[2] = elem.second[2];
                    break;
                }
                tension_command_.push_back(t);
            }

            std::vector<std::vector<double>> waypoints(wp.size(),std::vector<double>(3,0));
            for(int i = 0; i < wp.size(); i++)
            {
                waypoints[i][0] = wp[i][0];
                waypoints[i][1] = wp[i][1];
                waypoints[i][2] = wp[i][2];
            }

            std::cout<<"-------------------------------"<<wp.size()<<std::endl;

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->waypoint = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->waypoint->setMaterial("Gazebo/Red");
            this->waypoint->setVisibilityFlags(GZ_VISIBILITY_GUI);
            //way point 
            for(int i = 0; i<wp.size();i++)
                this->waypoint->AddPoint(ignition::math::Vector3d(waypoints[i][0], waypoints[i][1], waypoints[i][2]), 
                                           ignition::math::Color(0, 0, 1, 1.0));

            //cables
            if(!sim_cables_){
                this->cable0 = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
                this->cable1 = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
                this->cable2 = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
                this->cable3 = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
                this->cable0->AddPoint(ignition::math::Vector3d(tension_command_[0][0], 
                                                                tension_command_[0][1], 
                                                                tension_command_[0][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable0->AddPoint(ignition::math::Vector3d(tension_command_[0][0], 
                                                                tension_command_[0][1], 
                                                                tension_command_[0][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable1->AddPoint(ignition::math::Vector3d(tension_command_[1][0], 
                                                                tension_command_[1][1], 
                                                                tension_command_[1][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable1->AddPoint(ignition::math::Vector3d(tension_command_[1][0], 
                                                                tension_command_[1][1], 
                                                                tension_command_[1][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable2->AddPoint(ignition::math::Vector3d(tension_command_[2][0], 
                                                                tension_command_[2][1], 
                                                                tension_command_[2][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable2->AddPoint(ignition::math::Vector3d(tension_command_[2][0], 
                                                                tension_command_[2][1], 
                                                                tension_command_[2][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable3->AddPoint(ignition::math::Vector3d(tension_command_[3][0], 
                                                                tension_command_[3][1], 
                                                                tension_command_[3][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
                this->cable3->AddPoint(ignition::math::Vector3d(tension_command_[3][0], 
                                                                tension_command_[3][1], 
                                                                tension_command_[3][2]), 
                                            ignition::math::Color(0, 0, 0, 1.0));
            }
            
            // this->force_sub = this->rosnode->subscribe(this->topic_name+"/"+"the_force", 30, &UnitreeDrawForcePlugin::GetForceCallback, this);
            this->force_sub = this->rosnode->subscribe(this->topic_name, 30, &UnitreeDrawForcePlugin::GetForceCallback, this);
            this->update_connection = event::Events::ConnectPreRender(std::bind(&UnitreeDrawForcePlugin::OnUpdate, this));
            ros::spinOnce();
        }

        virtual void OnUpdate()
        {
            // this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
            this->line->AddPoint(ignition::math::Vector3d(Fx, Fy, Fz), ignition::math::Color(0, 0, 1, 1.0));
            if(!sim_cables_){
                this->cable0->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
                this->cable1->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
                this->cable2->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
                this->cable3->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
            }
            // ROS_INFO("Fx= %f, Fy= %f, Fz= %f", Fx, Fy, Fz);
        }

        void GetForceCallback(const gazebo_msgs::LinkState &pf_state_)
        {
            // this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            Fx = pf_state_.pose.position.x;
            Fy = pf_state_.pose.position.y;
            Fz = pf_state_.pose.position.z;

        }

        private:
            rendering::DynamicLines *line = nullptr;
            rendering::DynamicLines *waypoint = nullptr;
            rendering::DynamicLines *cable0 = nullptr;
            rendering::DynamicLines *cable1 = nullptr;
            rendering::DynamicLines *cable2 = nullptr;
            rendering::DynamicLines *cable3 = nullptr;
            ros::NodeHandle* rosnode;
            std::string topic_name;
            rendering::VisualPtr visual;
            rendering::ScenePtr scene;
            std::string visual_namespace;
            ros::Subscriber force_sub;
            double Fx = 0, Fy = 0, Fz = 0;
            event::ConnectionPtr update_connection;
            bool sim_cables_;
            std::vector<Eigen::Vector3d> tension_command_;
    };
    GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}
