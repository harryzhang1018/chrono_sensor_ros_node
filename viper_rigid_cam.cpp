// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou Harry Zhang
// =============================================================================
//
// Demo to show Viper Rover operated on Rigid Terrain
// This Demo includes operation to spawn a Viper rover, control wheel speed, and
// control rover steering
//
// =============================================================================
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "chrono_models/robot/viper/Viper.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"


// ros2 packages dependency
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::viper;
using namespace chrono::geometry;
using namespace chrono::sensor; 
using namespace std::chrono_literals;



// Use custom material for the Viper Wheel
bool use_custom_mat = false;
// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;
// setting parameters for camera sensors
float cam_update_rate = 30.f;
float exposure_time = 0.02f;
int super_samples = 2;
// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;
// Camera's horizontal field of view
float cam_fov = 1.408f;

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Simulation time step
double time_step = 1e-3;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Sensor_Camera_Node");
    //rclcpp::Node node("Sensor_Camera");
    //rclcpp::Publisher::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::Image>("my_topic", 10);
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("sensor_image", 10);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector<>(0, 0, -0.5));
    ground->SetBodyFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);
    // Construct a Viper rover and the asociated driver
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));


    viper.Initialize(ChFrame<>(ChVector<>(-3, 1, 0.5), QUNIT));

    std::cout << "Viper total mass: " << viper.GetRoverMass() << std::endl;
    std::cout << "  chassis:        " << viper.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  upper arm:      " << viper.GetUpperArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  lower arm:      " << viper.GetLowerArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  upright:        " << viper.GetUpright(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:          " << viper.GetWheel(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;
    // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChMaterialSurface> rockSufaceMaterial = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    // Add pre-defined 20 rocks
    for (int i = 0; i < 20; i++) {
        // create a rock
        std::string rock_obj_path;
        if (i % 3 == 0) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        } else if (i % 3 == 1) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock2.obj");
        } else {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        }

        double scale_ratio = 0.15;
        auto rock_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock_rot = Q_from_AngX(CH_C_PI / 2);
        ChVector<> rock_pos;

        // predefined customized location
        if (i == 0)
            rock_pos = ChVector<>(1.0, -0.5, 0.0);
        else if (i == 1)
            rock_pos = ChVector<>(-0.5, -0.5, 0.0);
        else if (i == 2)
            rock_pos = ChVector<>(2.4, 0.4, 0.0);
        else if (i == 3)
            rock_pos = ChVector<>(0.6, 1.0, 0.0);
        else if (i == 4)
            rock_pos = ChVector<>(5.5, 1.2, 0.0);
        else if (i == 5)
            rock_pos = ChVector<>(1.2, 2.1, 0.0);
        else if (i == 6)
            rock_pos = ChVector<>(-0.3, -2.1, 0.0);
        else if (i == 7)
            rock_pos = ChVector<>(0.4, 2.5, 0.0);
        else if (i == 8)
            rock_pos = ChVector<>(4.2, 1.4, 0.0);
        else if (i == 9)
            rock_pos = ChVector<>(5.0, 2.4, 0.0);
        else if (i == 10)
            rock_pos = ChVector<>(0.6, -1.2, 0.0);
        else if (i == 11)
            rock_pos = ChVector<>(4.8, -1.2, 0.0);
        else if (i == 12)
            rock_pos = ChVector<>(2.5, 2.2, 0.0);
        else if (i == 13)
            rock_pos = ChVector<>(4.7, -2.2, 0.0);
        else if (i == 14)
            rock_pos = ChVector<>(-1.7, 1.5, 0.0);
        else if (i == 15)
            rock_pos = ChVector<>(-2.0, -1.1, 0.0);
        else if (i == 16)
            rock_pos = ChVector<>(-5.0, -2.1, 0.0);
        else if (i == 17)
            rock_pos = ChVector<>(1.5, -0.8, 0.0);
        else if (i == 18)
            rock_pos = ChVector<>(-2.6, 1.6, 0.0);
        else if (i == 19)
            rock_pos = ChVector<>(-2.0, 1.8, 0.0);

        rock_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock_Body->SetInertiaXX(mdensity * principal_I);

        rock_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos), ChQuaternion<>(rock_rot)));
        sys.Add(rock_Body);

        rock_Body->SetBodyFixed(false);
        rock_Body->GetCollisionModel()->ClearModel();
        rock_Body->GetCollisionModel()->AddTriangleMesh(rockSufaceMaterial, rock_mmesh, false, false, VNULL,
                                                        ChMatrix33<>(1), 0.005);
        rock_Body->GetCollisionModel()->BuildModel();
        rock_Body->SetCollide(true);

        auto rock_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        rocks.push_back(rock_Body);
    }

    // create a sensor manager module here and add a point light for camera sensor later
    auto manager = chrono_types::make_shared<ChSensorManager>(viper.GetSystem());
    manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 5000);
    manager->SetVerbose(false);
    // add a camera here with predefined camera parameters
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(),// body camera is attached to
        cam_update_rate,   // update rate in Hz

        // ----------------------below is the line to adjust camera pos ---------------------- //    
        chrono::ChFrame<double>({0, -2, 1}, Q_from_AngAxis(.2, {-2, 3, 9.75})), 
        // ----------------------above is the line to adjust camera pos ---------------------- // 
        image_width,                                                         // image width
        image_height,                                                        // image height
        cam_fov,
        super_samples);  // fov, lag, exposure
    cam->SetName("Camera Sensor");
    cam->SetCollectionWindow(exposure_time);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);
    float ch_time = 0.0f;
    float end_time = 200.0f;
    sensor_msgs::msg::Image out_image;
    out_image.height = image_height;
    out_image.width  = image_width;
    out_image.encoding = "rgb8";
    out_image.is_bigendian = 1;
    out_image.step = 3 * out_image.width;
    //add a camera buffer here
    std::vector<uint8_t> image_buffer(3*image_width*image_height);
    out_image.data = image_buffer;
    // Simulation loop
    while (rclcpp::ok()) {
        ch_time = (float)sys.GetChTime();
        
        // Hard coded it to make steering work here.
        double time = viper.GetSystem()->GetChTime();
        double max_steering = CH_C_PI / 6;
        double steering = 0;
        if (time > 2 && time < 7)
            steering = max_steering * (time - 2) / 5;
        else if (time > 7 && time < 12)
            steering = max_steering * (12 - time) / 5;
        driver->SetSteering(steering);

        // Access the RGBA8 buffer from the camera
        UserRGBA8BufferPtr rgba8_ptr;
        rgba8_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr->Buffer){
            std::cout << "Publishing data ..." <<std::endl;
            for (size_t i=0; i<(image_height*image_width);i++){
                //std::cout<<"debug"<<std::endl;
                PixelRGBA8 data_pixel = rgba8_ptr->Buffer[i];
                image_buffer[i*3]=(data_pixel.R);
                image_buffer[i*3+1]=(data_pixel.G);
                image_buffer[i*3+2]=(data_pixel.B);
            }
        }
        else {std::cout<<"waiting for the data"<<std::endl;}
        // assign value from image_buffer (same data type as out_image.data) to the ros topic
        out_image.data = image_buffer;
        // publish data into ros topic here
        publisher->publish(out_image);
        // update in the simulation loop 
        // update sensor manager
        manager->Update();
        //update viper dynamics system
        viper.Update();
        sys.DoStepDynamics(time_step);
    }
    rclcpp::shutdown();
    return 0;
}