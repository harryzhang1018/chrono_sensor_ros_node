// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu Batagoda, Harry Zhang
// =============================================================================
//
// A demonstration of the Chrono camera sensor behaving in lunar environments
//
// =============================================================================


#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"


#include "chrono_models/robot/viper/Viper.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

// ros2 packages dependency
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

using namespace chrono::viper;
using namespace chrono::sensor;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL,     // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};
NoiseModel noise_model = PIXEL_DEPENDENT;

// Camera lens model
// Either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float)CH_C_PI / 2.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.0f;

// Exposure (in seconds) of each image
float exposure_time = 0.0f;

int alias_factor = 1;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 40.0f;

// Save camera images
bool save = true;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Sensor_Camera_Node");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("sensor_image", 10);
    auto publisher2 = node->create_publisher<sensor_msgs::msg::Image>("sensor_image_3rdperson", 10);
    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() << "Alias Factor: " << alias_factor << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({1,1,1}); //0.65f,0.65f,0.65f
    vis_mat2->SetDiffuseColor({1,1,1});
    vis_mat2->SetSpecularColor({1,1,1});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(1.0f);
    vis_mat2->SetUseHapke(true);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);


     // load mesh from obj file
    auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
    std::string terrain_mesh_path = GetChronoDataFile("robot/curiosity/rocks/terrain_and_rocks.obj");
    double terrain_scale_ratio = 1.0;
    auto terrain_mesh_loader = ChTriangleMeshConnected::CreateFromWavefrontFile(terrain_mesh_path, false, false);
    terrain_mesh_loader->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(terrain_scale_ratio));  // scale to a different size
    terrain_mesh_loader->RepairDuplicateVertexes(1e-9); // if meshes are not watertight
    
    // set terrain mesh
    terrain_mesh->SetMesh(terrain_mesh_loader);
    terrain_mesh->SetBackfaceCull(true);
    
    auto patch3_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    RigidTerrain terrain(&sys);
    auto mesh = terrain.AddPatch(patch3_mat, ChCoordsys<>(ChVector<>(0, 0, -0.5), QUNIT),
                                   GetChronoDataFile("robot/curiosity/rocks/terrain_and_rocks.obj"));
    {
       // printf("OK!\n");
        mesh->GetGroundBody()->AddVisualShape(terrain_mesh);
        auto shape = mesh->GetGroundBody()->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat2);
        }
        else{
            shape->GetMaterials()[0] = vis_mat2;
        }
    }
    terrain.Initialize();
   

    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetBodyFixed(true);
    sys.Add(ground_body);


    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100,100,100}, {1,1,1}, 1000.0f);
    Background b;
    b.mode = BackgroundMode::SOLID_COLOR;
    b.color_zenith = {0,0,0};
    manager->scene->SetBackground(b);


    
    // Create the rover
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.0f,0.0f);

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);

    viper.Initialize(ChFrame<>(ChVector<>(0,0,0.5), QUNIT));

    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------

    // --- Change pose of wheel camera here ----
    chrono::ChFrame<double> offset_pose1({-0.5, -1.7, 0.5}, Q_from_AngAxis(.2, {-2, 3, 9.75}));
    // -----------------------------------------
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(),         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose1,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,    // FOV
                                                          true);        // use global illumination or not
    cam1->SetName("Wheel Camera");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam1);

       // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------

    chrono::ChFrame<double> offset_pose2({0,-4,1}, Q_from_AngAxis(CH_C_PI/2, {0, 0, 1})); //-8, 0, 2
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(ground_body,         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,    // FOV
                                                          true);        // use global illumination or not
    cam2->SetName("3rd Person Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam2);

    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    float ch_time = 0.0;
    float orbit_radius = 1000.0f;
    float orbit_rate = 0.1f;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // adding publish topic initialization 
    sensor_msgs::msg::Image out_image;
    out_image.height = image_height;
    out_image.width  = image_width;
    out_image.encoding = "rgb8";
    out_image.is_bigendian = 1;
    out_image.step = 3 * out_image.width;
    //add a camera buffer here
    std::vector<uint8_t> image_buffer(3*image_width*image_height);
    out_image.data = image_buffer;

    sensor_msgs::msg::Image out_image_3rd;
    out_image_3rd.height = image_height;
    out_image_3rd.width  = image_width;
    out_image_3rd.encoding = "rgb8";
    out_image_3rd.is_bigendian = 1;
    out_image_3rd.step = 3 * out_image_3rd.width;
    //add a camera buffer here
    std::vector<uint8_t> image_buffer_3rd(3*image_width*image_height);
    out_image_3rd.data = image_buffer_3rd;

    while (rclcpp::ok()) {

        ch_time = (float)sys.GetChTime();
        
        // --- Comment these lines line if you don't want the "sun" moving
        PointLight p = {{orbit_radius * cos(ch_time * orbit_rate), 2.0f, orbit_radius * sin(ch_time * orbit_rate)}, {1.0f,1.0f,1.0f}, 1000.0f};
        manager->scene->ModifyPointLight(0,p);
        // -------------------------------------------------------

        // Access the RGBA8 buffer from the camera
        UserRGBA8BufferPtr rgba8_ptr;
        rgba8_ptr = cam1->GetMostRecentBuffer<UserRGBA8BufferPtr>();
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

        // Access the RGBA8 buffer from the camera
        UserRGBA8BufferPtr rgba8_ptr_3rd;
        rgba8_ptr_3rd = cam2->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr_3rd->Buffer){
            std::cout << "Publishing 3rd person view data ..." <<std::endl;
            for (size_t i=0; i<(image_height*image_width);i++){
                //std::cout<<"debug"<<std::endl;
                PixelRGBA8 data_pixel = rgba8_ptr_3rd->Buffer[i];
                image_buffer_3rd[i*3]=(data_pixel.R);
                image_buffer_3rd[i*3+1]=(data_pixel.G);
                image_buffer_3rd[i*3+2]=(data_pixel.B);
            }
        }
        else {std::cout<<"waiting for the data"<<std::endl;}
        // assign value from image_buffer (same data type as out_image.data) to the ros topic
        out_image_3rd.data = image_buffer_3rd;
        // publish data into ros topic here
        publisher2->publish(out_image_3rd);

        // Update sensor manager
        manager->Update();
        sys.DoStepDynamics(step_size);
   
    }
    rclcpp::shutdown();
    return 0;
}