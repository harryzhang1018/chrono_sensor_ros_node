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
// Authors: Jason Zhou, Harry Zhang
// =============================================================================
//
// Demo to show Viper Rover operated on SCM Terrain with obstacles
// A radar and a lidar sensors are used in this demo
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
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

#include "chrono_thirdparty/filesystem/path.h"


// ros2 packages dependency
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::viper;
using namespace chrono::sensor;

using namespace irr;

bool output = false;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = false;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

/////// setting parameters for camera sensors////////
float cam_update_rate = 30.f;
float exposure_time = 0.02f;
int super_samples = 2;
// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;
// Camera's horizontal field of view
float cam_fov = 1.408f;


// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMDeformableTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector<>& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Return customized wheel material parameters
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Sensor_Camera_Node_SCM");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("sensor_image", 10);
    auto publisher2 = node->create_publisher<sensor_msgs::msg::Image>("sensor_image_3rdperson", 10);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for moving patch size:
    double wheel_range = 0.5;
    ////double body_range = 1.2;

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    viper.Initialize(ChFrame<>(ChVector<>(-5, 0, -0.2), QUNIT));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

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

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMDeformableTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    terrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, -0.5)));

    // Use a regular grid:
    double length = 15;
    double width = 15;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // We need to add a moving patch under every wheel
    // Or we can define a large moving patch at the pos of the rover body
    if (enable_moving_patch) {
        terrain.AddMovingPatch(Wheel_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));

        for (int i = 0; i < 20; i++) {
            terrain.AddMovingPatch(rocks[i], ChVector<>(0, 0, 0), ChVector<>(0.5, 0.5, 0.5));
        }
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(false);


    //
    // SENSOR SIMULATION
    //

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {0.4, 0.4, 0.4}, 500);
    manager->SetVerbose(false);
    //Adding camera into manager
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(),                                              // body camera is attached to
        cam_update_rate,                                                     // update rate in Hz
        chrono::ChFrame<double>({0, -2, 1}, Q_from_AngAxis(.2, {-2, 3, 9.75})),  // offset pose
        image_width,                                                         // image width
        image_height,                                                        // image height
        cam_fov,
        super_samples);  // fov, lag, exposure
    cam->SetName("Camera Sensor");
    cam->SetCollectionWindow(exposure_time);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    auto cam_obs = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(),                                              // body camera is attached to
        cam_update_rate,  
        // this is the line to adjust camera pos                                                   // update rate in Hz
        chrono::ChFrame<double>({-5, -5, 5}, Q_from_AngAxis(.2, {-2, 3, 6})),  // offset pose
        image_width,                                                         // image width
        image_height,                                                        // image height
        cam_fov,
        super_samples);  // fov, lag, exposure
    cam_obs->SetName("Camera Sensor Thirdperson Viewpoint");
    cam_obs->SetCollectionWindow(exposure_time);
    cam_obs->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam_obs);    
    
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


    // Simulation loop
    while (rclcpp::ok()) {

        // update sensor manager
        manager->Update();

        if (output) {
            // write drive torques of all four wheels into file
            csv << sys.GetChTime() << viper.GetWheelTracTorque(ViperWheelID::V_LF)
                << viper.GetWheelTracTorque(ViperWheelID::V_RF) << viper.GetWheelTracTorque(ViperWheelID::V_LB)
                << viper.GetWheelTracTorque(ViperWheelID::V_RB) << std::endl;
        }

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

        // Access the RGBA8 buffer from the camera
        UserRGBA8BufferPtr rgba8_ptr_3rd;
        rgba8_ptr_3rd = cam_obs->GetMostRecentBuffer<UserRGBA8BufferPtr>();
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

        sys.DoStepDynamics(5e-4);
        viper.Update();
        
    }

    return 0;
}