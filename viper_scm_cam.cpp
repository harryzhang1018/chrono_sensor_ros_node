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
// Authors: Jason Zhou, Harry Zhang, Thomas Liang
// =============================================================================
//
// Demo to show Viper Rover operated on SCM Terrain
//
// =============================================================================
#include <iostream>

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"

#include "chrono/physics/ChSystemSMC.h"
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
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterCameraExposure.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

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

// Camera lens model
// Either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Update rate in Hz
float update_rate = 30;
// Image width and height
unsigned int image_width = 1920;
unsigned int image_height = 1080;
// Camera's horizontal field of view
float fov = (float)CH_C_PI / 2.;
// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.0f;
// Exposure (in seconds) of each image
float exposure_time = 0.0f;
int alias_factor = 1;
// Save camera images
bool save = true ;
// Render camera images
bool vis = false ;
// Exposure correction filter
bool exposure_correction_switch = false;

const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Number of SCM and collision threads
int nthreads = 40;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type (RealWheel, SimpleWheel, or CylWheel)
ViperWheelType wheel_type = ViperWheelType::RealWheel;

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
    float mu = 0.7f;   // coefficient of friction
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
    if (argc < 5) {
        std::cout << "Missing parameters: {Sun Position: 1,2,3,4} {Hapke: 0,1} {Third Rock Size: [0,1]} {Exposure Correction: 0,1}";
        return 0;
    }
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Sensor_Camera_Node_SCM");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("sensor_image_scm", 10);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // define different postion of the sun
    auto pos1 = ChVector<>(0.0,-20.0,20.0);
    auto pos2 = ChVector<>{0.0,20.0,20.0};
    auto pos3 = ChVector<>{20.0,0.0,20.0};
    auto pos4 = ChVector<>{-20.0,0.0,20.0};
    auto sun_pose = ChVector<>{0.0,0.0,0.0};
    if (std::atoi(argv[1])==1){
        sun_pose = pos1;
    }
    else if (std::atoi(argv[1])==2){
        sun_pose = pos2;
    }
    else if (std::atoi(argv[1])==3){
        sun_pose = pos3;
    }
    else {
        sun_pose = pos4;
    }
    //enable Hapke or not?---0(disable hapke)--1(enable hapke)
    int enable_hapke = std::atoi(argv[2]);

    // initilize visual material
    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({1,1,1}); //0.65f,0.65f,0.65f
    vis_mat2->SetDiffuseColor({1,1,1});
    vis_mat2->SetSpecularColor({1,1,1});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(1.0f);
    vis_mat2->SetUseHapke(enable_hapke);
    /*
    if (enable_hapke = 1){
        vis_mat2->SetUseHapke(true);}
    else {
        vis_mat2->SetUseHapke(false);
    }
    */
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);

    // Global parameter for moving patch size:
    double wheel_range = 0.25;
    ////double body_range = 1.2;

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetNumThreads(nthreads, nthreads, 1);

    // Create the rover
    //auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(0.1,0.37f);
    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);
    viper.SetChassisCollide(true);
    viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    viper.Initialize(ChFrame<>(ChVector<>(-1.5, -4.1, 0.2), QUNIT));
    //Used for moving rover closer to last rock
    // viper.Initialize(ChFrame<>(ChVector<>(2.4, 4, 0.05), Q_from_AngZ(CH_C_PI / 2)));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

    // Debugging the rock 
    auto vis_mat3 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat3->SetAmbientColor({1,1,1}); //0.65f,0.65f,0.65f
    vis_mat3->SetDiffuseColor({1,1,1});
    vis_mat3->SetSpecularColor({1,1,1});
    vis_mat3->SetUseSpecularWorkflow(true);
    vis_mat3->SetRoughness(1.0f);
    vis_mat3->SetUseHapke(1);

    // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChMaterialSurface> rockSufaceMaterial = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());
    rockSufaceMaterial->SetFriction(0.7f);
    // Add rock objects given as command line argument
    for (int i = 0; i < 3; i++) {
        // create a rock
        std::string rock_obj_path;

        // predefined customized location
        ChVector<> rock_pos;

        ChQuaternion<> rock_rot = Q_from_AngX(CH_C_PI / 2);
        double scale_ratio = 0.18;

        if (i == 0) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
            rock_pos = ChVector<>(2.5, -1, 0.1);
        } else if (i == 1) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/new_rock_2.obj");
            rock_pos = ChVector<>(2.935, 7.3, 0.2);
            rock_rot = Q_ROTATE_X_TO_Z;
            scale_ratio = std::stod(argv[3]);
        } else {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
            rock_pos = ChVector<>(2.9, 5, 0.1);
        }

        
        //double scale_ratio = 0.25;
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
        {
        if(rock_mesh->GetNumMaterials() == 0){
            rock_mesh->AddMaterial(vis_mat2);
            // rock_mesh->AddMaterial(vis_mat3);
        }
        else{
            rock_mesh->GetMaterials()[0] = vis_mat2;
            // rock_mesh->GetMaterials()[0] = vis_mat3;
        }
        }  
        rock_Body->AddVisualShape(rock_mesh);

        rocks.push_back(rock_Body);
    }

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMDeformableTerrain terrain(&sys);
    // load mesh from obj file
    std::string terrain_mesh_path = GetChronoDataFile("robot/curiosity/rocks/lunar_env_12.obj");
    //std::string terrain_mesh_path = GetChronoDataFile("robot/curiosity/rocks/crater.obj");
    terrain.Initialize(terrain_mesh_path,mesh_resolution);
    terrain.SetMeshWireframe(false);
    auto mesh = terrain.GetMesh();
    {
        if(mesh->GetNumMaterials() == 0){
            mesh->AddMaterial(vis_mat2);
        }
        else{
            mesh->GetMaterials()[0] = vis_mat2;
        }
    }
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
                                  3e2     // Damping (Pa s/m), proportional to negative vertical speed (optional)
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
        //for Viper moving on the terrain
        terrain.AddMovingPatch(Wheel_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        //3 rocks standing on the terrain
        for (int i = 0; i < 3; i++) {
            terrain.AddMovingPatch(rocks[i], ChVector<>(0, 0, 0), ChVector<>(0.5, 0.5, 0.5));
        }
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    //terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);

    //terrain.SetMeshWireframe(true);

    //create ground body attach camera
    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetBodyFixed(true);
    sys.Add(ground_body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight(sun_pose, {1,1,1}, 2000.0f);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_16k.hdr");
    manager->scene->SetBackground(b);

    std::vector<float> expsr_a_arr(3, 0.); // exposure correction coefficients of a-term
    std::vector<float> expsr_b_arr(3, 0.); // exposure correction coefficients of b-term

    expsr_a_arr = {0.29556334914850335, -0.2425751791534932, -0.9214774573420131}; // complex Hapke model, Terrain 01
    expsr_b_arr = {1.3639594220085027, -0.35420470527525494, -9.464210018888805}; // complex Hapke model, Terrain 01

    chrono::ChFrame<double> wheel_poses[4] = {chrono::ChFrame<double>({-1.0, -1.5, 0.5}, Q_from_AngAxis(.2, {-2, 3, 9.75})), 
                                                chrono::ChFrame<double>({1.0,-1.5,0.5}, Q_from_AngAxis(.2, {-2, 3, 9.75})),
                                                chrono::ChFrame<double>({1.0,1.5,0.5}, Q_from_AngAxis(.2, {2, 3, -9.75})),
                                                chrono::ChFrame<double>({-1.0,1.5,0.5}, Q_from_AngAxis(.2, {2, 3, -9.75}))};

    // for (int i = 0; i < 4; i++) {
    //     auto cam = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(),         // body camera is attached to
    //                                                         update_rate,   // update rate in Hz
    //                                                         wheel_poses[i],  // offset pose
    //                                                         image_width,   // image width
    //                                                         image_height,  // image height
    //                                                         fov,           // camera's horizontal field of view
    //                                                         alias_factor,  // supersample factor for antialiasing
    //                                                         lens_model,    // FOV
    //                                                         false);        // use global illumination or not
    //     cam->SetName("Wheel Camera " + std::to_string(i));
    //     cam->SetLag(lag);
    //     cam->SetCollectionWindow(exposure_time);
    //     // if (exposure_correction_switch == true) {
    //     //     cam->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
    //     //                                                                              expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], std::atof(argv[4])));
    //     // }
    //     if (vis)
    //         cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Wheel Camera " + std::to_string(i)));
    //     if (save)
    //         cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "_" + argv[1] + "_" + argv[2] + "_" + argv[3] + "/" + "Wheel_Cam_" + std::to_string(i)));
    //     manager->AddSensor(cam);
    // }

    chrono::ChFrame<double> offset_pose5({0,-6,1.5}, Q_from_AngAxis(CH_C_PI/2, {0, 0, 1})); //-8, 0, 2
    auto cam5 = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(),         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose5,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,    // FOV
                                                          false);        // use global illumination or not
    cam5->SetName("3rd Person Camera");
    cam5->SetLag(lag);
    cam5->SetCollectionWindow(exposure_time);
    if (exposure_correction_switch == true) {
        cam5->PushFilter(chrono_types::make_shared<ChFilterCameraExposureCorrect>(expsr_a_arr[0], expsr_a_arr[1], expsr_a_arr[2],
                                                                                  expsr_b_arr[0], expsr_b_arr[1], expsr_b_arr[2], std::atof(argv[4])));
    }
    if (vis)
        cam5->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "3rd Person Cam"));
    if (save)
         cam5->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "_" + argv[1] + "_" + argv[2] + "_" + argv[3] + "/" + "ThirdPerson"));
    cam5->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam5);

    // // Create the Irrlicht visualization sys
    // auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();Speed
    // vis->AddCamera(ChVector<>(2.0, 0.0, 1.4), ChVector<>(0, 0, wheel_range));
    // vis->AddTypicalLights();
    // vis->AddLightWithShadow(ChVector<>(-5.0, -0.5, 8.0), ChVector<>(-1, 0, 0), 100, 1, 35, 85, 512,
    //                         ChColor(0.8f, 0.8f, 0.8f));
    // vis->EnableShadows();
        // std::chrono::high_resolution_clock::time_point tm1 = std::chrono::high_resolution_clock::now();
    ChTimer<> timer;
    // Simulation end time
    float end_time = 180.0f;
    float ch_time = 0.0;
    float orbit_radius = 100.0f;
    float orbit_rate = 0.5f;

    sensor_msgs::msg::Image out_image;
    out_image.height = image_height;
    out_image.width  = image_width;
    out_image.encoding = "rgb8";
    out_image.is_bigendian = 1;
    out_image.step = 3 * out_image.width;
    //add a camera buffer here
    std::vector<uint8_t> image_buffer(3*image_width*image_height);
    out_image.data = image_buffer;

    while (rclcpp::ok()) {
    // while (vis->Run()) {   
    //     vis->BeginScene();
    //     vis->GetActiveCamera()->setTarget(core::vector3dfCH(Body_1->GetPos()));
    //     vis->Render();
    //     tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1180);
    //     vis->EndScene();# Make ssh dir
        // --- Comment these lines line if you don't want the "sun" moving
        ch_time = (float)sys.GetChTime();
        // PointLight p = {{orbit_radius * cos(ch_time * orbit_rate), orbit_radius * sin(ch_time * orbit_rate), 15.0f} , {1.0f,1.0f,1.0f}, 2000.0f};
        // manager->scene->ModifyPointLight(0,p);

        manager->Update();
        
        // std::chrono::high_resolution_clock::time_point tm1 = std::chrono::high_resolution_clock::now();
        

        // Access the RGBA8 buffer from the camera
        UserRGBA8BufferPtr rgba8_ptr;
        rgba8_ptr = cam5->GetMostRecentBuffer<UserRGBA8BufferPtr>();
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

        double max_steering = CH_C_PI / 3;
        double steering = 0;
        if (ch_time > 4 && ch_time < 45) {
            steering = max_steering * (ch_time - 4) / 4;
        } else if (ch_time > 75 && ch_time < 80) {
            steering = max_steering * (ch_time - 75) / 4;
        }

        driver->SetSteering(steering);

        timer.start();
        sys.DoStepDynamics(2.5e-4);
        timer.stop();

        viper.Update();

        // std::chrono::high_resolution_clock::time_point tm2 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> wall_time_m = std::chrono::duration_cast<std::chrono::duration<double>>(tm2 - tm1);
        // std::cout << "wall time: " << wall_time_m.count() << "s.\n";

        ////terrain.PrintStepStatistics(std::cout);
    }
    std::cout << "Sim time: " << ch_time << " RTF: " << timer() / ch_time << std::endl;


    return 0;
}
