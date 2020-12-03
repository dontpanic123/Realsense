// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example_usb.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include <atomic>
#include <map>
#include <string>
#include <thread>

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
using namespace std;

/**
Helper class for controlling the filter's GUI element
*/
struct filter_slider_ui
{
    std::string name;
    std::string label;
    std::string description;
    bool is_int;
    float value;
    rs2::option_range range;

    bool render(const float3& location, bool enabled);
    static bool is_all_integers(const rs2::option_range& range);
};

/**
Class to encapsulate a filter alongside its options
*/
class filter_options
{
public:
    filter_options(const std::string name, rs2::filter& filter);
    filter_options(filter_options&& other);
    std::string filter_name;                                   //Friendly name of the filter
    rs2::filter& filter;                                       //The filter in use
    std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

// Helper functions for rendering the UI
void render_ui(float w, float h, std::vector<filter_options>& filters);
// Helper function for getting data from the queues and updating the view
void update_data(rs2::frame_queue& data, rs2::frame& depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map);

int main(int argc, char* argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");

     ImGui_ImplGlfw_Init(app, false);
    rs2::context                          ctx;
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
    {
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        cout << "serial_number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;

    }
    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_RGB8, 30);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8  424 240
        cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8
        cfg.enable_device(serial);

        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }
    // Construct an object to manage view state
    glfw_state  app_state;
    glfw_state  app_state1;
    glfw_state  app_state2;
    glfw_state  app_state3;
    glfw_state  app_state4;
    //std::vector<glfw_state> app_states;
    int pc_index = 0; // In order to distinguish the currently controlled point cloud  
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);
    //register_glfw_callbacks(app1, app_state);
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;



    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

                                        // Declare disparity transform from depth to disparity and vice versa
    const std::string disparity_filter_name = "Disparity";
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // Initialize a vector that holds filters and their options
    std::vector<filter_options> filters;

    // The following order of emplacement will dictate the orders in which filters are applied
    filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Threshold", thr_filter);
    filters.emplace_back(disparity_filter_name, depth_to_disparity);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);



    while (app) // Application still alive?
    {
        // Collect the new frames from all the connected devices
        //std::vector<rs2::frame> new_frames;

        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());

        // Render the GUI
        render_ui(w, h, filters);

        draw_text(10, 20, "camera 1 upper_left   serial_number: 745412070908");
        draw_text(10, 40, "camera 2 upper_right  serial_number: 947522071890");
        draw_text(10, 60, "camera 3 driver_persp serial_number: 936322070874");
        draw_text(10, 80, "camera 4 forks_persp   serial_number: 936322071095");
        draw_text(10, 100, "camera 5 basis_persp   serial_number: 923322072240");
        draw_text(10, 140, "press the nember keys form one to five to select the point cloud you want to manipulate");
        draw_text(10, 160, "press the nember keys 0 to select the all point clouds to manipulate");
        draw_text(10, 200, "rotation           x:w,s       y:a,d       z:q,e");
        draw_text(10, 220, "translation:       x:y,x       y:c,v       z:r,f");

        int index = 1; //This index is used to update the displayed point cloud
        for (auto pipe : pipelines)
        {
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)  color = frames.get_infrared_frame();
            pc.map_to(color);
            // Tell pointcloud object to map to this color frame
            auto depth = frames.get_depth_frame();
            
            // filter                       §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
            bool revert_disparity = false;
            for (auto&& filter : filters)
            {
                 if (filter.is_enabled)
                  {
                      depth = filter.filter.process(depth);
                      if (filter.filter_name == disparity_filter_name)
                      {
                          revert_disparity = true;
                      }
                  }
            }
            if (revert_disparity)
            {
                depth = disparity_to_depth.process(depth);
            }
            
            // Generate the pointcloud and texture mappings
            points = pc.calculate(depth);
           
            // Upload the color frame to OpenGL
            glDepthFunc(GL_LESS);

            // register callbacks to allow manipulation of the pointcloud
         //   register_glfw_callbacks(app, *app_state);


            if (index == 1) {
                glViewport(0, 0, 1280, 720);
                app_state.tex.upload(color);
                draw_pointcloud(1280, 720, app_state, points);
            }
            if (index == 2) {
                glViewport(0, 0, 1280, 720);
                app_state1.tex.upload(color);
                draw_pointcloud(1280, 720, app_state1, points);
            }
            if (index == 3) {
                // glViewport(1000, 0, 960, 540);

                glViewport(0, 0, 1280, 720);
                app_state2.tex.upload(color);
                draw_pointcloud(1280, 720, app_state2, points);
            }
            if (index == 4) {

                // glViewport(1000, 540, 960, 540);

                glViewport(0, 0, 1280, 720);
                app_state3.tex.upload(color);
                draw_pointcloud(1280, 720, app_state3, points);
            }
            if (index == 5) {
                glViewport(0, 0, 1280, 720);
                app_state4.tex.upload(color);
                draw_pointcloud(1280, 720, app_state4, points);
            }

            index++;
        }

        /* bool isVisible = true;
         if (isVisible)
         {
             glViewport(0, 0, 1280, 720);
             glColor3f(255, 1, 1);
             GLfloat curSizeLine = 5;
             glLineWidth(curSizeLine);
             glBegin(GL_LINE_STRIP);

             glVertex3f(2.3f, 2.3f, 0.0f);
             glVertex3f(-2.15f, 2.3f, 0.0f);
             glVertex3f(25.f, 15.f, 0.0f);

             glEnd();

         }*/

        bool print_state = false;// When this variable is true, output the current position status
        app.on_key_release = [&](int key)
        {
            cout << key << endl;
            if (key == 32) // Escape, move all the pointcloud to a defaut position.
            {
                app_state.yaw = -6;
                app_state.pitch = -43;
                app_state.roll = 0;
                app_state.offset_x = -20;
                app_state.offset_y = -24;
                app_state.offset_z = 15;

                app_state1.yaw = 2;
                app_state1.pitch = -37;
                app_state1.roll = 1;
                app_state1.offset_x = 0.0;
                app_state1.offset_y = -21.0;
                app_state1.offset_z = 12.0;

                app_state2.yaw = 0;
                app_state2.pitch = -7;
                app_state2.roll = -2;
                app_state2.offset_x = -11.0;
                app_state2.offset_y = -4.0;
                app_state2.offset_z = -6.0;

                app_state3.yaw = -1;
                app_state3.pitch = 7;
                app_state3.roll = 183;
                app_state3.offset_x = -10.0;
                app_state3.offset_y = 19.0;
                app_state3.offset_z = 41.0;

                app_state4.yaw = 0;
                app_state4.pitch = 0;
                app_state4.roll = 0;
                app_state4.offset_x = -10;
                app_state4.offset_y = 0;
                app_state4.offset_z = 7;




                print_state = true;
            }

            if (key == 79) // o, move all the pointcloud to opencv position.
            {
                cout << "OpenCV position" << endl;
                app_state.yaw = -6.1502;
                app_state.pitch = -38.8882;
                app_state.roll = -0.5021;
                app_state.offset_x = -22.0178;
                app_state.offset_y = -22.7578;
                app_state.offset_z = 17.4156;

                app_state1.yaw = 0.6730;
                app_state1.pitch = -37.6007;
                app_state1.roll = -0.6624;
                app_state1.offset_x = 1.5844;
                app_state1.offset_y = -23.08;
                app_state1.offset_z = 18.1111;

                app_state2.yaw = -4, 9777;
                app_state2.pitch = -23, 7910;
                app_state2.roll = -3, 4316;
                app_state2.offset_x = -10.5867;
                app_state2.offset_y = -22.3267;
                app_state2.offset_z = -12.6178;

                app_state3.yaw = -0.2638;
                app_state3.pitch = 9.0734;
                app_state3.roll = -178.6431;
                app_state3.offset_x = -10.3667;
                app_state3.offset_y = 24.65;
                app_state3.offset_z = 45.706;

                app_state4.yaw = 0;
                app_state4.pitch = 0;
                app_state4.roll = 0;
                app_state4.offset_x = -10;
                app_state4.offset_y = 0;
                app_state4.offset_z = 7;




                print_state = true;
            }

            if (key == 75) // K, move all the pointcloud to kalibr position.
            {
                cout << "Kalibr position" << endl;
                app_state.yaw = 6.2785;
                app_state.pitch = -38, 4940;
                app_state.roll = 0.3527;
                app_state.offset_x = -34.2958;
                app_state.offset_y = -22.325;
                app_state.offset_z = 15.4583;

                app_state1.yaw = 3.9538;
                app_state1.pitch = -37.0704;
                app_state1.roll = -0.2495;
                app_state1.offset_x = 0.7421;
                app_state1.offset_y = -22.5875;
                app_state1.offset_z = 13.3770;

                app_state2.yaw = -0.9249;
                app_state2.pitch = -19.1966;
                app_state2.roll = -3.0028;
                app_state2.offset_x = -11.2354;
                app_state2.offset_y = -17.5375;
                app_state2.offset_z = -13.2917;

                app_state3.yaw = -7.3435;
                app_state3.pitch = 0.6026;
                app_state3.roll = 178.2782;
                app_state3.offset_x = -3.6354;
                app_state3.offset_y = 13.0667;
                app_state3.offset_z = 37.1167;

                app_state4.yaw = 0;
                app_state4.pitch = 0;
                app_state4.roll = 0;
                app_state4.offset_x = -10;
                app_state4.offset_y = 0;
                app_state4.offset_z = 7;




                print_state = true;
            }


            if (key == 48) // 0
            {


                pc_index = 8;
                // app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
            }
            if (key == 49) // 1
            {
                print_state = true;
                register_glfw_callbacks(app, app_state);
                pc_index = 0;
                // app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
            }
            if (key == 50) // 2
            {
                print_state = true;
                register_glfw_callbacks(app, app_state1);
                pc_index = 1;
                //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
            }
            if (key == 51) // 3
            {
                print_state = true;
                register_glfw_callbacks(app, app_state2);
                pc_index = 2;
                //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
            }
            if (key == 52) // 4
            {
                print_state = true;
                register_glfw_callbacks(app, app_state3);
                pc_index = 3;
                //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
            }
            if (key == 53) // 5
            {
                print_state = true;
                register_glfw_callbacks(app, app_state4);
                pc_index = 4;
                //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
            }

            if (key == 87) // w  y rotation +
            {
                if (pc_index == 0) app_state.pitch += 1;
                if (pc_index == 1) app_state1.pitch += 1;
                if (pc_index == 2) app_state2.pitch += 1;
                if (pc_index == 3) app_state3.pitch += 1;
                if (pc_index == 4) app_state4.pitch += 1;


                if (pc_index == 8)
                {
                    app_state.pitch += 1;
                    app_state1.pitch += 1;
                    app_state2.pitch += 1;
                    app_state3.pitch += 1;
                    app_state4.pitch += 1;

                }
            }
            if (key == 83) // s  y rotation -
            {
                if (pc_index == 0) app_state.pitch -= 1;
                if (pc_index == 1) app_state1.pitch -= 1;
                if (pc_index == 2) app_state2.pitch -= 1;
                if (pc_index == 3) app_state3.pitch -= 1;
                if (pc_index == 4) app_state4.pitch -= 1;

                if (pc_index == 8)
                {
                    app_state.pitch -= 1;
                    app_state1.pitch -= 1;
                    app_state2.pitch -= 1;
                    app_state3.pitch -= 1;
                    app_state4.pitch -= 1;
                }
            }
            if (key == 65) // a : x rotation -
            {
                if (pc_index == 0) app_state.yaw -= 1;
                if (pc_index == 1) app_state1.yaw -= 1;
                if (pc_index == 2) app_state2.yaw -= 1;
                if (pc_index == 3) app_state3.yaw -= 1;
                if (pc_index == 4) app_state4.yaw -= 1;

                if (pc_index == 8)
                {
                    app_state.yaw -= 1;
                    app_state1.yaw -= 1;
                    app_state2.yaw -= 1;
                    app_state3.yaw -= 1;
                    app_state4.yaw -= 1;
                }
            }
            if (key == 68) // d : x rotation +
            {
                if (pc_index == 0) app_state.yaw += 1;
                if (pc_index == 1) app_state1.yaw += 1;
                if (pc_index == 2) app_state2.yaw += 1;
                if (pc_index == 3) app_state3.yaw += 1;
                if (pc_index == 4) app_state4.yaw += 1;


                if (pc_index == 8)
                {
                    app_state.yaw += 1;
                    app_state1.yaw += 1;
                    app_state2.yaw += 1;
                    app_state3.yaw += 1;
                    app_state4.yaw += 1;
                }
            }
            if (key == 81) // q : z rotation -
            {
                if (pc_index == 0) app_state.roll -= 1;
                if (pc_index == 1) app_state1.roll -= 1;
                if (pc_index == 2) app_state2.roll -= 1;
                if (pc_index == 3) app_state1.roll -= 1;
                if (pc_index == 4) app_state2.roll -= 1;


                if (pc_index == 8)
                {
                    app_state.roll -= 1;
                    app_state1.roll -= 1;
                    app_state2.roll -= 1;
                    app_state3.roll -= 1;
                    app_state4.roll -= 1;
                }
            }
            if (key == 69) // e : z rotation +
            {
                if (pc_index == 0) app_state.roll += 1;
                if (pc_index == 1) app_state1.roll += 1;
                if (pc_index == 2) app_state2.roll += 1;
                if (pc_index == 1) app_state3.roll += 1;
                if (pc_index == 2) app_state4.roll += 1;

                if (pc_index == 8)
                {
                    app_state.roll += 1;
                    app_state1.roll += 1;
                    app_state2.roll += 1;
                    app_state3.roll += 1;
                    app_state4.roll += 1;
                }
            }
            if (key == 90) // y: x translation-
            {
                if (pc_index == 0) app_state.offset_x -= 1.f;
                if (pc_index == 1) app_state1.offset_x -= 1.f;
                if (pc_index == 2) app_state2.offset_x -= 1.f;
                if (pc_index == 3) app_state3.offset_x -= 1.f;
                if (pc_index == 4) app_state4.offset_x -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x -= 1.f;
                    app_state1.offset_x -= 1.f;
                    app_state2.offset_x -= 1.f;
                    app_state3.offset_x -= 1.f;
                    app_state4.offset_x -= 1.f;
                }
            }
            if (key == 88) // x: x translation+
            {
                if (pc_index == 0) app_state.offset_x += 1.f;
                if (pc_index == 1) app_state1.offset_x += 1.f;
                if (pc_index == 2) app_state2.offset_x += 1.f;
                if (pc_index == 3) app_state3.offset_x += 1.f;
                if (pc_index == 4) app_state4.offset_x += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x += 1.f;
                    app_state1.offset_x += 1.f;
                    app_state2.offset_x += 1.f;
                    app_state3.offset_x += 1.f;
                    app_state4.offset_x += 1.f;
                }
            }
            if (key == 67) // c: y translation-
            {
                if (pc_index == 0) app_state.offset_y -= 1.f;
                if (pc_index == 1) app_state1.offset_y -= 1.f;
                if (pc_index == 2) app_state2.offset_y -= 1.f;
                if (pc_index == 3) app_state3.offset_y -= 1.f;
                if (pc_index == 4) app_state4.offset_y -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_y -= 1.f;
                    app_state1.offset_y -= 1.f;
                    app_state2.offset_y -= 1.f;
                    app_state3.offset_y -= 1.f;
                    app_state4.offset_y -= 1.f;
                }
            }
            if (key == 86) // v: y translation+
            {
                if (pc_index == 0) app_state.offset_y += 1.f;
                if (pc_index == 1) app_state1.offset_y += 1.f;
                if (pc_index == 2) app_state2.offset_y += 1.f;
                if (pc_index == 3) app_state3.offset_y += 1.f;
                if (pc_index == 4) app_state4.offset_y += 1.f;


                if (pc_index == 8)
                {
                    app_state.offset_y += 1.f;
                    app_state1.offset_y += 1.f;
                    app_state2.offset_y += 1.f;
                    app_state3.offset_y += 1.f;
                    app_state4.offset_y += 1.f;
                }
            }
            if (key == 82) // r: z tranlation+
            {
                if (pc_index == 0) app_state.offset_z += 1.f;
                if (pc_index == 1) app_state1.offset_z += 1.f;
                if (pc_index == 2) app_state2.offset_z += 1.f;
                if (pc_index == 3) app_state3.offset_z += 1.f;
                if (pc_index == 4) app_state4.offset_z += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z += 1.f;
                    app_state1.offset_z += 1.f;
                    app_state2.offset_z += 1.f;
                    app_state3.offset_z += 1.f;
                    app_state4.offset_z += 1.f;
                }
            }
            if (key == 70) // f: z tranlation-
            {
                if (pc_index == 0) app_state.offset_z -= 1.f;
                if (pc_index == 1) app_state1.offset_z -= 1.f;
                if (pc_index == 2) app_state2.offset_z -= 1.f;
                if (pc_index == 3) app_state3.offset_z -= 1.f;
                if (pc_index == 4) app_state4.offset_z -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z -= 1.f;
                    app_state1.offset_z -= 1.f;
                    app_state2.offset_z -= 1.f;
                    app_state3.offset_z -= 1.f;
                    app_state4.offset_z -= 1.f;
                }
            }
            if (print_state == true)// print app state
            {
                if (pc_index == 0) cout << "cam 1 position(x,y,z):" << app_state.offset_x << "," << app_state.offset_y << "," << app_state.offset_z << endl << "rotation(x,y,z):" << app_state.yaw << "," << app_state.pitch << "," << app_state.roll << endl;
                if (pc_index == 1) cout << "cam 2 position(x,y,z):" << app_state1.offset_x << "," << app_state1.offset_y << "," << app_state1.offset_z << endl << "rotation(x,y,z):" << app_state1.yaw << "," << app_state1.pitch << "," << app_state1.roll << endl;
                if (pc_index == 2) cout << "cam 3 position(x,y,z):" << app_state2.offset_x << "," << app_state2.offset_y << "," << app_state2.offset_z << endl << "rotation(x,y,z):" << app_state2.yaw << "," << app_state2.pitch << "," << app_state2.roll << endl;
                if (pc_index == 3) cout << "cam 4 position(x,y,z):" << app_state3.offset_x << "," << app_state3.offset_y << "," << app_state3.offset_z << endl << "rotation(x,y,z):" << app_state3.yaw << "," << app_state3.pitch << "," << app_state3.roll << endl;
                if (pc_index == 4) cout << "cam 5 position(x,y,z):" << app_state4.offset_x << "," << app_state4.offset_y << "," << app_state4.offset_z << endl << "rotation(x,y,z):" << app_state4.yaw << "," << app_state4.pitch << "," << app_state4.roll << endl;
            }
        };


    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
void update_data(rs2::frame_queue& data, rs2::frame& colorized_depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map)
{
    rs2::frame f;
    if (data.poll_for_frame(&f))  // Try to take the depth and points from the queue
    {
        points = pc.calculate(f); // Generate pointcloud from the depth data
        colorized_depth = color_map.process(f);     // Colorize the depth frame with a color map
        pc.map_to(colorized_depth);         // Map the colored depth to the point cloud
        view.tex.upload(colorized_depth);   //  and upload the texture to the view (without this the view will be B&W)
    }
}

void render_ui(float w, float h, std::vector<filter_options>& filters)
{
    // Flags for displaying ImGui window
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;

    ImGui_ImplGlfw_NewFrame(1);
    ImGui::SetNextWindowSize({ w, h });
    ImGui::Begin("app", nullptr, flags);

    // Using ImGui library to provide slide controllers for adjusting the filter options
    const float offset_x = 2.5*w / 4;
    const int offset_from_checkbox = 120;
    float offset_y = 20;
    float elements_margin =25;
    for (auto& filter : filters)
    {
        // Draw a checkbox per filter to toggle if it should be applied
        ImGui::SetCursorPos({ offset_x, offset_y });
        ImGui::PushStyleColor(ImGuiCol_CheckMark, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
        bool tmp_value = filter.is_enabled;
        ImGui::Checkbox(filter.filter_name.c_str(), &tmp_value);
        filter.is_enabled = tmp_value;
        ImGui::PopStyleColor();

        if (filter.supported_options.size() == 0)
        {
            offset_y += elements_margin;
        }
        // Draw a slider for each of the filter's options
        for (auto& option_slider_pair : filter.supported_options)
        {
            filter_slider_ui& slider = option_slider_pair.second;
            if (slider.render({ offset_x + offset_from_checkbox, offset_y, w / 4 }, filter.is_enabled))
            {
                filter.filter.set_option(option_slider_pair.first, slider.value);
            }
            offset_y += elements_margin;
        }
    }

    ImGui::End();
    ImGui::Render();
}

bool filter_slider_ui::render(const float3& location, bool enabled)
{
    bool value_changed = false;
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 20 / 255.f, 150 / 255.f, 70 / 255.f, 1 });
    ImGui::GetStyle().GrabRounding = 12;
    if (!enabled)
    {
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_Text, { 0.6f, 0.6f, 0.6f, 1 });
    }

    ImGui::PushItemWidth(location.z);
    ImGui::SetCursorPos({ location.x, location.y + 3 });
    ImGui::TextUnformatted(label.c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", description.c_str());

    ImGui::SetCursorPos({ location.x + 170, location.y });

    if (is_int)
    {
        int value_as_int = static_cast<int>(value);
        value_changed = ImGui::SliderInt(("##" + name).c_str(), &value_as_int, static_cast<int>(range.min), static_cast<int>(range.max), "%.0f");
        value = static_cast<float>(value_as_int);
    }
    else
    {
        value_changed = ImGui::SliderFloat(("##" + name).c_str(), &value, range.min, range.max, "%.3f", 1.0f);
    }

    ImGui::PopItemWidth();

    if (!enabled)
    {
        ImGui::PopStyleColor(3);
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
    return value_changed;
}

/**
  Helper function for deciding on int ot float slider
*/
bool filter_slider_ui::is_all_integers(const rs2::option_range& range)
{
    const auto is_integer = [](float f)
    {
        return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
    };

    return is_integer(range.min) && is_integer(range.max) &&
        is_integer(range.def) && is_integer(range.step);
}

/**
Constructor for filter_options, takes a name and a filter.
*/
filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name),
    filter(flt),
    is_enabled(true)
{
    const std::array<rs2_option, 5> possible_filter_options = {
        RS2_OPTION_FILTER_MAGNITUDE,
        RS2_OPTION_FILTER_SMOOTH_ALPHA,
        RS2_OPTION_MIN_DISTANCE,
        RS2_OPTION_MAX_DISTANCE,
        RS2_OPTION_FILTER_SMOOTH_DELTA
    };

    //Go over each filter option and create a slider for it
    for (rs2_option opt : possible_filter_options)
    {
        if (flt.supports(opt))
        {
            rs2::option_range range = flt.get_option_range(opt);
            supported_options[opt].range = range;
            supported_options[opt].value = range.def;
            supported_options[opt].is_int = filter_slider_ui::is_all_integers(range);
            supported_options[opt].description = flt.get_option_description(opt);
            std::string opt_name = flt.get_option_name(opt);
            supported_options[opt].name = name + "_" + opt_name;
            std::string prefix = "Filter ";
            supported_options[opt].label = opt_name;
        }
    }
}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)),
    filter(other.filter),
    supported_options(std::move(other.supported_options)),
    is_enabled(other.is_enabled.load())
{
}
