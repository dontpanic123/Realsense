// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example_usb.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
using namespace std;

int main(int argc, char* argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");


    rs2::context                          ctx;
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
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
    //std::vector<glfw_state> app_states;
    int pc_index = 0; // In order to distinguish the currently controlled point cloud  
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    while (app) // Application still alive?
    {
        // Collect the new frames from all the connected devices
        //std::vector<rs2::frame> new_frames;

        draw_text(10, 20, "camera 1 serial_number: 936322070874");
        draw_text(10, 40, "camera 2 serial_number: 936322071095");
        draw_text(10, 60, "camera 3 serial_number: 936322071095");
        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());
       
        int index = 1; //This index is used to update the displayed point cloud
        for (auto pipe : pipelines)
        {


            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            pc.map_to(color);

            auto depth = frames.get_depth_frame();

            


            // Generate the pointcloud and texture mappings
            points = pc.calculate(depth);

            // Upload the color frame to OpenGL
            glDepthFunc(GL_LESS);

            // register callbacks to allow manipulation of the pointcloud
         //   register_glfw_callbacks(app, *app_state);


            if (index == 1) {
                glViewport(20, 20, 1280, 720);
                app_state.tex.upload(color);
                draw_pointcloud(1280, 720, app_state, points);

            }

            if (index == 2) {
                //glViewport(0, app.height() / 2, app.width() / 2, app.height() / 2);
                app_state1.tex.upload(color);
                draw_pointcloud(1280, 720, app_state1, points);

            }

            if (index == 3) {
                //glViewport(0, app.height() / 2, app.width() / 2, app.height() / 2);
                app_state2.tex.upload(color);
                draw_pointcloud(1280, 720, app_state2, points);

            }
            if (index == 4) {
                //glViewport(0, app.height() / 2, app.width() / 2, app.height() / 2);
                app_state3.tex.upload(color);
                draw_pointcloud(1280, 720, app_state3, points);

            }

            index++;
        }

        bool print_state = false;// When this variable is true, output the current position status
        app.on_key_release = [&](int key)
        {

            cout << key << endl;
            if (key == 32) // Escape, move all the pointcloud to a defaut position.
            {
                app_state.yaw = -2;
                app_state.pitch = 1;
                app_state.roll = 0;
                app_state.offset_x = -10;
                app_state.offset_y = -10;
                app_state.offset_z = 0.0;

                app_state1.yaw = -16;
                app_state1.pitch = 0;
                app_state1.roll = -2;
                app_state1.offset_x = -17.0;
                app_state1.offset_y = -18.0;
                app_state1.offset_z = 2.0;

                app_state2.yaw = -16;
                app_state2.pitch = 0;
                app_state2.roll = -2;
                app_state2.offset_x = -17.0;
                app_state2.offset_y = -18.0;
                app_state2.offset_z = 2.0;


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

           
            if (key == 87) // w  y rotation +
            {
                if (pc_index == 0) app_state.pitch += 1;
                if (pc_index == 1) app_state1.pitch += 1;
                if (pc_index == 2) app_state2.pitch += 1;

                if (pc_index == 8)
                {
                    app_state.pitch += 1;
                    app_state1.pitch += 1;
                    app_state2.pitch += 1;
                }
            }
            if (key == 83) // s  y rotation -
            {
                if (pc_index == 0) app_state.pitch -= 1;
                if (pc_index == 1) app_state1.pitch -= 1;
                if (pc_index == 2) app_state2.pitch -= 1;

                if (pc_index == 8)
                {
                    app_state.pitch -= 1;
                    app_state1.pitch -= 1;
                    app_state2.pitch -= 1;
                }
            }
            if (key == 65) // a : x rotation -
            {
                if (pc_index == 0) app_state.yaw -= 1;
                if (pc_index == 1) app_state1.yaw -= 1;
                if (pc_index == 2) app_state2.yaw -= 1;

                if (pc_index == 8)
                {
                    app_state.yaw -= 1;
                    app_state1.yaw -= 1;
                    app_state2.yaw -= 1;
                }
            }
            if (key == 68) // d : x rotation +
            {
                if (pc_index == 0) app_state.yaw += 1;
                if (pc_index == 1) app_state1.yaw += 1;
                if (pc_index == 2) app_state2.yaw += 1;

                if (pc_index == 8)
                {
                    app_state.yaw += 1;
                    app_state1.yaw += 1;
                    app_state2.yaw += 1;
                }
            }
            if (key == 81) // q : z rotation -
            {
                if (pc_index == 0) app_state.roll -= 1;
                if (pc_index == 1) app_state1.roll -= 1;
                if (pc_index == 2) app_state2.roll -= 1;

                if (pc_index == 8)
                {
                    app_state.roll -= 1;
                    app_state1.roll -= 1;
                    app_state2.roll -= 1;
                }
            }
            if (key == 69) // e : z rotation +
            {
                if (pc_index == 0) app_state.roll += 1;
                if (pc_index == 1) app_state1.roll += 1;
                if (pc_index == 2) app_state2.roll += 1;

                if (pc_index == 8)
                {
                    app_state.roll += 1;
                    app_state1.roll += 1;
                    app_state2.roll += 1;
                }
            }
            if (key == 90) // y: x translation+
            {
                if (pc_index == 0) app_state.offset_x += 1.f;
                if (pc_index == 1) app_state1.offset_x += 1.f;
                if (pc_index == 2) app_state2.offset_x += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x += 1.f;
                    app_state1.offset_x += 1.f;
                    app_state2.offset_x += 1.f;
                }
            }
            if (key == 88) // x: x translation-
            {
                if (pc_index == 0) app_state.offset_x -= 1.f;
                if (pc_index == 1) app_state1.offset_x -= 1.f;
                if (pc_index == 2) app_state2.offset_x -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x -= 1.f;
                    app_state1.offset_x -= 1.f;
                    app_state2.offset_x -= 1.f;
                }
            }
            if (key == 67) // c: y translation+
            {
                if (pc_index == 0) app_state.offset_y += 1.f;
                if (pc_index == 1) app_state1.offset_y += 1.f;
                if (pc_index == 2) app_state2.offset_y += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_y += 1.f;
                    app_state1.offset_y += 1.f;
                    app_state2.offset_y += 1.f;
                }
            }
            if (key == 86) // v: y translation+
            {
                if (pc_index == 0) app_state.offset_y -= 1.f;
                if (pc_index == 1) app_state1.offset_y -= 1.f;
                if (pc_index == 2) app_state2.offset_y -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_y -= 1.f;
                    app_state1.offset_y -= 1.f;
                    app_state2.offset_y -= 1.f;
                }
            }
            if (key == 82) // r: z tranlation+
            {
                if (pc_index == 0) app_state.offset_z += 1.f;
                if (pc_index == 1) app_state1.offset_z += 1.f;
                if (pc_index == 2) app_state2.offset_z += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z += 1.f;
                    app_state1.offset_z += 1.f;
                    app_state2.offset_z += 1.f;
                }
            }
            if (key == 70) // f: z tranlation-
            {
                if (pc_index == 0) app_state.offset_z -= 1.f;
                if (pc_index == 1) app_state1.offset_z -= 1.f;
                if (pc_index == 2) app_state2.offset_z -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z -= 1.f;
                    app_state1.offset_z -= 1.f;
                    app_state2.offset_z -= 1.f;
                }
            }
            if (print_state == true)// print app state
            {
                if (pc_index == 0) cout << "cam 1 position(x,y,z):" << app_state.offset_x << "," << app_state.offset_y << "," << app_state.offset_z << endl << "rotation(x,y,z):" << app_state.yaw << "," << app_state.pitch << "," << app_state.roll << endl;
                if (pc_index == 1) cout << "cam 2 position(x,y,z):" << app_state1.offset_x << "," << app_state1.offset_y << "," << app_state1.offset_z << endl << "rotation(x,y,z):" << app_state1.yaw << "," << app_state1.pitch << "," << app_state1.roll << endl;
                if (pc_index == 2) cout << "cam 3 position(x,y,z):" << app_state2.offset_x << "," << app_state2.offset_y << "," << app_state2.offset_z << endl << "rotation(x,y,z):" << app_state2.yaw << "," << app_state2.pitch << "," << app_state2.roll << endl;
            
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
