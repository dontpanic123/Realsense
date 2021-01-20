// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "parallel_example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <thread>
#include <mutex>

std::mutex mu;
void shared_print(std::string msg, int id)
{
    mu.lock();
    std::cout << msg << id << std::endl;
    mu.unlock();
}
// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
//using namespace std;

rs2::pointcloud pc;
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;

glfw_state  app_state;
glfw_state  app_state1;
glfw_state  app_state2;
glfw_state  app_state3;
glfw_state  app_state4;


rs2::video_frame pc_thread(rs2::pipeline pipe, glfw_state app_state)
{

    auto frames = pipe.wait_for_frames();
    auto color = frames.get_color_frame();
    // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    glViewport(0, 0, 1280, 720);
    //app_state.tex.upload(color);
    draw_pointcloud(1280, 720, app_state, points);
    //shared_print("pc_thread ", i);
    return color;
}

void pc_thread2(rs2::pipeline& pipe, glfw_state& app_state)
{
    //std::lock_guard<std::mutex> g1(mu);
    window app(1280, 720, "RealSense Pointcloud Example");
   
    register_glfw_callbacks(app, app_state);

    while(app){
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        glViewport(0, 0, 1280, 720);
        app_state.tex.upload(color);
        
        draw_pointcloud(1280, 720, app_state, points);
        //shared_print("pc_thread ", i);
        //return color;
    }
}

int main(int argc, char* argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    register_glfw_callbacks(app, app_state);

    // ImGui_ImplGlfw_Init(app, false);
    rs2::context                          ctx;
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
    {
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::cout << "serial_number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

    }
    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8  424 240
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8
        cfg.enable_device(serial);

        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }
    // Construct an object to manage view state

    //std::vector<glfw_state> app_states;


   


    int pc_index = 0; // In order to distinguish the currently controlled point cloud  
    // register callbacks to allow manipulation of the pointcloud
    
    
    std::thread t1( pc_thread2, std::ref(pipelines[0]), std::ref(app_state));
    std::cout << "thread t1 is presenting : " << t1.get_id() << std::endl;

    std::thread t2(pc_thread2, std::ref(pipelines[1]), std::ref(app_state1));
    std::cout << "thread t2 is presenting : " << t2.get_id() << std::endl;
   
    std::cout<<"Maximum Threads: "<<std::thread::hardware_concurrency()<<std::endl;

    //while (app) // Application still alive?
    //{
        // Collect the new frames from all the connected devices
        //std::vector<rs2::frame> new_frames;


        draw_text(10, 20, "camera 1 upper_left    serial_number: 745412070908");
        draw_text(10, 40, "camera 2 upper_right   serial_number: 947522071890");
        draw_text(10, 60, "camera 3 driver_persp  serial_number: 936322070874");
        draw_text(10, 80, "camera 4 forks_persp   serial_number: 936322071095");
        draw_text(10, 100, "camera 5 basis_persp   serial_number: 923322072240");
        draw_text(10, 140, "press the nember keys form one to five to select the point cloud you want to manipulate");
        draw_text(10, 160, "press the nember keys 0 to select the all point clouds to manipulate");
        draw_text(10, 200, "rotation           x:w,s       y:a,d       z:q,e");
        draw_text(10, 220, "translation:       x:y,x       y:c,v       z:r,f");

        
    

         //std::thread t2(pc_thread, pipelines[1], app_state1, 2);
         //t2.join();
        // auto color1 = pc_thread(pipelines[1], app_state1);


         //app_state.tex.upload(color);
        // app_state1.tex.upload(color1);

                 

        //app_state.tex.upload(pc_thread(pipelines[0], app_state, 1));
        //app_state1.tex.upload(pc_thread(pipelines[1], app_state1, 2));
        //app_state2.tex.upload(pc_thread(pipelines[2], app_state2,3));
        //app_state3.tex.upload(pc_thread(pipelines[3], app_state3,4));

        //pc_thread(pipelines[0], app_states[0]);


        /*int index = 1; //This index is used to update the displayed point cloud
        for (auto pipe : pipelines)
        {
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)  color = frames.get_infrared_frame();

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
        }*/

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

        //bool print_state = false;// When this variable is true, output the current position status
        //app.on_key_release = [&](int key)
        //{
        //    std::cout << key << std::endl;
        //    if (key == 32) // Escape, move all the pointcloud to a defaut position.
        //    {
        //        app_state.yaw = -6;
        //        app_state.pitch = -43;
        //        app_state.roll = 0;
        //        app_state.offset_x = -20;
        //        app_state.offset_y = -24;
        //        app_state.offset_z = 15;

        //        app_state1.yaw = 2;
        //        app_state1.pitch = -37;
        //        app_state1.roll = 1;
        //        app_state1.offset_x = 0.0;
        //        app_state1.offset_y = -21.0;
        //        app_state1.offset_z = 12.0;

        //        app_state2.yaw = 0;
        //        app_state2.pitch = -7;
        //        app_state2.roll = -2;
        //        app_state2.offset_x = -11.0;
        //        app_state2.offset_y = -4.0;
        //        app_state2.offset_z = -6.0;

        //        app_state3.yaw = -1;
        //        app_state3.pitch = 7;
        //        app_state3.roll = 183;
        //        app_state3.offset_x = -10.0;
        //        app_state3.offset_y = 19.0;
        //        app_state3.offset_z = 41.0;

        //        app_state4.yaw = 0;
        //        app_state4.pitch = 0;
        //        app_state4.roll = 0;
        //        app_state4.offset_x = -10;
        //        app_state4.offset_y = 0;
        //        app_state4.offset_z = 7;




        //        print_state = true;
        //    }

        //    if (key == 79) // o, move all the pointcloud to opencv position.
        //    {
        //        std::cout << "OpenCV position" << std::endl;
        //        app_state.yaw = -6.1502;
        //        app_state.pitch = -38.8882;
        //        app_state.roll = -0.5021;
        //        app_state.offset_x = -22.0178;
        //        app_state.offset_y = -22.7578;
        //        app_state.offset_z = 17.4156;

        //        app_state1.yaw = 0.6730;
        //        app_state1.pitch = -37.6007;
        //        app_state1.roll = -0.6624;
        //        app_state1.offset_x = 1.5844;
        //        app_state1.offset_y = -23.08;
        //        app_state1.offset_z = 18.1111;

        //        app_state2.yaw = -4, 9777;
        //        app_state2.pitch = -23, 7910;
        //        app_state2.roll = -3, 4316;
        //        app_state2.offset_x = -10.5867;
        //        app_state2.offset_y = -22.3267;
        //        app_state2.offset_z = -12.6178;

        //        app_state3.yaw = -0.2638;
        //        app_state3.pitch = 9.0734;
        //        app_state3.roll = -178.6431;
        //        app_state3.offset_x = -10.3667;
        //        app_state3.offset_y = 24.65;
        //        app_state3.offset_z = 45.706;

        //        app_state4.yaw = 0;
        //        app_state4.pitch = 0;
        //        app_state4.roll = 0;
        //        app_state4.offset_x = -10;
        //        app_state4.offset_y = 0;
        //        app_state4.offset_z = 7;




        //        print_state = true;
        //    }

        //    if (key == 75) // K, move all the pointcloud to kalibr position.
        //    {
        //        std::cout << "Kalibr position" << std::endl;
        //        app_state.yaw = 6.2785;
        //        app_state.pitch = -38, 4940;
        //        app_state.roll = 0.3527;
        //        app_state.offset_x = -34.2958;
        //        app_state.offset_y = -22.325;
        //        app_state.offset_z = 15.4583;

        //        app_state1.yaw = 3.9538;
        //        app_state1.pitch = -37.0704;
        //        app_state1.roll = -0.2495;
        //        app_state1.offset_x = 0.7421;
        //        app_state1.offset_y = -22.5875;
        //        app_state1.offset_z = 13.3770;

        //        app_state2.yaw = -0.9249;
        //        app_state2.pitch = -19.1966;
        //        app_state2.roll = -3.0028;
        //        app_state2.offset_x = -11.2354;
        //        app_state2.offset_y = -17.5375;
        //        app_state2.offset_z = -13.2917;

        //        app_state3.yaw = -7.3435;
        //        app_state3.pitch = 0.6026;
        //        app_state3.roll = 178.2782;
        //        app_state3.offset_x = -3.6354;
        //        app_state3.offset_y = 13.0667;
        //        app_state3.offset_z = 37.1167;

        //        app_state4.yaw = 0;
        //        app_state4.pitch = 0;
        //        app_state4.roll = 0;
        //        app_state4.offset_x = -10;
        //        app_state4.offset_y = 0;
        //        app_state4.offset_z = 7;




        //        print_state = true;
        //    }


        //    if (key == 48) // 0
        //    {


        //        pc_index = 8;
        //        // app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        //    }
        //    if (key == 49) // 1
        //    {
        //        print_state = true;
        //        register_glfw_callbacks(app, app_state);
        //        pc_index = 0;
        //        // app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        //    }
        //    if (key == 50) // 2
        //    {
        //        print_state = true;
        //        register_glfw_callbacks(app, app_state1);
        //        pc_index = 1;
        //        //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
        //    }
        //    if (key == 51) // 3
        //    {
        //        print_state = true;
        //        register_glfw_callbacks(app, app_state2);
        //        pc_index = 2;
        //        //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
        //    }
        //    if (key == 52) // 4
        //    {
        //        print_state = true;
        //        register_glfw_callbacks(app, app_state3);
        //        pc_index = 3;
        //        //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
        //    }
        //    if (key == 53) // 5
        //    {
        //        print_state = true;
        //        register_glfw_callbacks(app, app_state4);
        //        pc_index = 4;
        //        //app_state1.yaw = app_state1.pitch = 0; app_state1.offset_x = app_state1.offset_y = 0.0;
        //    }

        //    if (key == 87) // w  y rotation +
        //    {
        //        if (pc_index == 0) app_state.pitch += 1;
        //        if (pc_index == 1) app_state1.pitch += 1;
        //        if (pc_index == 2) app_state2.pitch += 1;
        //        if (pc_index == 3) app_state3.pitch += 1;
        //        if (pc_index == 4) app_state4.pitch += 1;


        //        if (pc_index == 8)
        //        {
        //            app_state.pitch += 1;
        //            app_state1.pitch += 1;
        //            app_state2.pitch += 1;
        //            app_state3.pitch += 1;
        //            app_state4.pitch += 1;

        //        }
        //    }
        //    if (key == 83) // s  y rotation -
        //    {
        //        if (pc_index == 0) app_state.pitch -= 1;
        //        if (pc_index == 1) app_state1.pitch -= 1;
        //        if (pc_index == 2) app_state2.pitch -= 1;
        //        if (pc_index == 3) app_state3.pitch -= 1;
        //        if (pc_index == 4) app_state4.pitch -= 1;

        //        if (pc_index == 8)
        //        {
        //            app_state.pitch -= 1;
        //            app_state1.pitch -= 1;
        //            app_state2.pitch -= 1;
        //            app_state3.pitch -= 1;
        //            app_state4.pitch -= 1;
        //        }
        //    }
        //    if (key == 65) // a : x rotation -
        //    {
        //        if (pc_index == 0) app_state.yaw -= 1;
        //        if (pc_index == 1) app_state1.yaw -= 1;
        //        if (pc_index == 2) app_state2.yaw -= 1;
        //        if (pc_index == 3) app_state3.yaw -= 1;
        //        if (pc_index == 4) app_state4.yaw -= 1;

        //        if (pc_index == 8)
        //        {
        //            app_state.yaw -= 1;
        //            app_state1.yaw -= 1;
        //            app_state2.yaw -= 1;
        //            app_state3.yaw -= 1;
        //            app_state4.yaw -= 1;
        //        }
        //    }
        //    if (key == 68) // d : x rotation +
        //    {
        //        if (pc_index == 0) app_state.yaw += 1;
        //        if (pc_index == 1) app_state1.yaw += 1;
        //        if (pc_index == 2) app_state2.yaw += 1;
        //        if (pc_index == 3) app_state3.yaw += 1;
        //        if (pc_index == 4) app_state4.yaw += 1;


        //        if (pc_index == 8)
        //        {
        //            app_state.yaw += 1;
        //            app_state1.yaw += 1;
        //            app_state2.yaw += 1;
        //            app_state3.yaw += 1;
        //            app_state4.yaw += 1;
        //        }
        //    }
        //    if (key == 81) // q : z rotation -
        //    {
        //        if (pc_index == 0) app_state.roll -= 1;
        //        if (pc_index == 1) app_state1.roll -= 1;
        //        if (pc_index == 2) app_state2.roll -= 1;
        //        if (pc_index == 3) app_state1.roll -= 1;
        //        if (pc_index == 4) app_state2.roll -= 1;


        //        if (pc_index == 8)
        //        {
        //            app_state.roll -= 1;
        //            app_state1.roll -= 1;
        //            app_state2.roll -= 1;
        //            app_state3.roll -= 1;
        //            app_state4.roll -= 1;
        //        }
        //    }
        //    if (key == 69) // e : z rotation +
        //    {
        //        if (pc_index == 0) app_state.roll += 1;
        //        if (pc_index == 1) app_state1.roll += 1;
        //        if (pc_index == 2) app_state2.roll += 1;
        //        if (pc_index == 1) app_state3.roll += 1;
        //        if (pc_index == 2) app_state4.roll += 1;

        //        if (pc_index == 8)
        //        {
        //            app_state.roll += 1;
        //            app_state1.roll += 1;
        //            app_state2.roll += 1;
        //            app_state3.roll += 1;
        //            app_state4.roll += 1;
        //        }
        //    }
        //    if (key == 90) // y: x translation-
        //    {
        //        if (pc_index == 0) app_state.offset_x -= 1.f;
        //        if (pc_index == 1) app_state1.offset_x -= 1.f;
        //        if (pc_index == 2) app_state2.offset_x -= 1.f;
        //        if (pc_index == 3) app_state3.offset_x -= 1.f;
        //        if (pc_index == 4) app_state4.offset_x -= 1.f;

        //        if (pc_index == 8)
        //        {
        //            app_state.offset_x -= 1.f;
        //            app_state1.offset_x -= 1.f;
        //            app_state2.offset_x -= 1.f;
        //            app_state3.offset_x -= 1.f;
        //            app_state4.offset_x -= 1.f;
        //        }
        //    }
        //    if (key == 88) // x: x translation+
        //    {
        //        if (pc_index == 0) app_state.offset_x += 1.f;
        //        if (pc_index == 1) app_state1.offset_x += 1.f;
        //        if (pc_index == 2) app_state2.offset_x += 1.f;
        //        if (pc_index == 3) app_state3.offset_x += 1.f;
        //        if (pc_index == 4) app_state4.offset_x += 1.f;

        //        if (pc_index == 8)
        //        {
        //            app_state.offset_x += 1.f;
        //            app_state1.offset_x += 1.f;
        //            app_state2.offset_x += 1.f;
        //            app_state3.offset_x += 1.f;
        //            app_state4.offset_x += 1.f;
        //        }
        //    }
        //    if (key == 67) // c: y translation-
        //    {
        //        if (pc_index == 0) app_state.offset_y -= 1.f;
        //        if (pc_index == 1) app_state1.offset_y -= 1.f;
        //        if (pc_index == 2) app_state2.offset_y -= 1.f;
        //        if (pc_index == 3) app_state3.offset_y -= 1.f;
        //        if (pc_index == 4) app_state4.offset_y -= 1.f;

        //        if (pc_index == 8)
        //        {
        //            app_state.offset_y -= 1.f;
        //            app_state1.offset_y -= 1.f;
        //            app_state2.offset_y -= 1.f;
        //            app_state3.offset_y -= 1.f;
        //            app_state4.offset_y -= 1.f;
        //        }
        //    }
        //    if (key == 86) // v: y translation+
        //    {
        //        if (pc_index == 0) app_state.offset_y += 1.f;
        //        if (pc_index == 1) app_state1.offset_y += 1.f;
        //        if (pc_index == 2) app_state2.offset_y += 1.f;
        //        if (pc_index == 3) app_state3.offset_y += 1.f;
        //        if (pc_index == 4) app_state4.offset_y += 1.f;


        //        if (pc_index == 8)
        //        {
        //            app_state.offset_y += 1.f;
        //            app_state1.offset_y += 1.f;
        //            app_state2.offset_y += 1.f;
        //            app_state3.offset_y += 1.f;
        //            app_state4.offset_y += 1.f;
        //        }
        //    }
        //    if (key == 82) // r: z tranlation+
        //    {
        //        if (pc_index == 0) app_state.offset_z += 1.f;
        //        if (pc_index == 1) app_state1.offset_z += 1.f;
        //        if (pc_index == 2) app_state2.offset_z += 1.f;
        //        if (pc_index == 3) app_state3.offset_z += 1.f;
        //        if (pc_index == 4) app_state4.offset_z += 1.f;

        //        if (pc_index == 8)
        //        {
        //            app_state.offset_z += 1.f;
        //            app_state1.offset_z += 1.f;
        //            app_state2.offset_z += 1.f;
        //            app_state3.offset_z += 1.f;
        //            app_state4.offset_z += 1.f;
        //        }
        //    }
        //    if (key == 70) // f: z tranlation-
        //    {
        //        if (pc_index == 0) app_state.offset_z -= 1.f;
        //        if (pc_index == 1) app_state1.offset_z -= 1.f;
        //        if (pc_index == 2) app_state2.offset_z -= 1.f;
        //        if (pc_index == 3) app_state3.offset_z -= 1.f;
        //        if (pc_index == 4) app_state4.offset_z -= 1.f;

        //        if (pc_index == 8)
        //        {
        //            app_state.offset_z -= 1.f;
        //            app_state1.offset_z -= 1.f;
        //            app_state2.offset_z -= 1.f;
        //            app_state3.offset_z -= 1.f;
        //            app_state4.offset_z -= 1.f;
        //        }
        //    }
        //    if (print_state == true)// print app state
        //    {
        //        if (pc_index == 0) std::cout << "cam 1 position(x,y,z):" << app_state.offset_x << "," << app_state.offset_y << "," << app_state.offset_z << std::endl << "rotation(x,y,z):" << app_state.yaw << "," << app_state.pitch << "," << app_state.roll << std::endl;
        //        if (pc_index == 1) std::cout << "cam 2 position(x,y,z):" << app_state1.offset_x << "," << app_state1.offset_y << "," << app_state1.offset_z << std::endl << "rotation(x,y,z):" << app_state1.yaw << "," << app_state1.pitch << "," << app_state1.roll << std::endl;
        //        if (pc_index == 2) std::cout << "cam 3 position(x,y,z):" << app_state2.offset_x << "," << app_state2.offset_y << "," << app_state2.offset_z << std::endl << "rotation(x,y,z):" << app_state2.yaw << "," << app_state2.pitch << "," << app_state2.roll << std::endl;
        //        if (pc_index == 3) std::cout << "cam 4 position(x,y,z):" << app_state3.offset_x << "," << app_state3.offset_y << "," << app_state3.offset_z << std::endl << "rotation(x,y,z):" << app_state3.yaw << "," << app_state3.pitch << "," << app_state3.roll << std::endl;
        //        if (pc_index == 4) std::cout << "cam 5 position(x,y,z):" << app_state4.offset_x << "," << app_state4.offset_y << "," << app_state4.offset_z << std::endl << "rotation(x,y,z):" << app_state4.yaw << "," << app_state4.pitch << "," << app_state4.roll << std::endl;
        //    }
        //};

       
    /*}*/

    t1.join();
    //t2.join();
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
