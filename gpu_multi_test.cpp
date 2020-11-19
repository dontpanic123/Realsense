// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <librealsense2-gl/rs_processing_gl.hpp> // Include GPU-Processing API

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
void prepare_matrices(glfw_state& app_state, float width, float height, float* projection, float* view);
bool handle_input(window& app, bool& use_gpu_processing);
// This helper class will be used to keep track of FPS and print instructions:
struct instructions_printer
{
    instructions_printer();
    void print_instructions(window& app, bool use_gpu_processing);

    std::chrono::high_resolution_clock::time_point last_clock;
    int rendered_frames;
    int last_fps;
};

int main(int argc, char* argv[]) try
{
    using namespace std;
    // The following toggle is going to control
    // if we will use CPU or GPU for depth data processing
    bool use_gpu_processing = true;

    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense GPU-Processing Example");

    // Once we have a window, initialize GL module
    // Pass our window to enable sharing of textures between processed frames and the window
    rs2::gl::init_processing(app, use_gpu_processing);
    // Initialize rendering module:
    rs2::gl::init_rendering();

    // Construct an object to manage view state
    glfw_state app_state;
    glfw_state app_state1;
    glfw_state app_state2;
    glfw_state app_state3;
    glfw_state app_state4;
    int pc_index = 0; // In order to distinguish the currently controlled point cloud 
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    texture tex; // For when not using GPU frames, we will need standard texture object
    // to render pointcloud texture (similar to pointcloud example)

    instructions_printer printer;

    // Every iteration the demo will render 3D pointcloud that will be stored in points object:
    rs2::points points;
    // The pointcloud will use colorized depth as a texture:
    rs2::frame  depth_texture;

    // ---- Declare GL processing blocks ----
    rs2::gl::pointcloud pc;         // similar to rs2::pointcloud
    rs2::gl::colorizer  colorizer;  // similar to rs2::colorizer
    rs2::gl::uploader   upload;     // used to explicitly copy frame to the GPU

    // ---- Declare rendering block      ----
    rs2::gl::pointcloud_renderer pc_renderer; // Will handle rendering points object to the screen

    // We will manage two matrices - projection and view
    // to handle mouse input and perspective
    float proj_matrix[16];
    float view_matrix[16];


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
        //cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8  424 240
        cfg.enable_stream(RS2_STREAM_DEPTH,848,480,RS2_FORMAT_Z16);//RS2_FORMAT_Y16   RS2_FORMAT_BGR8
        cfg.enable_device(serial);

        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }
    /* Enable pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    // For this example, we are only interested in depth:
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
    pipe.start(cfg);*/

    while (app) // Application still alive?
    {
        // Any new frames?
        int index = 1; //This index is used to update the displayed point cloud
        for (auto pipe : pipelines)
        {
            rs2::frameset frames;
            if (pipe.poll_for_frames(&frames))
            {
                auto depth = frames.get_depth_frame();

                // Since both colorizer and pointcloud are going to use depth
                // It helps to explicitly upload it to the GPU
                // Otherwise, each block will upload the same depth to GPU separately 
                // [ This step is optional, but can speed things up on some cards ]
                // The result of this step is still a rs2::depth_frame, 
                // but now it is also extendible to rs2::gl::gpu_frame
                depth = upload.process(depth);

                // Apply color map with histogram equalization
                depth_texture = colorizer.colorize(depth);

                // Tell pointcloud object to map to this color frame
                pc.map_to(depth_texture);

                // Generate the pointcloud and texture mappings
                points = pc.calculate(depth);
            }
            if (index == 1) {
                // Populate projection and view matrices based on user input
                prepare_matrices(app_state,
                    app.width(), app.height(),
                    proj_matrix, view_matrix);
            }
            if (index == 2) {
                // Populate projection and view matrices based on user input
                prepare_matrices(app_state1,
                    app.width(), app.height(),
                    proj_matrix, view_matrix);
            }
            if (index == 3) {
                // Populate projection and view matrices based on user input
                prepare_matrices(app_state2,
                    app.width(), app.height(),
                    proj_matrix, view_matrix);
            }
            if (index == 4) {
                // Populate projection and view matrices based on user input
                prepare_matrices(app_state3,
                    app.width(), app.height(),
                    proj_matrix, view_matrix);
            }
            if (index == 5) {
                // Populate projection and view matrices based on user input
                prepare_matrices(app_state4,
                    app.width(), app.height(),
                    proj_matrix, view_matrix);
            }
            // We need to get OpenGL texture ID for pointcloud texture
            uint32_t texture_id = 0;
            // First, check if the frame is already a GPU-frame
            if (auto gpu_frame = depth_texture.as<rs2::gl::gpu_frame>())
            {
                // If it is (and you have passed window for sharing to init_processing
                // you can just get the texture ID of the GPU frame
                texture_id = gpu_frame.get_texture_id(0);
            }
            else
            {
                // Otherwise, we need to upload texture like in all other examples
                tex.upload(depth_texture);
                texture_id = tex.get_gl_handle();
            }

            // Clear screen
            glClearColor(0.2f, 0.2f, 0.2f, 1.f);
            glClear(GL_DEPTH_BUFFER_BIT);

            // We need Depth-Test unless you want pointcloud to intersect with itself
            glEnable(GL_DEPTH_TEST);

            // Set texture to our selected texture ID
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texture_id);

            // Inform pointcloud-renderer of the projection and view matrices:
            pc_renderer.set_matrix(RS2_GL_MATRIX_PROJECTION, proj_matrix);
            pc_renderer.set_matrix(RS2_GL_MATRIX_TRANSFORMATION, view_matrix);

            // If we have something to render, use pointcloud-renderer to do it
            if (points) pc_renderer.process(points);

            // Disable texturing
            glDisable(GL_TEXTURE_2D);

            // Print textual information
            printer.print_instructions(app, use_gpu_processing);

            // Handle user input and check if reset is required
            if (handle_input(app, use_gpu_processing))
            {
                // First, shutdown processing and rendering
                rs2::gl::shutdown_rendering();
                rs2::gl::shutdown_processing();

                // Next, reinitialize processing and rendering with new settings
                rs2::gl::init_processing(app, use_gpu_processing);
                rs2::gl::init_rendering();
            }
            index += 1;
        }
        bool print_state = false;// When this variable is true, output the current position status
        app.on_key_release = [&](int key)
        {
            cout << key << endl;
            if (key == 32) // Escape, move all the pointcloud to a defaut position.
            {
                app_state.yaw = -3;
                app_state.pitch = -31;
                app_state.roll = 0;
                app_state.offset_x = -22;
                app_state.offset_y = -18;
                app_state.offset_z = 12;

                app_state1.yaw = 2;
                app_state1.pitch = -29;
                app_state1.roll = 1;
                app_state1.offset_x = 1.0;
                app_state1.offset_y = -16.0;
                app_state1.offset_z = 14.0;

                app_state2.yaw = 0;
                app_state2.pitch = -7;
                app_state2.roll = -2;
                app_state2.offset_x = -11.0;
                app_state2.offset_y = -4.0;
                app_state2.offset_z = -6.0;

                app_state3.yaw = -1;
                app_state3.pitch = -11;
                app_state3.roll = 183;
                app_state3.offset_x = -10.0;
                app_state3.offset_y = 10.0;
                app_state3.offset_z = 30.0;

                app_state4.yaw = 0;
                app_state4.pitch = 0;
                app_state4.roll = 0;
                app_state4.offset_x = -10;
                app_state4.offset_y = 0;
                app_state4.offset_z = 7;

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
            if (key == 90) // y: x translation+
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
            if (key == 88) // x: x translation-
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
            if (key == 67) // c: y translation+
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
            if (key == 86) // v: y translation+
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

instructions_printer::instructions_printer()
{
    // Initialize running clock to keep track of time (for FPS calculation)
    auto last_clock = std::chrono::high_resolution_clock::now();
    int rendered_frames = 0;    // Counts number of frames since last update
    int last_fps = 0;           // Stores last calculated FPS
    glfwSwapInterval(0);        // This is functionally disabling V-Sync, 
                                // allowing application FPS to go beyond monitor refresh rate
}

void instructions_printer::print_instructions(window& app, bool use_gpu_processing)
{
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, app.width(), app.height(), 0, -1, +1);

    glColor3f(1.f, 1.f, 1.f);
    std::stringstream ss;
    ss << "Performing Point-Cloud and Histogram-Equalization (Colorize) on the " << (use_gpu_processing ? "GPU" : "CPU");
    draw_text(20, 20, ss.str().c_str());

    ss.str("");
    ss << "( Press " << (use_gpu_processing ? "[C] - Switch to processing on the CPU )" : "[G] - Switch to processing on the GPU )");
    draw_text(20, 36, ss.str().c_str());

    ss.str("");
    ss << "This demo is being rendered at " << last_fps << " frames per second, ~" << (1000 / (last_fps + 1)) << "ms for single rendered frame";
    draw_text(20, 52, ss.str().c_str());

    rendered_frames++;

    auto since_last_clock = std::chrono::high_resolution_clock::now() - last_clock;
    auto sec_elapsed = std::chrono::duration_cast<std::chrono::seconds>(since_last_clock).count();
    if (sec_elapsed > 0)
    {
        last_clock = std::chrono::high_resolution_clock::now();
        last_fps = rendered_frames;
        rendered_frames = 0;
    }
}

bool handle_input(window& app, bool& use_gpu_processing)
{
    bool reset = false;

    if (glfwGetKey(app, GLFW_KEY_C) == GLFW_PRESS)
    {
        reset = use_gpu_processing;
        use_gpu_processing = false;
    }
    if (glfwGetKey(app, GLFW_KEY_G) == GLFW_PRESS)
    {
        reset = !use_gpu_processing;
        use_gpu_processing = true;
    }

    return reset;
}

void prepare_matrices(glfw_state& app_state, float width, float height, float* projection, float* view)
{
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
    glTranslatef(0, 0, 0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);
    glGetFloatv(GL_MODELVIEW_MATRIX, view);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}
