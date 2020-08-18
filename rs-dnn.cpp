#include <librealsense2/rs.hpp>
#include "../cv-helpers.hpp"
#include <string>
#include<sstream>
#include <fstream>              // File IO
#include <map>


int main(int argc, char** argv) try
{
    using namespace cv;
    using namespace rs2;

   
    
    std::cout << "Press 's' key to save photos. " << std::endl;
    
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;
    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);

        // Map from each device's serial number to a different colorizer
        colorizers[dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = rs2::colorizer(); 
    }


    //Detect the number of connected cameras and generate a sufficient number of windows
    const auto window_name = "camera0";
    for (int w = 0; w != pipelines.size(); w++)
    {
        std::string j = std::to_string(w);
        const auto window_name = "camera" + j;
        namedWindow(window_name, WINDOW_AUTOSIZE);

    }

    int  camcv_counter = 0;

    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        int w = 0;
        for (auto&& pipe : pipelines)
        {
            // Wait for the next set of frames
            auto data = pipe.wait_for_frames();
            // Make sure the frames are spatially aligned

            auto color_frame = data.get_color_frame();
            Mat color(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            Mat resize1;
            // If we only received new depth frame, 
            // but the color did not update, continue
            static int last_frame_number = 0;
            if (color_frame.get_frame_number() == last_frame_number) continue;
            last_frame_number = color_frame.get_frame_number();

            // Convert RealSense frame to OpenCV matrix:
            auto color_mat = frame_to_mat(color_frame);
            resize(color, resize1, Size(color.cols / 2, color.rows / 2), 0, 0, INTER_LINEAR);

            //send photo to the right window
            std::string j = std::to_string(w);
            const auto window_name = "camera" + j;
            imshow(window_name, resize1);
            w += 1;

        }

        if (waitKey(1) == 115)
        {   
                std::cout << "Please enter the photo counter: ";
                int photo_counter;
                std::cin >> photo_counter;


                int cam_counter = 0;
                for (auto&& pipe : pipelines)
                {
                    // Wait for the next set of frames
                    auto data = pipe.wait_for_frames();
                    // Make sure the frames are spatially aligned

                    auto color_frame = data.get_color_frame();
                    Mat color(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
                    // If we only received new depth frame, 
                    // but the color did not update, continue
                    static int last_frame_number = 0;
                    if (color_frame.get_frame_number() == last_frame_number) continue;
                    last_frame_number = color_frame.get_frame_number();

                    // Convert RealSense frame to OpenCV matrix:
                    auto color_mat = frame_to_mat(color_frame);

                    std::string photo_name = std::to_string(cam_counter) + "-" + std::to_string(photo_counter) + ".png";
                    imwrite(photo_name, color_mat);
                    std::cout << photo_name << "-Saved " << std::endl;
                    cam_counter += 1;
                }

            
        }
    }


   
    waitKey();
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

