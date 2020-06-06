
#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include "../cv-helpers.hpp"
#include <string>
#include<sstream>
#include <fstream>              // File IO
#include <iostream>             // Terminal IO         
#include <map>
// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

int main(int argc, char** argv) try
{
    using namespace cv;
    using namespace rs2;

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
    int i = 0;//camera window's index
    for (auto&& pipe : pipelines)
    {
        // Start streaming from Intel RealSense Camera
       // pipeline pipe;
        //rs2::config cfg;
        //Add desired streams to configuration
       // cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
       // auto config = pipe.start(cfg);

        std:: string j = std::to_string(i);
        i += 1;
        const auto window_name = "camera"+j;
        namedWindow(window_name, WINDOW_AUTOSIZE);
        int  camcv_counter = 0;
        while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
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
            imshow(window_name, resize1);
            //imwrite("0-1.png", color);
            //std::cout << "Saved " << std::endl;

           
            if (waitKey(1) >= 0)
            {
                camcv_counter += 1;
                std::string photo_name = camcv_counter + ".png";
                //imwrite(photo_name, color_mat);
                std::cout << "Saved " << std::endl;
                break;
            }
        }
    }

    rs2::colorizer color_map;
    int cam_counter = 0, photo_counter = 12;
    for (auto&& pipe : pipelines)
    {   
        for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
        // Wait for the next set of frames from the camera. Now that autoexposure, etc.
        // has settled, we will write these to disk
        for (auto&& frame : pipe.wait_for_frames())
        {
            // We can only save video frames as pngs, so we skip the rest
            if (auto vf = frame.as<rs2::video_frame>())
            {
                auto stream = frame.get_profile().stream_type();
                // Use the colorizer to get an rgb image for the depth stream
                
                //vf = color_map.process(frame);
                cam_counter += 1;
                
                //imwrite(cam_counter+"-"+photo_counter,)
                // Write images to disk
                std::stringstream png_file;
                png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << cam_counter << "-" << photo_counter << ".png";
                stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                    vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                std::cout << "Saved " << png_file.str() << std::endl;

                // Record per-frame metadata for UVC streams
                std::stringstream csv_file;
                csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
                    << "-metadata.csv";
                metadata_to_csv(vf, csv_file.str());

            }
        }
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

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
