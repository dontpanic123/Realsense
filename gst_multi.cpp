#include <gst/gst.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "exampler.hpp"          // Include short list of convenience functions for rendering
#include <algorithm>            // std::min, std::max
#include <convert.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <librealsense2/hpp/rs_internal.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <int-rs-splash.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define CHUNK_SIZE 7372800
using namespace std;


const int W = 640;
const int H = 480;
const int BPP = 2;
const int BPP1 = 8;

struct synthetic_frame
{
    int x, y, bpp;
    std::vector<uint8_t> frame;
};

class custom_frame_source
{
public:
    custom_frame_source()
    {
        depth_frame.x = W;
        depth_frame.y = H;
        depth_frame.bpp = BPP;

        color_frame.x = W;
        color_frame.y = H;
        color_frame.bpp = BPP1;
    }

    synthetic_frame& get_synthetic_texture(int r,int g, int b)
    {
        auto realsense_logo = stbi_load_from_memory(splash, (int)splash_size, &color_frame.x, &color_frame.y, &color_frame.bpp, false);

        std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y * color_frame.bpp, 0);

        memcpy(pixels_color.data(), realsense_logo, color_frame.x * color_frame.y * 4);

        for (auto i = 0; i < color_frame.y; i++)
            for (auto j = 0; j < color_frame.x * 4; j += 4)
            {
                if (pixels_color.data()[i * color_frame.x * 4 + j] == 0)
                {
                    pixels_color.data()[i * color_frame.x * 4 + j] = r;
                    pixels_color.data()[i * color_frame.x * 4 + j + 1] = g;
                    pixels_color.data()[i * color_frame.x * 4 + j + 2] = b;

                }
            }
        color_frame.frame = std::move(pixels_color);

        return color_frame;
    }

    synthetic_frame& get_gst_color(GstAppSink* sink1)
    {
        //auto realsense_logo = stbi_load_from_memory(splash, (int)splash_size, &color_frame.x, &color_frame.y, &color_frame.bpp, false);

        std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y*2 , 0);
        color_frame.frame = std::move(pixels_color);
        //std::cout << "W1:" << color_frame.x << std::endl << "H1:" << color_frame.y << std::endl << "BPP1:" << color_frame.bpp << std::endl;
        //memcpy(pixels_color.data(), realsense_logo, color_frame.x * color_frame.y*4);

        GstSample* sample = gst_app_sink_pull_sample(sink1);

        if (sample) 
        {
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            // g_print(" get gst color ! ");

            // memcpy(pixels_color.data(), (char*)map2.data, color_frame.x * color_frame.y * 4);

            for (auto i = 0; i < color_frame.y; i++)
            {
                for (auto j = 0; j < color_frame.x ; j++)
                {
                    ((uint16_t*)color_frame.frame.data())[i * color_frame.x + j] = (int)(map.data[i * color_frame.x + j] * 0xff);
                    //(pixels_color.data())[i * color_frame.x*4 + j] = (int)(map2.data[i * color_frame.x + j] );
                   // (pixels_color.data())[i * color_frame.x*4+ j + 1] = (int)(map2.data[i * color_frame.x + j+1] );
                  //  (pixels_color.data())[i * color_frame.x*4 + j + 2] = (int)(map2.data[i * color_frame.x + j+2] );

                }
            }

            //color_frame.frame = std::move(pixels_color);

            gst_buffer_unmap(buffer, &map);

        }

        gst_sample_unref(sample);
        return color_frame;
    }
    synthetic_frame& get_gst_color_pixel(GstAppSink* sink1)
    {
        //auto realsense_logo = stbi_load_from_memory(splash, (int)splash_size, &color_frame.x, &color_frame.y, &color_frame.bpp, false);

        std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y * color_frame.bpp*4, 0);
        
        //std::cout << "W1:" << color_frame.x << std::endl << "H1:" << color_frame.y << std::endl << "BPP1:" << color_frame.bpp << std::endl;
        //memcpy(pixels_color.data(), realsense_logo, color_frame.x * color_frame.y*4);

        GstSample* sample = gst_app_sink_pull_sample(sink1);

        if (sample)
        {
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            // g_print(" get gst color ! ");

            // memcpy(pixels_color.data(), (char*)map2.data, color_frame.x * color_frame.y * 4);

            for (auto i = 0; i < color_frame.y; i++)
            {
                for (auto j = 0; j < color_frame.x*4; j+=4)
                {
                   // ((uint16_t*)color_frame.frame.data())[i * color_frame.x + j] = (int)(map.data[i * color_frame.x + j] * 0xff);
                    pixels_color.data()[i * color_frame.x * 4 + j] = map.data[i * color_frame.x * 4 + j] * 0xff;
                    pixels_color.data()[i * color_frame.x * 4 + j + 1] = map.data[i * color_frame.x * 4 + j+1] * 0xff;
                    pixels_color.data()[i * color_frame.x * 4 + j + 2] = map.data[i * color_frame.x * 4 + j+2] * 0xff;

                }
            }

            
            color_frame.frame = std::move(pixels_color);


            gst_buffer_unmap(buffer, &map);

        }

        gst_sample_unref(sample);
        return color_frame;
    }



    synthetic_frame& get_synthetic_depth(glfw_state& app_state)
    {
        draw_text(50, 50, "This point-cloud is generated from a synthetic device:");

        std::vector<uint8_t> pixels_depth(depth_frame.x * depth_frame.y * depth_frame.bpp, 0);
        depth_frame.frame = std::move(pixels_depth);

        auto now = std::chrono::high_resolution_clock::now();
        if (now - last > std::chrono::milliseconds(1))
        {
            app_state.yaw -= 1;
            wave_base += 0.1f;
            last = now;

            for (int i = 0; i < depth_frame.y; i++)
            {
                for (int j = 0; j < depth_frame.x; j++)
                {
                    auto d = 2 + 0.1 * (1 + sin(wave_base + j / 50.f));
                    ((uint16_t*)depth_frame.frame.data())[i * depth_frame.x + j] = (int)(d * 0xff);
                }
            }
        }
        return depth_frame;
    }

    synthetic_frame& get_gst_depth(GstAppSink* sink2)
    {
        draw_text(50, 50, "This point-cloud is generated from a gstreamer pipeline:");


        std::vector<uint8_t> pixels_depth(depth_frame.x * depth_frame.y * depth_frame.bpp, 0);
        depth_frame.frame = std::move(pixels_depth);


        //GstSample* sample1;
        GstSample* sample1 = gst_app_sink_pull_sample(sink2);

        // g_signal_emit_by_name(sink, "pull-sample", &sample1);
        if (sample1)
        {

            // sample1 = gst_app_sink_pull_sample(sink);
            GstBuffer* buffer = gst_sample_get_buffer(sample1);
            // const GstStructure* info = gst_sample_get_info(sample1);

            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            //g_print(" get gst depth ! ");
            //(uint16_t*)depth_frame.frame.data() = (uint16_t)map.data;
            for (auto i = 0; i < depth_frame.y; i++)
            {
                for (auto j = 0; j < depth_frame.x; j++)
                {
                    //auto d = 2;
                    ((uint16_t*)depth_frame.frame.data())[i * depth_frame.x + j] = (int)(map.data[i * depth_frame.x + j] * 0xff);
                }
            }

            gst_buffer_unmap(buffer, &map);

        }

        gst_sample_unref(sample1);
        //gst_object_unref(sink);

        return depth_frame;
    }


    rs2_intrinsics create_texture_intrinsics()
    {
        rs2_intrinsics intrinsics = { color_frame.x, color_frame.y,
            (float)color_frame.x / 2, (float)color_frame.y / 2,
            (float)color_frame.x /0.7, (float)color_frame.y/0.7 ,
            RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

        return intrinsics;
    }

    rs2_intrinsics create_depth_intrinsics()
    {
        rs2_intrinsics intrinsics = { depth_frame.x, depth_frame.y,
            (float)depth_frame.x / 2, (float)depth_frame.y / 2,
            (float)depth_frame.x , (float)depth_frame.y ,
            RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

        return intrinsics;
    }

private:
    synthetic_frame depth_frame;
    synthetic_frame color_frame;

    std::chrono::high_resolution_clock::time_point last;
    float wave_base = 0.f;
};

//#define CHUNK_SIZE  614400       /* Amount of bytes we are sending in each buffer */
//#define SAMPLE_RATE 44100       /* Samples per second we are sending */




typedef struct _CustomData
{
    GstElement* pipeline1, * source1, * buffer1, * depay1, * decode1, * convert1, * sink1,
              * pipeline2, * source2, * buffer2, * depay2, * decode2, * convert2, * sink2,
              * pipeline3, * source3, * buffer3, * depay3, * decode3, * convert3, * sink3,
              * pipeline4, * source4, * buffer4, * depay4, * decode4, * convert4, * sink4,
              * pipeline5, * source5, * buffer5, * depay5, * decode5, * convert5, * sink5,
              * pipeline6, * source6, * buffer6, * depay6, * decode6, * convert6, * sink6;

    guint64 num_samples;          /* Number of samples generated so far (for timestamp generation) */
    gfloat a, b, c, d;            /* For waveform generation */

    guint sourceid;               /* To control the GSource */

    GMainLoop* main_loop;         /* GLib's Main Loop */
} CustomData;


int main(int argc, char* argv[]) try {
    CustomData data;
    
    //GstCaps* audio_caps;
    GstBus* bus;

    /* Initialize cumstom data structure */
    memset(&data, 0, sizeof(data));
   
    /* Initialize GStreamer */
    gst_init(&argc, &argv);

    /* Create the elements */
    data.source1 = gst_element_factory_make("udpsrc", "source1");
    data.buffer1 = gst_element_factory_make("rtpjitterbuffer", "buffer1");
    data.depay1 = gst_element_factory_make("rtph264depay", "depay");
    data.decode1 = gst_element_factory_make("avdec_h264", "decode");
    data.convert1 = gst_element_factory_make("videoconvert", "convert");
    data.sink1 = gst_element_factory_make("autovideosink", "sink1");
    //data.sink1 = gst_element_factory_make("appsink", "sink1");

    data.source2 = gst_element_factory_make("udpsrc", "source2");
    data.buffer2 = gst_element_factory_make("rtpjitterbuffer", "buffer2");
    data.depay2 = gst_element_factory_make("rtph264depay", "depay");
    data.decode2 = gst_element_factory_make("avdec_h264", "decode");
    data.convert2 = gst_element_factory_make("videoconvert", "convert");
    //data.sink2 = gst_element_factory_make("autovideosink", "sink2");
    data.sink2 = gst_element_factory_make("appsink", "sink2");


    data.source3 = gst_element_factory_make("udpsrc", "source3");
    data.buffer3 = gst_element_factory_make("rtpjitterbuffer", "buffer3");
    data.depay3 = gst_element_factory_make("rtph264depay", "depay3");
    data.decode3 = gst_element_factory_make("avdec_h264", "decode3");
    data.convert3 = gst_element_factory_make("videoconvert", "convert3");
    data.sink3 = gst_element_factory_make("autovideosink", "sink3");
    //data.sink3 = gst_element_factory_make("appsink", "sink3");

    data.source4 = gst_element_factory_make("udpsrc", "source4");
    data.buffer4 = gst_element_factory_make("rtpjitterbuffer", "buffer4");
    data.depay4 = gst_element_factory_make("rtph264depay", "depay4");
    data.decode4 = gst_element_factory_make("avdec_h264", "decode4");
    data.convert4 = gst_element_factory_make("videoconvert", "convert4");
    //data.sink4 = gst_element_factory_make("autovideosink", "sink4");
     data.sink4 = gst_element_factory_make("appsink", "sink4");

     data.source5 = gst_element_factory_make("udpsrc", "source5");
     data.buffer5 = gst_element_factory_make("rtpjitterbuffer", "buffer5");
     data.depay5 = gst_element_factory_make("rtph264depay", "depay5");
     data.decode5 = gst_element_factory_make("avdec_h264", "decode5");
     data.convert5 = gst_element_factory_make("videoconvert", "convert5");
     data.sink5 = gst_element_factory_make("autovideosink", "sink5");
     //data.sink5 = gst_element_factory_make("appsink", "sink5");

     data.source6 = gst_element_factory_make("udpsrc", "source6");
     data.buffer6 = gst_element_factory_make("rtpjitterbuffer", "buffer6");
     data.depay6 = gst_element_factory_make("rtph264depay", "depay6");
     data.decode6 = gst_element_factory_make("avdec_h264", "decode6");
     data.convert6 = gst_element_factory_make("videoconvert", "convert6");
     //data.sink6 = gst_element_factory_make("autovideosink", "sink6");
     data.sink6 = gst_element_factory_make("appsink", "sink6");

    /* Create the empty pipeline */
    data.pipeline1 = gst_pipeline_new("test-pipeline1");
    data.pipeline2 = gst_pipeline_new("test-pipeline2");

    data.pipeline3 = gst_pipeline_new("test-pipeline3");
    data.pipeline4 = gst_pipeline_new("test-pipeline4");

    data.pipeline5 = gst_pipeline_new("test-pipeline5");
    data.pipeline6 = gst_pipeline_new("test-pipeline6");

    if (!data.pipeline1 || !data.source1 || !data.buffer1 || !data.depay1 || !data.decode1 || !data.convert1 || !data.sink1 ||
        !data.pipeline2 || !data.source2 || !data.buffer2 || !data.depay2 || !data.decode2 || !data.convert2 || !data.sink2 ||
        !data.pipeline3 || !data.source3 || !data.buffer3 || !data.depay3 || !data.decode3 || !data.convert3 || !data.sink3 ||
        !data.pipeline4 || !data.source4 || !data.buffer4 || !data.depay4 || !data.decode4 || !data.convert4 || !data.sink4 ||
        !data.pipeline5 || !data.source5 || !data.buffer5 || !data.depay5 || !data.decode5 || !data.convert5 || !data.sink5 ||
        !data.pipeline6 || !data.source6 || !data.buffer6 || !data.depay6 || !data.decode6 || !data.convert6 || !data.sink6){
        g_printerr("Not all elements could be created.\n");
        return -1;
    }

    /* Configure appsrc */
    GstCaps* caps;
    caps = gst_caps_new_simple("application/x-rtp", "encoding-name", G_TYPE_STRING, "H264", NULL);
    if (!GST_IS_CAPS(caps)) {
        g_printerr("caps null ??.\n");
        return -1;
    }
    /* Modify the source's properties */

    g_object_set(data.source1, "port", 8554, NULL);
    g_object_set(data.source1, "caps", caps, NULL);
    g_object_set(data.source2, "port", 8564, NULL);
    g_object_set(data.source2, "caps", caps, NULL);

    g_object_set(data.source3, "port", 4400, NULL);
    g_object_set(data.source3, "caps", caps, NULL);
    g_object_set(data.source4, "port", 4510, NULL);
    g_object_set(data.source4, "caps", caps, NULL);

    g_object_set(data.source5, "port", 4200, NULL);
    g_object_set(data.source5, "caps", caps, NULL);
    g_object_set(data.source6, "port", 4250, NULL);
    g_object_set(data.source6, "caps", caps, NULL);

    /* Configure appsink */
    GstCaps* sink_caps;
    sink_caps = gst_caps_new_simple("video/x-raw", "Format", G_TYPE_STRING, "GRAY16_LE", NULL);
    g_object_set(data.sink2, "caps", sink_caps, NULL);
    
    g_object_set(data.sink4, "caps", sink_caps, NULL);

    g_object_set(data.sink6, "caps", sink_caps, NULL);
   
    /* Link all elements that can be automatically linked because they have "Always" pads */
    gst_bin_add_many(GST_BIN(data.pipeline1), data.source1, data.buffer1, data.depay1, data.decode1, data.convert1, data.sink1, NULL);
    if (gst_element_link_many(data.source1, data.buffer1, data.depay1, data.decode1, data.convert1, data.sink1)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline1);
        return -1;
    }
    gst_bin_add_many(GST_BIN(data.pipeline2), data.source2, data.buffer2, data.depay2, data.decode2, data.convert2, data.sink2, NULL);
    if (gst_element_link_many(data.source2, data.buffer2, data.depay2, data.decode2, data.convert2, data.sink2)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline2);
        return -1;
    }

    gst_bin_add_many(GST_BIN(data.pipeline3), data.source3, data.buffer3, data.depay3, data.decode3, data.convert3, data.sink3, NULL);
    if (gst_element_link_many(data.source3, data.buffer3, data.depay3, data.decode3, data.convert3, data.sink3)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline3);
        return -1;
    }
    gst_bin_add_many(GST_BIN(data.pipeline4), data.source4, data.buffer4, data.depay4, data.decode4, data.convert4, data.sink4, NULL);
    if (gst_element_link_many(data.source4, data.buffer4, data.depay4, data.decode4, data.convert4, data.sink4)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline4);
        return -1;
    }

    gst_bin_add_many(GST_BIN(data.pipeline5), data.source5, data.buffer5, data.depay5, data.decode5, data.convert5, data.sink5, NULL);
    if (gst_element_link_many(data.source5, data.buffer5, data.depay5, data.decode5, data.convert5, data.sink5)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline5);
        return -1;
    }
    gst_bin_add_many(GST_BIN(data.pipeline6), data.source6, data.buffer6, data.depay6, data.decode6, data.convert6, data.sink6, NULL);
    if (gst_element_link_many(data.source6, data.buffer6, data.depay6, data.decode6, data.convert6, data.sink6)) {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline6);
        return -1;
    }



    /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
    bus = gst_element_get_bus(data.pipeline1);
    gst_bus_add_signal_watch(bus);
    gst_object_unref(bus);

    /* Start playing the pipeline */
    gst_element_set_state(data.pipeline1, GST_STATE_PLAYING);
    gst_element_set_state(data.pipeline2, GST_STATE_PLAYING);
    gst_element_set_state(data.pipeline3, GST_STATE_PLAYING);
    gst_element_set_state(data.pipeline4, GST_STATE_PLAYING);
    gst_element_set_state(data.pipeline5, GST_STATE_PLAYING);
    gst_element_set_state(data.pipeline6, GST_STATE_PLAYING);


    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$      Software Device        §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§


    window app(1280, 1500, "Gstreamer Capture Example");
    glfw_state app_state;
    glfw_state app_state1;
    glfw_state app_state2;
    glfw_state app_state3;
    int pc_index = 0;
    register_glfw_callbacks(app, app_state);
    //rs2::colorizer color_map; // Save colorized depth for preview

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pointcloud pc1;
    rs2::points points1;
    rs2::pointcloud pc2;
    rs2::points points2;
    int frame_number = 0;
    int frame_number1 = 0;
    custom_frame_source app_data;
    custom_frame_source app_data1;
    custom_frame_source app_data2;

    rs2_intrinsics color_intrinsics = app_data.create_texture_intrinsics();
    rs2_intrinsics depth_intrinsics = app_data.create_depth_intrinsics();

    //§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§       Device 0               §§§§§§§§§§§§§§§§§§§§§§§§§§§§

    rs2::software_device dev; // Create software-only device
    auto depth_sensor = dev.add_sensor("Depth"); // Define single sensor
    auto color_sensor = dev.add_sensor("Color"); // Define single sensor

    auto depth_stream = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                                W, H, 60, BPP,
                                RS2_FORMAT_Z16, depth_intrinsics });

    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

    //cout << "texture_x:" << texture.x << endl << "texture_y:" << texture.y << endl << "texture.bpp:" << texture.bpp << endl;
   
    auto color_stream = color_sensor.add_video_stream({ RS2_STREAM_COLOR, 0, 1,
                                W, H, 60, BPP,                        //THESE parameter is for the size of color frame
                                RS2_FORMAT_RGBA8, color_intrinsics }); //Y10BPACK

    dev.create_matcher(RS2_MATCHER_DLR_C);

    rs2::syncer sync;
    
    depth_sensor.open(depth_stream);
    color_sensor.open(color_stream);

    depth_sensor.start(sync);
    color_sensor.start(sync);

    depth_stream.register_extrinsics_to(color_stream, { { 0.999927,-0.0116198,-0.00342654,
                                                          0.0116159,0.999932,-0.00114862,
                                                          0.00343965,0.00110873,0.999993 },{ 0,0.00013954,0.000115604 } });

    auto color_frame = app_data.get_synthetic_texture(200, 100, 0);



    //§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§       Device 1               §§§§§§§§§§§§§§§§§§§§§§§§§§§§
    
    rs2::software_device dev1; // Create software-only device
    auto depth_sensor1 = dev1.add_sensor("Depth"); // Define single sensor
    auto color_sensor1 = dev1.add_sensor("Color"); // Define single sensor

    auto depth_stream1 = depth_sensor1.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                                W, H, 60, BPP,
                                RS2_FORMAT_Z16, depth_intrinsics });

    depth_sensor1.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

    //cout << "texture_x:" << texture.x << endl << "texture_y:" << texture.y << endl << "texture.bpp:" << texture.bpp << endl;

    auto color_stream1 = color_sensor1.add_video_stream({ RS2_STREAM_COLOR, 0, 1,
                                W, H, 60, BPP,                        //THESE parameter is for the size of color frame
                                RS2_FORMAT_RGBA8, color_intrinsics }); //Y10BPACK

    dev1.create_matcher(RS2_MATCHER_DLR_C);

    rs2::syncer sync1;

    depth_sensor1.open(depth_stream1);
    color_sensor1.open(color_stream1);

    depth_sensor1.start(sync1);
    color_sensor1.start(sync1);

    depth_stream1.register_extrinsics_to(color_stream1, { { 0.999927,-0.0116198,-0.00342654,
                                                          0.0116159,0.999932,-0.00114862,
                                                          0.00343965,0.00110873,0.999993 },{ 0,0.00013954,0.000115604 } });

    auto color_frame1 = app_data1.get_synthetic_texture(100, 200, 0);
   
    //§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§       Device 2               §§§§§§§§§§§§§§§§§§§§§§§§§§§§

    rs2::software_device dev2; // Create software-only device
    auto depth_sensor2 = dev2.add_sensor("Depth"); // Define single sensor
    auto color_sensor2 = dev2.add_sensor("Color"); // Define single sensor

    auto depth_stream2 = depth_sensor2.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                                W, H, 60, BPP,
                                RS2_FORMAT_Z16, depth_intrinsics });

    depth_sensor2.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

    //cout << "texture_x:" << texture.x << endl << "texture_y:" << texture.y << endl << "texture.bpp:" << texture.bpp << endl;

    auto color_stream2 = color_sensor2.add_video_stream({ RS2_STREAM_COLOR, 0, 1,
                                W, H, 60, BPP,                        //THESE parameter is for the size of color frame
                                RS2_FORMAT_RGBA8, color_intrinsics }); //Y10BPACK

    dev2.create_matcher(RS2_MATCHER_DLR_C);

    rs2::syncer sync2;

    depth_sensor2.open(depth_stream2);
    color_sensor2.open(color_stream2);

    depth_sensor2.start(sync2);
    color_sensor2.start(sync2);

    depth_stream2.register_extrinsics_to(color_stream2, { { 0.999927,-0.0116198,-0.00342654,
                                                          0.0116159,0.999932,-0.00114862,
                                                          0.00343965,0.00110873,0.999993 },{ 0,0.00013954,0.000115604 } });

    auto color_frame2 = app_data2.get_synthetic_texture(0, 200, 100);
   
    while (app) // Application still alive?
    {
        // synthetic_frame& depth_frame = app_data.get_synthetic_depth(app_state);
        //auto color_frame = app_data.get_synthetic_texture(200,100,0);


       // auto color_frame = app_data.get_gst_color((GstAppSink*)data.sink1);
        //auto color_frame = app_data.get_gst_color_pixel((GstAppSink*)data.sink1);
       auto depth_frame = app_data.get_gst_depth((GstAppSink*)data.sink2);
       
      

        //auto color_frame = app_data.get_gst_color((GstAppSink*)data.sink1);

       // Device sensor 0
        depth_sensor.on_video_frame({ depth_frame.frame.data(), // Frame pixels from capture API
            [](void*) {}, // Custom deleter (if required)
            depth_frame.x * depth_frame.bpp, depth_frame.bpp, // Stride and Bytes-per-pixel
            (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
            depth_stream });

        color_sensor.on_video_frame({ color_frame.frame.data(), // Frame pixels from capture API
             [](void*) {}, // Custom deleter (if required)
             color_frame.x * color_frame.bpp, color_frame.bpp, // Stride and Bytes-per-pixel
             (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
             color_stream });
        
       
        
        rs2::frameset fset = sync.wait_for_frames();
        rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);
        rs2::frame color = fset.first_or_default(RS2_STREAM_COLOR);
        if (depth && color)
        {
            if (auto as_depth = depth.as<rs2::depth_frame>())
                points = pc.calculate(as_depth);
            pc.map_to(color);
            //std::cout << "got synthetic streams! " << std::endl;
            // Upload the color frame to OpenGL
            app_state.tex.upload(color);
        }
        draw_pointcloud(app.width(), app.height(), app_state, points);
        
        
        // Device sensor 1

        auto depth_frame1 = app_data.get_gst_depth((GstAppSink*)data.sink4);


        depth_sensor1.on_video_frame({ depth_frame1.frame.data(), // Frame pixels from capture API
           [](void*) {}, // Custom deleter (if required)
           depth_frame1.x * depth_frame1.bpp, depth_frame1.bpp, // Stride and Bytes-per-pixel
           (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
           depth_stream1 });

        color_sensor1.on_video_frame({ color_frame1.frame.data(), // Frame pixels from capture API
             [](void*) {}, // Custom deleter (if required)
             color_frame1.x * color_frame1.bpp, color_frame1.bpp, // Stride and Bytes-per-pixel
             (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
             color_stream1 });
      
             
        rs2::frameset fset1 = sync1.wait_for_frames();
        rs2::frame depth1 = fset1.first_or_default(RS2_STREAM_DEPTH);
        rs2::frame color1 = fset1.first_or_default(RS2_STREAM_COLOR);
        if (depth1 && color1)
        {
            if (auto as_depth1 = depth1.as<rs2::depth_frame>())
                points1 = pc1.calculate(as_depth1);
            pc1.map_to(color1);
            //std::cout << "got synthetic streams! " << std::endl;
            // Upload the color frame to OpenGL
            app_state1.tex.upload(color1);
        }
        draw_pointcloud(app.width(), app.height(), app_state1, points1);

        // Device sensor 2

        auto depth_frame2 = app_data.get_gst_depth((GstAppSink*)data.sink6);

        depth_sensor2.on_video_frame({ depth_frame2.frame.data(), // Frame pixels from capture API
           [](void*) {}, // Custom deleter (if required)
           depth_frame2.x * depth_frame2.bpp, depth_frame2.bpp, // Stride and Bytes-per-pixel
           (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
           depth_stream2 });

        color_sensor2.on_video_frame({ color_frame2.frame.data(), // Frame pixels from capture API
             [](void*) {}, // Custom deleter (if required)
             color_frame2.x * color_frame2.bpp, color_frame2.bpp, // Stride and Bytes-per-pixel
             (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
             color_stream2 });
   

        rs2::frameset fset2 = sync2.wait_for_frames();
        rs2::frame depth2 = fset2.first_or_default(RS2_STREAM_DEPTH);
        rs2::frame color2 = fset2.first_or_default(RS2_STREAM_COLOR);
        if (depth2 && color2)
        {
            if (auto as_depth2 = depth2.as<rs2::depth_frame>())
                points2 = pc2.calculate(as_depth2);
            pc2.map_to(color2);
            //std::cout << "got synthetic streams! " << std::endl;
            // Upload the color frame to OpenGL
            app_state2.tex.upload(color2);
        }
        draw_pointcloud(app.width(), app.height(), app_state2, points2);

        ++frame_number;

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
           

            if (key == 87) // w  y rotation +
            {
                if (pc_index == 0) app_state.pitch += 1;
                if (pc_index == 1) app_state1.pitch += 1;
                if (pc_index == 2) app_state2.pitch += 1;
                if (pc_index == 3) app_state3.pitch += 1;
             //   if (pc_index == 4) app_state4.pitch += 1;


                if (pc_index == 8)
                {
                    app_state.pitch += 1;
                    app_state1.pitch += 1;
                    app_state2.pitch += 1;
                    app_state3.pitch += 1;
                 //   app_state4.pitch += 1;

                }
            }
            if (key == 83) // s  y rotation -
            {
                if (pc_index == 0) app_state.pitch -= 1;
                if (pc_index == 1) app_state1.pitch -= 1;
                if (pc_index == 2) app_state2.pitch -= 1;
                if (pc_index == 3) app_state3.pitch -= 1;
               // if (pc_index == 4) app_state4.pitch -= 1;

                if (pc_index == 8)
                {
                    app_state.pitch -= 1;
                    app_state1.pitch -= 1;
                    app_state2.pitch -= 1;
                    app_state3.pitch -= 1;
                //    app_state4.pitch -= 1;
                }
            }
            if (key == 65) // a : x rotation -
            {
                if (pc_index == 0) app_state.yaw -= 1;
                if (pc_index == 1) app_state1.yaw -= 1;
                if (pc_index == 2) app_state2.yaw -= 1;
                if (pc_index == 3) app_state3.yaw -= 1;
             //   if (pc_index == 4) app_state4.yaw -= 1;

                if (pc_index == 8)
                {
                    app_state.yaw -= 1;
                    app_state1.yaw -= 1;
                    app_state2.yaw -= 1;
                    app_state3.yaw -= 1;
                  //  app_state4.yaw -= 1;
                }
            }
            if (key == 68) // d : x rotation +
            {
                if (pc_index == 0) app_state.yaw += 1;
                if (pc_index == 1) app_state1.yaw += 1;
                if (pc_index == 2) app_state2.yaw += 1;
                if (pc_index == 3) app_state3.yaw += 1;
             //   if (pc_index == 4) app_state4.yaw += 1;


                if (pc_index == 8)
                {
                    app_state.yaw += 1;
                    app_state1.yaw += 1;
                    app_state2.yaw += 1;
                    app_state3.yaw += 1;
                 //   app_state4.yaw += 1;
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
                  //  app_state4.roll -= 1;
                }
            }
            if (key == 69) // e : z rotation +
            {
                if (pc_index == 0) app_state.roll += 1;
                if (pc_index == 1) app_state1.roll += 1;
                if (pc_index == 2) app_state2.roll += 1;
                if (pc_index == 1) app_state3.roll += 1;
              //  if (pc_index == 2) app_state4.roll += 1;

                if (pc_index == 8)
                {
                    app_state.roll += 1;
                    app_state1.roll += 1;
                    app_state2.roll += 1;
                    app_state3.roll += 1;
                   // app_state4.roll += 1;
                }
            }
            if (key == 90) // y: x translation+
            {
                if (pc_index == 0) app_state.offset_x += 1.f;
                if (pc_index == 1) app_state1.offset_x += 1.f;
                if (pc_index == 2) app_state2.offset_x += 1.f;
                if (pc_index == 3) app_state3.offset_x += 1.f;
              //  if (pc_index == 4) app_state4.offset_x += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x += 1.f;
                    app_state1.offset_x += 1.f;
                    app_state2.offset_x += 1.f;
                    app_state3.offset_x += 1.f;
                   // app_state4.offset_x += 1.f;
                }
            }
            if (key == 88) // x: x translation-
            {
                if (pc_index == 0) app_state.offset_x -= 1.f;
                if (pc_index == 1) app_state1.offset_x -= 1.f;
                if (pc_index == 2) app_state2.offset_x -= 1.f;
                if (pc_index == 3) app_state3.offset_x -= 1.f;
              //  if (pc_index == 4) app_state4.offset_x -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_x -= 1.f;
                    app_state1.offset_x -= 1.f;
                    app_state2.offset_x -= 1.f;
                    app_state3.offset_x -= 1.f;
                   // app_state4.offset_x -= 1.f;
                }
            }
            if (key == 67) // c: y translation+
            {
                if (pc_index == 0) app_state.offset_y += 1.f;
                if (pc_index == 1) app_state1.offset_y += 1.f;
                if (pc_index == 2) app_state2.offset_y += 1.f;
                if (pc_index == 3) app_state3.offset_y += 1.f;
               // if (pc_index == 4) app_state4.offset_y += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_y += 1.f;
                    app_state1.offset_y += 1.f;
                    app_state2.offset_y += 1.f;
                    app_state3.offset_y += 1.f;
                  //  app_state4.offset_y += 1.f;
                }
            }
            if (key == 86) // v: y translation+
            {
                if (pc_index == 0) app_state.offset_y -= 1.f;
                if (pc_index == 1) app_state1.offset_y -= 1.f;
                if (pc_index == 2) app_state2.offset_y -= 1.f;
                if (pc_index == 3) app_state3.offset_y -= 1.f;
                //if (pc_index == 4) app_state4.offset_y -= 1.f;


                if (pc_index == 8)
                {
                    app_state.offset_y -= 1.f;
                    app_state1.offset_y -= 1.f;
                    app_state2.offset_y -= 1.f;
                    app_state3.offset_y -= 1.f;
                   // app_state4.offset_y -= 1.f;
                }
            }
            if (key == 82) // r: z tranlation+
            {
                if (pc_index == 0) app_state.offset_z += 1.f;
                if (pc_index == 1) app_state1.offset_z += 1.f;
                if (pc_index == 2) app_state2.offset_z += 1.f;
                if (pc_index == 3) app_state3.offset_z += 1.f;
               // if (pc_index == 4) app_state4.offset_z += 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z += 1.f;
                    app_state1.offset_z += 1.f;
                    app_state2.offset_z += 1.f;
                    app_state3.offset_z += 1.f;
                //    app_state4.offset_z += 1.f;
                }
            }
            if (key == 70) // f: z tranlation-
            {
                if (pc_index == 0) app_state.offset_z -= 1.f;
                if (pc_index == 1) app_state1.offset_z -= 1.f;
                if (pc_index == 2) app_state2.offset_z -= 1.f;
                if (pc_index == 3) app_state3.offset_z -= 1.f;
                //if (pc_index == 4) app_state4.offset_z -= 1.f;

                if (pc_index == 8)
                {
                    app_state.offset_z -= 1.f;
                    app_state1.offset_z -= 1.f;
                    app_state2.offset_z -= 1.f;
                    app_state3.offset_z -= 1.f;
                   
                }
            }
            if (print_state == true)// print app state
            {
                if (pc_index == 0) cout << "cam 1 position(x,y,z):" << app_state.offset_x << "," << app_state.offset_y << "," << app_state.offset_z << endl << "rotation(x,y,z):" << app_state.yaw << "," << app_state.pitch << "," << app_state.roll << endl;
                if (pc_index == 1) cout << "cam 2 position(x,y,z):" << app_state1.offset_x << "," << app_state1.offset_y << "," << app_state1.offset_z << endl << "rotation(x,y,z):" << app_state1.yaw << "," << app_state1.pitch << "," << app_state1.roll << endl;
                if (pc_index == 2) cout << "cam 3 position(x,y,z):" << app_state2.offset_x << "," << app_state2.offset_y << "," << app_state2.offset_z << endl << "rotation(x,y,z):" << app_state2.yaw << "," << app_state2.pitch << "," << app_state2.roll << endl;
                if (pc_index == 3) cout << "cam 4 position(x,y,z):" << app_state3.offset_x << "," << app_state3.offset_y << "," << app_state3.offset_z << endl << "rotation(x,y,z):" << app_state3.yaw << "," << app_state3.pitch << "," << app_state3.roll << endl;
               // if (pc_index == 4) cout << "cam 5 position(x,y,z):" << app_state4.offset_x << "," << app_state4.offset_y << "," << app_state4.offset_z << endl << "rotation(x,y,z):" << app_state4.yaw << "," << app_state4.pitch << "," << app_state4.roll << endl;
            }
        };
       
    }

    /* Free resources */
    gst_element_set_state(data.pipeline1, GST_STATE_NULL);
    gst_object_unref(data.pipeline1);
    gst_element_set_state(data.pipeline2, GST_STATE_NULL);
    gst_object_unref(data.pipeline2);

    gst_element_set_state(data.pipeline3, GST_STATE_NULL);
    gst_object_unref(data.pipeline3);
    gst_element_set_state(data.pipeline4, GST_STATE_NULL);
    gst_object_unref(data.pipeline4);

    gst_element_set_state(data.pipeline5, GST_STATE_NULL);
    gst_object_unref(data.pipeline5);
    gst_element_set_state(data.pipeline6, GST_STATE_NULL);
    gst_object_unref(data.pipeline6);
    return 0;



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
