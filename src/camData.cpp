// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_advanced_mode.h>
#include <opencv2/opencv.hpp> // Include OpenCV API
#include <iostream>
#include <fstream>
#include <string>
#include <sstream> // Stringstreams
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <thread>
#include <vector>
// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
//#include "camTCP.hpp"

#define PORT 9111

using namespace cv;

Mat frame;
// Point3d pt3d(-1,-1,-1);
bool newCoords = false;

struct imageParams
{
    float x1;
    float y1;
    float z1;
    float x2;
    float y2;
    float z2;
};

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile> &streams);
bool saveFrameRawData(const std::string &filename, rs2::frame frame);
void remove_background(rs2::video_frame &other_frame, const rs2::depth_frame &depth_frame, float depth_scale, float clipping_dist);

// Helper function for writing metadata to disk as a csv file
//void metadata_to_csv(const rs2::frame& frm, const int x, const int y, const float z, const int i);
void mouse_callback(int event, int x, int y, int flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        std::vector<Point2i> *pt = static_cast<std::vector<Point2i> *>(param);
        pt->at(0).x = x;
        pt->at(0).y = y;
        newCoords = true;
    }
    if (event == EVENT_LBUTTONUP)
    {
        std::vector<Point2i> *pt = static_cast<std::vector<Point2i> *>(param);
        pt->at(1).x = x;
        pt->at(1).y = y;
        std::cout << pt->at(0).x << ", " << pt->at(0).y << ", " << pt->at(1).x << ", " << pt->at(0).y << std::endl;
        newCoords = true;
    }
}

int main(int argc, char const *argv[]) try
{
     //------------------------Find correct device, reset & apply settings--------------
    //Get available cameras
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No camera detected. Is it plugged in?");
    std::cout << list.size() << " devices detected" << std::endl;

    //Find camera andf reset it
    rs2::device dev = list[0];
    std::string dev_serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    std::string dev_firmware_version(dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
    std::cout << "Found camera, S/N: " << dev_serial_number << "; FW: " << dev_firmware_version << std::endl;

    // Set up TCP/IP connection to PLC
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    std::cout << "Trying to connect to PLC..." << std::endl;
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 9111
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 9111
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                             (socklen_t *)&addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    std::cout << "Connected to PLC" << std::endl;
   

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_ANY, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_ANY, 30);
    // cfg.enable_stream(RS2_STREAM_INFRARED, 848, 480, RS2_FORMAT_ANY, 90);

    cfg.enable_device(dev_serial_number);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start(cfg);
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    // sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);

    float depth_scale = get_depth_scale(profile.get_device());
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    rs2::align align(align_to);
    float depth_clipping_distance = .87f;

    //Opencv window
    // const std::string window_name_raw = "Raw image";
    // cv::namedWindow(window_name_raw, cv::WINDOW_AUTOSIZE);
    // //set the callback function for any mouse event
    // cv::setMouseCallback(window_name_raw, mouse_callback, NULL);

    cv::Point2i pt1(-1, -1), pt2(-1, -1);
    std::vector<cv::Point2i> selectedPoints;
    selectedPoints.push_back(pt1);
    selectedPoints.push_back(pt2);
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    for (auto i = 0; i < 30; ++i)
        pipe.wait_for_frames();

    std::ofstream out;
    std::ofstream depthFile;

    int i = 0;
    float distance1, distance2;
    imageParams imageParams_;

    // rs2::frame_queue queue(2);
    rs2::frame_queue postprocessed_frames;

    // Apply filters
    // rs2::decimation_filter dec_filter;
    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thres_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter tempo_filter;
    rs2::disparity_transform depth2disparity(true);
    rs2::disparity_transform disparity2depth(false);

    thres_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.66f);
    thres_filter.set_option(RS2_OPTION_MAX_DISTANCE, 0.85f);
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.25f);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 1.0f);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 2);
    tempo_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.12f);
    tempo_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 69.0f);
    tempo_filter.set_option(RS2_OPTION_HOLES_FILL, 4);

    bool alive = true;

    std::thread video_processing_thread([&]() {
        // In order to generate new composite frames, we have to wrap the processing
        // code in a lambda
        rs2::processing_block frame_processor(
            [&](rs2::frameset data,        // Input frameset (from the pipeline)
                rs2::frame_source &source) // Frame pool that can allocate new frames
            {
                // First make the frames spatially aligned

                // Next, apply depth post-processing
                rs2::frame depth = data.get_depth_frame();
                // Decimation will reduce the resultion of the depth image,
                // closing small holes and speeding-up the algorithm
                depth = depth2disparity.process(depth);
                // depth = dec_filter.process(depth);
                // To make sure far-away objects are filtered proportionally
                // we try to switch to disparity domain
                // depth = depth2disparity.process(depth);
                depth = thres_filter.process(depth);
                // Apply spatial filtering
                depth = spat_filter.process(depth);
                // Apply temporal filtering
                depth = tempo_filter.process(depth);
                // If we are in disparity domain, switch back to depth
                depth = disparity2depth.process(depth);

                //checking the size before align process, due to decmiation set to 2, it will redcue the size as 1/2
                float width = depth.as<rs2::video_frame>().get_width();
                float height = depth.as<rs2::video_frame>().get_height();

                auto color = data.get_color_frame();
                rs2::frameset combined = source.allocate_composite_frame({depth, color});
                combined = align.process(combined);

                //checking the size after align process, the size align to color 640*480
                width = combined.get_depth_frame().as<rs2::video_frame>().get_width();
                height = combined.get_depth_frame().as<rs2::video_frame>().get_height();

                source.frame_ready(combined);
            });
        // Indicate that we want the results of frame_processor
        // to be pushed into postprocessed_frames queue
        frame_processor >> postprocessed_frames;

        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset fs = pipe.wait_for_frames();
            if (fs.size() != 0)
                frame_processor.invoke(fs);
        }
    });

    while (1)
    {
        bool towelGrabable = false;
        // Read TCP/IP buffer
        char buffer[1024] = {0};
        valread = read(new_socket, buffer, 1024);
        printf("%s\n", buffer);

        // Restart while loop if buffer doesn't contain correct message
        if (strcmp(buffer, "~1;") != 0)
            continue;
        std::cout << "Got past!" << std::endl;
        sleep(1); // Wait until towel settles

        // Using the align object, we block the application until a frameset is available

        static rs2::frameset frameset;
        postprocessed_frames.poll_for_frame(&frameset);

        if (frameset.size() == 0)
            continue;

        //Get processed aligned frame
        rs2::video_frame other_frame = frameset.get_color_frame();
        rs2::depth_frame aligned_depth_frame = frameset.get_depth_frame();

        float width = aligned_depth_frame.get_width();
        float height = aligned_depth_frame.get_height();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // rs2::frameset processed = align.process(data);
        // rs2::video_frame other_frame = processed.first(align_to);
        // rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
        rs2::frame color = data.get_color_frame();
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);

        // // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        const int wC = color.as<rs2::video_frame>().get_width();
        const int hC = color.as<rs2::video_frame>().get_height();

        const int wO = other_frame.as<rs2::video_frame>().get_width();
        const int hO = other_frame.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void *)depth.get_data(), Mat::AUTO_STEP);
        Mat imageColor(Size(wC, hC), CV_8UC3, (void *)color.get_data(), Mat::AUTO_STEP);
        Mat imageFiltered(Size(wO, hO), CV_8UC3, (void *)other_frame.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        // imageParams_.img = image;
        imshow(window_name, imageFiltered);
        setMouseCallback(window_name, mouse_callback, (void *)&selectedPoints);

        waitKey(0);

        distance1 = aligned_depth_frame.get_distance(selectedPoints[0].x, selectedPoints[0].y);
        distance2 = aligned_depth_frame.get_distance(selectedPoints[1].x, selectedPoints[1].y);

        std::cout << "values: " << imageFiltered.at<cv::Vec3b>(300,300)[0] << ", " << imageFiltered.at<cv::Vec3b>(300,300)[1] << ", " << imageFiltered.at<cv::Vec3b>(300,300)[2] << std::endl;

        std::string basePath = "/media/laus/USB DISK/images-newfilter";
        std::string filenameD;
        std::string filenameC;
        std::string filenameF;
        std::string filenameRawDepth;
        std::string filenameRawColor;
        std::string filenameRawFiltered;

        out.open(basePath + "/result.txt", std::ios::app);
        depthFile.open(basePath + "/depth.txt", std::ios::app);
        if (newCoords)
        {
            
            filenameD = basePath + "/True/depth" + std::to_string(i) + ".png";
            filenameC = basePath + "/True/color" + std::to_string(i) + ".png";
            filenameF = basePath + "/True/filtered" + std::to_string(i) + ".png";
            filenameRawDepth = basePath + "/True/RawDepth" + std::to_string(i) + ".bin";
            filenameRawColor = basePath + "/True/RawColor" + std::to_string(i) + ".bin";
            filenameRawFiltered = basePath + "/True/RawFiltered" + std::to_string(i) + ".bin";

            std::cout << "i: " << i << " - Good towel!" << std::endl;

            imageParams_.x1 = selectedPoints[0].x;
            imageParams_.y1 = selectedPoints[0].y;
            imageParams_.z1 = distance1;
            imageParams_.x2 = selectedPoints[1].x;
            imageParams_.y2 = selectedPoints[1].y;
            imageParams_.z2 = distance2;

            std::cout << imageParams_.x1 << "," << imageParams_.y1 << "," << imageParams_.x2 << "," << imageParams_.y2 << std::endl;
            out << filenameF << ",";
            out << imageParams_.x1 << ",";
            out << imageParams_.y1 << ",";
            out << imageParams_.x2 << ",";
            out << imageParams_.y2 << ",";
            out << "True";
            out << "\n";

            depthFile << filenameF << ",";
            depthFile << imageParams_.x1 << ",";
            depthFile << imageParams_.y1 << ",";
            depthFile << imageParams_.z1 << ",";
            depthFile << imageParams_.x2 << ",";
            depthFile << imageParams_.y2 << ",";
            depthFile << imageParams_.z2 << ",";
            depthFile << "True";
            depthFile << "\n";

            newCoords = false;
        }
        else
        {
            // towelGrabable = false;
            filenameD = basePath + "/False/depth" + std::to_string(i) + ".png";
            filenameC = basePath + "/False/color" + std::to_string(i) + ".png";
            filenameF = basePath + "/False/filtered" + std::to_string(i) + ".png";
            filenameRawDepth = basePath + "/False/RawDepth" + std::to_string(i) + ".bin";
            filenameRawColor = basePath + "/False/RawColor" + std::to_string(i) + ".bin";
            filenameRawFiltered = basePath + "/False/RawFiltered" + std::to_string(i) + ".bin";

            std::cout << "i: " << i << " - Bad towel!" << std::endl;
            out << filenameF << ",";
            out << 630 << ",";
            out << 470 << ",";
            out << 639 << ",";
            out << 439 << ",";
            out << "False";
            out << "\n";

            depthFile << filenameF << ",";
            depthFile << 630 << ",";
            depthFile << 470 << ",";
            depthFile << 0 << ",";
            depthFile << 639 << ",";
            depthFile << 439 << ",";
            depthFile << 0 << ",";
            depthFile << "False";
            depthFile << "\n";
        }
        out.close();
        depthFile.close();

        imwrite(filenameD, image);
        imwrite(filenameC, imageColor);
        imwrite(filenameF, imageFiltered);
        saveFrameRawData(filenameRawDepth, depth);
        saveFrameRawData(filenameRawColor, color);
        saveFrameRawData(filenameRawFiltered, other_frame);

        i++;

        destroyAllWindows();
    }
    alive = false;
    video_processing_thread.join();

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor &sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile> &streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found) //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool saveFrameRawData(const std::string &filename, rs2::frame frame)
{
    bool ret = false;
    auto image = frame.as<rs2::video_frame>();
    if (image)
    {
        std::ofstream outfile(filename.data(), std::ofstream::binary);
        outfile.write(static_cast<const char *>(image.get_data()), image.get_height() * image.get_stride_in_bytes());

        outfile.close();
        ret = true;
    }

    return ret;
}

void remove_background(rs2::video_frame &other_frame, const rs2::depth_frame &depth_frame, float depth_scale, float clipping_dist)
{
    const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
    uint8_t *p_other_frame = reinterpret_cast<uint8_t *>(const_cast<void *>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            // Check if the depth value is invalid (<=0) or greater than the threashold
            if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
            {
                // Calculate the offset in other frame's buffer to current pixel
                auto offset = depth_pixel_index * other_bpp;

                // Set pixel to "background" color (0x999999)
                std::memset(&p_other_frame[offset], 0x99, other_bpp);
            }
        }
    }
}
