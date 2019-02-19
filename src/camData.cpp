// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>              // Stringstreams
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
//#include "camTCP.hpp"

#define PORT 9111

using namespace cv;


Mat frame;
Point3d pt3d(-1,-1,-1);
bool newCoords = false;

// struct imageParams
// {
//     Mat img;
//     bool newCoords = false;
// };

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool saveFrameRawData(const std::string& filename, rs2::frame frame);

// Helper function for writing metadata to disk as a csv file
//void metadata_to_csv(const rs2::frame& frm, const int x, const int y, const float z, const int i);
void mouse_callback(int event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        Point* pt = (Point*)param;
        pt->x = x;
        pt->y = y;
        newCoords = true;
        std::cout << *pt << std::endl;
    }
}


int main(int argc, char const *argv[]) try
{
    int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
 
       
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
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 9111 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
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
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 


    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start();

    cv::Point2i pt(-1,-1);
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

    

    std::ofstream out;

    int i = 0;
    float distance;
    bool firstRun = true;
    // imageParams imageParams_;
    

    while (1)
    {
        // Read TCP/IP buffer
        char buffer[1024] = {0}; 
        valread = read(new_socket, buffer, 1024);
        printf("%s\n", buffer);
        
        // Restart while loop if buffer doesn't contain correct message 
        if (strcmp(buffer, "~1;") != 0) 
            continue;
        std::cout << "Got past!" << std::endl;
        waitKey(400); // Wait until towel settles

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame();
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        const int wC = color.as<rs2::video_frame>().get_width();
        const int hC = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat imageColor(Size(wC, hC), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        // imageParams_.img = image;
        imshow(window_name, image);
        setMouseCallback(window_name, mouse_callback, &pt); 
        // if (firstRun)
        waitKey(0);

        out.open("../result.txt", std::ios::app);
        if (newCoords)
        {
            distance = data.get_depth_frame().get_distance(pt.x, pt.y);
            pt3d.x = pt.x;
            pt3d.y = pt.y;
            pt3d.z = distance;

            std::cout << i << pt3d << std::endl;
            out << "Frame," << i;
            out << "\nx," << pt.x;
            out << "\ny," << pt.y;
            out << "\nz," << distance;
            out << "\n";

            newCoords = false;

        }
        else //if (!firstRun)
        {
            std::cout << i << "Bad towel!" << std::endl;
            out << "Frame," << i;
            out << "\nx," << -1;
            out << "\ny," << -1;
            out << "\nz," << -1;
            out << "\n";
        }
        out.close();
        firstRun = false;

        std::string filenameD = "/home/laus/uni/thesis/cam/Test/images/depth" + std::to_string(i) + ".png";
        std::string filenameC = "/home/laus/uni/thesis/cam/Test/images/color" + std::to_string(i) + ".png";
        imwrite(filenameD, image);
        imwrite(filenameC, imageColor);

        std::string filenameRawDepth = "/home/laus/uni/thesis/cam/Test/images/RawDepth" + std::to_string(i) + ".bin";
        saveFrameRawData(filenameRawDepth, depth);
        std::string filenameRawColor = "/home/laus/uni/thesis/cam/Test/images/RawColor" + std::to_string(i) + ".bin";
        saveFrameRawData(filenameRawColor, color);

        i++;
        
        destroyAllWindows();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
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
            if (!color_stream_found)         //Prefer color
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

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool saveFrameRawData(const std::string& filename, rs2::frame frame)
{
    bool ret = false;
    auto image = frame.as<rs2::video_frame>();
    if (image)
    {
        std::ofstream outfile(filename.data(), std::ofstream::binary);
        outfile.write(static_cast<const char*>(image.get_data()), image.get_height()*image.get_stride_in_bytes());

        outfile.close();
        ret = true;
    }

    return ret;
}
