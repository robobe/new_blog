---
title: Gstreamer c/cpp bindings
tags:
  - gstreamer
  - bindings
---

```bash titlle="install dependencies"
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

```c
#include <gst/gst.h>
#include <glib.h>
#include <iostream>

static void on_error(
    GstBus *bus,
    GstMessage *msg,
    gpointer user_data)
{
    GMainLoop *loop = static_cast<GMainLoop *>(user_data);

    GError *err = nullptr;
    gchar *debug = nullptr;
    gst_message_parse_error(msg, &err, &debug);

    std::cerr << "ERROR: " << err->message << std::endl;
    if (debug)
        std::cerr << "Debug info: " << debug << std::endl;

    g_error_free(err);
    g_free(debug);

    g_main_loop_quit(loop);
}

static void on_eos(
    GstBus *bus,
    GstMessage *msg,
    gpointer user_data)
{
    GMainLoop *loop = static_cast<GMainLoop *>(user_data);
    std::cout << "EOS" << std::endl;
    g_main_loop_quit(loop);
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    GMainLoop *loop = g_main_loop_new(nullptr, FALSE);

    GstElement *pipeline =
        gst_parse_launch("videotestsrc ! autovideosink", nullptr);

    GstBus *bus = gst_element_get_bus(pipeline);

    /* Attach bus to GLib signals (no manual polling) */
    gst_bus_add_signal_watch(bus);

    g_signal_connect(
        bus, "message::error",
        G_CALLBACK(on_error), loop);

    g_signal_connect(
        bus, "message::eos",
        G_CALLBACK(on_eos), loop);

    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}


```

```c
cmake_minimum_required(VERSION 3.16)
project(gstreamer_cpp_example LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-base-1.0
)

add_executable(gst_example main.cpp)

target_include_directories(gst_example PRIVATE
    ${GST_INCLUDE_DIRS}
)

target_link_libraries(gst_example
    ${GST_LIBRARIES}
)

# Needed on some systems
target_compile_options(gst_example PRIVATE
    ${GST_CFLAGS_OTHER}
)

```