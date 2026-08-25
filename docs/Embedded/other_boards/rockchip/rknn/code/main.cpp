#include <iostream>

#include "rknn_api.h"


void print_tensor_attr(const rknn_tensor_attr& attr)
{
    std::cout << "index: " << attr.index << '\n';
    std::cout << "name: " << attr.name << '\n';

    std::cout << "shape: [";

    for (uint32_t i = 0; i < attr.n_dims; ++i)
    {
        std::cout << attr.dims[i];

        if (i + 1 < attr.n_dims)
            std::cout << ", ";
    }

    std::cout << "]\n";

    std::cout << "elements: "
              << attr.n_elems << '\n';

    std::cout << "size: "
              << attr.size
              << " bytes\n";

    std::cout << "fmt: "
              << attr.fmt << '\n';

    std::cout << "type: "
              << attr.type << '\n';

    std::cout << "zp: "
              << attr.zp << '\n';

    std::cout << "scale: "
              << attr.scale << '\n';

    std::cout << '\n';
}


int main()
{
    const char* model =
        "yolo26n-rk3566.rknn";

    rknn_context ctx = 0;

    // ------------------------------------------
    // Load model
    // ------------------------------------------

    int ret = rknn_init(
        &ctx,
        const_cast<char*>(model),
        0,
        0,
        nullptr
    );

    if (ret != RKNN_SUCC)
    {
        std::cerr
            << "rknn_init failed: "
            << ret << '\n';

        return 1;
    }


    // ------------------------------------------
    // Number of inputs / outputs
    // ------------------------------------------

    rknn_input_output_num io_num{};

    ret = rknn_query(
        ctx,
        RKNN_QUERY_IN_OUT_NUM,
        &io_num,
        sizeof(io_num)
    );

    if (ret != RKNN_SUCC)
    {
        std::cerr
            << "query failed\n";

        rknn_destroy(ctx);
        return 1;
    }

    std::cout
        << "Number of inputs: "
        << io_num.n_input << '\n';

    std::cout
        << "Number of outputs: "
        << io_num.n_output << '\n';


    // ------------------------------------------
    // Input tensors
    // ------------------------------------------

    std::cout << "\n=== INPUT ===\n";

    for (uint32_t i = 0;
         i < io_num.n_input;
         ++i)
    {
        rknn_tensor_attr attr{};

        attr.index = i;

        ret = rknn_query(
            ctx,
            RKNN_QUERY_INPUT_ATTR,
            &attr,
            sizeof(attr)
        );

        if (ret == RKNN_SUCC)
            print_tensor_attr(attr);
    }


    // ------------------------------------------
    // Output tensors
    // ------------------------------------------

    std::cout << "\n=== OUTPUT ===\n";

    for (uint32_t i = 0;
         i < io_num.n_output;
         ++i)
    {
        rknn_tensor_attr attr{};

        attr.index = i;

        ret = rknn_query(
            ctx,
            RKNN_QUERY_OUTPUT_ATTR,
            &attr,
            sizeof(attr)
        );

        if (ret == RKNN_SUCC)
            print_tensor_attr(attr);
    }


    // ------------------------------------------
    // Cleanup
    // ------------------------------------------

    rknn_destroy(ctx);

    return 0;
}