
#include "tmc2209.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include <viam/sdk/common/exception.hpp>
#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/registry/registry.hpp>


int main(int argc, char** argv) try {

    // Every Viam C++ SDK program must have one and only one Instance object which is created before
    // any other SDK objects and stays alive until all of them are destroyed.
    viam::sdk::Instance inst;

    // Write general log statements using the VIAM_SDK_LOG macro.
    VIAM_SDK_LOG(info) << "Starting up tmcstepper module";

    viam::sdk::Model model("490b556e-1c8e-4597-9ec2-c0a64f2ec6cf", "tmcstepper", "tmc2209");


    auto mr = std::make_shared<viam::sdk::ModelRegistration>(
        viam::sdk::API::get<viam::sdk::Motor>(),
        model,
        [](viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg) {
            return std::make_unique<tmcstepper::Tmc2209>(deps, cfg);
        },
        &tmcstepper::Tmc2209::validate);



    std::vector<std::shared_ptr<viam::sdk::ModelRegistration>> mrs = {mr};
    auto my_mod = std::make_shared<viam::sdk::ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
} catch (const viam::sdk::Exception& ex) {
    std::cerr << "main failed with exception: " << ex.what() << "\n";
    return EXIT_FAILURE;
}
