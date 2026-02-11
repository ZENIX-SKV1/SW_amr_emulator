#include "amr_server.h"

int main(int argc, char* argv[]) 
{
    // std::string config_path = "/home/zenix/amr_emulator_release/config/amr_params.yaml";
    std::string config_path;
    if (argc > 1) config_path = argv[1];
    AmrServerApp app;
    app.run(config_path);
    return 0;
}