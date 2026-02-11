#include "presentation/amrServer.h"
int main(int argc, char* argv[]) 
{
    std::string config_path = "config/amr_params.yaml";
    if (argc > 1) config_path = argv[1];
    AmrServerApp app;
    app.run(config_path);
    return 0;
}