#include "mujoco_client.h"
#include <vector>
#include <random>

int main() {
    const char* model_path = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
    mujoco_client::MujocoClient client(model_path);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> pos_rnd(-1.0f, 1.0f);


    while (true) {
        client.setConfig(std::vector<double>{pos_rnd(gen), pos_rnd(gen), pos_rnd(gen), pos_rnd(gen), pos_rnd(gen)});
        client.render();
        glfwPollEvents();
    };

    return 0;
}