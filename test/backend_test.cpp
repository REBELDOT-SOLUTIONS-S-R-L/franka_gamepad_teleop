// Exercise the production main/controller dispatch against a no-network SDK
// stand-in and SDL virtual inputs. No physical robot or gamepad is commanded.
#define main gamepad_program_main
#include "../src/main.cpp"
#undef main
#include <filesystem>
#include <fstream>
#include <unistd.h>

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    if(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)!=0) throw std::runtime_error(SDL_GetError());
    const int index=SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_GAMECONTROLLER,6,15,0);
    if(index<0) throw std::runtime_error(SDL_GetError());
    franka::api_test::joystick=SDL_JoystickOpen(index);
    const auto folder=std::filesystem::temp_directory_path()/("gamepad-backend-test-"+std::to_string(getpid()));
    std::filesystem::create_directories(folder);
    auto config=YAML::LoadFile(argv[1]);config["gamepad"]["name_contains"]="Virtual Controller";config["gripper"]["enabled"]=false;
    const auto path=(folder/"profile.yaml").string(),log=(folder/"session.csv").string();
    {std::ofstream out(path);out<<config;}
    std::vector<std::string> args={"gamepad_teleop","--config",path,"--robot-ip","fake-only","--execute","--log",log};
    std::vector<char*> pointers;for(auto& argument:args) pointers.push_back(argument.data());
    const int result=gamepad_program_main(static_cast<int>(pointers.size()),pointers.data());
    const auto& trace=franka::api_test::backends;
    if(result!=0||trace!=std::vector<std::string>{"pose","velocity","torque","pose"})
      throw std::runtime_error("X did not dispatch pose -> velocity -> torque -> pose through completed control loops");
    if(franka::api_test::model_loads!=1||franka::api_test::maximum_velocity<.001||franka::api_test::maximum_torque<.1)
      throw std::runtime_error("Speed/Force API outputs were not exercised");
    if(franka::api_test::maximum_pose_jump>1e-8) throw std::runtime_error("Pose backend restarted with a position jump");
    std::ifstream file(log);std::string header;std::getline(file,header);
    if(header.find("control_backend")==std::string::npos||header.find("requested_wrench_0")==std::string::npos)
      throw std::runtime_error("Backend diagnostics missing");
    SDL_JoystickDetachVirtual(index);SDL_JoystickClose(franka::api_test::joystick);SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
    std::filesystem::remove_all(folder);
    std::cout<<"PASS actual controller dispatch, complete round trip, velocity/torque actuation and pose handover continuity (fake SDK)\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<"FAIL backend: "<<error.what()<<'\n';return 1; }
}
