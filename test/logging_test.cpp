#include "franka_gamepad_teleop/logging.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

using namespace gamepad;
namespace {
void require(bool yes,const char* error) { if(!yes) throw std::runtime_error(error); }
}
int main() {
  try {
    auto queue=std::make_unique<DiagnosticQueue>();
    DiagnosticSample sample{};
    for(size_t i=0;i<DiagnosticQueue::capacity;++i) {
      sample.cycle=i; sample.step.target.fill(static_cast<double>(i));
      require(queue->push(sample),"queue filled early");
    }
    require(!queue->push(sample)&&queue->dropped()==1,"full queue did not drop without waiting");
    for(size_t i=0;i<DiagnosticQueue::capacity;++i) {
      DiagnosticSample out; require(queue->pop(out),"queue lost a sample");
      require(out.cycle==i&&out.step.target[12]==static_cast<double>(i),"queue corrupted FIFO sample");
    }
    require(!queue->pop(sample),"empty queue returned a sample");
    std::cout<<"PASS bounded queue overflow and FIFO\n";
    std::atomic<bool> producer_done{false};
    std::thread producer([&] {
      for(uint64_t i=0;i<20000;++i) {
        DiagnosticSample s{}; s.cycle=i; s.step.target.fill(static_cast<double>(i));
        queue->push(s);
      }
      producer_done.store(true);
    });
    bool coherent=true; uint64_t previous=0; bool first=true;
    while(true) {
      DiagnosticSample out;
      if(queue->pop(out)) {
        coherent=coherent&&(first||out.cycle>previous);
        for(double value:out.step.target) coherent=coherent&&value==static_cast<double>(out.cycle);
        first=false; previous=out.cycle;
      } else if(producer_done.load()) break;
    }
    producer.join(); require(coherent,"concurrent queue produced torn or unordered samples");
    std::cout<<"PASS concurrent diagnostic snapshots\n";
    const auto folder=std::filesystem::temp_directory_path()/("gamepad-log-test-"+std::to_string(getpid()));
    std::filesystem::create_directories(folder);
    const auto path=(folder/"session.csv").string();
    {
      DiagnosticLog logger(path,"schema_version: 1\nmode: simulate\n");
      for(int i=0;i<100;++i) {
        DiagnosticSample s{}; s.cycle=i; s.step.target=identity_pose(); s.step.target[12]=.5+i*.001;
        s.monotonic=i*.004; s.input.connected=true; logger.record(s);
      }
    }
    std::ifstream csv(path); std::string header,line; std::getline(csv,header);
    const auto columns=std::count(header.begin(),header.end(),',');
    require(header.find("target_12")!=std::string::npos&&header.find("measured_12")!=std::string::npos&&
      header.find("commanded_12")!=std::string::npos&&header.find("mapped_5")!=std::string::npos,"missing diagnostic fields");
    size_t rows=0;
    while(std::getline(csv,line)) { require(std::count(line.begin(),line.end(),',')==columns,"malformed CSV row"); ++rows; }
    const auto meta=YAML::LoadFile(path+".meta.yaml");
    require(rows==100&&meta["rows_written"].as<int>()==100,"destructor did not drain all samples");
    require(meta["dropped_samples"].as<int>()==0&&!meta["writer_failed"].as<bool>(),"writer lost samples");
    bool refused=false;
    try { DiagnosticLog again(path,"schema_version: 1\n"); } catch(...) { refused=true; }
    require(refused,"existing log could be overwritten");
    std::filesystem::remove_all(folder);
    std::cout<<"PASS CSV fields, complete drain, metadata and overwrite refusal\n3 logging tests passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<"FAIL logging: "<<error.what()<<'\n'; return 1; }
}
