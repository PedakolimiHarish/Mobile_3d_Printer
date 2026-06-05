// start of file: ros2_ws/src/wbp_supervisor/include/wbp_supervisor/ipc_job_writer.hpp
#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include "wbp_interfaces/ipc/job_ingress_ipc.h"

class IPCJobWriter
{
public:
  IPCJobWriter();
  bool write_job(const std::vector<uint8_t> &blob, const std::string &source);
  void mark_ingress_complete();

private:
  // int fd_{-1};
  // void* ptr_{nullptr};
  wbp_job_ingress_ipc_t *job_ipc_{nullptr};
};
