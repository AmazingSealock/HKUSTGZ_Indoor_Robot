#include "main.h"
#include <csignal>

FastLioSamQnClass* g_fastLioSamPtr = nullptr;

void signalHandler(int signum) {
  if (g_fastLioSamPtr) {
    g_fastLioSamPtr->saveResult();
  }
  ros::shutdown();
  exit(signum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_sam_qn_node");
  ros::NodeHandle nh_private("~");

  FastLioSamQnClass fast_lio_sam_qn_(nh_private);
  g_fastLioSamPtr = &fast_lio_sam_qn_;

  std::signal(SIGINT, signalHandler);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}