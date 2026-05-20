/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-07-28 13:32:57
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-03 13:37:48
 * @FilePath: /xt212Alpha/Hmi/hmiServer.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Rpc/include/rest_rpc.hpp"
#include <fstream>
#include "hmi.h"

using namespace rest_rpc;
using namespace rpc_service;

int main()
{
  bool isRunning = false;
  Hmi_t hmi;
  hmi.Start();
  rpc_server server(RpcPort_Hmi, std::thread::hardware_concurrency());

  server.register_handler("SetAppCmd", &Hmi_t::appCmdSet, &hmi);
  server.register_handler("SetStaCmd", &Hmi_t::staCmdSet, &hmi);
  server.register_handler("SetKeyCmd", &Hmi_t::keyCmdSet, &hmi);
  server.register_handler("SetKeyCode", &Hmi_t::KeyCodeSet, &hmi);

  server.register_handler("Quit", [&](rpc_conn conn)
                          { isRunning = false; });
  server.set_network_err_callback(
      [](std::shared_ptr<connection> conn, std::string reason)
      {
        std::cout << "remote client address: " << conn->remote_address()
                  << " networking error, reason: " << reason << "\n";
      });

  isRunning = true;
  server.async_run();
 

  while(isRunning)
  {

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  }
  printf("Prepare to Stop Server!\\rn");
  server.Stop();
  printf("Server Stopped!!!\r\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  return 0;
}