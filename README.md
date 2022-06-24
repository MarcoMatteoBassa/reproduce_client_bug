One node calls periodically (from a thread each 100ms) and sequentially N different services (4 in my example) using N clients. Each call is executed calling async_send_request and waited for using "wait_for" on the returned future. The node is added to an executor (tested with both Multithreaded and Singlehtreaded). Everything apparently works fine for a while, but seldomly the wait_for will hang and never get the result, even if the server generated a response. This happens much more frequently if the load on the system is increased (using for example an online multi-thread stress test like silver.urih) or on slow machines. Adding each client to a different callback group doesn't solve the issue, even with a multithreaded executor (but reduces its frequency). Using a timer to execute the calls periodically instead of a thread doesn't help either. If the service is called again after the timeout of wait_for expired, it will usually respond properly.

SETUP ENVIRONMENT:

Setup a docker environment from osrf/ros:humble-desktop:

./docker/build_image_humble.sh
./docker/run_docker_humble.sh

COMPILE:

run "colcon build" from the parent folder

EXECUTE:

source install/setup.sh
ros2 launch test_service_client launch_client_servers.launch.py 

TO RUN A STRESS TEST (and reproduce the bug):

Go to https://silver.urih.com/ and click STRESS_TEST or use any stress test using multi-threading.

EXAMPLE OF OUTPUT:

[service_client-5] [INFO] [1655999164.254612342] [service_client]: Calling service /service_server_1

[service_server-1] [INFO] [1655999164.254873437] [service_server_1]: Server service_server_1 Responding

[service_client-5] [INFO] [1655999164.255034186] [service_client]: Waiting succeeded for server /service_server_1

[service_client-5] [INFO] [1655999164.255078482] [service_client]: Calling service /service_server_2

[service_server-2] [INFO] [1655999164.255172132] [service_server_2]: Server service_server_2 Responding

[service_client-5] [INFO] [1655999164.255276784] [service_client]: Waiting succeeded for server /service_server_2

[service_client-5] [INFO] [1655999164.255302071] [service_client]: Calling service /service_server_3

[service_server-3] [INFO] [1655999164.255386534] [service_server_3]: Server service_server_3 Responding

[service_client-5] [INFO] [1655999164.255486535] [service_client]: Waiting succeeded for server /service_server_3

[service_client-5] [INFO] [1655999164.255526302] [service_client]: Calling service /service_server_4

[service_server-4] [INFO] [1655999164.255631679] [service_server_4]: Server service_server_4 Responding

[service_client-5] [INFO] [1655999164.255744601] [service_client]: Waiting succeeded for server /service_server_4

[service_client-5] [INFO] [1655999164.355946003] [service_client]: Calling service /service_server_1

[service_server-1] [INFO] [1655999164.356461594] [service_server_1]: Server service_server_1 Responding

[service_client-5] [INFO] [1655999164.356853143] [service_client]: Waiting succeeded for server /service_server_1

[service_client-5] [INFO] [1655999164.356904203] [service_client]: Calling service /service_server_2

[service_server-2] [INFO] [1655999164.357118180] [service_server_2]: Server service_server_2 Responding

[service_client-5] [ERROR] [1655999167.357424851] [service_client]: Waiting result failed for server /service_server_2

The execution will stop after getting the first error.

I could not reproduce using galactic or a non-docker environment.
Tested with rmw_fastrtps and cyclone-dds.

Maybe related issue: https://github.com/ros2/rmw_fastrtps/issues/392
