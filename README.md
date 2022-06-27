One node calls periodically (from a thread each 100ms) and sequentially N different services (4 in my example) using N clients. Each call is executed calling async_send_request and waited for using "wait_for" on the returned future. The node is added to an executor (tested with both Multithreaded and Singlehtreaded). Everything apparently works fine for a while, but seldomly the wait_for will hang and never get the result, even if the server generated a response. This happens much more frequently if the load on the system is increased (using for example an online multi-thread stress test like silver.urih) or on slow machines. Adding each client to a different callback group doesn't solve the issue, even with a multithreaded executor (but reduces its frequency). Using a timer to execute the calls periodically instead of a thread doesn't help either. If the service is called again after the timeout of wait_for expired, it will usually respond properly.

## SETUP ENVIRONMENT:

Setup a docker environment:

```
docker build -t ros2_bug --pull .
```

## EXECUTE:

Execute a docker environment:

```
docker run -it --rm ros2_bug
```

TO RUN A STRESS TEST (and reproduce the bug):

Go to https://silver.urih.com/ and click STRESS_TEST or use any stress test using multi-threading.

## EXAMPLE OF OUTPUT:

```
$ docker run -it --rm ros2_bug
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-27-16-06-03-034508-090365e31010-1
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [service_server-1]: process started with pid [58]
[INFO] [service_server-2]: process started with pid [60]
[INFO] [service_server-3]: process started with pid [62]
[INFO] [service_server-4]: process started with pid [64]
[INFO] [service_client-5]: process started with pid [66]
[service_client-5] [INFO] [1656345963.173264234] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.317127563] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.317405294] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.317481737] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.317651615] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.317850668] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.317882087] [service_client]: Calling service /service_server_3
[service_server-3] [INFO] [1656345963.317998044] [service_server_3]: Server service_server_3 Responding
[service_client-5] [INFO] [1656345963.318160339] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.318188161] [service_client]: Calling service /service_server_4
[service_server-4] [INFO] [1656345963.318329455] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.318485738] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.418638715] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.418863897] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.419069623] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.419112243] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.419252976] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.419403779] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.419443463] [service_client]: Calling service /service_server_3
[service_client-5] [INFO] [1656345963.419683403] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.419724530] [service_client]: Calling service /service_server_4
[service_server-3] [INFO] [1656345963.419558389] [service_server_3]: Server service_server_3 Responding
[service_server-4] [INFO] [1656345963.419856076] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.420009624] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.520156860] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.520348580] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.520502238] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.520541461] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.520684920] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.520835722] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.520874766] [service_client]: Calling service /service_server_3
[service_client-5] [INFO] [1656345963.521053220] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.521089879] [service_client]: Calling service /service_server_4
[service_server-3] [INFO] [1656345963.520957531] [service_server_3]: Server service_server_3 Responding
[service_server-4] [INFO] [1656345963.521215334] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.521368321] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.621515316] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.621728335] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.621933650] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.621972984] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.622092137] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.622233181] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.622292302] [service_client]: Calling service /service_server_3
[service_client-5] [INFO] [1656345963.622544825] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.622584480] [service_client]: Calling service /service_server_4
[service_server-3] [INFO] [1656345963.622409411] [service_server_3]: Server service_server_3 Responding
[service_server-4] [INFO] [1656345963.622697301] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.622834408] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.723051985] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.723415887] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.723663832] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.723722872] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.723871170] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.724126659] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.724153620] [service_client]: Calling service /service_server_3
[service_server-3] [INFO] [1656345963.724291638] [service_server_3]: Server service_server_3 Responding
[service_client-5] [INFO] [1656345963.724442731] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.724468049] [service_client]: Calling service /service_server_4
[service_server-4] [INFO] [1656345963.725560065] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.725691241] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.825866229] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.826278482] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.826464450] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.826493565] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.826609181] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.826731981] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.826753201] [service_client]: Calling service /service_server_3
[service_client-5] [INFO] [1656345963.826931215] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.826954629] [service_client]: Calling service /service_server_4
[service_server-3] [INFO] [1656345963.826845254] [service_server_3]: Server service_server_3 Responding
[service_server-4] [INFO] [1656345963.827107335] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.827203946] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345963.927390396] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345963.927856218] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345963.928218297] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345963.928252601] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345963.928487612] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345963.928680273] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345963.928708385] [service_client]: Calling service /service_server_3
[service_server-3] [INFO] [1656345963.928854840] [service_server_3]: Server service_server_3 Responding
[service_client-5] [INFO] [1656345963.929020480] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345963.929035799] [service_client]: Calling service /service_server_4
[service_server-4] [INFO] [1656345963.929161004] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345963.929269747] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345964.029491974] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345964.030107948] [service_server_1]: Server service_server_1 Responding
[service_client-5] [INFO] [1656345964.030404775] [service_client]: Waiting succeeded for server /service_server_1
[service_client-5] [INFO] [1656345964.030438237] [service_client]: Calling service /service_server_2
[service_server-2] [INFO] [1656345964.030625147] [service_server_2]: Server service_server_2 Responding
[service_client-5] [INFO] [1656345964.030775429] [service_client]: Waiting succeeded for server /service_server_2
[service_client-5] [INFO] [1656345964.030806147] [service_client]: Calling service /service_server_3
[service_server-3] [INFO] [1656345964.031038632] [service_server_3]: Server service_server_3 Responding
[service_client-5] [INFO] [1656345964.031260308] [service_client]: Waiting succeeded for server /service_server_3
[service_client-5] [INFO] [1656345964.031284744] [service_client]: Calling service /service_server_4
[service_server-4] [INFO] [1656345964.031466204] [service_server_4]: Server service_server_4 Responding
[service_client-5] [INFO] [1656345964.031666198] [service_client]: Waiting succeeded for server /service_server_4
[service_client-5] [INFO] [1656345964.131882092] [service_client]: Calling service /service_server_1
[service_server-1] [INFO] [1656345964.132467650] [service_server_1]: Server service_server_1 Responding
[service_client-5] [ERROR] [1656345967.132796249] [service_client]: Waiting result failed for server /service_server_1
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[service_client-5] [INFO] [1656345996.984127853] [rclcpp]: signal_handler(signum=2)
[service_server-3] [INFO] [1656345996.984127132] [rclcpp]: signal_handler(signum=2)
[service_server-2] [INFO] [1656345996.984127963] [rclcpp]: signal_handler(signum=2)
[service_server-1] [INFO] [1656345996.984130498] [rclcpp]: signal_handler(signum=2)
[service_server-4] [INFO] [1656345996.984149303] [rclcpp]: signal_handler(signum=2)
[INFO] [service_server-4]: process has finished cleanly [pid 64]
[INFO] [service_server-3]: process has finished cleanly [pid 62]
[INFO] [service_server-1]: process has finished cleanly [pid 58]
[INFO] [service_server-2]: process has finished cleanly [pid 60]
[INFO] [service_client-5]: process has finished cleanly [pid 66]
```

The execution will stop after getting the first error.

I could not reproduce using galactic or a non-docker environment.
Tested with rmw_fastrtps and cyclone-dds.

Maybe related issue: https://github.com/ros2/rmw_fastrtps/issues/392
