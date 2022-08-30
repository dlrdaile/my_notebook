# Micro-Ros使用流程

## 安装micro-ros

### 1. micro_ros安装

- 首先source一下ros2的环境

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

- 新建文件夹，并拉取代码

```bash
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

- 更新软件源并安装所需依赖

```bash
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
```

- 若无pip，安装pip

```bash
sudo apt-get install python3-pip
```

- 进行编译

```bash
colcon build
source install/local_setup.bash
```

- 编译成功后，创建ros agent

> 此处建议将agent和setup放在同一工作空间

```text
ros2 run micro_ros_setup create_agent_ws.sh
```

- 编译ros agent

```bash
ros2 run micro_ros_setup build_agent.sh  
source install/local_setup.bash
```

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-cc2381b0c344e849062677fc00aa86e7_1440w.jpg)

- 报图中这样的命令是正常的；
- 至此，ros2中的操作告一段落；

### 2.基于相关平台的开发流程

#### Arduino

首先安装arduino IDE，自行解决。。。

我这里采用的是M5 stack Atom Lite的开发板，是基于esp32的芯片，也可以用arduino框架来开发，所以下位机程序就基于arduino了

这里有基于不同框架的micro_ros嵌入式对应的代码：[https://github.com/orgs/micro-ROS/repositories](https://link.zhihu.com/?target=https%3A//github.com/orgs/micro-ROS/repositories)

- 下载micro_ros_arduino代码

在这个链接[https://github.com/micro-ROS/micro_ros_arduino/releases](https://link.zhihu.com/?target=https%3A//github.com/micro-ROS/micro_ros_arduino/releases)

下载对应ros2版本的代码zip压缩文件，比如我用的foxy，那么下载[v2.0.5-foxy](https://link.zhihu.com/?target=https%3A//github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.5-foxy)

- 在arduino中添加下载的zip文件：

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-ace2d561c5e0adb929f87f5623fb779b_1440w.jpg)image-

- 在示例中找到micro_ros_arduino，里面有很多example code，这里以publisher为例：

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-3a14343f70f9ea70dfded71c19da614d_1440w.jpg)

- 编译并上传

#### platformio

https://github.com/micro-ROS/micro_ros_platformio

1. 创建一个新项目

2. 修改platformio.ini为

   ```ini
   [env:esp32dev]
   platform = espressif32
   board = esp32dev
   framework = arduino
   monitor_speed = 115200
   board_microros_transport = wifi
   board_microros_distro = humble
   lib_deps = 
   	https://github.com/micro-ROS/micro_ros_platformio
   ```

   * `board_microros_transport`：micro-ros和上位机通讯的方式，常用的包括`serial(默认)`、`wifi`、`native_ethernet`等
   * `board_microros_distro`：指明ros的版本

3. Now to proceed with the PlatformIO workflow:

   ```shell
   pio lib install # Install dependencies
   pio run # Build the firmware
   pio run --target upload # Flash the firmware
   ```

4. After the library is compiled for first time the build process will be skipped, to trigger a library build and apply [library modifications](https://github.com/micro-ROS/micro_ros_platformio#library-configuration) on your next platformIO build:

   ```shell
   pio run --target clean_microros  # Clean library
   ```

5. 在程序中创建通讯方法

   * serial

     ```cpp
     Serial.begin(115200);
     set_microros_serial_transports(Serial);
     ```

   * wifi

     ```cpp
     IPAddress agent_ip(192, 168, 1, 113);
     size_t agent_port = 8888;
     
     char ssid[] = "WIFI_SSID";
     char psk[]= "WIFI_PSK";
     
     set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
     ```

   * native_ethernet

     ```cpp
     byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
     IPAddress local_ip(192, 168, 1, 177);
     IPAddress agent_ip(192, 168, 1, 113);
     size_t agent_port = 8888;
     
     set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
     ```

   * custom

     ```cpp
     bool platformio_transport_open(struct uxrCustomTransport * transport) {...};
     bool platformio_transport_close(struct uxrCustomTransport * transport) {...};
     size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {...};
     size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {...};
     
     rmw_uros_set_custom_transport(
       MICROROS_TRANSPORTS_FRAMING_MODE, // Set the MICROROS_TRANSPORTS_FRAMING_MODE or MICROROS_TRANSPORTS_PACKET_MODE mode accordingly
       NULL,
       platformio_transport_open,
       platformio_transport_close,
       platformio_transport_write,
       platformio_transport_read
     );
     ```

##### Extra packages

Colcon packages can be added to the build process using this two methods:

- Package directories copied on the `<Project_directory>/extra_packages` folder.
- Git repositories included on the `<Project_directory>/extra_packages/extra_packages.repos` yaml file.

This should be used for example when adding custom messages types or custom micro-ROS packages.

### 3. 下位机连接上位机

#### 使用usb-serial

- 将下位机硬件连接到上位机，采用lsusb命令观察是否正确连接，我这里正确识别了下位机M5 stack Atom Lite

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-cc03b3415428cd6b7542643f00e2f5b0_1440w.jpg)

- 确保连接成功的前提下，首先给usb串口提升读写权限：

```text
sudo chmod -R 777 /dev/ttyUSB0
```

- 不给usb串口提权限的话，会报下面的错

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-7839d29ae47576c159a141acd0db5bdc_1440w.png)

- 然后在ubuntu ros2中，首先source一下ros环境，再source一下安装的micro_ros_agent的环境
- 正式运行micro-agent:

```text
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

#### 使用wifi

```text
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

- 运行成功是这样的：

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-94f9cc4e3badcfc889e32ef716925dab_1440w.jpg)

此时，按一下下位机的**复位or重启按钮**

- 重新打开一个终端，source一下环境。然后运行`ros2 topic list`，出现了下位机程序中的publisher相关的topic

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-e4e8f73cdd8d9be48178afde6be6f6b2_1440w.jpg)

- 利用ros2 topic echo打印publisher发布的内容：

![img](/home/dllr/Desktop/notebook/ros2/Micro-Ros使用流程.assets/v2-d5bb4b2bb3bc39548e233670fda4edda_1440w.jpg)

至此，micro_ros的环境就算配置完成了。

## 使用micro_ros

### 节点相关

#### 1. 节点初始化

- 使用默认方式创建节点:

  ```cpp
  // 初始化micro_ros allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  // 初始化 support 对象
  rclc_support_t support;
  rcl_ret_t rc = rclc_support_init(&support, argc, argv, &allocator);
  
  // 创建 node 对象
  rcl_node_t node;
  const char * node_name = "test_node";
  
  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";
  
  // 初始化默认节点
  rc = rclc_node_init_default(&node, node_name, namespace, &support);
  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

- 创建带有自定义选项的节点:

  > 对于节点的配置也可以用于后续其他对象的配置 (Publishers, subscribers, services, …)
  >
  > **注意**不同版本的API配置会有所不同:

  Foxy: The `rcl_node_options_t` is used to configure the node

  ```cpp
  // Initialize allocator and support objects
  ...
  
  // Create node object
  rcl_node_t node;
  const char * node_name = "test_node";
  
  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";
  
  // Get default node options and modify them
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  
  // Set node ROS domain ID to 10
  node_ops.domain_id = (size_t)(10);
  
  // Init node with custom options
  rc = rclc_node_init_with_options(&node, node_name, namespace, &support, &node_ops);
  
  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

  Galactic and beyond: In this case, the node options are configured on the `rclc_support_t` object with a custom API

  ```cpp
  // Initialize micro-ROS allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  // Initialize and modify options (Set DOMAIN ID to 10)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);
  
  // Initialize rclc support object with custom options
  rclc_support_t support;
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
  // Create node object
  rcl_node_t node;
  const char * node_name = "test_node";
  
  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";
  
  // Init node with configured support object
  rclc_node_init_default(&node, node_name, namespace, &support);
  
  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

#### 2. 清除节点

> 在清楚一个节点之前必须先清除和其相关的所有发布，订阅，服务等对象

```cpp
// Destroy created entities (Example)
rcl_publisher_fini(&publisher, &node);
...

// Destroy the node
rcl_node_fini(&node);
```

This will delete the node from ROS2 graph, including any generated infrastructure on the agent (if possible) and used memory on the client.

### 话题相关

#### 发布者相关

##### 1. 初始化发布对象

> 根据不同的qos配置需求，micro_ros提供了三种不同的初始化发布者对象的方法

- Reliable (default):

  ```cpp
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Creates a reliable rcl publisher
  rcl_ret_t rc = rclc_publisher_init_default(
    &publisher, &node,
    type_support, topic_name);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```cpp
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Creates a best effort rcl publisher
  rcl_ret_t rc = rclc_publisher_init_best_effort(
    &publisher, &node,
    type_support, topic_name);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```cpp
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Set publisher QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;
  
  // Creates a rcl publisher with customized quality-of-service options
  rcl_ret_t rc = rclc_publisher_init(
    &publisher, &node,
    type_support, topic_name, qos_profile);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

  For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the [QoS tutorial](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/qos/).

##### 2. 发布一个消息

To publish messages to the topic:

```
// Int32 message object
std_msgs__msg__Int32 msg;

// Set message value
msg.data = 0;

// Publish message
rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}
```

For periodic publications, `rcl_publish` can be placed inside a timer callback. Check the [Executor and timers](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/executor/) section for details.

Note: `rcl_publish` is thread safe and can be called from multiple threads.

#### 订阅者相关

##### 1.初始化订阅者

> 订阅者的初始化和发布者相当:

- Reliable (default):

  ```
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Initialize a reliable subscriber
  rcl_ret_t rc = rclc_subscription_init_default(
    &subscriber, &node,
    type_support, topic_name);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Initialize best effort subscriber
  rcl_ret_t rc = rclc_subscription_init_best_effort(
    &subscriber, &node,
    type_support, topic_name);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  
  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;
  
  // Initialize a subscriber with customized quality-of-service options
  rcl_ret_t rc = rclc_subscription_init(
    &subscriber, &node,
    type_support, topic_name, qos_profile);
  
  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

<span id="qos_jump">For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the [QoS tutorial](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/qos/).</span>

##### 2. 设置订阅回调函数

> 当消息发布时，executor负责调用配置的回调函数，该函数将消息作为其唯一参数，包含发布者发送的值

```cpp
// 回调函数的函数制定定义形式
// void (* rclc_subscription_callback_t)(const void *);

// 应用示例:
void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
}
```

当`subscriber`和`executor` 被初始化了, `subscriber`的回调函数就必须添加到`executor`中，以便在`spinning`的时候接受到发布的话题信息并进行处理:

```cpp
// 待接受发布对象的话题信息
std_msgs__msg__Int32 msg;
// 创建执行器对象
rclc_executor_t executor;
// 初始化执行器
rcl_ret_t rc = rclc_executor_init(&executor, &support.context, 1, &allocator)
// 将话题和回调函数添加到执行器中
rcl_ret_t rc = rclc_executor_add_subscription(
  &executor, &subscriber, &msg,
  &subscription_callback, ON_NEW_DATA);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive messages
// rclc_executor_spin(&executor);
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))
```

#### 消息初始化(此处是大坑，挺麻烦的)

Before publishing or receiving a message, it may be necessary to initialize its memory for types with strings or sequences. Check the [Handling messages memory in micro-ROS](https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/) section for details.

> [自定义消息初始化的示例](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_types_handling/micro-ros_types_handling.ino)

#### 清除订阅者或发布者对象

After finishing the publisher/subscriber, the node will no longer be advertising that it is publishing/listening on the topic. To destroy an initialized publisher or subscriber:

```
// Destroy publisher
rcl_publisher_fini(&publisher, &node);

// Destroy subscriber
rcl_subscription_fini(&subscriber, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.

### 服务相关

#### 服务器相关

##### 1. 初始化服务器

和话题类似，有三种初始化服务的方法:

- Reliable (default):

  ```cpp
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Initialize server with default configuration
  rcl_ret_t rc = rclc_service_init_default(
    &service, &node,
    type_support, service_name);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```cpp
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Initialize server with default configuration
  rcl_ret_t rc = rclc_service_init_best_effort(
    &service, &node,
    type_support, service_name);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```cpp
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Set service QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;
  
  // Initialize server with customized quality-of-service options
  rcl_ret_t rc = rclc_service_init(
    &service, &node, type_support,
    service_name, qos_profile);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

​	和`QOS`相关的配置可以参考[话题处链接](#qos_jump)

##### 2. 服务器回调函数定义

当一个服务请求到来是，`executor`会调用所配置的`callback`函数，该函数以请求和响应的接口作为参数。

* 请求参数会包含客户端定义的消息值
* 响应消息会在该回调函数中被修改并作为返回值发布回去

Using `AddTwoInts.srv` type definition as an example:

```yaml
int64 a
int64 b
---
int64 sum
```

The client request message will contain two integers `a` and `b`, and expects the `sum` of them as a response:

```cpp
// Function prototype:
// void (* rclc_service_callback_t)(const void *, void *);

// Implementation example:
void service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  example_interfaces__srv__AddTwoInts_Request * req_in =
    (example_interfaces__srv__AddTwoInts_Request *) request_msg;
  example_interfaces__srv__AddTwoInts_Response * res_in =
    (example_interfaces__srv__AddTwoInts_Response *) response_msg;

  // Handle request message and set the response message values
  printf("Client requested sum of %d and %d.\n", (int) req_in->a, (int) req_in->b);
  res_in->sum = req_in->a + req_in->b;
}
```

> 此处注意将两种消息都转换为特定的类型

当`executor`和回调函数以及服务对象都被定义了之后，要将服务对象、服务接口对象、服务回调函数挂载到`executor`上:

```cpp
// Service message objects
example_interfaces__srv__AddTwoInts_Response response_msg;
example_interfaces__srv__AddTwoInts_Request request_msg;

// Add server callback to the executor
rc = rclc_executor_add_service(
  &executor, &service, &request_msg,
  &response_msg, service_callback);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive requests
rclc_executor_spin(&executor);
```

#### 客户端相关

##### 1. 初始化客户端

- Reliable (default):

  ```cpp
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Initialize client with default configuration
  rcl_ret_t rc = rclc_client_init_default(
    &client, &node,
    type_support, service_name);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```cpp
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Initialize client with default configuration
  rcl_ret_t rc = rclc_client_init_best_effort(
    &client, &node,
    type_support, service_name);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```cpp
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";
  
  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);
  
  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;
  
  // Initialize server with customized quality-of-service options
  rcl_ret_t rc = rclc_client_init(
    &client, &node, type_support,
    service_name, qos_profile);
  
  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```

##### 2. 定义客户端回调函数

当`executor`获得服务器的响应时，其会调用客户端的回调函数，该函数仅有响应数据作为其参数

> 注意将获得的数据类型转换为需要的类型

```cpp
// Function prototype:
// void (* rclc_client_callback_t)(const void *);

// Implementation example:
void client_callback(const void * response_msg){
  // Cast response message to expected type
  example_interfaces__srv__AddTwoInts_Response * msgin =
    (example_interfaces__srv__AddTwoInts_Response * ) response_msg;

  // Handle response message
  printf("Received service response %ld + %ld = %ld\n", req.a, req.b, msgin->sum);
}
```

当上述内容定义完成(执行器，客户端对象，消息类型，回调函数)，即可进行执行器关联，并调用spin相关函数进行消息检测

```cpp
// Response message object
example_interfaces__srv__AddTwoInts_Response res;

// Add client callback to the executor
rcl_ret_t rc = rclc_executor_add_client(&executor, &client, &res, client_callback);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive requests
rclc_executor_spin(&executor);
```

##### 3. 发送一个请求

当某一服务的服务器和客户端都进行了配置，则可以发布一个服务请求并调用spin函数去处理请求的响应报文。

Following the example on `AddTwoInts.srv`:

```cpp
// Request message object (Must match initialized client type support)
example_interfaces__srv__AddTwoInts_Request request_msg;

// Initialize request message memory and set its values
example_interfaces__srv__AddTwoInts_Request__init(&request_msg);
request_msg.a = 24;
request_msg.b = 42;

// Sequence number of the request (在rcl_send_request进行填充)
int64_t sequence_number;

// Send request
rcl_send_request(&client, &request_msg, &sequence_number);

// Spin the executor to get the response
rclc_executor_spin(&executor);
```

#### 清楚服务相关对象

To destroy an initialized service or client:

```
// Destroy service server and client
rcl_service_fini(&service, &node);
rcl_client_fini(&client, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.

### 参数配置相关

#### 参数服务器

## 一些比较重要的示例代码

* [自动重连接示例](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino)
* 