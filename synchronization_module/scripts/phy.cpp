#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <zlib.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <random>
#include <pybind11/embed.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>

#include "pyquaternion.hpp"

using namespace std;
namespace py = pybind11;

void run_protobuf_server(map<string, py::object> config);
phyud::PhysicsUpdate parse_request(const char* req, size_t req_size);
string gen_response(const phyud::PhysicsUpdate& time_update, int num_nodes);
string generate_data(int num_nodes);

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "usage: net_sim_dummy.cpp <config_file>" << endl;
        return 0;
    }
    else
    {
        ifstream config_file(argv[1]);
        if (!config_file.is_open())
        {
            cout << "Cannot open configuration file: " << argv[1] << endl;
            return 0;
        }
        py::scoped_interpreter guard{};  // start the interpreter and keep it alive
        py::dict config = py::dict(py::module::import("yaml").attr("safe_load")(config_file));
        if (config.contains("do_driver_transfer") && py::bool_(config["do_driver_transfer"]))
        {
            py::module::import("multiprocessing").attr("Process")(py::module::import("driver_process"), config).attr("start")();
        }
        run_protobuf_server(config);
        if (config.contains("do_driver_transfer") && py::bool_(config["do_driver_transfer"]))
        {
            py::module::import("ranging_process").attr("join")();
        }
    }
    return 0;
}

void run_protobuf_server(map<string, py::object> config)
{
    int sock, connection;
    struct sockaddr_un server_address;
    struct sockaddr_in ip_server_address;
    char buf[1024];
    ssize_t len;
    socklen_t addr_size;
    string server_address_str;
    if (py::bool_(config["phy_use_uds"]))
    {
        server_address.sun_family = AF_UNIX;
        server_address_str = py::str(config["phy_uds_server_address"]);
        strncpy(server_address.sun_path, server_address_str.c_str(), sizeof(server_address.sun_path) - 1);
        server_address.sun_path[sizeof(server_address.sun_path) - 1] = '\0';
        sock = socket(AF_UNIX, SOCK_STREAM, 0);
        unlink(server_address.sun_path);
        bind(sock, (struct sockaddr*)&server_address, sizeof(server_address));
    }
    else
    {
        server_address_str = py::str(config["phy_ip_server_address"]);
        memset(&ip_server_address, 0, sizeof(ip_server_address));
        ip_server_address.sin_family = AF_INET;
        ip_server_address.sin_port = htons(py::int_(config["phy_ip_server_port"]));
        inet_pton(AF_INET, server_address_str.c_str(), &ip_server_address.sin_addr);
        sock = socket(AF_INET, SOCK_STREAM, 0);
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&

std::mutex socket_list_mutex;
std::vector<int> socket_list;

void connect(int id, const std::string& addr) {
    struct sockaddr_un sa;
    sa.sun_family = AF_UNIX;
    std::strcpy(sa.sun_path, addr.c_str());
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    bind(fd, (struct sockaddr*)&sa, sizeof(sa));
    listen(fd, 1);
    int conn_fd = accept(fd, NULL, NULL);
    std::cout << "Connected to " << addr << std::endl;
    if (id == 1) std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::string src_mac = "1a", dst_mac = "2a";
    if (id == 1) std::swap(src_mac, dst_mac);
    std::string request = "{\"type\": \"driver_request\",\"src_mac\": \"" + src_mac + "\",\"dst_mac\": \"" + dst_mac + "\"}";
    try {
        while (true) {
            std::cout << "Sent: " << request << std::endl;
            send(conn_fd, request.c_str(), request.length(), 0);
            char buffer[1024];
            int n = recv(conn_fd, buffer, 1024, MSG_DONTWAIT);
            if (n > 0) std::cout << "Received: " << std::string(buffer, n) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    } catch (std::exception e) {
        return;
    }
}

void driver_process(const YAML::Node& config) {
    std::vector<std::string> addresses;
    for (int i = 0; i < config["mac_list"].size(); ++i) {
        std::string address = config["phy_driver_uds_server_address"].as<std::string>() + std::to_string(i);
        addresses.push_back(address);
    }

    std::vector<std::thread> threads;
    int id = 0;
    for (const auto& address : addresses) {
        threads.emplace_back(std::thread(connect, id, address));
        ++id;
    }
    for (auto& thread : threads) thread.join();

    std::cout << std::endl << "Exiting physics simulator dummy driver process" << std::endl;
    std::lock_guard<std::mutex> lock(socket_list_mutex);
    for (auto& sock : socket_list) close(sock);
    for (const auto& address : addresses) unlink(address.c_str());
}

void run_protobuf_server(const YAML::Node& config) {
    // implementation not provided
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "usage: phy.cpp <config_file>" << std::endl;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(argv[1]);
    if (config["do_driver_transfer"].as<bool>()) {
        std::thread driver_thread(driver_process, config);
        driver_thread.detach();
    }
    run_protobuf_server(config);
    if (config["do_driver_transfer"].as<bool>()) {
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
