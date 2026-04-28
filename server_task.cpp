#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>

static bool set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0;
}

static std::string read_file(const std::string& path)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return {};
    std::string out;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n > 0)
    {
        out.resize((size_t)n);
        std::fread(out.data(), 1, (size_t)n, f);
    }
    std::fclose(f);
    return out;
}

static const char* content_type_for_path(const std::string& path)
{
    if (path.size() >= 5 && path.rfind(".html") == path.size()-5) return "text/html; charset=utf-8";
    if (path.size() >= 3 && path.rfind(".js")   == path.size()-3) return "application/javascript; charset=utf-8";
    if (path.size() >= 4 && path.rfind(".css")  == path.size()-4) return "text/css; charset=utf-8";
    return "application/octet-stream";
}

static void send_all(int fd, const std::string& s)
{
    const char* p = s.data();
    size_t left = s.size();
    while (left > 0)
    {
        ssize_t n = ::send(fd, p, left, 0);
        if (n <= 0) break;
        p += n;
        left -= (size_t)n;
    }
}

static std::string telemetry_json(const SharedState::Snapshot& s)
{
    // Note: keep this JSON stable for the frontend.
    char buf[1400];

    std::snprintf(buf, sizeof(buf),
        "{"
        "\"t_us\":%llu,"
        "\"valid\":%s,"
        "\"eul\":[%.7g,%.7g,%.7g],"
        "\"q\":[%.7g,%.7g,%.7g,%.7g],"
        "\"omega\":[%.7g,%.7g,%.7g],"
        "\"wheel_present\":[%s,%s,%s],"
        "\"wheel_rpm\":[%.7g,%.7g,%.7g],"
        "\"wheel_torque_cmd_nm\":[%.7g,%.7g,%.7g]"
        "}",
        (unsigned long long)s.t_us,
        s.est_imu.valid ? "true" : "false",
        (s.est_euler.roll_rad * 180.0f / 3.14159265f), (s.est_euler.pitch_rad * 180.0f / 3.14159265f), (s.est_euler.yaw_rad * 180.0f / 3.14159265f),
        s.est_imu.q[0], s.est_imu.q[1], s.est_imu.q[2], s.est_imu.q[3],
        s.est_imu.omega[0], s.est_imu.omega[1], s.est_imu.omega[2],
        s.wheels[0].present ? "true" : "false",
        s.wheels[1].present ? "true" : "false",
        s.wheels[2].present ? "true" : "false",
        s.wheels[0].rpm, s.wheels[1].rpm, s.wheels[2].rpm,
        s.wheel_torque_cmd_nm[0], s.wheel_torque_cmd_nm[1], s.wheel_torque_cmd_nm[2]
    );

    return std::string(buf);
}

// Very small HTTP parser (good enough for GET /, /telemetry, and static files).
static bool parse_http_request_line(const std::string& req, std::string& method, std::string& path)
{
    auto end = req.find("\r\n");
    if (end == std::string::npos) return false;
    auto line = req.substr(0, end);

    auto sp1 = line.find(' ');
    if (sp1 == std::string::npos) return false;
    auto sp2 = line.find(' ', sp1 + 1);
    if (sp2 == std::string::npos) return false;

    method = line.substr(0, sp1);
    path   = line.substr(sp1 + 1, sp2 - (sp1 + 1));
    return true;
}

static void respond_404(int fd)
{
    const std::string hdr =
        "HTTP/1.1 404 Not Found\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n\r\n";
    send_all(fd, hdr);
}

static void respond_200(int fd, const char* ct, const std::string& body)
{
    char hdr[256];
    std::snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Cache-Control: no-cache\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n\r\n",
        ct, body.size());
    send_all(fd, std::string(hdr) + body);
}

void server_task(SharedState& shared, std::atomic<bool>& stop_flag, int port, const std::string& web_root)
{
    int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0)
    {
        std::perror("[server] socket");
        return;
    }

    int opt = 1;
    (void)::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (::bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        std::perror("[server] bind");
        ::close(srv);
        return;
    }

    if (::listen(srv, 8) < 0)
    {
        std::perror("[server] listen");
        ::close(srv);
        return;
    }

    if (!set_nonblocking(srv))
    {
        std::perror("[server] nonblocking");
        // continue anyway
    }

    std::printf("[server] Listening on http://0.0.0.0:%d\n", port);

    while (!stop_flag.load())
    {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int client = ::accept(srv, (sockaddr*)&caddr, &clen);
        if (client < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            std::perror("[server] accept");
            break;
        }

        char reqbuf[2048];
        ssize_t n = ::recv(client, reqbuf, sizeof(reqbuf)-1, 0);
        if (n <= 0) { ::close(client); continue; }
        reqbuf[n] = 0;
        std::string req(reqbuf);

        std::string method, path;
        if (!parse_http_request_line(req, method, path))
        {
            respond_404(client);
            ::close(client);
            continue;
        }

        if (method != "GET")
        {
            respond_404(client);
            ::close(client);
            continue;
        }

        if (path == "/" || path == "/index.html")
        {
            auto body = read_file(web_root + "/index.html");
            if (body.empty()) respond_404(client);
            else respond_200(client, "text/html; charset=utf-8", body);
        }
        else if (path == "/plots.js" || path == "/style.css")
        {
            auto body = read_file(web_root + path);
            if (body.empty()) respond_404(client);
            else respond_200(client, content_type_for_path(path), body);
        }
        else if (path == "/telemetry")
        {
            auto snap = shared.snapshot();
            auto body = telemetry_json(snap);
            respond_200(client, "application/json; charset=utf-8", body);
        }
        else
        {
            respond_404(client);
        }

        ::close(client);
    }

    ::close(srv);
}
