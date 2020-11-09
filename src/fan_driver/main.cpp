#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>
#include <thread>
#include <condition_variable>

#include <cerrno>
#include <csignal>
#include <climits>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/inotify.h>
#include <linux/serial.h>

using namespace std;

volatile bool exit_now = false;
volatile int temperature = 0;
condition_variable cv;
mutex cv_m, clog_m;
int thread_count = 0;

class arduino_error : public system_error {
private:
    string msg;
    string device;

public:
    arduino_error(int ev, const char *msg, const char *device) :
        system_error(ev, system_category()),
        msg(msg),
        device(device)
    { }

    virtual const char* what() const noexcept override {
        return msg.c_str();
    }

    void print() const {
        cerr << "{" << device << "} :: " << msg << "  --  [" << int(code().value()) << "] :: " << code().message() << '\n';
    }
};

class arduino_device {
private:
    int fd = -1;
    arduino_error error;

    bool wait_io(short event, chrono::time_point<chrono::steady_clock> deadline) {
        int ms = chrono::duration_cast<chrono::milliseconds>(deadline - chrono::steady_clock::now()).count();
        if (ms < 0)
            return false;

        struct pollfd p_fd = {
            .fd = fd,
            .events = event
        };

        int ret = poll(&p_fd, 1, ms);
        if (ret <= 0)
            return false;

        return p_fd.revents & event;
    }

public:
    const arduino_error &last_error = error;
    const string name;

    operator int() {
        return fd;
    }

    arduino_device(string device) :
        error(0, "success", device.c_str()),
        name(device)
    {
        struct termios toptions;

        fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC | O_SYNC);

        if (fd == -1)
            throw arduino_error(errno, "cannot open serial port", name.c_str());
        if (tcgetattr(fd, &toptions) < 0)
            throw arduino_error(errno, "cannot get attributes of the serial port", name.c_str());
        if (cfsetispeed(&toptions, B9600) < 0)
            throw arduino_error(errno, "cannot set serial port input speed", name.c_str());
        if (cfsetospeed(&toptions, B9600) < 0)
            throw arduino_error(errno, "cannot set serial port output speed", name.c_str());

        toptions.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        toptions.c_cflag |= CS8 | CREAD | CLOCAL | HUPCL;

        toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
        toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        toptions.c_oflag &= ~OPOST;

        toptions.c_cc[VMIN] = 0;
        toptions.c_cc[VTIME] = 20;

        if (tcsetattr(fd, TCSANOW, &toptions) < 0)
            throw arduino_error(errno, "cannot set attributes of the serial port", name.c_str());

        if (!geteuid()) {
            struct serial_struct serinfo;
            serinfo.reserved_char[0] = 0;

            if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
                throw arduino_error(errno, "cannot get serial port info", name.c_str());

            serinfo.closing_wait = 500; // 5 seconds

            if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
                throw arduino_error(errno, "cannot set serial port info", name.c_str());
        }

        tcflush(fd, TCIOFLUSH);
    }

    ~arduino_device() {
        if (fd >= 0) {
            tcflush(fd, TCIOFLUSH);
            close(fd);
            fd = -1;
        }
    }

    vector<uint8_t> read_raw(size_t max_read = 256, int timeout = 10000) {
        auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout);
        if (!wait_io(POLLIN, deadline)) {
            error = arduino_error(ETIMEDOUT, "raw read timeout", name.c_str());
            return {};
        }

        vector<uint8_t> buffer(max_read);
        uint8_t *data = buffer.data();

        ssize_t result = ::read(fd, data, max_read);

        if (result <= 0) {
            error = arduino_error(errno, "cannot read serial port", name.c_str());
            return {};
        }

        buffer.resize(result);
        buffer.shrink_to_fit();
        return buffer;
    }

    vector<uint8_t> read(int timeout = 10000) {
        auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout);
        if (!wait_io(POLLIN, deadline)) {
            error = arduino_error(ETIMEDOUT, "read timeout (length)", name.c_str());
            return {};
        }

        uint8_t len;
        ssize_t result = ::read(fd, &len, sizeof(uint8_t));

        if (result <= 0) {
            error = arduino_error(errno, "cannot read lenght of the command", name.c_str());
            return {};
        }

        vector<uint8_t> buffer(len);
        uint8_t *data = buffer.data();

        while (len) {
            if (!wait_io(POLLIN, deadline)) {
                error = arduino_error(ETIMEDOUT, "read timeout (body)", name.c_str());
                return {};
            }

            result = ::read(fd, data, len);

            if (result <= 0) {
                error = arduino_error(errno, "cannot read body of the command", name.c_str());
                return {};
            }

            data += result;
            len -= result;
        }

        return buffer;
    }

    bool write(const vector<uint8_t> &buffer) {
        if (buffer.size() > 255) {
            error = arduino_error(EMSGSIZE, "command data is too long", name.c_str());
            return false;
        }

        uint8_t len = buffer.size();

        if (::write(fd, &len, sizeof(uint8_t)) != 1) {
            error = arduino_error(errno, "cannot write serial port (length)", name.c_str());
            return false;
        }

        const uint8_t *data = buffer.data();
        while (len > 0) {
            ssize_t written = ::write(fd, data, len);

            if (written <= 0) {
                error = arduino_error(errno, "cannot write serial port (body)", name.c_str());
                return false;
            }

            data += written;
            len -= written;
        }

        return true;
    }
};

enum class command_code : uint16_t {
    connect     = 0x4849, // HI
    disconnect  = 0x4242, // BB
    reject      = 0x4e4f, // NO
    temperature = 0x544d, // TM
    speed       = 0x5350, // SP
    invalid     = 0
};

class command {
private:
    command_code code = command_code::invalid;
    uint32_t data = 0;

    vector<uint8_t> to_bytes() const {
        size_t len;
        switch (code) {
            case command_code::temperature:
                len = 6; break;
            case command_code::speed:
                len = 3; break;
            default:
                len = 2;
        }
        vector<uint8_t> bytes(len);
        bytes[0] = (uint16_t(code) & 0xff00) >> 8;
        bytes[1] = (uint16_t(code) & 0x00ff);

        switch (len) {
            case 3:
                bytes[2] = data & 0x000000ff;
                break;
            case 6:
                bytes[2] = (data & 0xff000000) >> 24;
                bytes[3] = (data & 0x00ff0000) >> 16;
                bytes[4] = (data & 0x0000ff00) >> 8;
                bytes[5] = (data & 0x000000ff);
                break;
        }

        return bytes;
    }

    command_code make_conform(command_code code) {
        switch (code) {
            case command_code::connect:
            case command_code::disconnect:
            case command_code::reject:
            case command_code::temperature:
            case command_code::speed:
                return code;
            default:
                return command_code::invalid;
        }
    }

public:
    operator vector<uint8_t>() const {
        return to_bytes();
    }

    operator command_code() const {
        return code;
    }

    command(const vector<uint8_t> &bytes) {
        if (bytes.size() < 2) {
            code = command_code::invalid;
            return;
        }

        code = make_conform(command_code((bytes[0] << 8) | bytes[1]));

        switch (code) {
            case command_code::temperature: {
                if (bytes.size() < 6)
                    break;

                data = (bytes[2] << 24) | (bytes[3] << 16) | (bytes[4] << 8) | bytes[5];
            } return;
            case command_code::speed: {
                if (bytes.size() < 3)
                    break;

                data = bytes[2];
            } return;
            default:
                return;
        }
    }

    command(command_code code, uint32_t data = 0) :
        data(data)
    {
        this->code = make_conform(code);
    }

    uint32_t get_data() const {
        return data;
    }
};

void read_temperature() {
    int fd = -1;
    char buffer[16] = {0};

    while (!exit_now) {
        fd = open("/sys/class/thermal/thermal_zone0/temp", O_RDONLY | O_NOCTTY);
        if (fd != -1) {
            int len = read(fd, buffer, sizeof(buffer));
            if (len > 0)
                temperature = stoi(buffer);

            close(fd);
        }
        usleep(1000 * 1000);
    }
}

bool connect(arduino_device &device) {
    command cmd = command(command_code::connect);

    if (!device.write(cmd)) {
        unique_lock<mutex> lk(clog_m);
        device.last_error.print();
    }

    cmd = command(device.read(5000));

    if (cmd == command_code::invalid) {
        unique_lock<mutex> lk(clog_m);
        device.last_error.print();
    }

    return cmd == command_code::connect;
}

void comm_loop(arduino_device &device) {
    int fail_count = 0;
    uint8_t fan_speed = 0;
    command cmd = command_code::invalid;
    auto log_time = chrono::steady_clock::now() + chrono::seconds(60);

    while (!exit_now) {
        if (fail_count > 9)
            return;

        if (chrono::steady_clock::now() >= log_time) {
            log_time = chrono::steady_clock::now() + chrono::seconds(60);
            unique_lock<mutex> lk(clog_m);
            clog << dec << "device {" << device.name << "} fan speed :: " << int(fan_speed) << "%\n";
        }

        cmd = command(command_code::temperature, temperature);
        if (!device.write(cmd)) {
            fail_count++;
            unique_lock<mutex> lk(clog_m);
            device.last_error.print();
            continue;
        }

        cmd = command(device.read());
        switch ((command_code)cmd) {
            case command_code::speed:
                fan_speed = cmd.get_data();
                break;
            case command_code::reject:
                fail_count++;
                break;
            case command_code::disconnect:
                return;
            case command_code::invalid: {
                unique_lock<mutex> lk(clog_m);
                device.last_error.print();
            } break;
            default:
                break;
        }

        usleep(1000 * 1000); // 1s
    }
}

void communicate(string path) {
    {
        unique_lock<mutex> lk(clog_m);
        clog << "establishing communications with :: {" << path << "}\n";
    }
    {
        unique_lock<mutex> lk(cv_m);
        thread_count++;
    }

    try {
        int fail_count = 0;
        arduino_device device(path);
        usleep(1000 * 1000 * 2); // 2s (waiting for arduino to restart)

        while (fail_count < 4 && !exit_now) {
            auto deadline = chrono::steady_clock::now() + chrono::milliseconds(5000);
            if (!connect(device)) {
		int ms = chrono::duration_cast<chrono::milliseconds>(deadline - chrono::steady_clock::now()).count();
                if (ms > 0)
		    usleep(1000 * ms);
                fail_count++;
                continue;
            }
            {
                unique_lock<mutex> lk(clog_m);
                clog << "connected to :: {" << path << "}\n";
            }

            comm_loop(device);
        }
    } catch (const arduino_error &e) {
        unique_lock<mutex> lk(clog_m);
        e.print();
    }

    {
        unique_lock<mutex> lk(clog_m);
        clog << "communication stopped with device :: {" << path << "}\n";
    }
    {
        unique_lock<mutex> lk(cv_m);
        thread_count--;
    }
    cv.notify_all();
}

static void interruption_handler(const int s) {
    exit_now = true;
}

int handle_signals() {
    struct sigaction sa;
    sa.sa_handler = interruption_handler;

    if(sigemptyset(&sa.sa_mask) == -1)
        return 1;

    sa.sa_flags = 0;

    if (sigaction(SIGINT, &sa, NULL) == -1)
        return 2;

    return 0;
}

int watch_devices() {
    int n_fd = -1;
    int w_fd = -1;
    int ret = 0;
    alignas(inotify_event) char buffer[sizeof(inotify_event) + NAME_MAX + 1];
    inotify_event *event = new (buffer) inotify_event;

    n_fd = inotify_init();
    if (n_fd < 0) {
        exit_now = true;
        ret = 1;
        goto stop_watching;
    }

    w_fd = inotify_add_watch(n_fd, "/dev", IN_CREATE);
    if (w_fd < 0) {
        exit_now = true;
        ret = 2;
        goto stop_watching;
    }

    while (true) {
        if (exit_now) {
            ret = 3;
            goto stop_watching;
        }

        ssize_t len = read(n_fd, buffer, sizeof(buffer));
        if (len < 1)
            continue;

        string path = event->name;
        if(path.starts_with("ttyACM"))
            thread(communicate, path.insert(0, "/dev/")).detach();
    }

stop_watching:
    inotify_rm_watch(n_fd, w_fd);
    if (n_fd != -1)
        close(n_fd);

    return ret;
}

int main(int argc, char *argv[]) {
    switch(handle_signals()) {
        case 1:
            cerr << "{handling signals} cannot initilize signal set\n"; break;
        case 2:
            cerr << "{handling signals} cannot change action on SIGINT\n"; break;
        default:
            break;
    }

    thread(read_temperature).detach();

    for (const auto& p : filesystem::directory_iterator("/dev")) {
        if(p.path().filename().string().starts_with("ttyACM")) {
            thread(communicate, p.path().string()).detach();
        }
    }

    switch(watch_devices()) {
        case 1:
            cerr << "{device watcher} cannot initilize inotify\n"; break;
        case 2:
            cerr << "{device watcher} cannot initilize watcher\n"; break;
        case 3:
            cerr << "{device watcher} watcher interrupted\n"; break;
        default:
            break;
    }

    {
        unique_lock<mutex> lk(cv_m);
        cv.wait(lk, []{ return thread_count == 0;});
    }

    return 0;
}
