#include <iostream>
#include <string.h>
#include <vector>

#include <controlIO/controlIO_interface.h>

#include <math.h>
#include <chrono>

#include <core/core.h>
#include <core/logger.h>
#include <core/CoreScheduler/tasking.h>

#include <utils/utils_control.h>

#define PLUGIN_IO_NAME KneeR_Joelheira_2
#define PLUGIN_IO_TYPE ControlIO_plugin

class PLUGIN_IO_NAME : public PLUGIN_IO_TYPE
{

    float kp, kd, ki;
    float Kv, Bv;

    std::chrono::nanoseconds ts;

    double theta_d = 0;
    double pos0 = 0;
    double pos1 = 0;
    double Ks = 104;
    double GR = 150.0f;

    double theta_out_hip_r = 0;

    double fd;
    double f;

    double t_ms, vel;

    float emg_values[16];
    double emg2, emg2_f = 0;
    double emg3, emg3_f = 0;
    double Kv_emg = 0;

    VariableTrace theta_in, theta_out, error, dx, param0;

    LowPassFilter lf_omega = LowPassFilter(20, 0.001f);
    LowPassFilter lf_f = LowPassFilter(40, 0.001f);
    LowPassFilter Kv_f = LowPassFilter(4, 0.001f);

    LowPassFilter lf_emg2 = LowPassFilter(1, 0.001f);
    LowPassFilter lf_emg3 = LowPassFilter(1, 0.001f);

    CommIO_plugin *tcp_ = nullptr;

public:
    virtual ~PLUGIN_IO_NAME() = default;

    virtual bool config(const std::string &cfg) override
    {
        CONTROL_IO_INIT_CONFIG(cfg);

        CONFIG_VALIDATE(_CFG_FIELD_GET(Ks));
        CONFIG_VALIDATE(_CFG_FIELD_GET(GR));

        CONFIG_VALIDATE(_CFG_FIELD_GET(kp));
        CONFIG_VALIDATE(_CFG_FIELD_GET(kd));
        CONFIG_VALIDATE(_CFG_FIELD_GET(ki));

        CONFIG_VALIDATE(_CFG_FIELD_GET(Kv));
        CONFIG_VALIDATE(_CFG_FIELD_GET(Bv));

        CONFIG_VALIDATE(_CFG_FIELD_GET(ts));

        set_logger();

        tcp_ = core.plugins.get_node<CommIO_plugin>("tcp0");

        CONTROL_IO_FINISH_CONFIG_FULL;
    }

    void set_logger()
    {
        if (!_logger.enabled)
            return;

        auto &core = HH::Core::instance();

        this->_logger.file.emplace_back();
        this->_logger.file.back().set_file_name(core.config().logger.path + _logger.file_name + ".log");

        _logger.file[0].vars.push_back(RecordVariable{.name = "time", .format = "%.4f", ._ptr = &t_ms});
        _logger.file[0].vars.push_back(RecordVariable{.name = "theta_in", .fnc = [this]()
                                                                          { return theta_in.v(); }});
        _logger.file[0].vars.push_back(RecordVariable{.name = "dtheta_in", .fnc = [this]()
                                                                           { return theta_in.d(); }});
        _logger.file[0].vars.push_back(RecordVariable{.name = "theta_out", .fnc = [this]()
                                                                           { return theta_out.v(); }});
        _logger.file[0].vars.push_back(RecordVariable{.name = "dtheta_out", .fnc = [this]()
                                                                            { return theta_out.d(); }});
        _logger.file[0].vars.push_back(RecordVariable{.name = "error", .fnc = [this]()
                                                                       { return error.v(); }});
        _logger.file[0].vars.push_back(RecordVariable{.name = "theta_d", ._ptr = &theta_d});
        _logger.file[0].vars.push_back(RecordVariable{.name = "vel", ._ptr = &vel});
        _logger.file[0].vars.push_back(RecordVariable{.name = "f", ._ptr = &f});
        _logger.file[0].vars.push_back(RecordVariable{.name = "fd", ._ptr = &fd});
        _logger.file[0].vars.push_back(RecordVariable{.name = "Kv",  .fnc = [this]()
                                                                            { return Kv_f.v(); }});

        _logger.file[0].vars.push_back(RecordVariable{.name = "emg2", ._ptr = &emg2});
        _logger.file[0].vars.push_back(RecordVariable{.name = "emg2_f", ._ptr = &emg2_f});
        _logger.file[0].vars.push_back(RecordVariable{.name = "emg3", ._ptr = &emg3});
        _logger.file[0].vars.push_back(RecordVariable{.name = "emg3_f", ._ptr = &emg3_f});
        _logger.file[0].vars.push_back(RecordVariable{.name = "Kv_emg", ._ptr = &Kv_emg});

        _logger.file[0].vars.push_back(RecordVariable{.name = "theta_out_hip_r", ._ptr = &theta_out_hip_r});
    }

    void print_event(std::chrono::nanoseconds now)
    {
        static auto next_time = now;
        if (now >= next_time)
        {
            next_time += 1s;
            hh_logi("+[SimpleImpedance] pos0: %8f\t pos1: %8f\t vel: %4f", pos0, pos1, vel);
        }

        static auto next_time_2 = now;
        if (now >= next_time_2)
            if (tcp_)
            {
                next_time_2 += 100ms;
                std::string foo;
                foo.resize(100);
                int num_bytes = snprintf(&foo[0], 100, "%8f %8f %8f %8f\n", now.count() * 1e-9, theta_out.v(), theta_out_hip_r, 0);
                if (num_bytes < 100)
                {
                    foo.resize(num_bytes);
                }
                tcp_->send((void *)foo.c_str(), num_bytes);
            }
    }

    void send_control_signal(double u)
    {
        // for Current mode
        // int16_t vel_int = static_cast<int16_t>(u);

        // for Velocity mode
        int32_t vel_int = static_cast<int32_t>(u);

        _actuators[0]->write(&vel_int, 4);
    }

    std::chrono::nanoseconds prev_time = 0ns;
    virtual int loop() override
    {

        // Do not modify — handles timing and required imports
        // __CONTROL_IO_BEGIN_LOOP();
        static auto &core = HH::Core::instance();
        using namespace std::chrono_literals;
        // static auto prev_time = core.runner.get_run_time();
        t_ms = core.runner.get_run_time_double(1ms);

        // Read sensors
        _sensors[0]->read(&pos0, 0);
        _sensors[1]->read(&pos1, 0);
        _sensors[2]->read(&emg_values, 0);   
        _sensors[3]->read(&theta_out_hip_r, 0);  

        theta_out_hip_r = theta_out_hip_r/ 180.0 *  M_PI;

        //if (_references.size() > 0)
        //{
        //   _references[0]->read(&theta_d, 0);
        //}

         theta_d = theta_out_hip_r;


        // For continuous-time controller: compute dt with lower bound
        __CONTROL_IO_GET_DT(1e-3);

        // Update tracked variables and compute their derivatives and integrals

        theta_in.update(pos0 / GR, dt);
        theta_out.update(pos1, dt);

        emg2 = abs(emg_values[1]);
        emg2_f = lf_emg2.update(emg2, dt);

        emg3 = abs(emg_values[2]);
        emg3_f = lf_emg3.update(emg3, dt);

        if ((emg3_f)*1000 > 0.3)
        {Kv_emg = 2.0*Kv;}
        else {Kv_emg = Kv;}

        // static LowPassFilter lf_omega(20, 0.001f);

        dx.update(theta_out.v() - theta_in.v(), dt);

        fd = Kv_emg * (theta_d - theta_out.v()) + Bv * (0 - lf_omega.update(theta_out.d(), dt));

        // static LowPassFilter lf_f(40, 0.001f);
        f = lf_f.update(Ks * dx.v());

        param0.update(fd - f, dt);

        /* P.Y.C.H. — Put Your Controller (Probably Overengineered) Here */
        vel = kp * param0.v() + ki * param0.i() + kd * param0.d();
        vel = vel * GR;

        // Apply control signal to actuator
        send_control_signal(vel);

        // Finalize loop timing and logging
        __CONTROL_IO_END_LOOP();

        print_event(now);
        _logger.loop();

        return 0;
    };

    virtual bool command(std::string cmd, const void *arg0, const void *arg1) override
    {
        if (cmd.compare("END") == 0)
        {
            send_control_signal(0);
            hh_logw("+[SimpleImpedance]: Command(%s) Stopping Motor ... ", cmd.c_str());
        }
        else
        {
            hh_logw("+[SimpleImpedance]: Command(%s) not defined, continuing with out errors. ", cmd.c_str());
        }
        return true;
    }
};

__FINISH_PLUGIN_IO;