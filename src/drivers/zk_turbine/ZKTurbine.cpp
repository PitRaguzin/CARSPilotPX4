#include "ZKTurbine.hpp"

#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>

//PARAM_DEFINE_INT32(SENS_ZK_CFG, 0);

// Инициализация дескриптора модуля. Связываем функции с дескриптором. Использует constexpr конструктор: Descriptor(task_spawn, custom_command, print_usage)
ModuleBase::Descriptor ZKTurbine::desc{task_spawn, custom_command, print_usage};

ZKTurbine::ZKTurbine(const char *port_name) : ModuleParams(nullptr)
{
    PX4_INFO("Constructor with port name: %s", port_name);
    strncpy(_port, port_name, sizeof(_port));
}

ZKTurbine::~ZKTurbine()
{
    closePort();
}

int ZKTurbine::task_main_trampoline(int argc, char *argv[])
{
    ZKTurbine *instance = (ZKTurbine *)desc.object.load(); // Извлекаем указатель на созданный объект из дескриптора
    if (instance) {
        return instance->task_main();
    }
    return PX4_ERROR;
}

int ZKTurbine::task_main()
{
    // Инициализация объекта класса
    if (!init()) {
        PX4_ERR("Init error");
        desc.object.store(nullptr);
        delete this;
        return PX4_ERROR;
    }

    // Основной цикл программы
    while (!should_exit()) {
        // 1. Проверка обновления параметров по подпискам
        // 1.1. Обновление параметров турбины пользователем из-вне
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams(); // Обновляет _param_sens_zk_out_ch
        }
        int32_t user_channel = _param_sens_zk_out_ch.get(); // Получаем выбранный пользователем канал (например, 1..16)
        if (user_channel != _last_out_channel_num)
            _last_out_channel_num = (user_channel >= 0 && user_channel <= 16) ? user_channel - 1 : -1;

        // 1.2. Значение arm
        if (_vehicle_status_sub.updated()) {
            vehicle_status_s status;
            _vehicle_status_sub.copy(&status);
            bool current_arm = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
            if (current_arm != _is_armed)
                _is_armed = current_arm;
        }

        // 1.3. Значение на выходе включения рабочего режима турбины
        if (_actuator_outputs_sub.updated() && _last_out_channel_num != -1) {
            actuator_outputs_s outputs;
            _actuator_outputs_sub.copy(&outputs);
            float raw_value = outputs.output[_last_out_channel_num]; // Получаем значение с нужного нам канала вывода

            // Универсальная логика определения состояния: если значение близко к 1 (чистый GPIO High) ИЛИ больше 1500 (ШИМ max)
            bool turbine_should_on = (raw_value > 0.5f && raw_value < 2.0f) || raw_value > 1500.0f;
            if (turbine_should_on != _is_work)
                _is_work = turbine_should_on;

            PX4_INFO("Activating: %s", turbine_should_on ? "ON" : "OFF");
        }

        // 1.4. Проверка обновления значения уровня подачи газа
        if (_thrust_sp_sub.updated()) {
            vehicle_thrust_setpoint_s thrust_sp;
            if (_thrust_sp_sub.copy(&thrust_sp))
                m_setThrottle = thrust_sp.xyz[0] * 1000; // Индекс 3 отвечает за газ (THROTTLE) во всех стандартных микшерах, значение от 0.0 до 1.0
        }

        // 2. Расчёт статуса работы турбины для отправки
        SW_1 new_turbine_status = (!_is_armed) ? SW_1::ControlEngineIntoStopState : (_is_work) ? SW_1::ControlEngineIntoRunningState : SW_1::ControlEngineIntoStandbyMode;
        if (new_turbine_status != m_mode) {
            m_mode = new_turbine_status;
            PX4_INFO("Set mode: %s", m_mode == SW_1::ControlEngineIntoStopState ? "STOP" : m_mode == SW_1::ControlEngineIntoRunningState ? "RUN" : "STANDBY");
        }

        // 3. Работа с турбиной
        if (_fd >= 0) {
            // 3.1. Отправка данных
            uint8_t wr_bytes[4];
            wr_bytes[0] = SendHeaderByte;
            SendPacketData wr_packet;

            wr_packet.Command.ID = (uint16_t)SendCommand::ID_1;
            wr_packet.ID1Data.SW = (uint16_t)m_mode;
            wr_packet.ID1Data.Throttle = m_setThrottle;//(m_mode == SW_1::ControlEngineIntoRunningState) ? static_cast<uint16_t>(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 10) : 0;
            wr_bytes[1] = wr_packet.bytes[1];
            wr_bytes[2] = wr_packet.bytes[0];
            wr_bytes[3] = crc8(&wr_bytes[1], 2, 0);

            ssize_t wr_res = write(_fd, wr_bytes, 4);
            if (wr_res == -1) {
                static uint64_t last_err_t = 0;
                if (hrt_absolute_time() - last_err_t > 1000000) {
                    struct stat st;
                    int fstat_res = fstat(_fd, &st);
                    PX4_ERR("Failed to write UART port %s: descriptor=%d, fstat_res=%d, errno=%d", _port, _fd, fstat_res, errno);
                    last_err_t = hrt_absolute_time();
                }
            }

            // 3.2. Чтение данных
            struct pollfd fds[1];
            fds[0].fd = _fd;
            fds[0].events = POLLIN;
            int pres = poll(fds, 1, 10); // Ждем данные до 10 мс внутри своего потока

            if (pres > 0 && (fds[0].revents & POLLIN)) {
                int rd_res = read(_fd, &_buffer[_offset], sizeof(_buffer) - sizeof(_buffer[0]) * _offset);

                if (rd_res > 0)
                {
                    _offset+= rd_res;
                    int local_offset = 0;
                    while (_offset > local_offset && _offset - local_offset >= 7) {

                        while (local_offset < _offset && _buffer[local_offset] >> 4 != ReadHeader)
                            local_offset++;

                        if (_offset - local_offset >= 7)
                        {
                            ReceivePacket rd_packet;
                            memcpy(&rd_packet.bytes, &_buffer[local_offset], sizeof(rd_packet.bytes));
                            if (rd_packet.data.Command < (uint8_t)ReadCommand::ID_1 ||  // Если идентификатор команды меньше известного нам
                                rd_packet.data.Command > (uint8_t)ReadCommand::ID_9 ||  // ... или больше известного нам
                                !checkPacket(rd_packet))                                // ... или пакет не прошёл проверку целостности
                            {                                                           // .., значит мы за заголовок приняли часть предыдущего пакета
                                local_offset++;
                                continue;
                            }

                            parseData(rd_packet);
                            if (rd_packet.data.Command == (uint8_t)ReadCommand::ID_9)
                                sendTelemetry();

                            local_offset+= sizeof(rd_packet.bytes);
                        }
                    }

                    if (local_offset > 0)
                    {
                        int remain = _offset - local_offset;
                        if (remain > 0)
                            memmove(_buffer, &_buffer[local_offset], remain);
                        _offset = remain;
                    }
                }
                else if (rd_res < 0) {
                    if (errno != EAGAIN) {
                        static uint64_t last_r_err = 0;
                        if (hrt_absolute_time() - last_r_err > 1000000) {
                            PX4_ERR("Failed to read port %s: errno=%d", _port, errno);
                            last_r_err = hrt_absolute_time();
                        }
                    }
                }
            }
        }

        px4_usleep(20000);
    }

    PX4_INFO("Finishing thread...");

    desc.task_id = -1;
    desc.object.store(nullptr);

    delete this;

    return PX4_OK;
}

bool ZKTurbine::init()
{
    PX4_INFO("Initializing...");

//	// Динамически ищем параметр в системе по его текстовому имени
//	param_t param_handle = param_find("SENS_ZK_CFG");
//
//	// Если параметр существует, читаем его значение в нашу переменную
//	if (param_handle != PARAM_INVALID) {
//		param_get(param_handle, &_turbine_port_val);
//	} else {
//		PX4_WARN("Parameter SENS_ZK_CFG not found, using default");
//		_turbine_port_val = 0; // Значение по умолчанию
//	}
//
//	// Пример использования: если параметр равен 0 (Disabled), можем сразу выйти
//	if (_turbine_port_val == 0) {
//		PX4_INFO("Turbine driver is disabled via parameter");
//		return false;
//	}

    if (_port == nullptr || strlen(_port) == 0) {
        PX4_ERR("Failed to open empty UART port name.");
        return false;
    }

    _fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        PX4_ERR("Failed to open UART port %s", _port);
        return false;
    }
    PX4_INFO("Port %s opened with descriptor = %d", _port, _fd);

    tcflush(_fd, TCIOFLUSH);

    // Настройка параметров порта (Baudrate, 8N1 и т.д.)
    struct termios uart_config;
    tcgetattr(_fd, &uart_config);
    cfsetispeed(&uart_config, B9600);
    cfsetospeed(&uart_config, B9600);
    uart_config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    uart_config.c_cflag |= CS8 | CLOCAL | CREAD;
    uart_config.c_cflag &= ~CRTSCTS;
    tcsetattr(_fd, TCSANOW, &uart_config);

    tcflush(_fd, TCIOFLUSH);

    PX4_INFO("Initialized successfully");

    return true;
}

bool ZKTurbine::checkPacket(const ReceivePacket &p) const
{
    return p.data.CRC == crc8(p.bytes, 6, 0);
}

void ZKTurbine::parseData(ReceivePacket p)
{
    PX4_INFO("Parsing data: ID=%d",p.data.Command);
    m_rpm = (uint32_t)p.data.RPM * 10;
    switch ((ReadCommand)p.data.Command)
    {
        case ReadCommand::ID_1:
        {
            ReceivePacket::Data::DiffParameters::ID_1 d = p.data.DiffParams.ID1Data;
            m_state = (EngineStatusCode)d.EngineState;
            m_error = (ErrorCode)((d.ECodeH << 3) + d.ECodeL);
            m_exhTemperature = ((int16_t)d.TempH << 8) + d.TempL - 50;
            m_hostStatus = (HostStatus)d.SwSt;
            break;
        }
        case ReadCommand::ID_2:
        {
            ReceivePacket::Data::DiffParameters::ID_2 d = p.data.DiffParams.ID2Data;
            m_radioVoltage = d.RadioVoltage * ((m_protocolVersion >= 4) ? 0.2 : 0.1);
            m_powerVoltage = d.PowerVoltage * ((m_protocolVersion >= 4) ? 0.2 : 0.1);
            m_pumpVoltage = d.PumpVoltage * ((m_protocolVersion >= 4) ? 0.2 : 0.1);
            break;
        }
        case ReadCommand::ID_3:
        {
            ReceivePacket::Data::DiffParameters::ID_3 d = p.data.DiffParams.ID3Data;
            m_throttle = d.Throttle;
            m_pressure = (uint32_t)d.Pressure * 2;
            break;
        }
        case ReadCommand::ID_4:
        {
            ReceivePacket::Data::DiffParameters::ID_4 d = p.data.DiffParams.ID4Data;
            m_current = d.Current * 0.1;
            m_thrust = ((d.ThrustH << 8) + d.ThrustL) * 0.1;
            break;
        }
        case ReadCommand::ID_5:
        {
            ReceivePacket::Data::DiffParameters::ID_5 d = p.data.DiffParams.ID5Data;
            m_ignPumpVoltage = d.IgnPumpVoltage * 2 * 0.01;
            m_acceleration = d.CurveINC;
            m_deceleration = d.CurveDEC;
            break;
        }
        case ReadCommand::ID_6:
        {
            ReceivePacket::Data::DiffParameters::ID_6 d = p.data.DiffParams.ID6Data;
            m_maxRPM = (uint32_t)d.MaxRPM * 1000;
            m_protocolVersion = d.ProtocolVersion;
            m_maxPumpVoltage = d.MaxPumpVoltage * ((m_protocolVersion >= 4) ? 0.2 : 0.1);
            m_updateRate = (DataUpdateRate)d.SRate;
            break;
        }
        case ReadCommand::ID_7:
        {
            ReceivePacket::Data::DiffParameters::ID_7 d = p.data.DiffParams.ID7Data;
            m_flowRate = d.FlowRate * 0.01;
            m_flowTotal = (((uint16_t)d.FlowTotalH << 6) + d.FlowTotalL) * 0.1;
            break;
        }
        case ReadCommand::ID_8:
        {
            ReceivePacket::Data::DiffParameters::ID_8 d = p.data.DiffParams.ID8Data;
            m_idleRPM = (uint32_t)d.IdleRPM * 1000;
            m_esr = (ESR)d.ESR;
            m_scl = (SCL)d.SCL;
            m_startupTime = (((uint32_t)d.StartupTimeH << 8) + d.StartupTimeL) * 100;
            break;
        }
        case ReadCommand::ID_9:
        {
            ReceivePacket::Data::DiffParameters::ID_9 d = p.data.DiffParams.ID9Data;
            m_ecuTemperature = (int16_t)d.ECUTemperature - 50;
            break;
        }
    }
}

uint8_t ZKTurbine::crc8(const uint8_t *puchMsg, uint8_t crc_len, uint8_t seed) const
{
    uint8_t k, crc8 = seed;
    for(uint8_t i = 0; i < crc_len; i++)
    {
        k = puchMsg[i] ^ crc8;
        crc8 = 0;
        if (k & 0x01) crc8 ^= 0x5e;
        if (k & 0x02) crc8 ^= 0xbc;
        if (k & 0x04) crc8 ^= 0x61;
        if (k & 0x08) crc8 ^= 0xc2;
        if (k & 0x10) crc8 ^= 0x9d;
        if (k & 0x20) crc8 ^= 0x23;
        if (k & 0x40) crc8 ^= 0x46;
        if (k & 0x80) crc8 ^= 0x8c;
    }
    return crc8;
}

void ZKTurbine::sendTelemetry()
{
    internal_combustion_engine_status_s efi;
    memset(&efi, 0, sizeof(efi));

	// Заполняем поля, которые подходят под параметры турбины:
	efi.engine_speed_rpm = static_cast<float>(m_rpm);       // Обороты турбины (RPM)
	efi.ecu_index = 0;                                      // Индекс ECU (для QGC)
	efi.fuel_consumption_rate_cm3pm = m_flowRate * 1000;    // Расход топлива (см³/мин)
	efi.exhaust_gas_temperature = m_exhTemperature;         // Температура (EGT) в °C

	_efi_status_pub.publish(efi); // Публикуем в uORB. Модуль MAVLink подхватит это автоматически!

//    efi.timestamp                            = hrt_absolute_time();    // Обязательный системный таймштамп PX4
//    efi.engine_state                         = (m_error != ErrorCode::NoError) ? Engine_State::FAULT :
//                                               (m_state == EngineStatusCode::Stop) ? Engine_State::STOPPED : Engine_State::RUNNING;
//    efi.general_error                        = (m_error != ErrorCode::NoError);
//    efi.crankshaft_sensor_status             = Crankshaft_Sensor_Status::NOT_SUPPORTED;
//    efi.temperature_status                   = (m_error == ErrorCode::ExhaustTemperatureIsHigh) ? Temperature_Status::ABOVE_NOMINAL :
//                                               (m_error == ErrorCode::PumpControllerTemperatureIsHigh ||
//                                                m_error == ErrorCode::StarterControllerTemperatureIsHigh) ? Temperature_Status::EGT_ABOVE_NOMINAL :
//                                               (m_error == ErrorCode::ExhaustTemperatureIsLow) ? Temperature_Status::BELOW_NOMINAL :
//                                               (m_error == ErrorCode::ExhaustGasTemperatureSensorFailure) ? Temperature_Status::OVERHEATING : Temperature_Status::OK;
//    efi.fuel_pressure_status                 = (m_error == ErrorCode::PumpFailure ||
//                                                m_error == ErrorCode::PumpControllerTemperatureIsHigh) ? Fuel_Pressure_Status::BELOW_NOMINAL : Fuel_Pressure_Status::OK;
//    efi.oil_pressure_status                  = (m_error == ErrorCode::IgnitionValveFailure ||
//                                                m_error == ErrorCode::MainValveFailure) ? Oil_Pressure_Status::BELOW_NOMINAL : Oil_Pressure_Status::OK;
//    efi.detonation_status                    = (m_error == ErrorCode::GlowplugFailure ||
//                                                m_error == ErrorCode::StarterFailure) ? Detonation_Status::OBSERVED : Detonation_Status::NOT_OBSERVED;
//    efi.misfire_status                       = (m_error == ErrorCode::ClutchFailure ||
//                                                m_error == ErrorCode::StarterFailure) ? Misfire_Status::OBSERVED : Misfire_Status::NOT_OBSERVED;
//    efi.debris_status                        = Debris_Status::NOT_SUPPORTED;
//    efi.engine_load_percent                  = (m_maxRPM == 0) ? 0 : static_cast<uint8_t>(m_rpm * 100 / m_maxRPM);
//    efi.engine_speed_rpm                     = m_rpm;
//    efi.spark_dwell_time_ms                  = 0;
//    efi.atmospheric_pressure_kpa             = 0;
//    efi.intake_manifold_pressure_kpa         = 0;
//    efi.intake_manifold_temperature          = 0;
//    efi.coolant_temperature                  = 0;
//    efi.oil_pressure                         = m_pressure / 1000.0;
//    efi.oil_temperature                      = 0;
//    efi.fuel_pressure                        = 0;
//    efi.fuel_consumption_rate_cm3pm          = m_flowRate * 1000;
//    efi.estimated_consumed_fuel_volume_cm3   = m_flowTotal * 1000;
//    efi.throttle_position_percent            = m_setThrottle / 10;
//    efi.ecu_index                            = 0;
//    efi.spark_plug_usage                     = (m_error == ErrorCode::MainValveFailure) ? Spark_Plug_Usage::SECOND_ACTIVE :
//                                               (m_error == ErrorCode::IgnitionValveFailure) ? Spark_Plug_Usage::FIRST_ACTIVE : Spark_Plug_Usage::BOTH_ACTIVE;
//    efi.cylinder_status.cylinder_head_temperature    = 0;
//    efi.cylinder_status.exhaust_gas_temperature      = m_exhTemperature + 273.15f;
//    efi.cylinder_status.ignition_timing_deg          = 0;
//    efi.cylinder_status.injection_time_ms            = 0;
//    efi.cylinder_status.lambda_coefficient           = 0;
//    efi.ignition_voltage                     = m_ignPumpVoltage;
//    efi.throttle_out                         = m_throttle;
//    efi.pt_compensation                      = 0;
}

void ZKTurbine::closePort()
{
    if (_fd >= 0) {
        if (close(_fd) == 0) {
            _fd = -1;
            PX4_INFO("Port %s closed successfully.", _port);
        }
        else
            PX4_ERR("Close port %s error: %d.", _port, errno);
    }
}

int ZKTurbine::task_spawn(int argc, char *argv[])
{
    char *port = nullptr;

    if (argc > 1)
    {
        int ch;
        while ((ch = getopt(argc, argv, "d:")) != EOF)
        {
            switch (ch)
            {
                case 'd':   port = optarg;  break;
                default:                    break;
            }
        }
    }
    optind = 0;

    if (port == nullptr || strlen(port) == 0) {
        PX4_ERR("Empty port name");
        return PX4_ERROR;
    }

    ZKTurbine *instance = new ZKTurbine(port);

    if (!instance) {
        PX4_ERR("Alloc failed");
        return PX4_ERROR;
    }

    desc.object.store(instance);

    int task_id = px4_task_spawn_cmd(
        "zk_turbine_thread",                // Имя потока в ОС
        SCHED_DEFAULT,                      // Планировщик
        SCHED_PRIORITY_DEFAULT,             // Приоритет драйвера
        2000,                               // Размер стека в байтах
        &ZKTurbine::task_main_trampoline,   // Точка входа
        (char *const *)argv
    );

    if (task_id < 0) {
        PX4_ERR("Task spawn failed");
        desc.object.store(nullptr);
        delete instance;
        return PX4_ERROR;
    }

    desc.task_id = task_id;

    return PX4_OK;
}

int ZKTurbine::custom_command(int argc, char *argv[])
{
	return print_usage("Unknown command"); // Если команда не распознана, возвращаем вызов справки
}

int ZKTurbine::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);
    PRINT_MODULE_DESCRIPTION("ZK Turbine Driver");
    PRINT_MODULE_USAGE_NAME("zk_turbine", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
    PRINT_MODULE_USAGE_ARG("<port>", "UART device port (e.g. /dev/ttyS3)", true);
    return 0;
}

extern "C" __EXPORT int zk_turbine_main(int argc, char *argv[]);

int zk_turbine_main(int argc, char *argv[])
{
    return ModuleBase::main(ZKTurbine::desc, argc, argv);
}
