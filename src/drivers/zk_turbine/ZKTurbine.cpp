#include "ZKTurbine.hpp"

#include <termios.h>
#include <fcntl.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>

//PARAM_DEFINE_INT32(SENS_ZK_CFG, 0);

// Инициализация дескриптора модуля. Связываем функции с дескриптором. Использует constexpr конструктор: Descriptor(task_spawn, custom_command, print_usage)
ModuleBase::Descriptor ZKTurbine::desc{task_spawn, custom_command, print_usage};

ZKTurbine::ZKTurbine(const char *port_name) :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    PX4_INFO("ZK Turbine constructor with port name: %s", port_name);
    strncpy(_port, port_name, sizeof(_port));
}

ZKTurbine::~ZKTurbine()
{
    ScheduleClear();
    if (_fd >= 0) {
        close(_fd);
    }
}

bool ZKTurbine::init()
{
    PX4_INFO("ZK Turbine initializing...");

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

    if (_port == nullptr || strlen(_port) == 0)
    {
        PX4_ERR("Failed to open empty UART port name.");
        return false;
    }

    _fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        PX4_ERR("Failed to open UART port %s", _port);
        return false;
    }

    // Настройка параметров порта (Baudrate, 8N1 и т.д.)
    struct termios uart_config;
    tcgetattr(_fd, &uart_config);
    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);
    uart_config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    uart_config.c_cflag |= CS8 | CLOCAL | CREAD;
    tcsetattr(_fd, TCSANOW, &uart_config);

//    ScheduleNow();
    ScheduleOnInterval(20000); // Запуск периодического выполнения (например, каждые 20 мс / 50 Гц)

    PX4_INFO("ZK Turbine initialized successfully");

    return true;
}

void ZKTurbine::Run()
{
    static int counter = 3;
    if (counter-- > 0)
        PX4_INFO("ZK Turbine periodic function start");

    if (should_exit()) {
        ScheduleClear();
        ModuleBase::exit_and_cleanup(desc);
        return;
    }

    // Отправка пакета
    uint8_t wr_bytes[4];
    wr_bytes[0] = SendHeaderByte;
    SendPacketData wr_packet;

//    // Уточнение значения отправляемого режима
//    float outputnorm = SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_rcin8);
//    switch (m_mode)
//    {
//    case SW_1::ControlEngineIntoStopState:
//    case SW_1::ControlEngineIntoStandbyMode:
//    {
//        if (outputnorm > 0)
//            m_mode = SW_1::ControlEngineIntoRunningState;
//        break;
//    }
//    case SW_1::ControlEngineIntoRunningState:
//    {
//        if (outputnorm < 0)
//            m_mode = SW_1::ControlEngineIntoStandbyMode;
//        break;
//    }
//    case SW_1::UartDoesNotControlEngine:
//        break;
//    }

    // Отправка данных
    wr_packet.Command.ID = (uint16_t)SendCommand::ID_1;
    wr_packet.ID1Data.SW = (uint16_t)SW_1::UartDoesNotControlEngine;//m_mode;
    wr_packet.ID1Data.Throttle = 0;//(m_mode == SW_1::ControlEngineIntoRunningState) ? static_cast<uint16_t>(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 10) : 0;
    m_setThrottle = wr_packet.ID1Data.Throttle;
    wr_bytes[1] = wr_packet.bytes[1];
    wr_bytes[2] = wr_packet.bytes[0];
    wr_bytes[3] = crc8(&wr_bytes[1], 2, 0);

    ssize_t wr_res = write(_fd, wr_bytes, 4);
    if (wr_res == -1)
        PX4_ERR("Failed to write UART port %s: errno=%d", _port, errno);

    // Чтение данных из UART
    _offset+= read(_fd, &_buffer[_offset], sizeof(_buffer) - sizeof(_buffer[0]) * _offset);

    int local_offset = 0;
    while (_offset > local_offset && _offset - local_offset >= 7) {

        while (_buffer[local_offset] != ReadHeader)
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
        for (int i = local_offset;i < _offset - local_offset;i++)
            _buffer[i] = _buffer[local_offset + i];
        _offset-= local_offset;
    }
}

bool ZKTurbine::checkPacket(const ReceivePacket &p) const
{
    return p.data.CRC == crc8(p.bytes, 6, 0);
}

void ZKTurbine::parseData(ReceivePacket p)
{
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

int ZKTurbine::task_spawn(int argc, char *argv[])
{
    const char *port = "/dev/ttyS3";

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
//        for (int i = 1;i < argc;i+=2)
//        {
//            if (strcmp("-d", argv[i]) == 0 && i+1 < argc)
//                port = argv[i+1];
//        }
    }

    ZKTurbine *instance = new ZKTurbine(port);
    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    if (!instance->init()) {
        delete instance;
        return PX4_ERROR;
    }

    desc.object.store(instance);
    desc.task_id = task_id_is_work_queue;

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
    PX4_INFO("ZKTurbine_main() starting...");
    return ModuleBase::main(ZKTurbine::desc, argc, argv);
}
