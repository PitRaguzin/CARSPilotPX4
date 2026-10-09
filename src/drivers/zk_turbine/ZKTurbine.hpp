#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/internal_combustion_engine_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

#define PACKED __attribute__((__packed__))

class ZKTurbine : public ModuleBase, public ModuleParams
{
public:
    static ModuleBase::Descriptor desc;  ///< Статический дескриптор модуля, который требует архитектура PX4

    ZKTurbine(const char *port_name);
    ~ZKTurbine() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    static int task_main_trampoline(int argc, char *argv[]);
    int task_main();

private:
////////////////////////////// ОТПРАВКА ДАННЫХ (начало) //////////////////////////////
    const uint8_t SendHeaderByte = 0xFF;    ///< Заголовочный байт отправляемого запроса

    /// @brief Отпрравляемые команды
    enum class SendCommand : uint16_t
    {
        ID_0 = 0,   ///< Empty loop
        ID_1 = 1,   ///< Control engine status (SW)
        ID_2 = 2,   ///< Control commands
        ID_3 = 3,   ///< Unlock settings parameters
        ID_4 = 4,   ///< Set ignition pump voltage
        ID_5 = 5,   ///< Set engine running acceleration curve
        ID_6 = 6,   ///< Control engine status (SW)
        ID_7 = 7,   ///< Control engine RPM
        ID_8 = 8    ///< Set atmospheric static pressure
    };

    /// @brief Структура данных, отправляемых в контроллер двигателя
    union PACKED SendPacketData
    {
        uint8_t bytes[2];                       ///< Данные в виде последовательности байт
        struct PACKED Command
        {
            uint16_t IDData : 12;               ///< Объединённые данные сообщения
            uint16_t ID : 4;                    ///< ID сообщения
        } Command;
        struct PACKED Command_0
        {
            uint16_t reserve : 12;              ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_0
        } ID0Data;
        struct PACKED Command_1
        {
            uint16_t Throttle : 10;             ///< Control engine throttle
            uint16_t SW : 2;                    ///< Control engine status
            uint16_t ID : 4;                    ///< SendCommand::ID_1
        } ID1Data;
        struct PACKED Command_2
        {
            uint16_t Param : 8;                 ///< Using mode parameter
            uint16_t reserve : 4;               ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_2
        } ID2Data;
        struct PACKED Command_3
        {
            uint16_t reserve : 12;              ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_3
        } ID3Data;
        struct PACKED Command_4
        {
            uint16_t IgnitionPumpVoltage : 8;   ///< Ignition pump voltage
            uint16_t reserve : 4;               ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_4
        } ID4Data;
        struct PACKED Command_5
        {
            uint16_t AccelerationCurve : 8;     ///< Running acceleration curve
            uint16_t reserve : 4;               ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_5
        } ID5Data;
        struct PACKED Command_6
        {
            uint16_t SW : 2;                    ///< Control engine status
            uint16_t RPM_X : 3;                 ///< RPM factor
            uint16_t reserve : 7;               ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_6
        } ID6Data;
        struct PACKED Command_7
        {
            uint16_t SetRPM : 12;               ///< Control engine RPM
            uint16_t ID : 4;                    ///< SendCommand::ID_7
        } ID7Data;
        struct PACKED Command_8
        {
            uint16_t AirPressure : 10;          ///< Atmospheric static pressure
            uint16_t reserve : 2;               ///< [reserve]
            uint16_t ID : 4;                    ///< SendCommand::ID_8
        } ID8Data;
    };

    /// @brief Режимы работы двигателя, передаваемые в команде SendCommand::ID_1
    enum class SW_1 : uint16_t
    {
        UartDoesNotControlEngine = 0,
        ControlEngineIntoStopState = 1,
        ControlEngineIntoStandbyMode = 2,
        ControlEngineIntoRunningState = 3
    };

    /// @brief Режимы работы двигателя
    enum class Param_2 : uint16_t
    {
        DrainOil = 1,
        TestGlowplug = 2,
        TestMainValve = 3,
        TestIgnitionValve = 4,
        TestFuelPump = 5,
        TestStarter = 6,
        SetStatusUpdateRate20Hz = 7,
        SetStatusUpdateRate50Hz = 8,
        SetStatusUpdateRate100Hz = 9,
        ResetConsumptionStatistics = 10,
        CalibrateTrustSensorTo0 = 11,
        OpenFuelPumpForLongTime = 12,
        ClosePump = 13
    };

    /// @brief Значения для RPM factor
    enum class RPM_X_6 : uint16_t
    {
        Invalid = 0,
        to40950 = 1,
        to81900 = 2,
        to122850 = 3,
        to163800 = 4,
        to204750 = 5,
        to245700 = 6
    };

    /// @brief Режимы работы двигателя, передаваемые в команде SendCommand::ID_6
    enum class SW_6 : uint16_t
    {
        Invalid = 0,
        ControlEngineIntoStopState = 1,
        ControlEngineIntoStandbyMode = 2,
        ControlEngineIntoRunningState = 3
    };
////////////////////////////// ОТПРАВКА ДАННЫХ (конец) ///////////////////////////////

/////////////////////////////// ЧТЕНИЕ ДАННЫХ (начало) ///////////////////////////////
    const uint8_t ReadHeader = 0xF;     ///< Заголовок читаемого пакета

    /// @brief Отпрравляемые команды
    enum class ReadCommand : uint8_t
    {
        ID_1 = 1,
        ID_2 = 2,
        ID_3 = 3,
        ID_4 = 4,
        ID_5 = 5,
        ID_6 = 6,
        ID_7 = 7,
        ID_8 = 8,
        ID_9 = 9
    };

    /// @brief Структура принимаемых данных
    union PACKED ReceivePacket
    {
        uint8_t bytes[7];                           ///< Данные в виде последовательности байт
        struct PACKED Data
        {
            uint8_t Command : 4;                    ///< Идентификатор команды
            uint8_t Header : 4;                     ///< Заголовок пакета
            uint16_t RPM;                           ///< Значение RPM (/10)
            union PACKED DiffParameters
            {
                uint8_t DiffData[3];                ///< Данные, отличающиеся в зависимости от команды
                struct PACKED ID_1
                {
                    uint8_t EngineState : 5;        ///< Статус работы двигателя
                    uint8_t ECodeL : 3;             ///< Код ошибки (младшие биты)

                    uint8_t ECodeH : 2;             ///< Код ошибки (старшие биты)
                    uint8_t TempH : 3;              ///< Температура выхлопных газов (град С + 50) (старшие биты)
                    uint8_t SwSt : 2;               ///< Control status of the host (computer, flight controller) to the ECU
                    uint8_t reserve : 1;            ///< [reserve]

                    uint8_t TempL;                  ///< Температура выхлопных газов (град С + 50) (младший байт)
                } ID1Data;
                struct PACKED ID_2
                {
                    uint8_t RadioVoltage;           ///< Control voltage (0.1 В (до версии 3 включительно) или 0.2 В (начиная с версии 4))
                    uint8_t PowerVoltage;           ///< Напряжение питания (0.1 В (до версии 3 включительно) или 0.2 В (начиная с версии 4))
                    uint8_t PumpVoltage;            ///< Напряжение насоса (0.1 В (до версии 3 включительно) или 0.2 В (начиная с версии 4))
                } ID2Data;
                struct PACKED ID_3
                {
                    uint8_t Throttle;               ///< Значение процента использования мощности (газ в %)
                    uint16_t Pressure;              ///< Давление (Па / 2)
                } ID3Data;
                struct PACKED ID_4
                {
                    uint16_t Current : 9;           ///< Электрический ток (0,1 А)
                    uint16_t ThrustH : 7;           ///< Тяга двигателя (0,1 кг) (старшие биты)
                    uint8_t ThrustL;                ///< Тяга двигателя (0,1 кг) (младший байт)
                } ID4Data;
                struct PACKED ID_5
                {
                    uint8_t IgnPumpVoltage;         ///< Ignition pump voltage (0.01 В / 2)
                    uint8_t CurveINC;               ///< Engine acceleration curve parameters
                    uint8_t CurveDEC;               ///< Engine deceleration curve parameters
                } ID5Data;
                struct PACKED ID_6
                {
                    uint8_t MaxRPM;                 ///< Engine maximum RPM (RPM / 1000)
                    uint8_t MaxPumpVoltage;         ///< Fuel pump maximum voltage (0.1 В (до версии 3 включительно) или 0.2 В (начиная с версии 4))
                    uint8_t SRate : 2;              ///< Current data update rate
                    uint8_t ProtocolVersion : 6;    ///< Версия протокола
                } ID6Data;
                struct PACKED ID_7
                {
                    uint16_t FlowRate : 10;         ///< Расход топлива (0,01 л/мин) (младший байт)
                    uint16_t FlowTotalL : 6;        ///< Суммарный расход топлива (0,1 л) (младшие биты)
                    uint8_t FlowTotalH;             ///< Суммарный расход топлива (0,1 л) (старший байт)
                } ID7Data;
                struct PACKED ID_8
                {
                    uint8_t IdleRPM;                ///< Engine IDLE RPM (RPM / 1000)
                    uint8_t reserve : 2;            ///< [reserve]
                    uint8_t ESR : 1;                ///< Request flight controller to send barometric pressure
                    uint8_t SCL : 1;                ///< Speed closed loop rate
                    uint8_t StartupTimeH : 4;       ///< Startup time (0,1 с) (старшие биты)
                    uint8_t StartupTimeL;           ///< Startup time (0,1 с) (младший байт)
                } ID8Data;
                struct PACKED ID_9
                {
                    uint8_t ECUTemperature;         ///< ECU temperature (град С + 50)
                    uint16_t reserve;               ///< [reserve]
                } ID9Data;
            } DiffParams;
            uint8_t CRC;
        } data;
    };

    /// @brief Статусы работы двигателя
    enum class EngineStatusCode
    {
        Stop = 0,
        Standby_AutoCooling = 1,
        IgnitionMinThrottle = 2,
        Ignition = 3,
        Preheat = 4,
        Fuelramp = 5,
        Running_LearningModeMaxThrottle = 6,
        Running_LearningModeMinThrottle = 7,
        Running_LearningIdle = 8,
        Running_MinThrottle = 9,
        Running_WarningFuelPumpLimit = 10,
        Running = 11,
        Cooling = 12,
        Restart = 13,
        TestGlowplug = 14,
        TestMainValve = 15,
        TestIgnitionValve = 16,
        TestPump = 17,
        TestStarter = 18,
        FuelExhausAir = 19
    };

    /// @brief  Коды ошибок
    enum class ErrorCode : uint8_t
    {
        NoError = 0,                                ///< Нет ошибок
        TimeOut = 1,                                ///< Время истекло
        VoltageLow = 2,                             ///< Низкое напряжение
        GlowplugFailure = 3,                        ///< Ошибка свечи накаливания
        PumpFailure = 4,                            ///< Ошибка насоса
        StarterFailure = 5,                         ///< Ошибка стартера
        RPMLow = 6,                                 ///< Низкие оборосты двигателя
        RPMInstability = 7,                         ///< Нестабильная скорость вращения двигателя
        ExhaustTemperatureIsHigh = 8,               ///< Высокая температура выхлопных газов
        ExhaustTemperatureIsLow = 9,                ///< Низкая температура выхлопных газов
        ExhaustGasTemperatureSensorFailure = 10,    ///< Ошибка датчика температуры выхлопных газов
        IgnitionValveFailure = 11,                  ///< Ошибка клапана зажигания
        MainValveFailure = 12,                      ///< Ошибка главного клапана
        LossOfControlSignal = 13,                   ///< Потеряm_setThrottle управляющего сигнала
        StarterControllerTemperatureIsHigh = 14,    ///< Высокая температура контроллера запуска
        PumpControllerTemperatureIsHigh = 15,       ///< Высокая температура контроллера насоса
        ClutchFailure = 16,                         ///< Ошибка сцепления (схватывания)
        CurrentOverload = 18,                       ///< Перегрузка по току
        EngineOffline = 19                          ///< Потеря связи
    };

    /// @brief Значения статусов управляющего (компьютер или полётный контроллер) ID 1
    enum class HostStatus : uint8_t
    {
        STOP = 0,
        STANDBY = 1,
        RUN = 2
    };

    /// @brief Частота обновления данных ID 6
    enum class DataUpdateRate : uint8_t
    {
        _20Hz = 0,
        _50Hz = 1,
        _100Hz = 2
    };

    /// @brief Необходимость отправлять значение атмосферного давления в ECU ID 8
    enum class ESR : uint8_t
    {
        NeedToSendAtmPressure = 0,
        DoNotSendAtmPressure = 1
    };

    /// @brief Speed closed loop state ID 8
    enum class SCL : uint8_t
    {
        OpenLoop = 0,
        CloseLoop = 1
    };
/////////////////////////////// ЧТЕНИЕ ДАННЫХ (конец) ////////////////////////////////

    /// @brief обработчик изменения состояния параметров управления включением работы турбины
    void updateControlOutput();

    /// @brief Проверка целостности принятого пакета
    /// @param p Принятый пакет
    /// @return Результат проверки целостности
    bool checkPacket(const ReceivePacket &p) const;

    /// @brief Разбор данных принятого пакета
    /// @param p Принятые данные
    void parseData(ReceivePacket p);

    /// @brief Подсчёт контрольной суммы
    /// @param puchMsg Указатель на данные
    /// @param crc_len Размер данных, участвующих в подсчёте контрольной суммы
    /// @param seed Смещение
    /// @return Контрольная сумма
    uint8_t crc8(const uint8_t *puchMsg, uint8_t crc_len, uint8_t seed) const;

    /// @brief Отправка телеметрии о турбине вверх
    void sendTelemetry();

    /// @brief Закрытие последовательного порта
    void closePort();

    char _port[32];                     ///< Имя порта UART в системе
    int _fd{-1};                        ///< Дескриптор порта UART
    int _offset = 0;                    ///< Смещение для чтения данных в буфер
    uint8_t _buffer[64];                ///< Буфер для чтения данных из порта UART
    int32_t _turbine_port_val{0};       ///< Переменная для хранения значения порта, прочитанного из параметров
    int32_t _last_out_channel_num{-1};  ///< Тип интерфейса для управления включением работы турбины
    bool _is_work{false};               ///< Статус включения рабочего режима
    bool _is_armed{false};              ///< Статус arm

    uORB::Publication<internal_combustion_engine_status_s> _efi_status_pub{ORB_ID(internal_combustion_engine_status)};
    uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)}; ///< Получение значения уровня газа
    uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)}; ///< Получение значений выходов GPIO и ШИМ
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};     ///< Получение значения arm\disarm
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)}; ///< Получение статуса обновления параметров
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SENS_ZK_OUT_CH>) _param_sens_zk_out_ch
    )

    SW_1 m_mode = SW_1::ControlEngineIntoStopState; ///< Значение режима, отправляемое в ECU
    uint16_t m_setThrottle;                         ///< Задаваемое значение газа (процента используемой мощности) (промилле)
    uint32_t m_rpm;                                 ///< Значение оборотов двигателя в минуту
    ErrorCode m_error;                              ///< Код ошибки
    EngineStatusCode m_state;                       ///< Статус работы двигателя
    HostStatus m_hostStatus;                        ///< Статус управляющего (компьютера\полётного контроллера)
    int16_t m_exhTemperature;                       ///< Температура выхлопных газов (*С)
    float m_radioVoltage;                           ///< Управляющее напряжение (В)
    float m_powerVoltage;                           ///< Напряжение питания (В)
    float m_pumpVoltage;                            ///< Напряжение насоса (В)
    uint8_t m_throttle;                             ///< Процент используемой мощности (%)
    uint32_t m_pressure;                            ///< Давление (Па)
    float m_current;                                ///< Ток (А)
    float m_thrust;                                 ///< Тяга (кг)
    float m_ignPumpVoltage;                         ///< Ignition pump voltage (В)
    uint8_t m_acceleration;                         ///< Ускорение
    uint8_t m_deceleration;                         ///< Замедление
    uint32_t m_maxRPM;                              ///< Максимальное значение оборотов двигателя в минуту
    float m_maxPumpVoltage;                         ///< Максимальное значение напряжения насоса (В)
    uint8_t m_protocolVersion;                      ///< Версия протокола
    DataUpdateRate m_updateRate;                    ///< Значение частоты обновления данных
    float m_flowRate;                               ///< Текущий расход топлива (л/мин)
    float m_flowTotal;                              ///< Общий расход (л)
    uint32_t m_idleRPM;                             ///< Холостые обороты двигателя
    ESR m_esr;                                      ///< Необходимость отправлять данные об атмосферном давлении
    SCL m_scl;                                      ///< Speed closed loop state
    uint32_t m_startupTime;                         ///< Время работы (мс)
    int16_t m_ecuTemperature;                       ///< Температура ECU (*С)

    int m_msgcounter = 0;
};
