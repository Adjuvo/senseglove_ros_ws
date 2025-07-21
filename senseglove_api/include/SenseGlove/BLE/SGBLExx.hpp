/**
 * @file
 *
 * @author  Mamadou Babaei <mamadou@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2025 SenseGlove
 *
 * @section DESCRIPTION
 *
 * The main header file for the SGBLExx library.
 */


#pragma once

#include <algorithm>
#include <cassert>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <SenseGlove/BLE/Platform.hpp>
#include <SenseGlove/BLE/sgble.hpp>

#define SGBLEXX_CHAR_PROP_FLAG_BROADCAST 0x01
#define SGBLEXX_CHAR_PROP_FLAG_READ 0x02
#define SGBLEXX_CHAR_PROP_FLAG_WRITE_WITHOUT_RESPONSE 0x04
#define SGBLEXX_CHAR_PROP_FLAG_WRITE 0x08
#define SGBLEXX_CHAR_PROP_FLAG_NOTIFY 0x10
#define SGBLEXX_CHAR_PROP_FLAG_INDICATE 0x20
#define SGBLEXX_CHAR_PROP_FLAG_AUTHENTICATED_SIGNED_WRITES 0x40
#define SGBLEXX_CHAR_PROP_FLAG_EXTENDED_PROPERTIES 0x80

#if !defined(SGBLEXX_CACHE_LOGS)
#define SGBLEXX_CACHE_LOGS 0
#endif /* ! defined ( SGBLEXX_CACHE_LOGS ) */

#if !defined(SGBLEXX_FFI_ALLOC_TRACKER)
#define SGBLEXX_FFI_ALLOC_TRACKER 0
#endif /* ! defined ( SGBLEXX_FFI_ALLOC_TRACKER ) */

namespace SGBLExx
{
    typedef std::function<void(const std::string&)>
            PeripheralEventSubscribeCallbackType;

    typedef std::function<void(const std::string&, const std::string&, const std::vector<uint8_t>&)>
            PeripheralCharacteristicSubscribeCallbackType;

    enum class EsgBleWriteType
    {
        WithResponse,
        WithoutResponse,
    };

    class SgBleDescriptor;
    class SgBleCharacteristic;
    class SgBleService;
    class SgBlePeripheralProperties;
    class SgBlePeripheral;

    namespace
    {
        inline static std::vector<std::shared_ptr<PeripheralEventSubscribeCallbackType>>
            PeripheralEventOnDiscoveredSubscribeCallbacks;
        inline static std::vector<std::shared_ptr<PeripheralEventSubscribeCallbackType>>
            PeripheralEventOnConnectedSubscribeCallbacks;
        inline static std::vector<std::shared_ptr<PeripheralEventSubscribeCallbackType>>
            PeripheralEventOnDisconnectedSubscribeCallbacks;

        void PeripheralEventOnDiscoveredSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId);
        void PeripheralEventOnConnectedSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId);
        void PeripheralEventOnDisconnectedSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId);
    }// namespace

    namespace
    {
        void DropFfiString(sgble::SgFfiString*& out_ffiString);
        void DropFfiOption(sgble::SgFfiOption<int16_t>*& out_ffiOption);
        void DropFfiOption(sgble::SgFfiOption<uint32_t>*& out_ffiOption);
        void DropFfiOption(sgble::SgFfiOption<sgble::SgFfiString*>*& out_ffiOption);
        void DropFfiEmptyResult(sgble::SgFfiEmptyResult*& out_ffiEmptyResult);
        void DropFfiResult(sgble::SgFfiResult<bool>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<uint32_t>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiString*>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiVec<uint8_t>*>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiVec<sgble::SgFfiBlePeripheral*>*>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBleDescriptor*>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>*& out_ffiResult);
        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*& out_ffiResult);
        void DropFfiVec(sgble::SgFfiVec<uint8_t>*& out_ffiVec);
        void DropFfiVec(sgble::SgFfiVec<sgble::SgFfiString*>*& out_ffiVec);
        void DropFfiVec(sgble::SgFfiVec<sgble::SgFfiBlePeripheral*>*& out_ffiVec);
        void DropFfiHashMap(sgble::SgFfiHashMap<uint16_t, sgble::SgFfiVec<uint8_t>*>*& out_ffiHashMap);
        void DropFfiHashMap(sgble::SgFfiHashMap<sgble::SgFfiString*, sgble::SgFfiVec<uint8_t>*>*& out_ffiHashMap);
        void DropFfiDescriptor(sgble::SgFfiBleDescriptor*& out_ffiDescriptor);
        void DropFfiCharacteristic(sgble::SgFfiBleCharacteristic*& out_ffiCharacteristic);
        void DropFfiService(sgble::SgFfiBleService*& out_ffiService);
        void DropFfiPeripheralProperties(sgble::SgFfiBlePeripheralProperties*& out_ffiPeripheralProperties);
        void DropFfiPeripheral(sgble::SgFfiBlePeripheral*& out_ffiPeripheral);
    }// namespace

    static bool IsInitialized();
    static bool Initialize(std::string& out_error);

    template<
        typename OnPeripheralDiscoveredCallback,
        typename OnPeripheralConnectedCallback,
        typename OnPeripheralDisconnectedCallback>
    static bool Initialize(
        OnPeripheralDiscoveredCallback&& onPeripheralDiscoveredCallback,
        OnPeripheralConnectedCallback&& onPeripheralConnectedCallback,
        OnPeripheralDisconnectedCallback&& onPeripheralDisconnectedCallback,
        std::string& out_error)
    {
        out_error.clear();

        std::shared_ptr onPeripheralDiscoveredCallbackPtr{
            std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(onPeripheralDiscoveredCallback) };
        std::shared_ptr onPeripheralConnectedCallbackPtr{
            std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(onPeripheralConnectedCallback) };
        std::shared_ptr onPeripheralDisconnectedCallbackPtr{
            std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(onPeripheralDisconnectedCallback) };

        sgble::SgFfiEmptyResult* result{
            sgble::sg_initialize_with_peripheral_events_callbacks(
                static_cast<void*>(onPeripheralDiscoveredCallbackPtr.get()),
                PeripheralEventOnDiscoveredSubscribeCallback,
                static_cast<void*>(onPeripheralConnectedCallbackPtr.get()),
                PeripheralEventOnConnectedSubscribeCallback,
                static_cast<void*>(onPeripheralDisconnectedCallbackPtr.get()),
                PeripheralEventOnDisconnectedSubscribeCallback
            )};
        assert(result && "Invalid FFI function call result!");

        if (result->error && result->error->value) {
            out_error = std::string{ result->error->value };
            DropFfiEmptyResult(result);

            return false;
        }

        PeripheralEventOnDiscoveredSubscribeCallbacks.emplace_back(std::move(onPeripheralDiscoveredCallbackPtr));
        PeripheralEventOnConnectedSubscribeCallbacks.emplace_back(std::move(onPeripheralConnectedCallbackPtr));
        PeripheralEventOnDisconnectedSubscribeCallbacks.emplace_back(std::move(onPeripheralDisconnectedCallbackPtr));

        DropFfiEmptyResult(result);

        return true;
    }

    static bool Terminate(std::string& out_error);

    template<typename Callback>
    static bool SubscribeToOnPeripheralDiscovered(
        Callback&& callback,
        std::string& out_error)
    {
        out_error.clear();

        std::shared_ptr callbackPtr{ std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(callback) };

        sgble::SgFfiEmptyResult* resultSubscribeToOnPeripheralDiscovered{
                sgble::sg_subscribe_to_on_peripheral_discovered(
                    static_cast<void*>(callbackPtr.get()), PeripheralEventOnDiscoveredSubscribeCallback) };
        assert(resultSubscribeToOnPeripheralDiscovered && "Invalid FFI function call result!");

        if (resultSubscribeToOnPeripheralDiscovered->error
            && resultSubscribeToOnPeripheralDiscovered->error->value) {
            out_error = std::string{ resultSubscribeToOnPeripheralDiscovered->error->value };
            DropFfiEmptyResult(resultSubscribeToOnPeripheralDiscovered);
            return false;
        }

        PeripheralEventOnDiscoveredSubscribeCallbacks.emplace_back(std::move(callbackPtr));

        DropFfiEmptyResult(resultSubscribeToOnPeripheralDiscovered);

        return true;
    }

    template<typename Callback>
    static bool SubscribeToOnPeripheralConnected(
        Callback&& callback,
        std::string& out_error)
    {
        out_error.clear();

        std::shared_ptr callbackPtr{ std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(callback) };

        sgble::SgFfiEmptyResult* resultSubscribeToOnPeripheralConnected{
                sgble::sg_subscribe_to_on_peripheral_connected(
                    static_cast<void*>(callbackPtr.get()), PeripheralEventOnConnectedSubscribeCallback) };
        assert(resultSubscribeToOnPeripheralConnected && "Invalid FFI function call result!");

        if (resultSubscribeToOnPeripheralConnected->error
            && resultSubscribeToOnPeripheralConnected->error->value) {
            out_error = std::string{ resultSubscribeToOnPeripheralConnected->error->value };
            DropFfiEmptyResult(resultSubscribeToOnPeripheralConnected);
            return false;
        }

        PeripheralEventOnConnectedSubscribeCallbacks.emplace_back(std::move(callbackPtr));

        DropFfiEmptyResult(resultSubscribeToOnPeripheralConnected);

        return true;
    }

    template<typename Callback>
    static bool SubscribeToOnPeripheralDisconnected(
        Callback&& callback,
        std::string& out_error)
    {
        out_error.clear();

        std::shared_ptr callbackPtr{ std::make_shared<SGBLExx::PeripheralEventSubscribeCallbackType>(callback) };

        sgble::SgFfiEmptyResult* resultSubscribeToOnPeripheralDisconnected{
                sgble::sg_subscribe_to_on_peripheral_disconnected(
                    static_cast<void*>(callbackPtr.get()), PeripheralEventOnDisconnectedSubscribeCallback) };
        assert(resultSubscribeToOnPeripheralDisconnected && "Invalid FFI function call result!");

        if (resultSubscribeToOnPeripheralDisconnected->error
            && resultSubscribeToOnPeripheralDisconnected->error->value) {
            out_error = std::string{ resultSubscribeToOnPeripheralDisconnected->error->value };
            DropFfiEmptyResult(resultSubscribeToOnPeripheralDisconnected);
            return false;
        }

        PeripheralEventOnDisconnectedSubscribeCallbacks.emplace_back(std::move(callbackPtr));

        DropFfiEmptyResult(resultSubscribeToOnPeripheralDisconnected);

        return true;
    }

    static bool UnsubscribeAllFromOnPeripheralDiscovered(std::string& out_error);
    static bool UnsubscribeAllFromOnPeripheralConnected(std::string& out_error);
    static bool UnsubscribeAllFromOnPeripheralDisconnected(std::string& out_error);

    static bool GetPeripherals(std::vector<SgBlePeripheral>& out_peripherals, std::string& out_error);
    static bool GetPeripheralByAddress(const std::string& address, SgBlePeripheral& out_peripheral, std::string& out_error);
    static bool GetPeripheralByLocalName(const std::string& localName, SgBlePeripheral& out_peripheral, std::string& out_error);

#if SGBLEXX_CACHE_LOGS
    static std::string GetLogs();
    static void ClearLogs();
#endif /* SGBLEXX_CACHE_LOGS */

#if SGBLEXX_FFI_ALLOC_TRACKER
    static std::vector<std::string> GetFfiAllocTrackerTypeNames();
#endif /* SGBLEXX_FFI_ALLOC_TRACKER */

    namespace
    {
        void PeripheralCharacteristicSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId,
                sgble::SgFfiString* characteristicId,
                sgble::SgFfiVec<uint8_t>* data);
    }// namespace

    class SgBleDescriptor
    {
    public:
        static SgBleDescriptor From(const sgble::SgFfiBleDescriptor* descriptor)
        {
            assert(descriptor && "Invalid descriptor!");
            assert(descriptor->uuid && "Invalid descriptor ID!");
            assert(descriptor->service_uuid && "Invalid descriptor service ID!");
            assert(descriptor->characteristic_uuid && "Invalid descriptor characteristic ID!");

            std::string id{descriptor->uuid->value};
            std::string serviceId{descriptor->service_uuid->value};
            std::string charactersticId{descriptor->characteristic_uuid->value};

            return SgBleDescriptor{
                    std::move(id),
                    std::move(serviceId),
                    std::move(charactersticId)};
        }

    private:
        std::string Id;
        std::string ServiceId;
        std::string CharacteristicId;

    public:
        // The default constructor.
        SgBleDescriptor()
            : Id(),
              ServiceId(),
              CharacteristicId()
        {
        }

        // The main constructor.
        SgBleDescriptor(
                const std::string& id,
                const std::string& serviceId,
                const std::string& characteristicId)
        {
            Id = id;
            ServiceId = serviceId;
            CharacteristicId = characteristicId;
        }

        // The copy constructor.
        SgBleDescriptor(const SgBleDescriptor& rhs) = default;

        // The move constructor.
        SgBleDescriptor(SgBleDescriptor&& rhs) noexcept = default;

        // The destructor.
        virtual ~SgBleDescriptor() = default;

    public:
        // The copy assignment operator.
        SgBleDescriptor& operator=(const SgBleDescriptor& rhs) = default;

        // The move assignment operator.
        SgBleDescriptor& operator=(SgBleDescriptor&& rhs) noexcept = default;

    public:
        // The equality comparison operator.
        bool operator==(const SgBleDescriptor& rhs) const
        {
            if (Id == rhs.Id) {
                return true;
            } else {
                return false;
            }
        }

        // The hash function.
        class HashFunction
        {
        public:
            size_t operator()(const SgBleDescriptor& descriptor) const
            {
                return std::hash<std::string>()(descriptor.GetId());
            }
        };

    public:
        [[nodiscard]] SG_FORCEINLINE const std::string& GetId() const
        {
            return Id;
        }

        [[nodiscard]] SG_FORCEINLINE const std::string& GetServiceId() const
        {
            return ServiceId;
        }

        [[nodiscard]] SG_FORCEINLINE const std::string& GetCharacteristicId() const
        {
            return CharacteristicId;
        }
    };

    class SgBleCharacteristic
    {
    public:
        typedef std::unordered_set<SgBleDescriptor, SgBleDescriptor::HashFunction> DescriptorsType;

    public:
        static SgBleCharacteristic From(const sgble::SgFfiBleCharacteristic* characteristic)
        {
            assert(characteristic && "Invalid characteristic!");
            assert(characteristic->uuid && "Invalid characteristic ID!");
            assert(characteristic->descriptors && "Invalid characteristic descriptors!");

            std::string id{characteristic->uuid->value};
            std::string serviceId{characteristic->service_uuid->value};

            DescriptorsType descriptors;
            {
                for (uintptr_t i = 0; i < characteristic->descriptors->size; ++i) {
                    if (characteristic->descriptors->elements[i]) {
                        SgBleDescriptor descriptor{SgBleDescriptor::From(characteristic->descriptors->elements[i])};
                        descriptors.emplace(std::move(descriptor));
                    }
                }
            }

            return SgBleCharacteristic{
                    std::move(id),
                    std::move(serviceId),
                    characteristic->properties,
                    std::move(descriptors)};
        }

    private:
        std::string Id;
        std::string ServiceId;
        uint8_t Properties;
        DescriptorsType Descriptors;

    public:
        // The default constructor.
        SgBleCharacteristic()
            : Id(),
              ServiceId(),
              Properties(),
              Descriptors()
        {
        }

        // The main constructor.
        SgBleCharacteristic(
                const std::string& id,
                const std::string& serviceId,
                const uint8_t properties,
                const DescriptorsType& descriptors)
        {
            Id = id;
            ServiceId = serviceId;
            Properties = properties;
            Descriptors = descriptors;
        }

        // The copy constructor.
        SgBleCharacteristic(const SgBleCharacteristic& rhs) = default;

        // The move constructor.
        SgBleCharacteristic(SgBleCharacteristic&& rhs) noexcept = default;

        // The destructor.
        virtual ~SgBleCharacteristic() = default;

    public:
        // The copy assignment operator.
        SgBleCharacteristic& operator=(const SgBleCharacteristic& rhs) = default;

        // The move assignment operator.
        SgBleCharacteristic& operator=(SgBleCharacteristic&& rhs) noexcept = default;

    public:
        // The equality comparison operator.
        bool operator==(const SgBleCharacteristic& rhs) const
        {
            if (Id == rhs.Id) {
                return true;
            } else {
                return false;
            }
        }

        // The hash function.
        class HashFunction
        {
        public:
            size_t operator()(const SgBleCharacteristic& characteristic) const
            {
                return std::hash<std::string>()(characteristic.GetId());
            }
        };

    public:
        [[nodiscard]] SG_FORCEINLINE const std::string& GetId() const
        {
            return Id;
        }

        [[nodiscard]] SG_FORCEINLINE const std::string& GetServiceId() const
        {
            return ServiceId;
        }

        [[nodiscard]] SG_FORCEINLINE uint8_t GetProperties() const
        {
            return Properties;
        }

        [[nodiscard]] SG_FORCEINLINE const DescriptorsType& GetDescriptors() const
        {
            return Descriptors;
        }
    };

    class SgBleService
    {
    public:
        typedef std::unordered_set<SgBleCharacteristic, SgBleCharacteristic::HashFunction> CharacteristicsType;

    public:
        static SgBleService From(const sgble::SgFfiBleService* service)
        {
            assert(service && "Invalid service!");
            assert(service->uuid && "Invalid service ID!");
            assert(service->characteristics && "Invalid service characteristics!");

            std::string id{service->uuid->value};

            CharacteristicsType characteristics;
            {
                for (uintptr_t i = 0; i < service->characteristics->size; ++i) {
                    if (service->characteristics->elements[i]) {
                        SgBleCharacteristic characteristic{
                                SgBleCharacteristic::From(service->characteristics->elements[i])};
                        characteristics.emplace(std::move(characteristic));
                    }
                }
            }

            return SgBleService{
                    std::move(id),
                    service->primary,
                    std::move(characteristics)};
        }


    private:
        std::string Id;
        bool bPrimary;
        CharacteristicsType Characteristics;

    public:
        // The default constructor.
        SgBleService()
            : Id(),
              bPrimary(false),
              Characteristics()
        {
        }

        // The main constructor.
        SgBleService(
                const std::string& id,
                const bool bInPrimary,
                const CharacteristicsType& characteristics)
        {
            Id = id;
            bPrimary = bInPrimary;
            Characteristics = characteristics;
        }

        // The copy constructor.
        SgBleService(const SgBleService& rhs) = default;

        // The move constructor.
        SgBleService(SgBleService&& rhs) noexcept = default;

        // The destructor.
        virtual ~SgBleService() = default;

    public:
        // The copy assignment operator.
        SgBleService& operator=(const SgBleService& rhs) = default;

        // The move assignment operator.
        SgBleService& operator=(SgBleService&& rhs) noexcept = default;

    public:
        // The equality comparison operator.
        bool operator==(const SgBleService& rhs) const
        {
            if (Id == rhs.Id) {
                return true;
            } else {
                return false;
            }
        }

        // The hash function.
        class HashFunction
        {
        public:
            size_t operator()(const SgBleService& service) const
            {
                return std::hash<std::string>()(service.GetId());
            }
        };

    public:
        [[nodiscard]] SG_FORCEINLINE const std::string& GetId() const
        {
            return Id;
        }

        [[nodiscard]] SG_FORCEINLINE bool IsPrimary() const
        {
            return bPrimary;
        }

        [[nodiscard]] SG_FORCEINLINE const CharacteristicsType& GetCharacteristics() const
        {
            return Characteristics;
        }
    };

    class SgBlePeripheralProperties
    {
    public:
        static SgBlePeripheralProperties From(const sgble::SgFfiBlePeripheralProperties* properties)
        {
            assert(properties && "Invalid peripheral properties!");

            std::string address;
            if (properties->address) {
                address.assign(properties->address->value);
            }

            std::optional<std::string> localName;
            if (properties->local_name && properties->local_name->has_value && properties->local_name->value) {
                localName.emplace(std::string{properties->local_name->value->value});
            }

            std::optional<int16_t> txPowerLevel;
            if (properties->tx_power_level && properties->tx_power_level->has_value && properties->tx_power_level->value) {
                txPowerLevel.emplace(properties->tx_power_level->value);
            }

            std::optional<int16_t> rssi;
            if (properties->rssi && properties->rssi->has_value && properties->rssi->value) {
                txPowerLevel.emplace(properties->tx_power_level->value);
            }

            std::unordered_map<uint16_t, std::vector<uint8_t>> manufacturerData;
            if (properties->manufacturer_data) {
                for (uintptr_t i = 0; i < properties->manufacturer_data->size; ++i) {
                    const uint16_t key = properties->manufacturer_data->keys[i];
                    const sgble::SgFfiVec<uint8_t>* valuesVec = properties->manufacturer_data->values[i];
                    std::vector<uint8_t> valuesVector;
                    if (valuesVec) {
                        for (uintptr_t j = 0; j < valuesVec->size; ++j) {
                            const uint8_t value = valuesVec->array[j];
                            valuesVector.emplace_back(value);
                        }
                    }
                    std::pair<uint16_t, std::vector<uint8_t>> pair{key, std::move(valuesVector)};
                    manufacturerData.insert(std::move(pair));
                }
            }

            std::unordered_map<std::string, std::vector<uint8_t>> serviceData;
            if (properties->service_data) {
                for (uintptr_t i = 0; i < properties->service_data->size; ++i) {
                    if (properties->service_data->keys[i]) {
                        std::string key{properties->service_data->keys[i]->value};
                        const sgble::SgFfiVec<uint8_t>* valuesVec = properties->service_data->values[i];
                        std::vector<uint8_t> valuesVector;
                        if (valuesVec) {
                            for (uintptr_t j = 0; j < valuesVec->size; ++j) {
                                const uint8_t value = valuesVec->array[j];
                                valuesVector.emplace_back(value);
                            }
                        }
                        std::pair<std::string, std::vector<uint8_t>> pair{key, std::move(valuesVector)};
                        serviceData.insert(std::move(pair));
                    }
                }
            }

            std::vector<std::string> services;
            if (properties->services) {
                for (uintptr_t i = 0; i < properties->services->size; ++i) {
                    if (properties->services->array[i]) {
                        std::string service{properties->services->array[i]->value};
                        services.emplace_back(std::move(service));
                    }
                }
            }

            std::optional<uint32_t> class_;
            if (properties->class_ && properties->class_->has_value && properties->class_->value) {
                class_.emplace(properties->class_->value);
            }

            return SgBlePeripheralProperties{
                    std::move(address),
                    std::move(localName),
                    std::move(txPowerLevel),
                    std::move(rssi),
                    std::move(manufacturerData),
                    std::move(serviceData),
                    std::move(services),
                    std::move(class_)};
        }

    private:
        std::string Address;
        std::optional<std::string> LocalName;
        std::optional<int16_t> TxPowerLevel;
        std::optional<int16_t> Rssi;
        std::unordered_map<uint16_t, std::vector<uint8_t>> ManufacturerData;
        std::unordered_map<std::string, std::vector<uint8_t>> ServiceData;
        std::vector<std::string> Services;
        std::optional<uint32_t> Class;

    public:
        // The default constructor.
        SgBlePeripheralProperties()
            : Address(),
              LocalName(),
              TxPowerLevel(),
              Rssi(),
              ManufacturerData(),
              ServiceData(),
              Services(),
              Class()
        {
        }

        // The main constructor.
        SgBlePeripheralProperties(
                const std::string& address,
                const std::optional<std::string>& localName,
                const std::optional<int16_t>& txPowerLevel,
                const std::optional<int16_t>& rssi,
                const std::unordered_map<uint16_t, std::vector<uint8_t>>& manufacturerData,
                const std::unordered_map<std::string, std::vector<uint8_t>>& serviceData,
                const std::vector<std::string>& services,
                const std::optional<uint32_t>& class_)
        {
            Address = address;
            LocalName = localName;
            TxPowerLevel = txPowerLevel;
            Rssi = rssi;
            ManufacturerData = manufacturerData,
            ServiceData = serviceData,
            Services = services;
            Class = class_;
        }

        // The copy constructor.
        SgBlePeripheralProperties(const SgBlePeripheralProperties& rhs) = default;

        // The move constructor.
        SgBlePeripheralProperties(SgBlePeripheralProperties&& rhs) noexcept = default;

        // The destructor.
        virtual ~SgBlePeripheralProperties() = default;

    public:
        // The copy assignment operator.
        SgBlePeripheralProperties& operator=(const SgBlePeripheralProperties& rhs) = default;

        // The move assignment operator.
        SgBlePeripheralProperties& operator=(SgBlePeripheralProperties&& rhs) noexcept = default;

    public:
        [[nodiscard]] SG_FORCEINLINE const std::string& GetAddress() const
        {
            return Address;
        }

        [[nodiscard]] SG_FORCEINLINE const std::optional<std::string>& GetLocalName() const
        {
            return LocalName;
        }

        [[nodiscard]] SG_FORCEINLINE const std::optional<int16_t>& GetTxPowerLevel() const
        {
            return TxPowerLevel;
        }

        [[nodiscard]] SG_FORCEINLINE const std::optional<int16_t>& GetRssi() const
        {
            return Rssi;
        }

        [[nodiscard]] SG_FORCEINLINE const std::unordered_map<uint16_t, std::vector<uint8_t>>&
        GetManufacturerData() const
        {
            return ManufacturerData;
        }

        [[nodiscard]] SG_FORCEINLINE const std::unordered_map<std::string, std::vector<uint8_t>>& GetServiceData() const
        {
            return ServiceData;
        }

        [[nodiscard]] SG_FORCEINLINE const std::vector<std::string>& GetServices() const
        {
            return Services;
        }

        [[nodiscard]] SG_FORCEINLINE const std::optional<uint32_t>& GetClass() const
        {
            return Class;
        }
    };

    class SgBlePeripheral
    {
    public:
        typedef std::unordered_set<SgBleService, SgBleService::HashFunction> ServicesType;
        typedef std::unordered_set<SgBleCharacteristic, SgBleCharacteristic::HashFunction> CharacteristicsType;

    public:
        static SgBlePeripheral From(const sgble::SgFfiBlePeripheral* peripheral)
        {
            assert(peripheral && "Invalid peripheral!");
            assert(peripheral->id && "Invalid peripheral ID!");
            assert(peripheral->properties && "Invalid peripheral properties!");
            assert(peripheral->services && "Invalid peripheral services!");
            assert(peripheral->characteristics && "Invalid peripheral characteristics!");

            std::string id{peripheral->id->value};
            SgBlePeripheralProperties properties{SgBlePeripheralProperties::From(peripheral->properties)};

            ServicesType services;
            {
                for (uintptr_t i = 0; i < peripheral->services->size; ++i) {
                    if (peripheral->services->elements[i]) {
                        SgBleService service{SgBleService::From(peripheral->services->elements[i])};
                        services.emplace(std::move(service));
                    }
                }
            }

            CharacteristicsType characteristics;
            {
                for (uintptr_t i = 0; i < peripheral->characteristics->size; ++i) {
                    if (peripheral->characteristics->elements[i]) {
                        SgBleCharacteristic characteristic{
                                SgBleCharacteristic::From(peripheral->characteristics->elements[i])};
                        characteristics.emplace(std::move(characteristic));
                    }
                }
            }

            return SgBlePeripheral{
                    std::move(id),
                    std::move(properties),
                    std::move(services),
                    std::move(characteristics)};
        }

    private:
        std::string Id;
        SgBlePeripheralProperties Properties;
        ServicesType Services;
        CharacteristicsType Characteristics;

        std::unordered_map<std::string, std::vector<std::shared_ptr<PeripheralCharacteristicSubscribeCallbackType>>> SubscribeCallbacks;

    public:
        // The default constructor.
        SgBlePeripheral()
            : Id(),
              Properties(),
              Services(),
              Characteristics()
        {
        }

        // The main constructor.
        SgBlePeripheral(
                const std::string& id,
                const SgBlePeripheralProperties& properties,
                const ServicesType& services,
                const CharacteristicsType& characteristics)
        {
            Id = id;
            Properties = properties;
            Services = services;
            Characteristics = characteristics;
        }

        // The copy constructor.
        SgBlePeripheral(const SgBlePeripheral& rhs) = default;

        // The move constructor.
        SgBlePeripheral(SgBlePeripheral&& rhs) noexcept = default;

        // The destructor.
        virtual ~SgBlePeripheral() = default;

    public:
        // The copy assignment operator.
        SgBlePeripheral& operator=(const SgBlePeripheral& rhs) = default;

        // The move assignment operator.
        SgBlePeripheral& operator=(SgBlePeripheral&& rhs) noexcept = default;

    public:
        [[nodiscard]] SG_FORCEINLINE const std::string& GetId() const
        {
            return Id;
        }

        [[nodiscard]] SG_FORCEINLINE const SgBlePeripheralProperties& GetProperties() const
        {
            return Properties;
        }

        [[nodiscard]] SG_FORCEINLINE const ServicesType& GetServices() const
        {
            return Services;
        }

        [[nodiscard]] SG_FORCEINLINE const CharacteristicsType& GetCharacteristics() const
        {
            return Characteristics;
        }

    public:
        bool IsConnected(std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            sgble::SgFfiResult<bool>* resultIsConnected{
                    sgble::sg_peripheral_is_connected(ffiPeripheral)};
            assert(resultIsConnected && "Invalid FFI function call result!");

            if (resultIsConnected->error &&
                resultIsConnected->error->value) {
                out_error = std::string{resultIsConnected->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultIsConnected);
                return false;
            }

            const bool bResult = resultIsConnected->value;

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultIsConnected);

            return bResult;
        }

        bool Connect(std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            sgble::SgFfiEmptyResult* resultConnect{
                    sgble::sg_peripheral_connect(ffiPeripheral)};
            assert(resultConnect && "Invalid FFI function call result!");

            if (resultConnect->error &&
                resultConnect->error->value) {
                out_error = std::string{resultConnect->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiEmptyResult(resultConnect);
                return false;
            }

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiEmptyResult(resultConnect);

            return true;
        }

        bool Disconnect(std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            sgble::SgFfiEmptyResult* resultDisconnect{
                    sgble::sg_peripheral_disconnect(ffiPeripheral)};
            assert(resultDisconnect && "Invalid FFI function call result!");

            if (resultDisconnect->error &&
                resultDisconnect->error->value) {
                out_error = std::string{resultDisconnect->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiEmptyResult(resultDisconnect);
                return false;
            }

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiEmptyResult(resultDisconnect);

            return true;
        }

    public:
        bool Read(const SgBleCharacteristic& characteristic,
                  std::vector<uint8_t>& out_data, std::string& out_error) const
        {
            out_data.clear();
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{characteristic.GetId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            sgble::SgFfiResult<sgble::SgFfiVec<uint8_t>*>* resultRead{
                    sgble::sg_peripheral_read(ffiPeripheral, ffiCharacteristic)};
            assert(resultRead && "Invalid FFI function call result!");

            if (resultRead->error && resultRead->error->value) {
                out_error = std::string{resultRead->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiResult(resultRead);
                return false;
            }

            sgble::SgFfiVec<uint8_t>* data{resultRead->value};
            out_data.assign(data->array, data->array + data->size);

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiResult(resultRead);

            return true;
        }

        bool Write(const SgBleCharacteristic& characteristic,
                   const std::vector<uint8_t>& data, const EsgBleWriteType writeType,
                   std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{characteristic.GetId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            sgble::SgFfiBleWriteType ffiWriteType;
            switch (writeType) {
                case EsgBleWriteType::WithResponse:
                    ffiWriteType = sgble::SgFfiBleWriteType::EsgWithResponse;
                    break;
                case EsgBleWriteType::WithoutResponse:
                    ffiWriteType = sgble::SgFfiBleWriteType::EsgWithoutResponse;
                    break;
                default:
                    assert(false && "The code flow should not reach here!");
                    break;
            }

            const uint8_t* dataPtr = &data[0];
            const std::size_t dataSize = data.size();

            sgble::SgFfiEmptyResult* resultWrite{
                    sgble::sg_peripheral_write(ffiPeripheral, ffiCharacteristic, dataPtr, dataSize, ffiWriteType)};
            assert(resultWrite && "Invalid FFI function call result!");

            if (resultWrite->error && resultWrite->error->value) {
                out_error = std::string{resultWrite->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiEmptyResult(resultWrite);
                return false;
            }

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiEmptyResult(resultWrite);

            return true;
        }

        template<typename Callback>
        bool Subscribe(const SgBleCharacteristic& characteristic,
                       Callback&& callback,
                       std::string& out_error)
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{characteristic.GetId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            std::shared_ptr callbackPtr{std::make_shared<SGBLExx::PeripheralCharacteristicSubscribeCallbackType>(callback)};

            sgble::SgFfiEmptyResult* resultSubscribe{
                    sgble::sg_peripheral_subscribe(
                        ffiPeripheral, ffiCharacteristic,
                        static_cast<void*>(callbackPtr.get()), PeripheralCharacteristicSubscribeCallback)};
            assert(resultSubscribe && "Invalid FFI function call result!");

            if (resultSubscribe->error && resultSubscribe->error->value) {
                out_error = std::string{resultSubscribe->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiEmptyResult(resultSubscribe);
                return false;
            }

            SubscribeCallbacks[characteristic.GetId()].emplace_back(std::move(callbackPtr));

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiEmptyResult(resultSubscribe);

            return true;
        }

        bool UnsubscribeAll(const SgBleCharacteristic& characteristic,
                            std::string& out_error)
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{characteristic.GetId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            sgble::SgFfiEmptyResult* resultUnunsubscribeAll{
                    sgble::sg_peripheral_unsubscribe_all(ffiPeripheral, ffiCharacteristic)};
            assert(resultUnunsubscribeAll && "Invalid FFI function call result!");

            if (resultUnunsubscribeAll->error && resultUnunsubscribeAll->error->value) {
                out_error = std::string{resultUnunsubscribeAll->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiEmptyResult(resultUnunsubscribeAll);
                return false;
            }

            SubscribeCallbacks.erase(characteristic.GetId());

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiEmptyResult(resultUnunsubscribeAll);

            return true;
        }

        bool ReadDescriptor(const SgBleDescriptor& descriptor,
                            std::vector<uint8_t>& out_data, std::string& out_error) const
        {
            out_data.clear();
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{descriptor.GetCharacteristicId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            const std::string& descriptorId{descriptor.GetId()};
            sgble::SgFfiString* ffiDescriptorUuidString{sgble::sg_ffi_new_string(descriptorId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleDescriptor*>* resultGetPeripheralDescriptorByUuidString{
                    sgble::get_peripheral_descriptor_by_uuid_string(ffiPeripheral, ffiCharacteristic,
                                                                    ffiDescriptorUuidString)};
            assert(resultGetPeripheralDescriptorByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiDescriptorUuidString);

            if (resultGetPeripheralDescriptorByUuidString->error &&
                resultGetPeripheralDescriptorByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralDescriptorByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiResult(resultGetPeripheralDescriptorByUuidString);
                return false;
            }
            sgble::SgFfiBleDescriptor* ffiDescriptor{resultGetPeripheralDescriptorByUuidString->value};

            sgble::SgFfiResult<sgble::SgFfiVec<uint8_t>*>* resultRead{
                    sgble::sg_peripheral_read_descriptor(ffiPeripheral, ffiDescriptor)};
            assert(resultRead && "Invalid FFI function call result!");

            if (resultRead->error && resultRead->error->value) {
                out_error = std::string{resultRead->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiResult(resultGetPeripheralDescriptorByUuidString);
                DropFfiResult(resultRead);
                return false;
            }

            sgble::SgFfiVec<uint8_t>* data{resultRead->value};
            out_data.assign(data->array, data->array + data->size);

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiResult(resultGetPeripheralDescriptorByUuidString);
            DropFfiResult(resultRead);

            return true;
        }

        bool WriteDescriptor(const SgBleDescriptor& descriptor,
                             const std::vector<uint8_t>& data, std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            const std::string& characteristicId{descriptor.GetCharacteristicId()};
            sgble::SgFfiString* ffiCharacteristicUuidString{sgble::sg_ffi_new_string(characteristicId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>* resultGetPeripheralCharacteristicByUuidString{
                    sgble::get_peripheral_characteristic_by_uuid_string(ffiPeripheral, ffiCharacteristicUuidString)};
            assert(resultGetPeripheralCharacteristicByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiCharacteristicUuidString);

            if (resultGetPeripheralCharacteristicByUuidString->error &&
                resultGetPeripheralCharacteristicByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralCharacteristicByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                return false;
            }
            sgble::SgFfiBleCharacteristic* ffiCharacteristic{resultGetPeripheralCharacteristicByUuidString->value};

            const std::string& descriptorId{descriptor.GetId()};
            sgble::SgFfiString* ffiDescriptorUuidString{sgble::sg_ffi_new_string(descriptorId.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBleDescriptor*>* resultGetPeripheralDescriptorByUuidString{
                    sgble::get_peripheral_descriptor_by_uuid_string(ffiPeripheral, ffiCharacteristic,
                                                                    ffiDescriptorUuidString)};
            assert(resultGetPeripheralDescriptorByUuidString && "Invalid FFI function call result!");
            DropFfiString(ffiDescriptorUuidString);

            if (resultGetPeripheralDescriptorByUuidString->error &&
                resultGetPeripheralDescriptorByUuidString->error->value) {
                out_error = std::string{resultGetPeripheralDescriptorByUuidString->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiResult(resultGetPeripheralDescriptorByUuidString);
                return false;
            }
            sgble::SgFfiBleDescriptor* ffiDescriptor{resultGetPeripheralDescriptorByUuidString->value};

            const uint8_t* dataPtr = &data[0];
            const std::size_t dataSize = data.size();

            sgble::SgFfiEmptyResult* resultWrite{
                    sgble::sg_peripheral_write_descriptor(ffiPeripheral, ffiDescriptor, dataPtr, dataSize)};
            assert(resultWrite && "Invalid FFI function call result!");

            if (resultWrite->error && resultWrite->error->value) {
                out_error = std::string{resultWrite->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
                DropFfiResult(resultGetPeripheralDescriptorByUuidString);
                DropFfiEmptyResult(resultWrite);
                return false;
            }

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPeripheralCharacteristicByUuidString);
            DropFfiResult(resultGetPeripheralDescriptorByUuidString);
            DropFfiEmptyResult(resultWrite);

            return true;
        }

        uint32_t GetPacketsPerSecond(std::string& out_error) const
        {
            out_error.clear();

            const std::string& address{GetProperties().GetAddress()};
            sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};
            sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*
                    resultGetPeripheralByAddress{sgble::sg_get_peripheral_by_address(ffiAddress)};
            assert(resultGetPeripheralByAddress && "Invalid FFI function call result!");
            DropFfiString(ffiAddress);

            if (resultGetPeripheralByAddress->error && resultGetPeripheralByAddress->error->value) {
                out_error = std::string{resultGetPeripheralByAddress->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                return false;
            }
            sgble::SgFfiBlePeripheral* ffiPeripheral{resultGetPeripheralByAddress->value};

            sgble::SgFfiResult<uint32_t>* resultGetPacketsPerSecond{
                    sgble::sg_peripheral_get_packets_per_second(ffiPeripheral)};
            assert(resultGetPacketsPerSecond && "Invalid FFI function call result!");

            if (resultGetPacketsPerSecond->error &&
                resultGetPacketsPerSecond->error->value) {
                out_error = std::string{resultGetPacketsPerSecond->error->value};
                DropFfiResult(resultGetPeripheralByAddress);
                DropFfiResult(resultGetPacketsPerSecond);
                return false;
            }

            const uint32_t packetsPerSecond = resultGetPacketsPerSecond->value;

            DropFfiResult(resultGetPeripheralByAddress);
            DropFfiResult(resultGetPacketsPerSecond);

            return packetsPerSecond;
        }
    };

    static SG_FORCEINLINE bool IsInitialized()
    {
        return sgble::sg_is_initialized();
    }

    static bool Initialize(std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiEmptyResult* result{sgble::sg_initialize()};
        assert(result && "Invalid FFI function call result!");

        if (result->error && result->error->value) {
            out_error = std::string{result->error->value};
            DropFfiEmptyResult(result);

            return false;
        }

        DropFfiEmptyResult(result);

        return true;
    }

    static bool Terminate(std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiEmptyResult* result{sgble::sg_terminate()};
        assert(result && "Invalid FFI function call result!");

        if (result->error && result->error->value) {
            out_error = std::string{result->error->value};
            DropFfiEmptyResult(result);

            return false;
        }

        DropFfiEmptyResult(result);

        return true;
    }

    bool UnsubscribeAllFromOnPeripheralDiscovered(std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiEmptyResult* resultUnunsubscribeAllFromOnPeripheralDiscovered{
                sgble::sg_unsubscribe_all_from_on_peripheral_discovered()};
        assert(resultUnunsubscribeAllFromOnPeripheralDiscovered && "Invalid FFI function call result!");

        if (resultUnunsubscribeAllFromOnPeripheralDiscovered->error
            && resultUnunsubscribeAllFromOnPeripheralDiscovered->error->value) {
            out_error = std::string{ resultUnunsubscribeAllFromOnPeripheralDiscovered->error->value };
            DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralDiscovered);
            return false;
        }

        PeripheralEventOnDiscoveredSubscribeCallbacks.clear();
        PeripheralEventOnDiscoveredSubscribeCallbacks.shrink_to_fit();

        DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralDiscovered);

        return true;
    }

    bool UnsubscribeAllFromOnPeripheralConnected(std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiEmptyResult* resultUnunsubscribeAllFromOnPeripheralConnected{
                sgble::sg_unsubscribe_all_from_on_peripheral_connected() };
        assert(resultUnunsubscribeAllFromOnPeripheralConnected && "Invalid FFI function call result!");

        if (resultUnunsubscribeAllFromOnPeripheralConnected->error
            && resultUnunsubscribeAllFromOnPeripheralConnected->error->value) {
            out_error = std::string{ resultUnunsubscribeAllFromOnPeripheralConnected->error->value };
            DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralConnected);
            return false;
        }

        PeripheralEventOnConnectedSubscribeCallbacks.clear();
        PeripheralEventOnConnectedSubscribeCallbacks.shrink_to_fit();

        DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralConnected);

        return true;
    }

    bool UnsubscribeAllFromOnPeripheralDisconnected(std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiEmptyResult* resultUnunsubscribeAllFromOnPeripheralDisconnected{
                sgble::sg_unsubscribe_all_from_on_peripheral_disconnected() };
        assert(resultUnunsubscribeAllFromOnPeripheralDisconnected && "Invalid FFI function call result!");

        if (resultUnunsubscribeAllFromOnPeripheralDisconnected->error
            && resultUnunsubscribeAllFromOnPeripheralDisconnected->error->value) {
            out_error = std::string{ resultUnunsubscribeAllFromOnPeripheralDisconnected->error->value };
            DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralDisconnected);
            return false;
        }

        PeripheralEventOnDisconnectedSubscribeCallbacks.clear();
        PeripheralEventOnDisconnectedSubscribeCallbacks.shrink_to_fit();

        DropFfiEmptyResult(resultUnunsubscribeAllFromOnPeripheralDisconnected);

        return true;
    }

    static bool GetPeripherals(std::vector<SgBlePeripheral>& out_peripherals, std::string& out_error)
    {
        out_peripherals.clear();
        out_error.clear();

        sgble::SgFfiResult<sgble::SgFfiVec<sgble::SgFfiBlePeripheral*>*>* result{sgble::sg_get_peripherals()};
        assert(result && "Invalid FFI function call result!");

        if (result->error && result->error->value) {
            out_error = std::string{result->error->value};

            DropFfiResult(result);

            return false;
        }

        if (result->value) {
            for (uintptr_t i = 0; i < result->value->size; ++i) {
                if (result->value->array[i]) {
                    SgBlePeripheral peripheral{SgBlePeripheral::From(result->value->array[i])};
                    out_peripherals.emplace_back(peripheral);
                }
            }
        }

        DropFfiResult(result);

        return true;
    }

    static bool GetPeripheralByAddress(const std::string& address, SgBlePeripheral& out_peripheral, std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiString* ffiAddress{sgble::sg_ffi_new_string(address.c_str())};

        sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>* result{sgble::sg_get_peripheral_by_address(ffiAddress)};
        assert(result && "Invalid FFI function call result!");

        DropFfiString(ffiAddress);

        if (result->error && result->error->value) {
            out_error = std::string{result->error->value};

            DropFfiResult(result);

            return false;
        }

        if (result->value) {
            out_peripheral = SgBlePeripheral::From(result->value);
        }

        DropFfiResult(result);

        return true;
    }

    static bool GetPeripheralByLocalName(const std::string& localName, SgBlePeripheral& out_peripheral, std::string& out_error)
    {
        out_error.clear();

        sgble::SgFfiString* ffiLocalName{sgble::sg_ffi_new_string(localName.c_str())};

        sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>* result{sgble::sg_get_peripheral_by_local_name(ffiLocalName)};
        assert(result && "Invalid FFI function call result!");

        DropFfiString(ffiLocalName);

        if (result->error && result->error->value) {
            out_error = std::string{result->error->value};

            DropFfiResult(result);

            return false;
        }

        out_peripheral = SgBlePeripheral::From(result->value);

        DropFfiResult(result);

        return true;
    }

#if SGBLEXX_CACHE_LOGS
    static std::string GetLogs()
    {
        sgble::SgFfiString* ffiLogs{sgble::sg_logs_get()};

        std::string logs{ffiLogs->value ? ffiLogs->value : ""};

        DropFfiString(ffiLogs);

        return std::move(logs);
    }

    static SG_FORCEINLINE void ClearLogs()
    {
        sgble::sg_logs_clear();
    }
#endif /* SGBLEXX_CACHE_LOGS */

#if SGBLEXX_FFI_ALLOC_TRACKER
    static std::vector<std::string> GetFfiAllocTrackerTypeNames()
    {
        std::vector<std::string> typeNames;

        sgble::SgFfiVec<sgble::SgFfiString*>* ffiTypeNames = sgble::sg_ffi_get_alloc_tracker_type_names();
        for (uintptr_t i = 0; i < ffiTypeNames->size; ++i) {
            if (ffiTypeNames->array[i]) {
                std::string typeName{ffiTypeNames->array[i]->value};
                typeNames.emplace_back(typeName);
            }
        }
        sgble::sg_ffi_drop_vec_string(ffiTypeNames);

        return std::move(typeNames);
    }
#endif /* SGBLEXX_FFI_ALLOC_TRACKER */

    namespace
    {
        void PeripheralEventOnDiscoveredSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId)
        {
            if (peripheralId->value) {
                std::string pId{peripheralId->value};

                SGBLExx::PeripheralEventSubscribeCallbackType callback{
                        *static_cast<SGBLExx::PeripheralEventSubscribeCallbackType*>(context)};
                callback(std::move(pId));
            }

            DropFfiString(peripheralId);
        }

        void PeripheralEventOnConnectedSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId)
        {
            if (peripheralId->value) {
                std::string pId{peripheralId->value};

                SGBLExx::PeripheralEventSubscribeCallbackType callback{
                        *static_cast<SGBLExx::PeripheralEventSubscribeCallbackType*>(context)};
                callback(std::move(pId));
            }

            DropFfiString(peripheralId);
        }

        void PeripheralEventOnDisconnectedSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId)
        {
            if (peripheralId->value) {
                std::string pId{peripheralId->value};

                SGBLExx::PeripheralEventSubscribeCallbackType callback{
                        *static_cast<SGBLExx::PeripheralEventSubscribeCallbackType*>(context)};
                callback(std::move(pId));
            }

            DropFfiString(peripheralId);
        }
    }// namespace

    namespace
    {
        void PeripheralCharacteristicSubscribeCallback(
                void* context,
                sgble::SgFfiString* peripheralId,
                sgble::SgFfiString* characteristicId,
                sgble::SgFfiVec<uint8_t>* data)
        {
            if (peripheralId->value && characteristicId->value && data->array) {
                std::string pId{peripheralId->value};
                std::string cId{characteristicId->value};
                std::vector<uint8_t> d{data->array, data->array + data->size};

                SGBLExx::PeripheralCharacteristicSubscribeCallbackType callback{
                        *static_cast<SGBLExx::PeripheralCharacteristicSubscribeCallbackType*>(context)};
                callback(std::move(pId), std::move(cId), std::move(d));
            }

            DropFfiString(peripheralId);
            DropFfiString(characteristicId);
            DropFfiVec(data);
        }
    }// namespace

    namespace
    {
        void DropFfiString(sgble::SgFfiString*& out_ffiString)
        {
            if (out_ffiString) {
                sgble::sg_ffi_drop_string(out_ffiString);
                out_ffiString = nullptr;
            }
        }

        void DropFfiOption(sgble::SgFfiOption<int16_t>*& out_ffiOption)
        {
            if (out_ffiOption) {
                sgble::sg_ffi_drop_option_i16(out_ffiOption);
                out_ffiOption = nullptr;
            }
        }

        void DropFfiOption(sgble::SgFfiOption<uint32_t>*& out_ffiOption)
        {
            if (out_ffiOption) {
                sgble::sg_ffi_drop_option_u32(out_ffiOption);
                out_ffiOption = nullptr;
            }
        }

        void DropFfiOption(sgble::SgFfiOption<sgble::SgFfiString*>*& out_ffiOption)
        {
            if (out_ffiOption) {
                sgble::sg_ffi_drop_option_string(out_ffiOption);
                out_ffiOption = nullptr;
            }
        }

        void DropFfiEmptyResult(sgble::SgFfiEmptyResult*& out_ffiEmptyResult)
        {
            if (out_ffiEmptyResult) {
                sgble::sg_ffi_drop_empty_result(out_ffiEmptyResult);
                out_ffiEmptyResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<bool>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_bool(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<uint32_t>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_u32(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiString*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_string(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiVec<uint8_t>*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_vec_u8(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiVec<sgble::SgFfiBlePeripheral*>*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_vec_peripheral(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBleDescriptor*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_descriptor(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBleCharacteristic*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_characteristic(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiResult(sgble::SgFfiResult<sgble::SgFfiBlePeripheral*>*& out_ffiResult)
        {
            if (out_ffiResult) {
                sgble::sg_ffi_drop_result_peripheral(out_ffiResult);
                out_ffiResult = nullptr;
            }
        }

        void DropFfiVec(sgble::SgFfiVec<uint8_t>*& out_ffiVec)
        {
            if (out_ffiVec) {
                sgble::sg_ffi_drop_vec_u8(out_ffiVec);
                out_ffiVec = nullptr;
            }
        }

        void DropFfiVec(sgble::SgFfiVec<sgble::SgFfiString*>*& out_ffiVec)
        {
            if (out_ffiVec) {
                sgble::sg_ffi_drop_vec_string(out_ffiVec);
                out_ffiVec = nullptr;
            }
        }

        void DropFfiVec(sgble::SgFfiVec<sgble::SgFfiBlePeripheral*>*& out_ffiVec)
        {
            if (out_ffiVec) {
                sgble::sg_ffi_drop_vec_peripheral(out_ffiVec);
                out_ffiVec = nullptr;
            }
        }

        void DropFfiHashMap(sgble::SgFfiHashMap<uint16_t, sgble::SgFfiVec<uint8_t>*>*& out_ffiHashMap)
        {
            if (out_ffiHashMap) {
                sgble::sg_ffi_drop_hashmap_u16_vec_u8(out_ffiHashMap);
                out_ffiHashMap = nullptr;
            }
        }

        void DropFfiHashMap(sgble::SgFfiHashMap<sgble::SgFfiString*, sgble::SgFfiVec<uint8_t>*>*& out_ffiHashMap)
        {
            if (out_ffiHashMap) {
                sgble::sg_ffi_drop_hashmap_string_vec_u8(out_ffiHashMap);
                out_ffiHashMap = nullptr;
            }
        }

        void DropFfiDescriptor(sgble::SgFfiBleDescriptor*& out_ffiDescriptor)
        {
            if (out_ffiDescriptor) {
                sgble::sg_ffi_drop_descriptor(out_ffiDescriptor);
                out_ffiDescriptor = nullptr;
            }
        }

        void DropFfiCharacteristic(sgble::SgFfiBleCharacteristic*& out_ffiCharacteristic)
        {
            if (out_ffiCharacteristic) {
                sgble::sg_ffi_drop_characteristic(out_ffiCharacteristic);
                out_ffiCharacteristic = nullptr;
            }
        }

        void DropFfiService(sgble::SgFfiBleService*& out_ffiService)
        {
            if (out_ffiService) {
                sgble::sg_ffi_drop_service(out_ffiService);
                out_ffiService = nullptr;
            }
        }

        void DropFfiPeripheralProperties(sgble::SgFfiBlePeripheralProperties*& out_ffiPeripheralProperties)
        {
            if (out_ffiPeripheralProperties) {
                sgble::sg_ffi_drop_peripheral_properties(out_ffiPeripheralProperties);
                out_ffiPeripheralProperties = nullptr;
            }
        }

        void DropFfiPeripheral(sgble::SgFfiBlePeripheral*& out_ffiPeripheral)
        {
            if (out_ffiPeripheral) {
                sgble::sg_ffi_drop_peripheral(out_ffiPeripheral);
                out_ffiPeripheral = nullptr;
            }
        }
    }// namespace
} /* namespace SGBLExx */
