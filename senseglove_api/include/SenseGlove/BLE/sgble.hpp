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
 * The main header file for the SGBLE C++ exported functions.
 */


#pragma once

#include <SenseGlove/BLE/Platform.hpp>

#if SG_PLATFORM_ANDROID
#include <jni.h>
#endif  /* SG_PLATFORM_ANDROID */

#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

namespace sgble {

enum class SgFfiBleWriteType {
  EsgWithResponse,
  EsgWithoutResponse,
};

struct SgFfiString {
  char *value;
  bool initialized;
};

struct SgFfiEmptyResult {
  SgFfiString *error;
  bool initialized;
};

using SgFfiPeripheralEventCallback = void(*)(void *context, SgFfiString *peripheral_id);

template<typename T>
struct SgFfiOption {
  bool has_value;
  T value;
  bool initialized;
};

template<typename T>
struct SgFfiVec {
  T *array;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
};

template<typename K, typename V>
struct SgFfiHashMap {
  K *keys;
  V *values;
  uintptr_t size;
  uintptr_t keys_capacity;
  uintptr_t values_capacity;
  bool initialized;
};

struct SgFfiBlePeripheralProperties {
  SgFfiString *address;
  SgFfiOption<SgFfiString*> *local_name;
  SgFfiOption<int16_t> *tx_power_level;
  SgFfiOption<int16_t> *rssi;
  SgFfiHashMap<uint16_t, SgFfiVec<uint8_t>*> *manufacturer_data;
  SgFfiHashMap<SgFfiString*, SgFfiVec<uint8_t>*> *service_data;
  SgFfiVec<SgFfiString*> *services;
  SgFfiOption<uint32_t> *class_;
  bool initialized;
};

struct SgFfiBleDescriptor {
  SgFfiString *uuid;
  SgFfiString *service_uuid;
  SgFfiString *characteristic_uuid;
  bool initialized;
};

template<typename T>
struct SgFfiBTreeSet {
  T *elements;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
};

struct SgFfiBleCharacteristic {
  SgFfiString *uuid;
  SgFfiString *service_uuid;
  uint8_t properties;
  SgFfiBTreeSet<SgFfiBleDescriptor*> *descriptors;
  bool initialized;
};

struct SgFfiBleService {
  SgFfiString *uuid;
  bool primary;
  SgFfiBTreeSet<SgFfiBleCharacteristic*> *characteristics;
  bool initialized;
};

struct SgFfiBlePeripheral {
  SgFfiString *id;
  SgFfiBlePeripheralProperties *properties;
  SgFfiBTreeSet<SgFfiBleService*> *services;
  SgFfiBTreeSet<SgFfiBleCharacteristic*> *characteristics;
  bool initialized;
};

template<typename T>
struct SgFfiResult {
  T value;
  SgFfiString *error;
  bool initialized;
};

using SgFfiBleSubscribeCallback = void(*)(void *context,
                                          SgFfiString *peripheral_id,
                                          SgFfiString *characteristic_id,
                                          SgFfiVec<uint8_t>*);

extern "C" {

#if defined(__ANDROID__)
#endif

bool sg_is_initialized();

SgFfiEmptyResult *sg_initialize();

SgFfiEmptyResult *sg_initialize_with_peripheral_events_callbacks(void *on_device_discovered_callback_context,
                                                                 SgFfiPeripheralEventCallback on_device_discovered_callback,
                                                                 void *on_device_connected_callback_context,
                                                                 SgFfiPeripheralEventCallback on_device_connected_callback,
                                                                 void *on_device_disconnected_callback_context,
                                                                 SgFfiPeripheralEventCallback on_device_disconnected_callback);

SgFfiEmptyResult *sg_terminate();

SgFfiEmptyResult *sg_subscribe_to_on_peripheral_discovered(void *callback_context,
                                                           SgFfiPeripheralEventCallback callback);

SgFfiEmptyResult *sg_subscribe_to_on_peripheral_connected(void *callback_context,
                                                          SgFfiPeripheralEventCallback callback);

SgFfiEmptyResult *sg_subscribe_to_on_peripheral_disconnected(void *callback_context,
                                                             SgFfiPeripheralEventCallback callback);

SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_discovered();

SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_connected();

SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_disconnected();

SgFfiResult<SgFfiVec<SgFfiBlePeripheral*>*> *sg_get_peripherals();

SgFfiResult<SgFfiBlePeripheral*> *sg_get_peripheral_by_address(SgFfiString *address);

SgFfiResult<SgFfiBlePeripheral*> *sg_get_peripheral_by_local_name(SgFfiString *local_name);

SgFfiResult<bool> *sg_peripheral_is_connected(SgFfiBlePeripheral *peripheral);

SgFfiEmptyResult *sg_peripheral_connect(SgFfiBlePeripheral *peripheral);

SgFfiEmptyResult *sg_peripheral_disconnect(SgFfiBlePeripheral *peripheral);

SgFfiResult<SgFfiBleCharacteristic*> *get_peripheral_characteristic_by_uuid_string(SgFfiBlePeripheral *peripheral,
                                                                                   SgFfiString *uuid_string);

SgFfiResult<SgFfiBleDescriptor*> *get_peripheral_descriptor_by_uuid_string(SgFfiBlePeripheral *peripheral,
                                                                           SgFfiBleCharacteristic *characteristic,
                                                                           SgFfiString *uuid_string);

SgFfiResult<SgFfiVec<uint8_t>*> *sg_peripheral_read(SgFfiBlePeripheral *peripheral,
                                                    SgFfiBleCharacteristic *characteristic);

SgFfiEmptyResult *sg_peripheral_write(SgFfiBlePeripheral *peripheral,
                                      SgFfiBleCharacteristic *characteristic,
                                      const uint8_t *data,
                                      uintptr_t data_len,
                                      SgFfiBleWriteType write_type);

SgFfiEmptyResult *sg_peripheral_unsubscribe_all(SgFfiBlePeripheral *peripheral,
                                                SgFfiBleCharacteristic *characteristic);

SgFfiEmptyResult *sg_peripheral_subscribe(SgFfiBlePeripheral *peripheral,
                                          SgFfiBleCharacteristic *characteristic,
                                          void *callback_context,
                                          SgFfiBleSubscribeCallback callback);

SgFfiResult<SgFfiVec<uint8_t>*> *sg_peripheral_read_descriptor(SgFfiBlePeripheral *peripheral,
                                                               SgFfiBleDescriptor *descriptor);

SgFfiEmptyResult *sg_peripheral_write_descriptor(SgFfiBlePeripheral *peripheral,
                                                 SgFfiBleDescriptor *descriptor,
                                                 const uint8_t *data,
                                                 uintptr_t data_len);

SgFfiResult<uint32_t> *sg_peripheral_get_packets_per_second(SgFfiBlePeripheral *peripheral);

SgFfiString *sg_logs_get();

void sg_logs_clear();

SgFfiVec<SgFfiString*> *sg_ffi_get_alloc_tracker_type_names();

#if defined(__ANDROID__)
jint initialize_android(JNIEnv *env_ptr);
#endif

SgFfiString *sg_ffi_new_string(const char *value);

void sg_ffi_drop_string(SgFfiString *string);

void sg_ffi_drop_option_i16(SgFfiOption<int16_t> *option);

void sg_ffi_drop_option_u32(SgFfiOption<uint32_t> *option);

void sg_ffi_drop_option_string(SgFfiOption<SgFfiString*> *option);

void sg_ffi_drop_empty_result(SgFfiEmptyResult *result);

void sg_ffi_drop_result_bool(SgFfiResult<bool> *result);

void sg_ffi_drop_result_u32(SgFfiResult<uint32_t> *result);

void sg_ffi_drop_result_string(SgFfiResult<SgFfiString*> *result);

void sg_ffi_drop_result_vec_u8(SgFfiResult<SgFfiVec<uint8_t>*> *result);

void sg_ffi_drop_result_vec_peripheral(SgFfiResult<SgFfiVec<SgFfiBlePeripheral*>*> *result);

void sg_ffi_drop_result_descriptor(SgFfiResult<SgFfiBleDescriptor*> *result);

void sg_ffi_drop_result_characteristic(SgFfiResult<SgFfiBleCharacteristic*> *result);

void sg_ffi_drop_result_peripheral(SgFfiResult<SgFfiBlePeripheral*> *result);

void sg_ffi_drop_vec_u8(SgFfiVec<uint8_t> *vec);

void sg_ffi_drop_vec_string(SgFfiVec<SgFfiString*> *vec);

void sg_ffi_drop_vec_peripheral(SgFfiVec<SgFfiBlePeripheral*> *vec);

void sg_ffi_drop_hashmap_u16_vec_u8(SgFfiHashMap<uint16_t, SgFfiVec<uint8_t>*> *map);

void sg_ffi_drop_hashmap_string_vec_u8(SgFfiHashMap<SgFfiString*, SgFfiVec<uint8_t>*> *map);

void sg_ffi_drop_btreeset_characteristic(SgFfiBTreeSet<SgFfiBleCharacteristic*> *set);

void sg_ffi_drop_btreeset_descriptor(SgFfiBTreeSet<SgFfiBleDescriptor*> *set);

void sg_ffi_drop_btreeset_service(SgFfiBTreeSet<SgFfiBleService*> *set);

void sg_ffi_drop_peripheral_properties(SgFfiBlePeripheralProperties *properties);

void sg_ffi_drop_descriptor(SgFfiBleDescriptor *descriptor);

void sg_ffi_drop_characteristic(SgFfiBleCharacteristic *characteristic);

void sg_ffi_drop_service(SgFfiBleService *service);

void sg_ffi_drop_peripheral(SgFfiBlePeripheral *peripheral);

}  // extern "C"

}  // namespace sgble


