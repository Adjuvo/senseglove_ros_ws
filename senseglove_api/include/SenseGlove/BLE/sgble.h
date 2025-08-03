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
 * The main header file for the SGBLE C exported functions.
 */


#pragma once

#include <SenseGlove/BLE/Platform.hpp>

#if SG_PLATFORM_ANDROID
#include <jni.h>
#endif  /* SG_PLATFORM_ANDROID */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum SgFfiBleWriteType {
  EsgWithResponse,
  EsgWithoutResponse,
} SgFfiBleWriteType;

typedef struct SgFfiString {
  char *value;
  bool initialized;
} SgFfiString;

typedef struct SgFfiEmptyResult {
  struct SgFfiString *error;
  bool initialized;
} SgFfiEmptyResult;

typedef void (*SgFfiPeripheralEventCallback)(void *context, struct SgFfiString *peripheral_id);

typedef struct SgFfiOption_____SgFfiString {
  bool has_value;
  struct SgFfiString *value;
  bool initialized;
} SgFfiOption_____SgFfiString;

typedef struct SgFfiOption_i16 {
  bool has_value;
  int16_t value;
  bool initialized;
} SgFfiOption_i16;

typedef struct SgFfiVec_u8 {
  uint8_t *array;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiVec_u8;

typedef struct SgFfiHashMap_u16______SgFfiVec_u8 {
  uint16_t *keys;
  struct SgFfiVec_u8 **values;
  uintptr_t size;
  uintptr_t keys_capacity;
  uintptr_t values_capacity;
  bool initialized;
} SgFfiHashMap_u16______SgFfiVec_u8;

typedef struct SgFfiHashMap_____SgFfiString______SgFfiVec_u8 {
  struct SgFfiString **keys;
  struct SgFfiVec_u8 **values;
  uintptr_t size;
  uintptr_t keys_capacity;
  uintptr_t values_capacity;
  bool initialized;
} SgFfiHashMap_____SgFfiString______SgFfiVec_u8;

typedef struct SgFfiVec_____SgFfiString {
  struct SgFfiString **array;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiVec_____SgFfiString;

typedef struct SgFfiOption_u32 {
  bool has_value;
  uint32_t value;
  bool initialized;
} SgFfiOption_u32;

typedef struct SgFfiBlePeripheralProperties {
  struct SgFfiString *address;
  struct SgFfiOption_____SgFfiString *local_name;
  struct SgFfiOption_i16 *tx_power_level;
  struct SgFfiOption_i16 *rssi;
  struct SgFfiHashMap_u16______SgFfiVec_u8 *manufacturer_data;
  struct SgFfiHashMap_____SgFfiString______SgFfiVec_u8 *service_data;
  struct SgFfiVec_____SgFfiString *services;
  struct SgFfiOption_u32 *class_;
  bool initialized;
} SgFfiBlePeripheralProperties;

typedef struct SgFfiBleDescriptor {
  struct SgFfiString *uuid;
  struct SgFfiString *service_uuid;
  struct SgFfiString *characteristic_uuid;
  bool initialized;
} SgFfiBleDescriptor;

typedef struct SgFfiBTreeSet_____SgFfiBleDescriptor {
  struct SgFfiBleDescriptor **elements;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiBTreeSet_____SgFfiBleDescriptor;

typedef struct SgFfiBleCharacteristic {
  struct SgFfiString *uuid;
  struct SgFfiString *service_uuid;
  uint8_t properties;
  struct SgFfiBTreeSet_____SgFfiBleDescriptor *descriptors;
  bool initialized;
} SgFfiBleCharacteristic;

typedef struct SgFfiBTreeSet_____SgFfiBleCharacteristic {
  struct SgFfiBleCharacteristic **elements;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiBTreeSet_____SgFfiBleCharacteristic;

typedef struct SgFfiBleService {
  struct SgFfiString *uuid;
  bool primary;
  struct SgFfiBTreeSet_____SgFfiBleCharacteristic *characteristics;
  bool initialized;
} SgFfiBleService;

typedef struct SgFfiBTreeSet_____SgFfiBleService {
  struct SgFfiBleService **elements;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiBTreeSet_____SgFfiBleService;

typedef struct SgFfiBlePeripheral {
  struct SgFfiString *id;
  struct SgFfiBlePeripheralProperties *properties;
  struct SgFfiBTreeSet_____SgFfiBleService *services;
  struct SgFfiBTreeSet_____SgFfiBleCharacteristic *characteristics;
  bool initialized;
} SgFfiBlePeripheral;

typedef struct SgFfiVec_____SgFfiBlePeripheral {
  struct SgFfiBlePeripheral **array;
  uintptr_t size;
  uintptr_t capacity;
  bool initialized;
} SgFfiVec_____SgFfiBlePeripheral;

typedef struct SgFfiResult_____SgFfiVec_____SgFfiBlePeripheral {
  struct SgFfiVec_____SgFfiBlePeripheral *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiVec_____SgFfiBlePeripheral;

typedef struct SgFfiResult_____SgFfiBlePeripheral {
  struct SgFfiBlePeripheral *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiBlePeripheral;

typedef struct SgFfiResult_bool {
  bool value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_bool;

typedef struct SgFfiResult_____SgFfiBleCharacteristic {
  struct SgFfiBleCharacteristic *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiBleCharacteristic;

typedef struct SgFfiResult_____SgFfiBleDescriptor {
  struct SgFfiBleDescriptor *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiBleDescriptor;

typedef struct SgFfiResult_____SgFfiVec_u8 {
  struct SgFfiVec_u8 *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiVec_u8;

typedef void (*SgFfiBleSubscribeCallback)(void *context,
                                          struct SgFfiString *peripheral_id,
                                          struct SgFfiString *characteristic_id,
                                          struct SgFfiVec_u8*);

typedef struct SgFfiResult_u32 {
  uint32_t value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_u32;

typedef struct SgFfiResult_____SgFfiString {
  struct SgFfiString *value;
  struct SgFfiString *error;
  bool initialized;
} SgFfiResult_____SgFfiString;

#if defined(__ANDROID__)
jint JNI_OnLoad(JavaVM vm, const void *_res);
#endif

bool sg_is_initialized(void);

struct SgFfiEmptyResult *sg_initialize(void);

struct SgFfiEmptyResult *sg_initialize_with_peripheral_events_callbacks(void *on_device_discovered_callback_context,
                                                                        SgFfiPeripheralEventCallback on_device_discovered_callback,
                                                                        void *on_device_connected_callback_context,
                                                                        SgFfiPeripheralEventCallback on_device_connected_callback,
                                                                        void *on_device_disconnected_callback_context,
                                                                        SgFfiPeripheralEventCallback on_device_disconnected_callback);

struct SgFfiEmptyResult *sg_terminate(void);

struct SgFfiEmptyResult *sg_subscribe_to_on_peripheral_discovered(void *callback_context,
                                                                  SgFfiPeripheralEventCallback callback);

struct SgFfiEmptyResult *sg_subscribe_to_on_peripheral_connected(void *callback_context,
                                                                 SgFfiPeripheralEventCallback callback);

struct SgFfiEmptyResult *sg_subscribe_to_on_peripheral_disconnected(void *callback_context,
                                                                    SgFfiPeripheralEventCallback callback);

struct SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_discovered(void);

struct SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_connected(void);

struct SgFfiEmptyResult *sg_unsubscribe_all_from_on_peripheral_disconnected(void);

struct SgFfiResult_____SgFfiVec_____SgFfiBlePeripheral *sg_get_peripherals(void);

struct SgFfiResult_____SgFfiBlePeripheral *sg_get_peripheral_by_address(struct SgFfiString *address);

struct SgFfiResult_____SgFfiBlePeripheral *sg_get_peripheral_by_local_name(struct SgFfiString *local_name);

struct SgFfiResult_bool *sg_peripheral_is_connected(struct SgFfiBlePeripheral *peripheral);

struct SgFfiEmptyResult *sg_peripheral_connect(struct SgFfiBlePeripheral *peripheral);

struct SgFfiEmptyResult *sg_peripheral_disconnect(struct SgFfiBlePeripheral *peripheral);

struct SgFfiResult_____SgFfiBleCharacteristic *get_peripheral_characteristic_by_uuid_string(struct SgFfiBlePeripheral *peripheral,
                                                                                            struct SgFfiString *uuid_string);

struct SgFfiResult_____SgFfiBleDescriptor *get_peripheral_descriptor_by_uuid_string(struct SgFfiBlePeripheral *peripheral,
                                                                                    struct SgFfiBleCharacteristic *characteristic,
                                                                                    struct SgFfiString *uuid_string);

struct SgFfiResult_____SgFfiVec_u8 *sg_peripheral_read(struct SgFfiBlePeripheral *peripheral,
                                                       struct SgFfiBleCharacteristic *characteristic);

struct SgFfiEmptyResult *sg_peripheral_write(struct SgFfiBlePeripheral *peripheral,
                                             struct SgFfiBleCharacteristic *characteristic,
                                             const uint8_t *data,
                                             uintptr_t data_len,
                                             enum SgFfiBleWriteType write_type);

struct SgFfiEmptyResult *sg_peripheral_unsubscribe_all(struct SgFfiBlePeripheral *peripheral,
                                                       struct SgFfiBleCharacteristic *characteristic);

struct SgFfiEmptyResult *sg_peripheral_subscribe(struct SgFfiBlePeripheral *peripheral,
                                                 struct SgFfiBleCharacteristic *characteristic,
                                                 void *callback_context,
                                                 SgFfiBleSubscribeCallback callback);

struct SgFfiResult_____SgFfiVec_u8 *sg_peripheral_read_descriptor(struct SgFfiBlePeripheral *peripheral,
                                                                  struct SgFfiBleDescriptor *descriptor);

struct SgFfiEmptyResult *sg_peripheral_write_descriptor(struct SgFfiBlePeripheral *peripheral,
                                                        struct SgFfiBleDescriptor *descriptor,
                                                        const uint8_t *data,
                                                        uintptr_t data_len);

struct SgFfiResult_u32 *sg_peripheral_get_packets_per_second(struct SgFfiBlePeripheral *peripheral);

struct SgFfiString *sg_logs_get(void);

void sg_logs_clear(void);

struct SgFfiVec_____SgFfiString *sg_ffi_get_alloc_tracker_type_names(void);

#if defined(__ANDROID__)
jint initialize_android(JNIEnv *env_ptr);
#endif

struct SgFfiString *sg_ffi_new_string(const char *value);

void sg_ffi_drop_string(struct SgFfiString *string);

void sg_ffi_drop_option_i16(struct SgFfiOption_i16 *option);

void sg_ffi_drop_option_u32(struct SgFfiOption_u32 *option);

void sg_ffi_drop_option_string(struct SgFfiOption_____SgFfiString *option);

void sg_ffi_drop_empty_result(struct SgFfiEmptyResult *result);

void sg_ffi_drop_result_bool(struct SgFfiResult_bool *result);

void sg_ffi_drop_result_u32(struct SgFfiResult_u32 *result);

void sg_ffi_drop_result_string(struct SgFfiResult_____SgFfiString *result);

void sg_ffi_drop_result_vec_u8(struct SgFfiResult_____SgFfiVec_u8 *result);

void sg_ffi_drop_result_vec_peripheral(struct SgFfiResult_____SgFfiVec_____SgFfiBlePeripheral *result);

void sg_ffi_drop_result_descriptor(struct SgFfiResult_____SgFfiBleDescriptor *result);

void sg_ffi_drop_result_characteristic(struct SgFfiResult_____SgFfiBleCharacteristic *result);

void sg_ffi_drop_result_peripheral(struct SgFfiResult_____SgFfiBlePeripheral *result);

void sg_ffi_drop_vec_u8(struct SgFfiVec_u8 *vec);

void sg_ffi_drop_vec_string(struct SgFfiVec_____SgFfiString *vec);

void sg_ffi_drop_vec_peripheral(struct SgFfiVec_____SgFfiBlePeripheral *vec);

void sg_ffi_drop_hashmap_u16_vec_u8(struct SgFfiHashMap_u16______SgFfiVec_u8 *map);

void sg_ffi_drop_hashmap_string_vec_u8(struct SgFfiHashMap_____SgFfiString______SgFfiVec_u8 *map);

void sg_ffi_drop_btreeset_characteristic(struct SgFfiBTreeSet_____SgFfiBleCharacteristic *set);

void sg_ffi_drop_btreeset_descriptor(struct SgFfiBTreeSet_____SgFfiBleDescriptor *set);

void sg_ffi_drop_btreeset_service(struct SgFfiBTreeSet_____SgFfiBleService *set);

void sg_ffi_drop_peripheral_properties(struct SgFfiBlePeripheralProperties *properties);

void sg_ffi_drop_descriptor(struct SgFfiBleDescriptor *descriptor);

void sg_ffi_drop_characteristic(struct SgFfiBleCharacteristic *characteristic);

void sg_ffi_drop_service(struct SgFfiBleService *service);

void sg_ffi_drop_peripheral(struct SgFfiBlePeripheral *peripheral);


