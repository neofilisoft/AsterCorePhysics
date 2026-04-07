#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *GDExtensionVariantPtr;
typedef const void *GDExtensionConstVariantPtr;
typedef void *GDExtensionUninitializedVariantPtr;
typedef void *GDExtensionStringNamePtr;
typedef const void *GDExtensionConstStringNamePtr;
typedef void *GDExtensionUninitializedStringNamePtr;
typedef void *GDExtensionStringPtr;
typedef const void *GDExtensionConstStringPtr;
typedef void *GDExtensionUninitializedStringPtr;
typedef void *GDExtensionObjectPtr;
typedef const void *GDExtensionConstObjectPtr;
typedef void *GDExtensionTypePtr;
typedef const void *GDExtensionConstTypePtr;
typedef int64_t GDExtensionInt;
typedef uint8_t GDExtensionBool;
typedef void *GDExtensionClassInstancePtr;
typedef void *GDExtensionClassLibraryPtr;
typedef void *GDExtensionInterfaceFunctionPtr;
typedef GDExtensionInterfaceFunctionPtr (*GDExtensionInterfaceGetProcAddress)(const char *p_name);

typedef enum {
    GDEXTENSION_VARIANT_TYPE_NIL,
    GDEXTENSION_VARIANT_TYPE_BOOL,
    GDEXTENSION_VARIANT_TYPE_INT,
    GDEXTENSION_VARIANT_TYPE_FLOAT,
    GDEXTENSION_VARIANT_TYPE_STRING,
    GDEXTENSION_VARIANT_TYPE_VECTOR2,
    GDEXTENSION_VARIANT_TYPE_VECTOR2I,
    GDEXTENSION_VARIANT_TYPE_RECT2,
    GDEXTENSION_VARIANT_TYPE_RECT2I,
    GDEXTENSION_VARIANT_TYPE_VECTOR3,
    GDEXTENSION_VARIANT_TYPE_VECTOR3I,
    GDEXTENSION_VARIANT_TYPE_TRANSFORM2D,
    GDEXTENSION_VARIANT_TYPE_VECTOR4,
    GDEXTENSION_VARIANT_TYPE_VECTOR4I,
    GDEXTENSION_VARIANT_TYPE_PLANE,
    GDEXTENSION_VARIANT_TYPE_QUATERNION,
    GDEXTENSION_VARIANT_TYPE_AABB,
    GDEXTENSION_VARIANT_TYPE_BASIS,
    GDEXTENSION_VARIANT_TYPE_TRANSFORM3D,
    GDEXTENSION_VARIANT_TYPE_PROJECTION,
    GDEXTENSION_VARIANT_TYPE_COLOR,
    GDEXTENSION_VARIANT_TYPE_STRING_NAME,
    GDEXTENSION_VARIANT_TYPE_NODE_PATH,
    GDEXTENSION_VARIANT_TYPE_RID,
    GDEXTENSION_VARIANT_TYPE_OBJECT,
    GDEXTENSION_VARIANT_TYPE_CALLABLE,
    GDEXTENSION_VARIANT_TYPE_SIGNAL,
    GDEXTENSION_VARIANT_TYPE_DICTIONARY,
    GDEXTENSION_VARIANT_TYPE_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_BYTE_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_INT32_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_INT64_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_FLOAT32_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_FLOAT64_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_STRING_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_VECTOR2_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_VECTOR3_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_COLOR_ARRAY,
    GDEXTENSION_VARIANT_TYPE_PACKED_VECTOR4_ARRAY,
    GDEXTENSION_VARIANT_TYPE_VARIANT_MAX
} GDExtensionVariantType;

typedef enum {
    GDEXTENSION_CALL_OK,
    GDEXTENSION_CALL_ERROR_INVALID_METHOD,
    GDEXTENSION_CALL_ERROR_INVALID_ARGUMENT,
    GDEXTENSION_CALL_ERROR_TOO_MANY_ARGUMENTS,
    GDEXTENSION_CALL_ERROR_TOO_FEW_ARGUMENTS,
    GDEXTENSION_CALL_ERROR_INSTANCE_IS_NULL,
    GDEXTENSION_CALL_ERROR_METHOD_NOT_CONST,
} GDExtensionCallErrorType;

typedef struct {
    GDExtensionCallErrorType error;
    int32_t argument;
    int32_t expected;
} GDExtensionCallError;

typedef enum {
    GDEXTENSION_INITIALIZATION_CORE,
    GDEXTENSION_INITIALIZATION_SERVERS,
    GDEXTENSION_INITIALIZATION_SCENE,
    GDEXTENSION_INITIALIZATION_EDITOR,
    GDEXTENSION_MAX_INITIALIZATION_LEVEL,
} GDExtensionInitializationLevel;

typedef void (*GDExtensionInitializeCallback)(void *p_userdata, GDExtensionInitializationLevel p_level);
typedef void (*GDExtensionDeinitializeCallback)(void *p_userdata, GDExtensionInitializationLevel p_level);

typedef struct {
    GDExtensionInitializationLevel minimum_initialization_level;
    void *userdata;
    GDExtensionInitializeCallback initialize;
    GDExtensionDeinitializeCallback deinitialize;
} GDExtensionInitialization;

typedef void (*GDExtensionVariantFromTypeConstructorFunc)(GDExtensionUninitializedVariantPtr, GDExtensionTypePtr);
typedef void (*GDExtensionTypeFromVariantConstructorFunc)(GDExtensionTypePtr, GDExtensionVariantPtr);
typedef void (*GDExtensionPtrDestructor)(GDExtensionTypePtr p_base);

typedef struct {
    GDExtensionVariantType type;
    GDExtensionStringNamePtr name;
    GDExtensionStringNamePtr class_name;
    uint32_t hint;
    GDExtensionStringPtr hint_string;
    uint32_t usage;
} GDExtensionPropertyInfo;

typedef void (*GDExtensionClassMethodCall)(void *method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstVariantPtr *p_args, GDExtensionInt p_argument_count, GDExtensionVariantPtr r_return, GDExtensionCallError *r_error);
typedef void (*GDExtensionClassMethodPtrCall)(void *method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstTypePtr *p_args, GDExtensionTypePtr r_ret);

typedef enum {
    GDEXTENSION_METHOD_FLAG_NORMAL = 1,
    GDEXTENSION_METHOD_FLAG_EDITOR = 2,
    GDEXTENSION_METHOD_FLAG_CONST = 4,
    GDEXTENSION_METHOD_FLAG_VIRTUAL = 8,
    GDEXTENSION_METHOD_FLAG_VARARG = 16,
    GDEXTENSION_METHOD_FLAG_STATIC = 32,
    GDEXTENSION_METHOD_FLAGS_DEFAULT = GDEXTENSION_METHOD_FLAG_NORMAL,
} GDExtensionClassMethodFlags;

typedef enum {
    GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT8,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT16,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT32,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_UINT8,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_UINT16,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_UINT32,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_UINT64,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_FLOAT,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_CHAR16,
    GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_CHAR32,
} GDExtensionClassMethodArgumentMetadata;

typedef struct {
    GDExtensionStringNamePtr name;
    void *method_userdata;
    GDExtensionClassMethodCall call_func;
    GDExtensionClassMethodPtrCall ptrcall_func;
    uint32_t method_flags;
    GDExtensionBool has_return_value;
    GDExtensionPropertyInfo *return_value_info;
    GDExtensionClassMethodArgumentMetadata return_value_metadata;
    uint32_t argument_count;
    GDExtensionPropertyInfo *arguments_info;
    GDExtensionClassMethodArgumentMetadata *arguments_metadata;
    uint32_t default_argument_count;
    GDExtensionVariantPtr *default_arguments;
} GDExtensionClassMethodInfo;

typedef GDExtensionObjectPtr (*GDExtensionClassCreateInstance2)(void *p_class_userdata, GDExtensionBool p_notify_postinitialize);
typedef void (*GDExtensionClassFreeInstance)(void *p_class_userdata, GDExtensionClassInstancePtr p_instance);

typedef struct {
    GDExtensionBool is_virtual;
    GDExtensionBool is_abstract;
    GDExtensionBool is_exposed;
    GDExtensionBool is_runtime;
    GDExtensionConstStringPtr icon_path;
    void *set_func;
    void *get_func;
    void *get_property_list_func;
    void *free_property_list_func;
    void *property_can_revert_func;
    void *property_get_revert_func;
    void *validate_property_func;
    void *notification_func;
    void *to_string_func;
    void *reference_func;
    void *unreference_func;
    GDExtensionClassCreateInstance2 create_instance_func;
    GDExtensionClassFreeInstance free_instance_func;
    void *recreate_instance_func;
    void *get_virtual_func;
    void *get_virtual_call_data_func;
    void *call_virtual_with_data_func;
    void *class_userdata;
} GDExtensionClassCreationInfo5;

typedef void (*GDExtensionInterfaceStringNameNewWithLatin1Chars)(GDExtensionUninitializedStringNamePtr r_dest, const char *p_contents, GDExtensionBool p_is_static);
typedef void (*GDExtensionInterfaceStringNewWithLatin1Chars)(GDExtensionUninitializedStringPtr r_dest, const char *p_contents);
typedef GDExtensionVariantFromTypeConstructorFunc (*GDExtensionInterfaceGetVariantFromTypeConstructor)(GDExtensionVariantType p_type);
typedef GDExtensionTypeFromVariantConstructorFunc (*GDExtensionInterfaceGetVariantToTypeConstructor)(GDExtensionVariantType p_type);
typedef GDExtensionPtrDestructor (*GDExtensionInterfaceVariantGetPtrDestructor)(GDExtensionVariantType p_type);
typedef GDExtensionObjectPtr (*GDExtensionInterfaceClassdbConstructObject)(GDExtensionConstStringNamePtr p_classname);
typedef GDExtensionObjectPtr (*GDExtensionInterfaceClassdbConstructObject2)(GDExtensionConstStringNamePtr p_classname);
typedef void (*GDExtensionInterfaceObjectSetInstance)(GDExtensionObjectPtr p_o, GDExtensionConstStringNamePtr p_classname, GDExtensionClassInstancePtr p_instance);
typedef void (*GDExtensionInterfaceClassdbRegisterExtensionClass4)(GDExtensionClassLibraryPtr p_library, GDExtensionConstStringNamePtr p_class_name, GDExtensionConstStringNamePtr p_parent_class_name, const GDExtensionClassCreationInfo5 *p_extension_funcs);
typedef void (*GDExtensionInterfaceClassdbRegisterExtensionClass5)(GDExtensionClassLibraryPtr p_library, GDExtensionConstStringNamePtr p_class_name, GDExtensionConstStringNamePtr p_parent_class_name, const GDExtensionClassCreationInfo5 *p_extension_funcs);
typedef void (*GDExtensionInterfaceClassdbRegisterExtensionClassMethod)(GDExtensionClassLibraryPtr p_library, GDExtensionConstStringNamePtr p_class_name, const GDExtensionClassMethodInfo *p_method_info);

#ifdef __cplusplus
}
#endif
