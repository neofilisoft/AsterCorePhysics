#include <Integrations/Godot/gdextension_minimal.h>
#include <Bindings/CAPI/AsterCoreCAPI.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <new>
#include <vector>

namespace
{
	struct GDVector3
	{
		float x;
		float y;
		float z;
	};

	template <size_t Size>
	struct OpaqueStorage
	{
		alignas(16) unsigned char data[Size];
		void *ptr() { return data; }
		const void *ptr() const { return data; }
	};

	using GDStringNameStorage = OpaqueStorage<64>;
	using GDStringStorage = OpaqueStorage<64>;

	struct BridgeAPI
	{
		GDExtensionInterfaceStringNameNewWithLatin1Chars string_name_new = nullptr;
		GDExtensionInterfaceStringNewWithLatin1Chars string_new = nullptr;
		GDExtensionInterfaceGetVariantFromTypeConstructor get_variant_from_type = nullptr;
		GDExtensionInterfaceGetVariantToTypeConstructor get_variant_to_type = nullptr;
		GDExtensionInterfaceVariantGetPtrDestructor get_variant_destructor = nullptr;
		GDExtensionInterfaceClassdbConstructObject construct_object = nullptr;
		GDExtensionInterfaceClassdbConstructObject2 construct_object2 = nullptr;
		GDExtensionInterfaceObjectSetInstance object_set_instance = nullptr;
		GDExtensionInterfaceClassdbRegisterExtensionClass4 register_class4 = nullptr;
		GDExtensionInterfaceClassdbRegisterExtensionClassMethod register_method = nullptr;

		GDExtensionVariantFromTypeConstructorFunc variant_from_bool = nullptr;
		GDExtensionVariantFromTypeConstructorFunc variant_from_int = nullptr;
		GDExtensionVariantFromTypeConstructorFunc variant_from_vector3 = nullptr;
		GDExtensionTypeFromVariantConstructorFunc variant_to_bool = nullptr;
		GDExtensionTypeFromVariantConstructorFunc variant_to_int = nullptr;
		GDExtensionTypeFromVariantConstructorFunc variant_to_float = nullptr;
		GDExtensionTypeFromVariantConstructorFunc variant_to_vector3 = nullptr;
		GDExtensionPtrDestructor string_destructor = nullptr;
		GDExtensionPtrDestructor string_name_destructor = nullptr;
	};

	struct GlobalStringName
	{
		GDStringNameStorage storage {};
		bool initialized = false;

		void init(const BridgeAPI &api, const char *text)
		{
			if (!initialized)
			{
				api.string_name_new(storage.ptr(), text, 1);
				initialized = true;
			}
		}

		GDExtensionConstStringNamePtr ptr() const
		{
			return reinterpret_cast<GDExtensionConstStringNamePtr>(storage.ptr());
		}
	};

	struct LocalStringName
	{
		const BridgeAPI &api;
		GDStringNameStorage storage {};

		LocalStringName(const BridgeAPI &in_api, const char *text) : api(in_api)
		{
			api.string_name_new(storage.ptr(), text, 0);
		}

		~LocalStringName()
		{
			api.string_name_destructor(storage.ptr());
		}

		GDExtensionStringNamePtr ptr()
		{
			return reinterpret_cast<GDExtensionStringNamePtr>(storage.ptr());
		}
	};

	struct LocalString
	{
		const BridgeAPI &api;
		GDStringStorage storage {};

		LocalString(const BridgeAPI &in_api, const char *text) : api(in_api)
		{
			api.string_new(storage.ptr(), text);
		}

		~LocalString()
		{
			api.string_destructor(storage.ptr());
		}

		GDExtensionStringPtr ptr()
		{
			return reinterpret_cast<GDExtensionStringPtr>(storage.ptr());
		}
	};

	struct PropertyInfoStorage
	{
		LocalStringName name;
		LocalStringName class_name;
		LocalString hint_string;
		GDExtensionPropertyInfo info {};

		PropertyInfoStorage(const BridgeAPI &api, GDExtensionVariantType type, const char *name_text) :
			name(api, name_text),
			class_name(api, ""),
			hint_string(api, "")
		{
			info.type = type;
			info.name = name.ptr();
			info.class_name = class_name.ptr();
			info.hint = 0;
			info.hint_string = hint_string.ptr();
			info.usage = 6;
		}
	};

	struct MethodArgument
	{
		GDExtensionVariantType type;
		GDExtensionClassMethodArgumentMetadata metadata;
		const char *name;
	};

	struct AsterCoreSpace3DInstance
	{
		ACPH_CAPIWorldHandle world = nullptr;
		ACPH_CAPIRaycastHit last_hit {};
	};

	BridgeAPI g_api;
	GDExtensionClassLibraryPtr g_library = nullptr;
	GlobalStringName g_class_name;
	GlobalStringName g_parent_name;

	template <typename T>
	static T GetProc(GDExtensionInterfaceGetProcAddress get_proc, const char *name)
	{
		return reinterpret_cast<T>(get_proc(name));
	}

	static void SetCallOk(GDExtensionCallError *error)
	{
		if (error != nullptr)
		{
			error->error = GDEXTENSION_CALL_OK;
			error->argument = 0;
			error->expected = 0;
		}
	}

	static bool ExpectArgumentCount(GDExtensionCallError *error, GDExtensionInt actual, GDExtensionInt expected)
	{
		if (actual == expected)
		{
			SetCallOk(error);
			return true;
		}

		if (error != nullptr)
		{
			error->error = actual < expected? GDEXTENSION_CALL_ERROR_TOO_FEW_ARGUMENTS : GDEXTENSION_CALL_ERROR_TOO_MANY_ARGUMENTS;
			error->argument = int32_t(actual);
			error->expected = int32_t(expected);
		}
		return false;
	}

	static GDExtensionBool ToGDBool(bool value)
	{
		return value? GDExtensionBool(1) : GDExtensionBool(0);
	}

	static ACPH_CAPIVec3 ToCapiVec3(const GDVector3 &value)
	{
		ACPH_CAPIVec3 result { value.x, value.y, value.z };
		return result;
	}

	static GDVector3 ToGDVector3(const ACPH_CAPIVec3 &value)
	{
		GDVector3 result { value.x, value.y, value.z };
		return result;
	}

	static bool VariantToBool(GDExtensionConstVariantPtr value)
	{
		GDExtensionBool result = 0;
		g_api.variant_to_bool(&result, const_cast<GDExtensionVariantPtr>(value));
		return result != 0;
	}

	static int64_t VariantToInt(GDExtensionConstVariantPtr value)
	{
		int64_t result = 0;
		g_api.variant_to_int(&result, const_cast<GDExtensionVariantPtr>(value));
		return result;
	}

	static double VariantToFloat(GDExtensionConstVariantPtr value)
	{
		double result = 0.0;
		g_api.variant_to_float(&result, const_cast<GDExtensionVariantPtr>(value));
		return result;
	}

	static GDVector3 VariantToVector3(GDExtensionConstVariantPtr value)
	{
		GDVector3 result { 0.0f, 0.0f, 0.0f };
		g_api.variant_to_vector3(&result, const_cast<GDExtensionVariantPtr>(value));
		return result;
	}

	static void ReturnBool(GDExtensionVariantPtr r_return, bool value)
	{
		if (r_return != nullptr)
		{
			GDExtensionBool temp = ToGDBool(value);
			g_api.variant_from_bool(r_return, &temp);
		}
	}

	static void ReturnInt(GDExtensionVariantPtr r_return, int64_t value)
	{
		if (r_return != nullptr)
			g_api.variant_from_int(r_return, &value);
	}

	static void ReturnVector3(GDExtensionVariantPtr r_return, const GDVector3 &value)
	{
		if (r_return != nullptr)
		{
			GDVector3 temp = value;
			g_api.variant_from_vector3(r_return, &temp);
		}
	}

	static AsterCoreSpace3DInstance *GetInstance(GDExtensionClassInstancePtr instance)
	{
		return reinterpret_cast<AsterCoreSpace3DInstance *>(instance);
	}

	static GDExtensionObjectPtr AsterCoreSpace3D_CreateInstance(void *, GDExtensionBool)
	{
		GDExtensionObjectPtr object = g_api.construct_object != nullptr? g_api.construct_object(g_parent_name.ptr()) : nullptr;
		if (object == nullptr && g_api.construct_object2 != nullptr)
			object = g_api.construct_object2(g_parent_name.ptr());
		if (object == nullptr)
			return nullptr;

		AsterCoreSpace3DInstance *instance = new AsterCoreSpace3DInstance();
		instance->world = ACPH_CAPICreateWorld(nullptr);
		g_api.object_set_instance(object, g_class_name.ptr(), instance);
		return object;
	}

	static void AsterCoreSpace3D_FreeInstance(void *, GDExtensionClassInstancePtr instance_ptr)
	{
		AsterCoreSpace3DInstance *instance = GetInstance(instance_ptr);
		if (instance != nullptr)
		{
			ACPH_CAPIDestroyWorld(instance->world);
			delete instance;
		}
	}

	static void reset_world_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 0))
			return;
		ACPH_CAPIResetWorld(GetInstance(instance)->world);
	}

	static void reset_world_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *, GDExtensionTypePtr)
	{
		ACPH_CAPIResetWorld(GetInstance(instance)->world);
	}

	static void is_runtime_ready_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 0))
			return;
		ReturnBool(r_return, ACPH_CAPIIsWorldValid(GetInstance(instance)->world) != 0);
	}

	static void is_runtime_ready_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDExtensionBool *>(r_ret) = ToGDBool(ACPH_CAPIIsWorldValid(GetInstance(instance)->world) != 0);
	}

	static void step_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 2))
			return;
		ACPH_CAPIWorldStep(GetInstance(instance)->world, float(VariantToFloat(args[0])), uint32_t(VariantToInt(args[1])));
	}

	static void step_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPIWorldStep(GetInstance(instance)->world, float(*reinterpret_cast<const double *>(args[0])), uint32_t(*reinterpret_cast<const int64_t *>(args[1])));
	}

	static void set_gravity_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ACPH_CAPISetGravity(GetInstance(instance)->world, ToCapiVec3(VariantToVector3(args[0])));
	}

	static void set_gravity_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPISetGravity(GetInstance(instance)->world, ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[0])));
	}

	static void get_gravity_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 0))
			return;
		ReturnVector3(r_return, ToGDVector3(ACPH_CAPIGetGravity(GetInstance(instance)->world)));
	}

	static void get_gravity_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDVector3 *>(r_ret) = ToGDVector3(ACPH_CAPIGetGravity(GetInstance(instance)->world));
	}

	static void create_box_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 4))
			return;
		const uint32_t id = ACPH_CAPICreateBoxBody(GetInstance(instance)->world, ToCapiVec3(VariantToVector3(args[0])), ToCapiVec3(VariantToVector3(args[1])), VariantToBool(args[2])? 1 : 0, float(VariantToFloat(args[3])));
		ReturnInt(r_return, int64_t(id));
	}

	static void create_box_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		const uint32_t id = ACPH_CAPICreateBoxBody(GetInstance(instance)->world, ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])), *reinterpret_cast<const GDExtensionBool *>(args[2]) != 0? 1 : 0, float(*reinterpret_cast<const double *>(args[3])));
		if (r_ret != nullptr)
			*reinterpret_cast<int64_t *>(r_ret) = int64_t(id);
	}

	static void create_sphere_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 4))
			return;
		const uint32_t id = ACPH_CAPICreateSphereBody(GetInstance(instance)->world, float(VariantToFloat(args[0])), ToCapiVec3(VariantToVector3(args[1])), VariantToBool(args[2])? 1 : 0, float(VariantToFloat(args[3])));
		ReturnInt(r_return, int64_t(id));
	}

	static void create_sphere_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		const uint32_t id = ACPH_CAPICreateSphereBody(GetInstance(instance)->world, float(*reinterpret_cast<const double *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])), *reinterpret_cast<const GDExtensionBool *>(args[2]) != 0? 1 : 0, float(*reinterpret_cast<const double *>(args[3])));
		if (r_ret != nullptr)
			*reinterpret_cast<int64_t *>(r_ret) = int64_t(id);
	}

	static void destroy_body_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ACPH_CAPIDestroyBody(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])));
	}

	static void destroy_body_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPIDestroyBody(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0])));
	}

	static void has_body_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ReturnBool(r_return, ACPH_CAPIHasBody(GetInstance(instance)->world, uint32_t(VariantToInt(args[0]))) != 0);
	}

	static void has_body_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDExtensionBool *>(r_ret) = ToGDBool(ACPH_CAPIHasBody(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0]))) != 0);
	}

	static void is_body_active_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ReturnBool(r_return, ACPH_CAPIIsBodyActive(GetInstance(instance)->world, uint32_t(VariantToInt(args[0]))) != 0);
	}

	static void is_body_active_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDExtensionBool *>(r_ret) = ToGDBool(ACPH_CAPIIsBodyActive(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0]))) != 0);
	}

	static void get_body_position_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ReturnVector3(r_return, ToGDVector3(ACPH_CAPIGetBodyPosition(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])))));
	}

	static void get_body_position_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDVector3 *>(r_ret) = ToGDVector3(ACPH_CAPIGetBodyPosition(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0]))));
	}

	static void set_body_position_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 2))
			return;
		ACPH_CAPISetBodyPosition(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])), ToCapiVec3(VariantToVector3(args[1])));
	}

	static void set_body_position_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPISetBodyPosition(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])));
	}

	static void set_body_linear_velocity_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 2))
			return;
		ACPH_CAPISetBodyLinearVelocity(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])), ToCapiVec3(VariantToVector3(args[1])));
	}

	static void set_body_linear_velocity_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPISetBodyLinearVelocity(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])));
	}

	static void get_body_linear_velocity_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 1))
			return;
		ReturnVector3(r_return, ToGDVector3(ACPH_CAPIGetBodyLinearVelocity(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])))));
	}

	static void get_body_linear_velocity_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDVector3 *>(r_ret) = ToGDVector3(ACPH_CAPIGetBodyLinearVelocity(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0]))));
	}

	static void apply_body_impulse_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 2))
			return;
		ACPH_CAPIApplyBodyImpulse(GetInstance(instance)->world, uint32_t(VariantToInt(args[0])), ToCapiVec3(VariantToVector3(args[1])));
	}

	static void apply_body_impulse_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr)
	{
		ACPH_CAPIApplyBodyImpulse(GetInstance(instance)->world, uint32_t(*reinterpret_cast<const int64_t *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])));
	}

	static void raycast_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *args, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 3))
			return;
		AsterCoreSpace3DInstance *space = GetInstance(instance);
		space->last_hit = {};
		const int hit = ACPH_CAPIRaycast(space->world, ToCapiVec3(VariantToVector3(args[0])), ToCapiVec3(VariantToVector3(args[1])), float(VariantToFloat(args[2])), &space->last_hit);
		ReturnInt(r_return, hit != 0? int64_t(space->last_hit.body_id) : -1);
	}

	static void raycast_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *args, GDExtensionTypePtr r_ret)
	{
		AsterCoreSpace3DInstance *space = GetInstance(instance);
		space->last_hit = {};
		const int hit = ACPH_CAPIRaycast(space->world, ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[0])), ToCapiVec3(*reinterpret_cast<const GDVector3 *>(args[1])), float(*reinterpret_cast<const double *>(args[2])), &space->last_hit);
		if (r_ret != nullptr)
			*reinterpret_cast<int64_t *>(r_ret) = hit != 0? int64_t(space->last_hit.body_id) : -1;
	}

	static void get_last_raycast_position_call(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstVariantPtr *, GDExtensionInt count, GDExtensionVariantPtr r_return, GDExtensionCallError *error)
	{
		if (!ExpectArgumentCount(error, count, 0))
			return;
		ReturnVector3(r_return, ToGDVector3(GetInstance(instance)->last_hit.position));
	}

	static void get_last_raycast_position_ptr(void *, GDExtensionClassInstancePtr instance, const GDExtensionConstTypePtr *, GDExtensionTypePtr r_ret)
	{
		if (r_ret != nullptr)
			*reinterpret_cast<GDVector3 *>(r_ret) = ToGDVector3(GetInstance(instance)->last_hit.position);
	}

	static void RegisterMethod(const char *name, GDExtensionClassMethodCall call, GDExtensionClassMethodPtrCall ptrcall, bool has_return, GDExtensionVariantType return_type, GDExtensionClassMethodArgumentMetadata return_metadata, std::initializer_list<MethodArgument> arguments)
	{
		LocalStringName method_name(g_api, name);
		std::vector<PropertyInfoStorage> arg_storage;
		std::vector<GDExtensionPropertyInfo> arg_infos;
		std::vector<GDExtensionClassMethodArgumentMetadata> arg_meta;
		arg_storage.reserve(arguments.size());
		arg_infos.reserve(arguments.size());
		arg_meta.reserve(arguments.size());

		for (const MethodArgument &argument : arguments)
		{
			arg_storage.emplace_back(g_api, argument.type, argument.name);
			arg_infos.push_back(arg_storage.back().info);
			arg_meta.push_back(argument.metadata);
		}

		PropertyInfoStorage return_storage(g_api, return_type, "return_value");

		GDExtensionClassMethodInfo method_info {};
		method_info.name = method_name.ptr();
		method_info.method_userdata = nullptr;
		method_info.call_func = call;
		method_info.ptrcall_func = ptrcall;
		method_info.method_flags = GDEXTENSION_METHOD_FLAGS_DEFAULT;
		method_info.has_return_value = has_return? 1 : 0;
		method_info.return_value_info = has_return? &return_storage.info : nullptr;
		method_info.return_value_metadata = return_metadata;
		method_info.argument_count = uint32_t(arg_infos.size());
		method_info.arguments_info = arg_infos.empty()? nullptr : arg_infos.data();
		method_info.arguments_metadata = arg_meta.empty()? nullptr : arg_meta.data();
		method_info.default_argument_count = 0;
		method_info.default_arguments = nullptr;
		g_api.register_method(g_library, g_class_name.ptr(), &method_info);
	}

	static void RegisterAsterCoreSpace3D()
	{
		GDExtensionClassCreationInfo5 class_info {};
		class_info.is_virtual = 0;
		class_info.is_abstract = 0;
		class_info.is_exposed = 1;
		class_info.is_runtime = 0;
		class_info.icon_path = nullptr;
		class_info.create_instance_func = &AsterCoreSpace3D_CreateInstance;
		class_info.free_instance_func = &AsterCoreSpace3D_FreeInstance;

		g_api.register_class4(g_library, g_class_name.ptr(), g_parent_name.ptr(), &class_info);

		RegisterMethod("reset_world", &reset_world_call, &reset_world_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {});
		RegisterMethod("is_runtime_ready", &is_runtime_ready_call, &is_runtime_ready_ptr, true, GDEXTENSION_VARIANT_TYPE_BOOL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {});
		RegisterMethod("step", &step_call, &step_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_FLOAT, GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE, "delta" },
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "collision_steps" }
		});
		RegisterMethod("set_gravity", &set_gravity_call, &set_gravity_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "gravity" }
		});
		RegisterMethod("get_gravity", &get_gravity_call, &get_gravity_ptr, true, GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {});
		RegisterMethod("create_box", &create_box_call, &create_box_ptr, true, GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, {
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "half_extents" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "position" },
			{ GDEXTENSION_VARIANT_TYPE_BOOL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "dynamic" },
			{ GDEXTENSION_VARIANT_TYPE_FLOAT, GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE, "mass" }
		});
		RegisterMethod("create_sphere", &create_sphere_call, &create_sphere_ptr, true, GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, {
			{ GDEXTENSION_VARIANT_TYPE_FLOAT, GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE, "radius" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "position" },
			{ GDEXTENSION_VARIANT_TYPE_BOOL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "dynamic" },
			{ GDEXTENSION_VARIANT_TYPE_FLOAT, GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE, "mass" }
		});
		RegisterMethod("destroy_body", &destroy_body_call, &destroy_body_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" }
		});
		RegisterMethod("has_body", &has_body_call, &has_body_ptr, true, GDEXTENSION_VARIANT_TYPE_BOOL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" }
		});
		RegisterMethod("is_body_active", &is_body_active_call, &is_body_active_ptr, true, GDEXTENSION_VARIANT_TYPE_BOOL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" }
		});
		RegisterMethod("get_body_position", &get_body_position_call, &get_body_position_ptr, true, GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" }
		});
		RegisterMethod("set_body_position", &set_body_position_call, &set_body_position_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "position" }
		});
		RegisterMethod("set_body_linear_velocity", &set_body_linear_velocity_call, &set_body_linear_velocity_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "velocity" }
		});
		RegisterMethod("get_body_linear_velocity", &get_body_linear_velocity_call, &get_body_linear_velocity_ptr, true, GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" }
		});
		RegisterMethod("apply_body_impulse", &apply_body_impulse_call, &apply_body_impulse_ptr, false, GDEXTENSION_VARIANT_TYPE_NIL, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {
			{ GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, "body_id" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "impulse" }
		});
		RegisterMethod("raycast", &raycast_call, &raycast_ptr, true, GDEXTENSION_VARIANT_TYPE_INT, GDEXTENSION_METHOD_ARGUMENT_METADATA_INT_IS_INT64, {
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "origin" },
			{ GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, "direction" },
			{ GDEXTENSION_VARIANT_TYPE_FLOAT, GDEXTENSION_METHOD_ARGUMENT_METADATA_REAL_IS_DOUBLE, "max_distance" }
		});
		RegisterMethod("get_last_raycast_position", &get_last_raycast_position_call, &get_last_raycast_position_ptr, true, GDEXTENSION_VARIANT_TYPE_VECTOR3, GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE, {});
	}

	static void AsterCoreInitialize(void *, GDExtensionInitializationLevel level)
	{
		if (level != GDEXTENSION_INITIALIZATION_SCENE)
			return;

		ACPH_CAPIInitialize();
		RegisterAsterCoreSpace3D();
	}

	static void AsterCoreDeinitialize(void *, GDExtensionInitializationLevel level)
	{
		if (level != GDEXTENSION_INITIALIZATION_SCENE)
			return;

		ACPH_CAPIShutdown();
	}
}

extern "C" __declspec(dllexport) GDExtensionBool astercore_godot_library_init(
	GDExtensionInterfaceGetProcAddress get_proc_address,
	GDExtensionClassLibraryPtr library,
	GDExtensionInitialization *initialization)
{
	if (get_proc_address == nullptr || initialization == nullptr)
		return 0;

	g_library = library;
	g_api.string_name_new = GetProc<GDExtensionInterfaceStringNameNewWithLatin1Chars>(get_proc_address, "string_name_new_with_latin1_chars");
	g_api.string_new = GetProc<GDExtensionInterfaceStringNewWithLatin1Chars>(get_proc_address, "string_new_with_latin1_chars");
	g_api.get_variant_from_type = GetProc<GDExtensionInterfaceGetVariantFromTypeConstructor>(get_proc_address, "get_variant_from_type_constructor");
	g_api.get_variant_to_type = GetProc<GDExtensionInterfaceGetVariantToTypeConstructor>(get_proc_address, "get_variant_to_type_constructor");
	g_api.get_variant_destructor = GetProc<GDExtensionInterfaceVariantGetPtrDestructor>(get_proc_address, "variant_get_ptr_destructor");
	g_api.construct_object = GetProc<GDExtensionInterfaceClassdbConstructObject>(get_proc_address, "classdb_construct_object");
	g_api.construct_object2 = GetProc<GDExtensionInterfaceClassdbConstructObject2>(get_proc_address, "classdb_construct_object2");
	g_api.object_set_instance = GetProc<GDExtensionInterfaceObjectSetInstance>(get_proc_address, "object_set_instance");
	g_api.register_class4 = GetProc<GDExtensionInterfaceClassdbRegisterExtensionClass4>(get_proc_address, "classdb_register_extension_class4");
	g_api.register_method = GetProc<GDExtensionInterfaceClassdbRegisterExtensionClassMethod>(get_proc_address, "classdb_register_extension_class_method");

	if (g_api.string_name_new == nullptr || g_api.string_new == nullptr || g_api.get_variant_from_type == nullptr || g_api.get_variant_to_type == nullptr || g_api.get_variant_destructor == nullptr || g_api.object_set_instance == nullptr || g_api.register_method == nullptr || g_api.register_class4 == nullptr)
		return 0;

	g_api.variant_from_bool = g_api.get_variant_from_type(GDEXTENSION_VARIANT_TYPE_BOOL);
	g_api.variant_from_int = g_api.get_variant_from_type(GDEXTENSION_VARIANT_TYPE_INT);
	g_api.variant_from_vector3 = g_api.get_variant_from_type(GDEXTENSION_VARIANT_TYPE_VECTOR3);
	g_api.variant_to_bool = g_api.get_variant_to_type(GDEXTENSION_VARIANT_TYPE_BOOL);
	g_api.variant_to_int = g_api.get_variant_to_type(GDEXTENSION_VARIANT_TYPE_INT);
	g_api.variant_to_float = g_api.get_variant_to_type(GDEXTENSION_VARIANT_TYPE_FLOAT);
	g_api.variant_to_vector3 = g_api.get_variant_to_type(GDEXTENSION_VARIANT_TYPE_VECTOR3);
	g_api.string_destructor = g_api.get_variant_destructor(GDEXTENSION_VARIANT_TYPE_STRING);
	g_api.string_name_destructor = g_api.get_variant_destructor(GDEXTENSION_VARIANT_TYPE_STRING_NAME);

	if (g_api.variant_from_bool == nullptr || g_api.variant_from_int == nullptr || g_api.variant_from_vector3 == nullptr || g_api.variant_to_bool == nullptr || g_api.variant_to_int == nullptr || g_api.variant_to_float == nullptr || g_api.variant_to_vector3 == nullptr || g_api.string_destructor == nullptr || g_api.string_name_destructor == nullptr)
		return 0;

	g_class_name.init(g_api, "AsterCoreSpace3D");
	g_parent_name.init(g_api, "RefCounted");

	initialization->minimum_initialization_level = GDEXTENSION_INITIALIZATION_SCENE;
	initialization->userdata = nullptr;
	initialization->initialize = &AsterCoreInitialize;
	initialization->deinitialize = &AsterCoreDeinitialize;
	return 1;
}
