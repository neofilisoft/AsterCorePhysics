using UnrealBuildTool;
using System.IO;

public class AsterCorePhysicsRuntime : ModuleRules
{
    public AsterCorePhysicsRuntime(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "Projects"
        });

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "ThirdParty", "AsterCore", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string PluginDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
            string BinaryDir = Path.Combine(PluginDir, "Binaries", "ThirdParty", "Win64");
            string DllPath = Path.Combine(BinaryDir, "AsterCoreCAPI.dll");

            PublicDelayLoadDLLs.Add("AsterCoreCAPI.dll");

            if (File.Exists(DllPath))
            {
                RuntimeDependencies.Add(DllPath);
            }
        }
    }
}
