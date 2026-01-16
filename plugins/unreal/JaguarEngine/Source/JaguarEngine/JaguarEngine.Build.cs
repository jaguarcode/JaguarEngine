// Copyright JaguarEngine Team. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class JaguarEngine : ModuleRules
{
    public JaguarEngine(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        // C++20 support
        CppStandard = CppStandardVersion.Cpp20;

        PublicIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Public"),
            }
        );

        PrivateIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Private"),
            }
        );

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "InputCore",
                "GeometryCore",
                "GeometryFramework",
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Slate",
                "SlateCore",
                "RenderCore",
                "RHI",
                "Projects",
                "DeveloperSettings",
            }
        );

        // JaguarEngine native library linking
        string JaguarLibPath = Path.Combine(ModuleDirectory, "..", "..", "..", "..", "lib");
        string JaguarIncludePath = Path.Combine(ModuleDirectory, "..", "..", "..", "..", "include");

        if (Directory.Exists(JaguarLibPath))
        {
            PublicIncludePaths.Add(JaguarIncludePath);

            if (Target.Platform == UnrealTargetPlatform.Win64)
            {
                PublicAdditionalLibraries.Add(Path.Combine(JaguarLibPath, "Win64", "jaguar.lib"));
                RuntimeDependencies.Add(Path.Combine(JaguarLibPath, "Win64", "jaguar.dll"));
            }
            else if (Target.Platform == UnrealTargetPlatform.Linux)
            {
                PublicAdditionalLibraries.Add(Path.Combine(JaguarLibPath, "Linux", "libjaguar.so"));
                RuntimeDependencies.Add(Path.Combine(JaguarLibPath, "Linux", "libjaguar.so"));
            }
            else if (Target.Platform == UnrealTargetPlatform.Mac)
            {
                PublicAdditionalLibraries.Add(Path.Combine(JaguarLibPath, "Mac", "libjaguar.dylib"));
                RuntimeDependencies.Add(Path.Combine(JaguarLibPath, "Mac", "libjaguar.dylib"));
            }
        }

        // Enable RTTI for JaguarEngine compatibility
        bUseRTTI = true;
        bEnableExceptions = true;

        // Preprocessor definitions
        PublicDefinitions.Add("JAGUAR_UE5_PLUGIN=1");

        if (Target.Configuration == UnrealTargetConfiguration.Debug ||
            Target.Configuration == UnrealTargetConfiguration.DebugGame)
        {
            PublicDefinitions.Add("JAGUAR_DEBUG=1");
        }
    }
}
