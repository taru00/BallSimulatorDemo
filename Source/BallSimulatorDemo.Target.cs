// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

using UnrealBuildTool;
using System.Collections.Generic;

public class BallSimulatorDemoTarget : TargetRules
{
	public BallSimulatorDemoTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new string[] { "BallSimulatorDemo" } );
	}
}
