// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

using UnrealBuildTool;
using System.Collections.Generic;

public class BallSimulatorDemoEditorTarget : TargetRules
{
	public BallSimulatorDemoEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new string[] { "BallSimulatorDemo" } );
	}
}
