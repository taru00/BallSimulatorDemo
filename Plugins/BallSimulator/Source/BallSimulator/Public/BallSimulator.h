// © 2025 UnrealStudy. All rights reserved.
// Author: taru00@gmail.com | https://x.com/3devnote

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FBallSimulatorModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
